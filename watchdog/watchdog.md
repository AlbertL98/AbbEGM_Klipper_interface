# Watchdog / Snapshot-Logger für EGM + Extruder

## Ziel

Es soll **kein zeitlich absolut synchronisiertes Messsystem** gebaut werden, sondern ein Logger, der **möglichst frische und möglichst gleichzeitig gültige Istwerte** von

- dem **Roboter** (`x`, `y`, `z` aus EGM-Feedback)
- dem **Extruder** (`e` bzw. optional abgeleitet `v_e` aus Klipper)

in **einer gemeinsamen Logzeile** speichert.

Der Fokus ist also nicht auf absoluter Zeit, sondern auf:

1. **möglichst wenig Zeit zwischen beiden Werten**
2. **möglichst frische Daten**
3. **einfacher nachträglicher Vergleich in CSV**

---

## Ausgangslage im aktuellen Setup

### Roboterseite

Das vorhandene Skript `EGM_READ.py` hört direkt auf den EGM-Feedback-Port `6510` und liest aus jedem eingehenden `EgmRobot`-Paket:

- `seq`
- `tm`
- `x`
- `y`
- `z`

Damit liegt der Roboter-Istwert bereits in einem guten, frischen Datenpfad vor. fileciteturn5file0

### Bridge / bestehende Architektur

In der vorhandenen Bridge existiert derselbe Grundgedanke bereits systemisch:

- EGM-Feedback kommt als eigener RX-Strom herein
- Telemetrie schreibt `rx.csv`
- der `SyncMonitor` bekommt fortlaufend Soll- und Istwerte

Die Bridge trennt also bereits klar zwischen Klipper-Plan, EGM-Senden und EGM-Feedback. fileciteturn5file3turn5file4turn5file6turn5file7

### Klipperseite

Die `printer.cfg` enthält einen normalen `[extruder]` sowie `[move_export]`. Der Extruder läuft also in Klipper regulär mit, und `move_export` streamt zusätzlich geplante Segmente auf TCP-Port `7200`. `move_export` ist für geplante Bahnsegmente nützlich, aber **nicht** die beste Quelle für den gesuchten **frischen Istwert** des Extruders. fileciteturn5file16

---

## Zielarchitektur

Der Watchdog wird als **Snapshot-Logger** aufgebaut.

### Grundidee

Es gibt **zwei Eingänge**:

1. **EGM-RX** für frische Roboter-Istwerte
2. **Klipper-Status-Feed** für den letzten bekannten Extruderwert

Sobald ein **neues EGM-Feedback** empfangen wird, wird **sofort** ein Snapshot erzeugt:

- `robot_x`
- `robot_y`
- `robot_z`
- `extruder_e`
- optional `extruder_v`
- `dt_ms`
- `extruder_age_ms`

Diese Werte landen gemeinsam in **einer CSV-Zeile**.

---

## Warum nicht bei jedem EGM-Paket aktiv Klipper abfragen?

Die naive Variante wäre:

1. EGM-Paket empfangen
2. Request an Klipper senden
3. Antwort abwarten
4. gemeinsam loggen

Das ist **nicht** die empfohlene Lösung, weil dadurch zusätzlicher Versatz entsteht:

- Request/Response-Latenz
- Scheduling-Jitter
- die Klipper-Antwort kann älter sein als der zuletzt bereits bekannte Status

Für dieses Problem ist es besser, den Extruderwert **kontinuierlich im Hintergrund aktuell zu halten** und ihn beim EGM-Ereignis nur noch aus einem Shared State auszulesen.

---

## Empfohlene Datenquelle für den Extruder

### Erste Wahl: Klipper-Subscription auf `toolhead.position`

Empfohlene Startlösung:

- Python-Thread verbindet sich auf den Klipper-API-Socket
- `objects/subscribe` auf `toolhead.position`
- aus der Position wird die **E-Komponente** gelesen
- der letzte Wert wird thread-sicher gespeichert

Vorteile:

- geringer Implementierungsaufwand
- robuster API-Weg
- keine aktive Anfrage pro EGM-Paket
- ausreichend genau für einen Snapshot-Logger

### Was genau geloggt wird

Minimal:

- `E` = aktuelle Extruderposition aus Klipper

Optional zusätzlich:

- `v_e` = aus zwei aufeinanderfolgenden `E`-Werten numerisch abgeleitete Extrudergeschwindigkeit

### Zweite Wahl: Low-Level-Stepper-Stream

Falls sich später zeigt, dass `toolhead.position` zu träge oder zu grob ist, kann als zweite Ausbaustufe ein näherer MCU-/Stepper-Statuspfad verwendet werden.

Das ist aber ausdrücklich **nicht** die erste Umsetzungsstufe, weil der Aufwand deutlich höher ist.

---

## Funktionsprinzip des Watchdogs

### Threads / Aufgaben

#### 1. EGM-Empfangsthread

Bestehend aus `EGM_READ.py`:

- wartet auf UDP auf Port `6510`
- parst `EgmRobot`
- extrahiert `seq`, `tm`, `x`, `y`, `z`
- triggert pro Paket genau **einen Snapshot**

#### 2. Klipper-Subscriber-Thread

Neuer Thread:

- verbindet sich auf den lokalen Klipper-API-Socket
- subscribed auf `toolhead.position`
- extrahiert `e`
- aktualisiert den Shared State:
  - `latest_e`
  - `latest_e_recv_mono`
  - optional `latest_e_eventtime`
  - optional `latest_e_velocity`

#### 3. CSV-Logger

Kann im EGM-Thread direkt erfolgen.

Bei jedem EGM-Paket wird geschrieben:

- frischer Robot-Wert aus dem Paket
- letzter Extruderwert aus dem Shared State
- Zeitdifferenzen / Alterswerte

---

## Datenmodell des Shared State

Empfohlenes Minimalmodell:

```python
latest_extruder = {
    "e": None,
    "recv_mono": None,
    "eventtime": None,
    "vel": None,
}
```

Zusätzlich wird ein Lock verwendet:

```python
state_lock = threading.Lock()
```

---

## Trigger-Logik

### Snapshot-Regel

**Trigger ist immer ein neues EGM-Feedback.**

Ablauf:

1. `EgmRobot` empfangen
2. `now_mono = time.monotonic()` setzen
3. `x`, `y`, `z`, `seq`, `tm` aus Paket lesen
4. unter Lock den letzten Extruderwert lesen
5. `extruder_age_ms = now_mono - latest_e_recv_mono`
6. Logzeile schreiben

### Warum Robot als Trigger?

Weil der Roboter-Istwert in deinem Use Case der härtere Echtzeitpfad ist.

Der Snapshot bedeutet dann praktisch:

> „Zum Zeitpunkt dieses frischen Robot-Feedbacks war der zuletzt bekannte Extruderwert E.“

Das ist näher an „gleichzeitig“ als zwei nacheinander ausgeführte Abfragen.

---

## Empfohlenes CSV-Format

### Minimal

```text
host_mono_s,dt_ms,egm_seq,egm_tm_ms,robot_x,robot_y,robot_z,extruder_e,extruder_age_ms
```

### Empfohlen

```text
host_mono_s,dt_ms,egm_seq,egm_tm_ms,robot_x,robot_y,robot_z,extruder_e,extruder_v,extruder_age_ms,klipper_eventtime
```

### Bedeutung der Felder

- `host_mono_s`  
  lokale monotone Zeit beim Schreiben des Snapshots

- `dt_ms`  
  Zeit seit der letzten Logzeile; gut um später unregelmäßige Abstände zu erkennen

- `egm_seq`  
  Sequenznummer des EGM-Feedbacks

- `egm_tm_ms`  
  Roboter-Zeitfeld aus dem EGM-Paket

- `robot_x`, `robot_y`, `robot_z`  
  Istposition des Roboters

- `extruder_e`  
  letzter bekannter Extruderstand aus Klipper

- `extruder_v`  
  optionale numerisch abgeleitete Extrudergeschwindigkeit

- `extruder_age_ms`  
  wie alt der verwendete Extruderwert zum Zeitpunkt des Snapshots war

- `klipper_eventtime`  
  optionaler Zeitstempel aus dem Klipper-Status

---

## Wichtigste Qualitätsmetrik

### `extruder_age_ms`

Dieses Feld ist für die spätere Bewertung zentral.

Es beantwortet die Frage:

> „Wie frisch war der verwendete Extruderwert, als das Robot-Feedback geloggt wurde?“

Interpretation:

- **klein** → guter Snapshot
- **groß** → Extruderwert war schon alt

Beispiel:

- `extruder_age_ms = 1.2` → sehr gut
- `extruder_age_ms = 4.8` → wahrscheinlich noch gut
- `extruder_age_ms = 18.0` → deutlich schlechter

Ohne dieses Feld wäre später schwer zu beurteilen, ob Roboter- und Extruderwert wirklich sinnvoll zusammengehören.

---

## Umgang mit Extrudergeschwindigkeit

### Variante A: Nur `E` loggen

Für die erste Version reicht es, nur die Extruderposition `E` zu loggen.

Vorteile:

- einfach
- robust
- direkt aus Klipper verfügbar

### Variante B: `v_e` aus `E` numerisch ableiten

Wenn zusätzlich die Extrudergeschwindigkeit gewünscht ist:

```text
v_e = (e_now - e_prev) / (t_now - t_prev)
```

Wichtig:

- `v_e` wird im Klipper-Subscriber berechnet, nicht erst im EGM-Thread
- dadurch basiert `v_e` auf zwei echten Extruder-Updates

Empfehlung:

- zuerst `E` stabil loggen
- `v_e` als zweiten Schritt ergänzen

---

## Umgang mit fehlenden oder alten Extruderwerten

Der Logger darf auch dann weiterlaufen, wenn noch kein frischer Extruderwert vorhanden ist.

### Regeln

#### Fall 1: noch kein Extruderwert vorhanden

CSV schreibt:

- `extruder_e = leer`
- `extruder_v = leer`
- `extruder_age_ms = leer`

#### Fall 2: Extruderwert ist vorhanden, aber alt

CSV schreibt den Wert trotzdem, aber `extruder_age_ms` zeigt die Alterung an.

Optional kann zusätzlich eine Warnung ausgegeben werden, z. B. ab:

- `extruder_age_ms > 10 ms`
- `extruder_age_ms > 20 ms`

---

## Umbau von `EGM_READ.py`

### Aktueller Stand

Das Skript liest derzeit nur:

- UDP-EGM auf Port `6510`
- Parse `EgmRobot`
- Ausgabe von `seq`, `tm`, `x`, `y`, `z` auf Konsole fileciteturn5file0

### Geplante Erweiterung

Das Skript wird um diese Bausteine erweitert:

1. `csv`-Import
2. `json`-Import
3. `threading`-Import
4. Klipper-Subscriber-Thread
5. Shared State + Lock
6. CSV-Datei mit Snapshot-Format
7. Berechnung von `dt_ms`
8. Berechnung von `extruder_age_ms`
9. optional Berechnung von `extruder_v`

---

## Warum `move_export` hierfür nicht die Hauptquelle sein sollte

`move_export` exportiert Segmente direkt aus Klippers Bewegungsplanung und streamt sie per TCP an Port `7200`. Das ist ideal für den Planpfad der Bridge. fileciteturn5file1turn5file7turn5file16

Für den geplanten Watchdog ist es aber nicht die beste Hauptquelle, weil hier **kein frischer Istwert**, sondern primär **Plan-/Segmentinformation** vorliegt.

Der Snapshot-Logger soll hingegen möglichst nah an den tatsächlichen momentanen Zuständen liegen:

- Roboter: EGM-Feedback
- Extruder: laufender Klipper-Status

---

## Erweiterungsstufen

### Stufe 1 — Minimal lauffähig

- EGM-RX bleibt wie bisher
- Klipper-Subscription auf `toolhead.position`
- `extruder_e` loggen
- `extruder_age_ms` loggen
- CSV schreiben

### Stufe 2 — Mehr Aussagekraft

- `extruder_v` numerisch berechnen
- Warnung bei altem Extruderwert
- Statistik über mittlere und maximale `extruder_age_ms`

### Stufe 3 — Strenger Watchdog

Zusätzlich zur reinen Logfunktion können Regeln eingeführt werden wie:

- Roboter bewegt sich, Extruder bleibt stehen
- Extruder fördert, Roboter steht nahezu still
- Extruderwert zu alt

Diese Regeln sollten am Anfang nur **warnen**, nicht sofort stoppen.

---

## Empfohlene Akzeptanzkriterien

Der Watchdog ist in Version 1 dann erfolgreich, wenn:

1. das Skript stabil sowohl EGM-Feedback als auch Klipper-Extruderstatus empfängt
2. jede EGM-Nachricht genau eine CSV-Zeile erzeugt
3. `robot_x/y/z` und `extruder_e` gemeinsam in derselben Zeile stehen
4. `dt_ms` plausibel ist
5. `extruder_age_ms` überwiegend klein bleibt
6. keine aktive Klipper-Abfrage pro EGM-Paket nötig ist

---

## Technische Empfehlung

Für die erste Implementierung gilt daher klar:

- **kein Polling beider Seiten in 5 ms Schleife**
- **keine Request/Response-Abfrage an Klipper pro EGM-Ereignis**
- **EGM als Trigger verwenden**
- **Extruderwert kontinuierlich abonnieren und puffern**
- **Alter des Extruderwerts immer mitloggen**

Das ist für den gewünschten Zweck die beste Kombination aus:

- geringem Aufwand
- hoher Frische
- kleinem Versatz
- guter späterer Auswertbarkeit

---

## Konkrete Umsetzung im Projekt

### Bereits vorhanden

- `EGM_READ.py` für Robot-Feedback auf UDP `6510` fileciteturn5file0
- `[extruder]` in `printer.cfg`
- `[move_export]` in `printer.cfg` auf TCP `7200` fileciteturn5file16

### Neu zu implementieren

- Klipper-API-Client im Python-Skript
- Subscription auf den Toolhead-/Extruderstatus
- Shared State für `latest_e`
- CSV-Snapshot-Logging

---

## Kurzfazit

Der Watchdog soll **kein klassischer Fehler-Watchdog** im ersten Schritt sein, sondern ein **Snapshot-Logger**.

Die richtige Grundarchitektur dafür ist:

- **Robot-Feedback triggert den Snapshot**
- **Extruderwert kommt aus einem laufend aktualisierten Klipper-Statuskanal**
- **beide Werte werden gemeinsam geloggt**
- **die Frische des Extruderwerts wird explizit mitgeschrieben**

Damit erhält man genau das, was für die Analyse benötigt wird:

- `x, y, z` vom Roboter
- `e` bzw. optional `v_e` vom Extruder
- gemeinsamer Snapshot pro Robot-Feedback
- nachvollziehbare Zeitabstände zwischen den Logzeilen
- Bewertung der Güte über `extruder_age_ms`
