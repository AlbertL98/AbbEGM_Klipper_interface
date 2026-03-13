# EGM-Bridge — Architektur und Modulreferenz (v0.4.0)

Dieses Dokument beschreibt den Aufbau der EGM-Bridge, die Klipper mit einem ABB-Roboter über das Externally Guided Motion (EGM) Protokoll verbindet. Es soll als einzige Anlaufstelle dienen, um zu verstehen, welche Dateien für welche Funktion zuständig sind und wo Änderungen vorgenommen werden müssen.

---

## Gesamtarchitektur

Die Bridge übersetzt Klippers Bahnplanung in Echtzeit-Sollwerte für den ABB-Roboter. Der Datenfluss ist linear:

```
Klipper (G-Code → Trapez-Segmente)
    │
    │  TCP (JSON-Lines, Port 7200)
    ▼
move_export.py (Klipper-Extra)
    │
    │  TCP-Stream
    ▼
TcpSegmentReceiver (segment_source.py)
    │
    │  Plan-Queue (FIFO)
    ▼
TrajectoryPlanner (trajectory_planner.py)
    │   ▲
    │   │ get_lookahead() (adaptiver Offset, 50Hz)
    │   │
    ├───┤
    │  SyncMonitor (sync_monitor.py)
    │   ▲
    │   │ on_feedback_received() (250Hz)
    │
    │  EGM-Samples (alle 4–20ms)
    ▼
EgmClient (egm_client.py)
    │
    │  UDP Protobuf/JSON
    ▼
ABB Controller (EGM)
    │
    │  UDP Feedback
    ▼
SyncMonitor (sync_monitor.py)
    │
    │  Level-Änderungen (OK/WARN/DEGRADE/STOP)
    ▼
BridgeStateMachine (state_machine.py)
    │
    │  Stop/Pause-Befehle
    ▼
KlipperCommandClient (klipper_command.py)
    │
    │  TCP (JSON-Lines, Port 7201)
    ▼
bridge_watchdog.py (Klipper-Extra)
    │
    │  PAUSE / EMERGENCY_STOP
    ▼
Klipper (Print anhalten)
```

**Führungsprinzip:** Klipper ist der Motion-Master. Die Bridge übersetzt nur — sie plant keine eigenen Bahnen.

**Closed-Loop-Prinzip:** Der `SyncMonitor` integriert einen adaptiven Latenz-Estimator. Er beobachtet bei 250Hz die Ist-Positionen des Roboters, matched sie positionsbasiert mit den gesendeten Soll-Samples und schätzt daraus kontinuierlich die tatsächliche Controller-Latenz T_delay. Der `TrajectoryPlanner` holt diesen Wert via `get_lookahead()` und interpoliert entsprechend in die Zukunft — ohne manuelle Kalibrierung.

**Sicherheitsprinzip:** Zwei Schutzrichtungen. Wenn die Bridge ein Problem erkennt (Sync-Fehler, Workspace-Verletzung, Watchdog), weist sie Klipper an den Print zu stoppen. Wenn die Bridge stirbt, erkennt Klippers bridge_watchdog den fehlenden Heartbeat und pausiert eigenständig.

---

## Dateiübersicht

Alle Bridge-Dateien liegen unter `bridge/`. Die Kernmodule sind in `bridge/data/`.

| Datei | Zweck | Wann ändern? |
|-------|-------|-------------|
| `run_bridge.py` | CLI-Einstiegspunkt, startet die Bridge | Neues CLI-Argument, anderer Startup-Flow |
| `bridge_config.json` | Laufzeit-Konfiguration (JSON, Profil v1.4) | Neue Parameter, andere Grenzwerte |
| `data/bridge.py` | Orchestrator — verbindet alle Komponenten | Neues Feature in den Hauptloop einbauen |
| `data/clock.py` | Einheitliche Zeitquelle (`bridge_now`) | Nur wenn Clock-Basis geändert werden soll |
| `data/config.py` | Konfigurations-Dataclasses + Validierung | Neuen Parameter hinzufügen |
| `data/egm_client.py` | UDP-Senden/Empfangen (Protobuf + JSON) | EGM-Protokoll ändern, Orientierung |
| `data/egm_pb2.py` | Generierter Protobuf-Code (ABB EGM) | Nur neu generieren wenn .proto sich ändert |
| `data/klipper_command.py` | Heartbeat + Stop/Pause an Klipper senden | Neuer Befehlstyp, anderes Protokoll |
| `data/moonraker_client.py` | Websocket-Client für Extruder-E-Wert | Neue Moonraker-Daten abfragen |
| `data/segment_source.py` | TCP-Empfang + TrapezSegment-Datenmodell + Validierung | Neues Feld im Segment, anderes Protokoll |
| `data/state_machine.py` | Zustandsmaschine (6 States) | Neuer State, andere Transitions |
| `data/sync_monitor.py` | Closed-Loop Offset-Estimator + Sync-Bewertung | Neue Metrik, andere Grenzwerte, Estimator-Parameter |
| `data/telemetry.py` | CSV-Logging in 6 Streams | Neuer Log-Stream, anderes Format |
| `data/trajectory_planner.py` | Trapez-Interpolation + adaptiver Lookahead | Interpolationslogik, Korrekturstrategie |
| `tests/test_core.py` | Offline-Smoke-Tests | Bei jeder Codeänderung mitpflegen |
| `tests/test_egm_direct.py` | Live-EGM-Kreisbahn-Test | EGM-Protokoll testen |
| `tests/test_watchdog.py` | Watchdog-/Command-Tests | Bei Watchdog-Änderungen |

Außerhalb der Bridge, aber direkt relevant:

| Datei | Zweck | Wann ändern? |
|-------|-------|-------------|
| `klippy_extras/move_export.py` | Klipper-Extra: fängt `trapq_append` ab, exportiert Segmente per TCP + CSV | Neues Feld exportieren, anderes Format |
| `klippy_extras/bridge_watchdog.py` | Klipper-Extra: empfängt Heartbeats, pausiert bei Timeout/Bridge-Stop | Timeout-Logik, andere Aktion |
| `config/printer.cfg` | Klipper-Config inkl. `[move_export]` und `[bridge_watchdog]` Sektionen | TCP-Port ändern, Extruder-Parameter, Watchdog-Timeout |

---

## Module im Detail

### 0. `data/clock.py` — Einheitliche Zeitquelle (`bridge_now`)

Zentrale Funktion `bridge_now()`, die `time.perf_counter()` kapselt. **Jede Komponente** importiert diese Funktion für Timestamps — niemand ruft `time.perf_counter()` oder `time.monotonic()` direkt auf.

**Warum:** `time.monotonic()` hat auf Windows nur 15ms Auflösung. Das Mischen beider Clocks führt zu falschen RTT- und Lag-Werten, weil die Epochen auf Windows divergieren.

```python
from .clock import bridge_now
ts = bridge_now()   # → float, Sekunden, µs-Auflösung
```

Ausnahme: `time.sleep()`-Timeouts dürfen `time.monotonic()` verwenden, da dort nur die Dauer zählt.

---

### 1. `run_bridge.py` — Einstiegspunkt

CLI-Tool das die Bridge startet. Unterstützt:

```bash
python run_bridge.py                          # Standard-Config
python run_bridge.py --config my.json         # Eigene Config
python run_bridge.py --dry-run                # Nur Config validieren
python run_bridge.py --generate-config        # Default-Config erzeugen
python run_bridge.py --log-level DEBUG        # Verbose Logging
```

Ablauf: Config laden → Validieren → `EgmBridge` erzeugen → `start()` → Warten auf erstes Segment → `run_job()` → Loop bis STOP/FAULT.

Signale SIGINT/SIGTERM werden abgefangen und lösen ein sauberes Shutdown aus.

---

### 2. `data/bridge.py` — Orchestrator (`EgmBridge`)

Die zentrale Klasse, die alle Komponenten verbindet. Lifecycle:

```
__init__(config)  →  Komponenten erzeugen (SyncMonitor, Planner, EgmClient, Telemetry, KlipperCmd)
start()           →  Verbindungen herstellen, INIT → READY
run_job()         →  READY → RUN, EGM-Loop-Thread starten
stop() / shutdown()  →  Alles herunterfahren
```

Der **EGM-Loop** (`_egm_loop`) ist der Kern und läuft im festen Takt (konfigurierbar, default 20ms = 50Hz):

1. Aktuellen Lookahead holen (`sync.get_lookahead()` oder fixer `offset_s`)
2. Sample aus dem TrajectoryPlanner holen (Interpolation mit Lookahead)
3. **Workspace-Envelope prüfen** — Position außerhalb der Box → FAULT, kein Senden
4. An EgmClient senden (UDP an Roboter)
5. `sync.record_sent_sample()` — Sample in Estimator-Buffer eintragen
6. SyncMonitor mit letztem Feedback updaten (Sync-Level-Bewertung)
7. Telemetrie schreiben
8. Auf Sync-Level reagieren (DEGRADE, STOP, Recovery)
9. Bei leerem Buffer: letzte Position halten, nach Starvation-Timeout Job beenden

**Workspace Envelope Check** (`_check_workspace_envelope`): Wird VOR jedem `send_target()` aufgerufen. Bei Verletzung: FAULT setzen, Klipper stoppen, Telemetrie-Event `"WORKSPACE_VIOLATION"` schreiben.

**Callbacks**, die die Komponenten verbinden:
- `_on_segment_received` — neues Segment von Klipper → Zeitkopplung (beim ersten) + Planner-Queue
- `_on_feedback` — Feedback vom Roboter → `sync.on_feedback_received()` (Estimator, 250Hz) + Telemetrie RX
- `_on_egm_timeout` — Watchdog → FAULT
- `_on_sync_level_change` — Sync-Level geändert → Telemetrie-Event
- `_on_state_change` — State-Machine-Übergang → Heartbeat-State aktualisieren, bei FAULT/DEGRADE Stop/Pause an Klipper senden

**Starvation-Erkennung:** Wenn `starvation_timeout_s` (konfigurierbar, default 3s) lang keine neuen Segmente kommen und der Buffer leer ist, wird der Job als abgeschlossen betrachtet.

**Klipper-Benachrichtigung bei Fehlern:** Bei FAULT wird über zwei redundante Kanäle gestoppt: primär über den dedizierten TCP (Port 7201 → `bridge_watchdog.py`) und als Fallback über die Moonraker HTTP-API (`POST /printer/emergency_stop`).

---

### 3. `data/config.py` — Konfiguration (`BridgeConfig`)

Acht verschachtelte Dataclasses:

- **`EgmConnectionConfig`** — UDP-Verbindung: IPs, Ports, Zykluszeit, Watchdog, Protokoll (protobuf/json/auto), Default-Quaternion für Orientierung
- **`WorkspaceEnvelopeConfig`** — Begrenzungsbox für Zielpositionen (mm). Methoden `contains(x,y,z)` und `violation_reason(x,y,z)`. Bei `enabled=False` wird der Check in `bridge.py` übersprungen.
- **`QueueConfig`** — Buffer-Größen, Watermarks, Underflow-Verhalten ("degrade"/"stop"), `starvation_timeout_s`
- **`SyncConfig`** — Tracking-Error (mm), Offset-Grenzwerte (ms), Jitter (ms), Normal-Error (mm); Korrektur-Limits; Offset-Modell (Legacy-Felder für fixen Offset)
- **`KlipperSourceConfig`** — TCP-Verbindung zu move_export.py (Host, Port, Reconnect)
- **`MoonrakerConfig`** — Websocket-Verbindung für Extruder-E-Wert (Host, Port, Poll-Intervall)
- **`WatchdogConfig`** — Klipper-Watchdog: Heartbeat-Intervall, Stop/Pause-Verhalten, Moonraker-Fallback
- **`LatencyEstimatorConfig`** — Estimator-Parameter: Initialwert T_delay, EMA-Raten (slow/fast/output), Suchfenster, TX-Buffer-Größe
- **`TelemetryConfig`** — Log-Verzeichnis, Level, CSV-Export, Metrik-Intervall

Die JSON-Config (`bridge_config.json`) wird mit `load_config()` geladen. `validate_config()` prüft vor Jobstart auf Plausibilität (Ranges, Monotonie der Grenzwerte, Port-Kollisionen, Workspace-Konsistenz) und gibt eine Liste von Fehlern zurück.

**Neuen Parameter hinzufügen:**
1. Dataclass in `config.py` erweitern
2. In `validate_config()` Range-Check hinzufügen
3. In `bridge_config.json` Default setzen
4. In der nutzenden Komponente verwenden

---

### 4. `data/egm_client.py` — EGM-Kommunikation (`EgmClient`)

Zwei getrennte UDP-Sockets (wie vom ABB EGM Standard gefordert):

- **TX-Socket:** Bindet auf `(robot_ip, local_send_port)` → sendet an `(robot_ip, send_port)`. Default: `127.0.0.1:6512 → 127.0.0.1:6599`.
- **RX-Socket:** Bindet auf `(0.0.0.0, recv_port)` → empfängt Feedback. Default: Port `6510`.

Das Protokoll ist **Protobuf** (wenn `egm_pb2` verfügbar) mit **JSON-Fallback**. Die Protobuf-Nachrichten nutzen `EgmSensor` (Senden) und `EgmRobot` (Empfangen) aus der ABB EGM Spezifikation.

Datenstrukturen:
- `EgmTarget` — Sollposition (x, y, z, Quaternion) die an den Roboter gesendet wird
- `EgmFeedback` — Istposition die vom Roboter zurückkommt (inkl. `robot_time` in ms)
- `EgmConnectionStats` — TX/RX-Zähler, Timeouts, RTT (per Sequence-ID-Matching)

Der RX-Thread läuft permanent und ruft bei jedem empfangenen Paket den `on_feedback`-Callback auf. RTT wird per `sequence_id`-Matching korrekt berechnet (beide Zeiten via `bridge_now()`).

**Orientierung:** Aktuell wird eine feste Quaternion-Orientierung gesendet (konfigurierbar in `EgmConnectionConfig.default_q0..q3`). Soll die Orientierung dynamisch sein, muss `EgmTarget` in `bridge.py` entsprechend befüllt werden.

---

### 5. `data/segment_source.py` — Segment-Empfang

#### `TrapezSegment` (Datenmodell)

Repräsentiert ein einzelnes Trapezgeschwindigkeitsprofil-Segment. Enthält Start-/Endposition, Geschwindigkeiten (start_v, cruise_v, end_v), Beschleunigung und die drei Phasenzeiten (accel_t, cruise_t, decel_t).

Wichtigste Methoden:
- `validate()` — Plausibilitätsprüfung (duration > 0, keine negativen Geschwindigkeiten, Phasensumme == duration, normierter Richtungsvektor). Wird automatisch in `from_dict()` und `from_csv_row()` aufgerufen. Wirft `SegmentValidationError` bei Fehler.
- `position_at(t)` — Berechnet Position zum Zeitpunkt t. Distanz wird auf `[0, distance]` geclampt (verhindert Überschießen durch Rundungsfehler in der Decel-Phase).
- `velocity_at(t)` — Skalare Geschwindigkeit zum Zeitpunkt t.
- `from_dict(d)` — Erzeugt Segment aus JSON-Dict (TCP-Empfang).
- `from_csv_row(row)` — Erzeugt Segment aus CSV-Zeile (Batch-Tests).

#### `TcpSegmentReceiver`

Verbindet sich per TCP mit dem `move_export.py` Server (default `127.0.0.1:7200`). Läuft in eigenem Thread. Empfängt JSON-Lines, parst sie zu `TrapezSegment`-Objekten, validiert Monotonie der `print_time` und ruft den `on_segment`-Callback auf. Reconnect bei Verbindungsverlust. Zählt abgelehnte Segmente (`segments_rejected`).

#### `CsvSegmentSource`

Liest Segmente aus einer CSV-Datei. Für Offline-Tests und Replay. Kann mit `realtime=True` die originalen Zeitabstände einhalten.

---

### 6. `data/trajectory_planner.py` — Interpolation (`TrajectoryPlanner`)

Wandelt diskrete Trapez-Segmente in zeitkontinuierliche Sollwerte um.

**Funktionsweise (Time-Indexed Playback):**

Die Bridge koppelt beim ersten empfangenen Segment einmalig die Bridge-Zeitlinie mit Klippers `print_time` (`init_time_sync()`). Danach gilt:

```
t_klipper_now = (bridge_time - t0_bridge) + t0_klipper + lookahead_s
```

`next_sample(bridge_time, lookahead_s)` berechnet mit dieser Formel den aktuellen Klipper-Zeitpunkt und findet das passende Segment. `lookahead_s` kommt von außen (Bridge übergibt `sync.get_lookahead()`), der Planner kennt den Estimator nicht — saubere Trennung.

**Queue-Management:**
- Segmente werden per `add_segment()` in eine FIFO-Queue (max `plan_queue_size`, default 2000) eingefügt
- Veraltete Segmente (Endzeit < t_klipper_now) werden automatisch übersprungen
- Zeitlücken zwischen Segmenten werden mit Position-Hold überbrückt (`velocity=0`)

**Rolling-Horizon-Korrektur:**
Per `apply_correction(dx, dy, dz)` kann eine Positionskorrektur eingebracht werden. Die Korrektur wird rate-limited und geglättet angewandt. Begrenzung durch `correction_max_mm` und `correction_rate_limit_mm_per_s`.

**Ausgabe:** `EgmSample` mit Timestamp, Position (x, y, z), Geschwindigkeit, Segment-Nr, Progress, Sequence-ID und `t_klipper` (für Telemetrie/Debug).

---

### 7. `data/state_machine.py` — Zustandsmaschine (`BridgeStateMachine`)

Sechs Zustände mit explizit definierten Übergängen:

```
INIT ──→ READY ──→ RUN ──⇄── DEGRADED
  │        │        │           │
  │        │        ▼           ▼
  │        └──→ STOP ←─────────┘
  │               │
  ▼               ▼
FAULT ←─── (von überall erreichbar)
  │
  └──→ INIT (nur per explizitem Reset)
```

- **INIT** — Konfiguration geladen, Verbindungen noch nicht aktiv
- **READY** — Verbindungen stehen, warten auf Segmente/Jobstart
- **RUN** — Aktiver Betrieb, EGM-Loop läuft
- **DEGRADED** — Sync-Problem erkannt, System läuft weiter aber mit Warnung. Recovery nach RUN möglich wenn Sync wieder OK.
- **STOP** — Geordneter Halt (Job fertig, manueller Stop, Underflow)
- **FAULT** — Harter Fehler (Watchdog, Sync-Stop, Workspace-Verletzung). **Löst automatisch Stop/Pause an Klipper aus.**

Thread-sicher. Listener-System für State-Changes. History der letzten 500 Übergänge.

---

### 8. `data/sync_monitor.py` — Closed-Loop Sync-Überwachung (`SyncMonitor`)

Konsolidiert den Offset-Estimator und die Sync-Bewertung in einer Klasse. Zwei Update-Pfade mit unterschiedlicher Rate:

**Pfad 1: `on_feedback_received(feedback)` — 250Hz (RX-Thread)**

Der eigentliche Closed-Loop-Kern. Läuft mit voller Feedback-Rate für bestmögliches Matching.

Ablauf pro Feedback-Paket:
1. Ist-Geschwindigkeit aus Positionsdifferenz berechnen
2. Erwartete TX-Sendezeit schätzen: `t_tx_guess = (t_rx - OFFSET_BACK) - T_delay`
3. Im TX-Ringbuffer per `bisect` ein Suchfenster (±60ms um `t_tx_guess`) aufspannen
4. Nächsten TX-Sample im Suchfenster per XY-Abstand matchen
5. Tangential/Normal-Zerlegung des Fehlers: Tangentialfehler → Timing-Signal; Normalfehler → Bahnabweichung
6. Gewichtung: hohes Tangential/Normal-Verhältnis → weight ≈ 1, Stillstand → weight = 0
7. EMA-Update: `T_delay += ema_rate * weight * (T_delay_refined - T_delay)`. Bei erkannter Beschleunigung wird `ema_fast` statt `ema_slow` verwendet.

**Pfad 2: `update(sample, feedback, buffer_depth, buffer_time_s)` — 50Hz (EGM-Loop)**

Sync-Level-Bewertung und Logging. Die schwere Arbeit (Matching) ist bereits in Pfad 1 erledigt.

Bewertung hierarchisch anhand konfigurierbarer Grenzwerte:

| Level | Offset | Normal-Error | TCP-Error | Reaktion |
|-------|--------|-------------|-----------|----------|
| OK | < warn | < warn | < stop | Normalbetrieb |
| WARN | > warn | > warn | — | Logging |
| DEGRADE | > degrade | > degrade | — | RUN → DEGRADED |
| STOP | > stop | — | > stop | → FAULT → Klipper Stop |

Jedes Level hat konfigurierbare Confirm-Cycles (wie viele aufeinanderfolgende Bad-Cycles nötig sind). STOP ist Default sofort (1 Cycle), DEGRADE benötigt 3, WARN benötigt 5.

**`get_lookahead()` — Thread-sicher, 50Hz**

Gibt den geglätteten Output-Offset zurück: `T_delay_output += EMA_OUTPUT * (T_delay - T_delay_output)`. Der Output bewegt sich sanft auf den Roh-Offset zu — kein Ruckeln beim Senden.

**Warmup:** Die ersten 100 Zyklen (~2s bei 50Hz) werden nicht bewertet.

**Grenzwerte anpassen:** In `bridge_config.json` unter `sync` und `estimator`, oder zur Laufzeit per `bridge.set_param("sync", "offset_warn_ms", 400.0)`.

---

### 9. `data/klipper_command.py` — Klipper-Steuerung (`KlipperCommandClient`)

Sendet Heartbeats und Stop/Pause-Befehle an das `bridge_watchdog.py` Klipper-Extra über eine dedizierte TCP-Verbindung (Port 7201).

**Heartbeat:** Läuft in eigenem Thread, sendet alle N Sekunden (default 1s) ein JSON-Line `{"type": "heartbeat", "state": "RUN", "ts": ...}`. Der Bridge-State wird bei jedem State-Machine-Übergang aktualisiert.

**Stop/Pause:** Wird von `bridge.py` aufgerufen wenn die State-Machine nach FAULT oder DEGRADED wechselt. Sendet `{"type": "stop", "reason": "..."}` oder `{"type": "pause", "reason": "..."}`.

**Fallback:** Wenn der TCP-Kanal nicht verfügbar ist, wird über die Moonraker HTTP-API (`POST /printer/emergency_stop` bzw. `POST /printer/gcode/script?script=PAUSE`) gestoppt.

**Reconnect-Zählung:** `_connect_count` zählt alle Verbindungen, `_reconnect_count` nur Reconnects nach der ersten Verbindung (Bug-Fix gegenüber v0.2).

---

### 10. `klippy_extras/bridge_watchdog.py` — Klipper-Watchdog

Klipper-Extra das auf Heartbeats von der Bridge lauscht und bei Timeout oder explizitem Stop-Befehl den Print pausiert.

**Sicherheitslogik:**
- Heartbeat-Timeout löst NUR aus wenn ein Print aktiv ist UND die Bridge WÄHREND dieses Prints verbunden war
- Ohne Bridge-Verbindung kann normal gedruckt werden (z.B. Extruder-Tests)
- Explizite Stop-Befehle von der Bridge wirken immer sofort

**GCode-Befehle:**
- `BRIDGE_STATUS` — Zeigt Bridge-Verbindungsstatus in der Mainsail-Konsole
- `BRIDGE_WATCHDOG_RESET` — Setzt den Watchdog zurück (nach einem Trigger)

Konfiguration in `printer.cfg`:
```ini
[bridge_watchdog]
tcp_port: 7201
heartbeat_timeout: 2.0
action_on_timeout: pause
action_on_bridge_stop: pause
enabled: true
```

---

### 11. `data/telemetry.py` — Telemetrie (`TelemetryWriter`)

Erzeugt pro Job ein Verzeichnis unter `logs/` mit sechs CSV-Streams:

| Stream | Datei | Inhalt | Frequenz |
|--------|-------|--------|----------|
| PLAN | `plan.csv` | Empfangene Segmente (Nr, Dauer, Position, Geschwindigkeit) | Pro Segment |
| TX | `tx.csv` | Gesendete Sollwerte (Seq, Position, Velocity, Segment-Info, t_klipper, E-Wert, Lookahead) | Jeder EGM-Zyklus |
| RX | `rx.csv` | Empfangene Istpositionen (Seq, Robot-Time, Position, Quaternion, E-Wert) | Jedes Feedback-Paket |
| SYNC | `sync.csv` | Offset (raw+output), Normal/Tangential-Error, TCP-Error, Buffer-Tiefe, Sync-Level | Konfigurierbares Intervall (default 1s) |
| EVENT | `event.csv` | State-Changes, Warnungen, Fehler, Workspace-Violations, Klipper-Stop-Befehle | Bei Auftreten |
| ESTIMATOR | `estimator.csv` | T_delay (raw+output), Tang/Norm-Error, Weight, EMA-Rate, Correction | Pro RX-Paket (~250Hz) |

Zusätzlich wird ein `config_snapshot.json` gespeichert mit der zum Jobstart aktiven Konfiguration.

Low-Volume-Streams (PLAN, EVENT, SYNC) werden sofort geflusht. High-Volume-Streams (TX, RX) alle 50 Zeilen. ESTIMATOR alle 50 Zeilen (250Hz würde sonst den RX-Thread bremsen).

---

### 12. `klippy_extras/move_export.py` — Klipper-Seite

Ein Klipper-Extra das sich per Monkey-Patch in `toolhead.trapq_append` einklinkt. Für jedes Trapez-Segment:

1. Berechnet Endposition, Enddistanz und Endgeschwindigkeit
2. Schreibt eine CSV-Zeile (für Offline-Analyse)
3. Sendet ein JSON-Line über TCP an alle verbundenen Clients

Konfiguration in `printer.cfg`:
```ini
[move_export]
output_path: /home/klippy/printer_data/logs/move_segments.csv
tcp_port: 7200
tcp_enabled: true
log_to_console: true
```

Der TCP-Server akzeptiert mehrere Clients gleichzeitig. Jeder Client erhält zuerst ein `{"type": "hello", ...}` und danach `{"type": "segment", ...}` für jedes Segment.

---

## Konfigurationsprofil (`bridge_config.json`, v1.4)

Die wichtigsten Sektionen:

```json
{
  "profile_name": "robotstudio_dev",
  "profile_version": "1.4",
  "connection": {
    "robot_ip": "127.0.0.1",
    "send_port": 6599,
    "recv_port": 6510,
    "local_send_port": 6512,
    "cycle_ms": 20,
    "timeout_ms": 100,
    "watchdog_cycles": 250,
    "protocol": "protobuf"
  },
  "workspace": {
    "enabled": true,
    "min_x": -300.0, "max_x": 300.0,
    "min_y": -300.0, "max_y": 300.0,
    "min_z": 0.0,    "max_z": 400.0
  },
  "queues": {
    "plan_queue_size": 2000,
    "low_watermark": 5,
    "high_watermark": 1800,
    "underflow_action": "degrade",
    "starvation_timeout_s": 3.0
  },
  "sync": {
    "offset_warn_ms": 350.0,
    "offset_degrade_ms": 500.0,
    "offset_stop_ms": 800.0,
    "norm_error_warn_mm": 5.0,
    "norm_error_degrade_mm": 15.0,
    "tracking_stop_mm": 50.0,
    "correction_enabled": true,
    "correction_max_mm": 3.0
  },
  "estimator": {
    "enabled": true,
    "t_delay_init_ms": 50.0,
    "ema_slow": 0.04,
    "ema_fast": 0.25,
    "ema_output": 0.08,
    "offset_back_ms": 5.0,
    "alpha_weight": 10.0,
    "tx_buffer_size": 500
  },
  "watchdog": {
    "enabled": true,
    "tcp_host": "127.0.0.1",
    "tcp_port": 7201,
    "heartbeat_interval_s": 1.0,
    "stop_on_fault": true,
    "pause_on_degrade": false,
    "moonraker_fallback": true
  },
  "telemetry": {
    "log_dir": "./logs",
    "csv_export": true,
    "metric_interval_s": 1.0
  }
}
```

**Hinweis:** Die aktuellen Default-Werte sind für RobotStudio-Simulation ausgelegt (großzügige Sync-Grenzen, `t_delay_init_ms: 50`). Für echte Hardware sollten Tracking- und Offset-Grenzen deutlich enger gesetzt werden und `t_delay_init_ms` auf die bekannte Controller-Zykluszeit des ABB IRB 1100 eingestellt werden.

**Estimator deaktivieren:** `"estimator": {"enabled": false}` — dann verwendet der Planner den fixen `sync.time_offset_ms` als Fallback-Offset.

---

## Bekannte Probleme und Plattform-Hinweise

### Windows Timer-Auflösung (WICHTIG)

Windows `time.sleep()` hat per Default nur 15.625ms Auflösung. `sleep(19ms)` wird auf 31.25ms aufgerundet, wodurch der EGM-Loop nur mit ~32Hz statt 50Hz läuft. Die Bridge aktiviert `timeBeginPeriod(1)` automatisch auf Windows um 1ms Auflösung zu erzwingen.

**Symptome wenn der Fix nicht greift:** Alle TX-Zykluszeiten liegen bei 30-32ms statt 20ms. Lag-Werte sind exakte Vielfache von 15.625ms.

**Diagnose:** Im TX-Log prüfen ob die Zeitdifferenzen zwischen aufeinanderfolgenden Samples ~20ms betragen.

### Clock-Konsistenz

Alle internen Timestamps nutzen `bridge_now()` (= `time.perf_counter()`) für µs-Auflösung und einheitliche Clock-Epoch. Das Mischen mit `time.monotonic()` ist ein bekannter Bug aus v0.2 und wurde in v0.3 behoben.

---

## Telemetrie-Analyse (Anleitung)

**Schnell-Check nach einem Run:**
1. `event.csv` — Was ist schiefgelaufen? (SYNC_WARN, SYNC_STOP, WORKSPACE_VIOLATION etc.)
2. `sync.csv` — Offset, Normal-Error und TCP-Error über die Zeit
3. `estimator.csv` — Konvergiert T_delay? Ist das Weight > 0 (Roboter bewegt sich)?
4. `tx.csv` — Zykluszeiten prüfen (Differenzen zwischen Timestamps sollten ≈ cycle_ms sein)

**TX Zykluszeit prüfen (Python):**
```python
import csv
times = [float(r['timestamp']) for r in csv.DictReader(open('tx.csv'))]
diffs = [(times[i+1]-times[i])*1000 for i in range(len(times)-1)]
print(f"Soll: {cycle_ms}ms, Ist: {sum(diffs)/len(diffs):.1f}ms")
```

**Estimator-Konvergenz prüfen:**
```python
import csv
rows = list(csv.DictReader(open('estimator.csv')))
delays = [float(r['t_delay_ms']) for r in rows]
print(f"T_delay: start={delays[0]:.1f}ms → end={delays[-1]:.1f}ms "
      f"(Δ={delays[-1]-delays[0]:.1f}ms)")
```

---

## Häufige Änderungsszenarien

### Neuen Prozessparameter in die Segmente aufnehmen (z.B. Extrusionsmenge)

1. **`klippy_extras/move_export.py`** — Wert in `_export_move()` berechnen, in CSV-Zeile und JSON-Dict aufnehmen
2. **`bridge/data/segment_source.py`** — Feld in `TrapezSegment` Dataclass hinzufügen, `from_dict()` und `from_csv_row()` anpassen
3. **`bridge/data/trajectory_planner.py`** — Falls der Wert interpoliert werden muss: in `EgmSample` aufnehmen und in `next_sample()` berechnen
4. **`bridge/data/telemetry.py`** — CSV-Header und `log_tx()`/`log_plan()` erweitern

### Orientierung dynamisch steuern (statt fixer Quaternion)

1. **`bridge/data/segment_source.py`** — Orientierungsdaten ins `TrapezSegment` aufnehmen
2. **`bridge/data/trajectory_planner.py`** — Orientierung interpolieren (Slerp), in `EgmSample` ausgeben
3. **`bridge/data/bridge.py`** — In `_egm_loop()` die Quaternion aus dem Sample statt aus der Config nehmen

### Estimator deaktivieren / Fixen Offset verwenden

1. `"estimator": {"enabled": false}` in `bridge_config.json`
2. `"sync": {"time_offset_ms": 80.0}` — fixer Lookahead in ms
3. Der Planner verwendet dann `self.planner.offset_s` statt `sync.get_lookahead()`

### Estimator-Verhalten anpassen

- `t_delay_init_ms` — Startwert, sollte ungefähr der bekannten Controller-Latenz entsprechen
- `ema_slow` (default 0.04) — kleiner = stabileres, langsameres Lernen
- `ema_fast` (default 0.25) — greift bei erkannter Beschleunigung (schnellere Anpassung)
- `ema_output` (default 0.08) — glättet den Lookahead-Output, verhindert Ruckeln
- `alpha_weight` — größer = Normalfehler wird stärker bestraft → weight sinkt schneller bei schlechtem Match

### Workspace Envelope für echte Hardware konfigurieren

1. In `bridge_config.json` unter `workspace` die Grenzen für den tatsächlichen Arbeitsraum des ABB IRB 1100 eintragen — **konservativ, lieber kleiner**
2. Validierung prüft automatisch: min < max pro Achse, Volumen nicht zu groß
3. Bei Verletzung: `event.csv` enthält `"WORKSPACE_VIOLATION"` mit Position und betroffenen Achsen

### Andere Sync-Strategie / Neue Metrik

1. **`bridge/data/sync_monitor.py`** — Neue Berechnung in `on_feedback_received()` oder `update()`, neues Feld in `SyncMetrics`, Bewertung in `_evaluate_sync_level()` anpassen
2. **`bridge/data/config.py`** — Neue Grenzwerte in `SyncConfig`
3. **`bridge/data/telemetry.py`** — Neues Feld im SYNC- oder ESTIMATOR-Stream

### Watchdog-Verhalten anpassen

1. **`bridge/data/config.py`** — `WatchdogConfig`: `stop_on_fault`, `pause_on_degrade`, `moonraker_fallback`
2. **`bridge/data/bridge.py`** — `_on_state_change()` Callback
3. **`klippy_extras/bridge_watchdog.py`** — `action_on_timeout`, `action_on_bridge_stop`, `heartbeat_timeout`
4. **`config/printer.cfg`** — `[bridge_watchdog]` Sektion

---

## Docker-Infrastruktur (Kurzübersicht)

- **`Dockerfile`** — Baut Ubuntu 24.04 mit Klipper, Moonraker, Mainsail. Patcht die Linux-MCU via `gpio_patch.py`.
- **`start.sh`** — Startet die simulierte MCU per `socat`, verlinkt Custom Extras aus `klippy_extras/` nach Klipper, startet Supervisor.
- **`supervisord.conf`** — Managed Klipper, Moonraker und Nginx als Prozesse.
- **`gpio_patch.py`** — Ersetzt alle Hardware-Zugriffsdateien (GPIO, ADC, SPI, I2C, PWM) in Klippers Linux-MCU durch Stubs.

Exponierte Ports: 80 (Mainsail), 7125 (Moonraker), 7200 (MoveExport), 7201 (Bridge-Watchdog).
