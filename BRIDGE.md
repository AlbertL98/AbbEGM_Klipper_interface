# EGM-Bridge — Architektur und Modulreferenz

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
```

**Führungsprinzip:** Klipper ist der Motion-Master. Die Bridge übersetzt nur — sie plant keine eigenen Bahnen. Der Roboter folgt den Sollwerten, die Klipper vorgibt.

---

## Dateiübersicht

Alle Bridge-Dateien liegen unter `bridge/`. Die Kernmodule sind in `bridge/data/`.

| Datei | Zweck | Wann ändern? |
|-------|-------|-------------|
| `run_bridge.py` | CLI-Einstiegspunkt, startet die Bridge | Neues CLI-Argument, anderer Startup-Flow |
| `bridge_config.json` | Laufzeit-Konfiguration (JSON) | Neue Parameter, andere Grenzwerte |
| `data/bridge.py` | Orchestrator — verbindet alle Komponenten | Neues Feature in den Hauptloop einbauen |
| `data/config.py` | Konfigurations-Dataclasses + Validierung | Neuen Parameter hinzufügen |
| `data/egm_client.py` | UDP-Senden/Empfangen (Protobuf + JSON) | EGM-Protokoll ändern, Orientierung |
| `data/egm_pb2.py` | Generierter Protobuf-Code (ABB EGM) | Nur neu generieren wenn .proto sich ändert |
| `data/segment_source.py` | TCP-Empfang + TrapezSegment-Datenmodell | Neues Feld im Segment, anderes Protokoll |
| `data/state_machine.py` | Zustandsmaschine (6 States) | Neuer State, andere Transitions |
| `data/sync_monitor.py` | Tracking-Error, Lag, Jitter | Neue Metrik, andere Grenzwerte |
| `data/telemetry.py` | CSV-Logging in 5 Streams | Neuer Log-Stream, anderes Format |
| `data/trajectory_planner.py` | Trapez-Interpolation + Korrektur | Interpolationslogik, Korrekturstrategie |
| `tests/test_core.py` | Offline-Smoke-Tests | Bei jeder Codeänderung mitpflegen |
| `tests/test_egm_direct.py` | Live-EGM-Kreisbahn-Test | EGM-Protokoll testen |

Außerhalb der Bridge, aber direkt relevant:

| Datei | Zweck | Wann ändern? |
|-------|-------|-------------|
| `klippy_extras/move_export.py` | Klipper-Extra: fängt `trapq_append` ab, exportiert Segmente per TCP + CSV | Neues Feld exportieren, anderes Format |
| `config/printer.cfg` | Klipper-Config inkl. `[move_export]` Sektion | TCP-Port ändern, Extruder-Parameter |

---

## Module im Detail

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
__init__(config)  →  Komponenten erzeugen (Planner, EgmClient, SyncMonitor, Telemetry)
start()           →  Verbindungen herstellen, INIT → READY
run_job()         →  READY → RUN, EGM-Loop-Thread starten
stop() / shutdown()  →  Alles herunterfahren
```

Der **EGM-Loop** (`_egm_loop`) ist der Kern und läuft im festen Takt (konfigurierbar, default 20ms = 50Hz):

1. Sample aus dem TrajectoryPlanner holen (Interpolation)
2. An EgmClient senden (UDP an Roboter)
3. SyncMonitor mit dem letzten Feedback updaten
4. Telemetrie schreiben
5. Auf Sync-Level reagieren (DEGRADE, STOP, Recovery)
6. Bei leerem Buffer: letzte Position halten, nach Timeout Job beenden

**Callbacks**, die die Komponenten verbinden:
- `_on_segment_received` — neues Segment von Klipper → in Planner-Queue
- `_on_feedback` — Feedback vom Roboter → für SyncMonitor
- `_on_egm_timeout` — Watchdog → FAULT
- `_on_sync_level_change` — Sync-Level geändert → Telemetrie-Event

**Starvation-Erkennung:** Wenn 3 Sekunden lang keine neuen Segmente kommen und der Buffer leer ist, wird der Job als abgeschlossen betrachtet.

---

### 3. `data/config.py` — Konfiguration (`BridgeConfig`)

Fünf verschachtelte Dataclasses:

- **`EgmConnectionConfig`** — UDP-Verbindung: IPs, Ports, Zykluszeit, Watchdog, Protokoll (protobuf/json/auto), Default-Quaternion für Orientierung
- **`QueueConfig`** — Buffer-Größen, Watermarks, Underflow-Verhalten ("degrade" oder "stop")
- **`SyncConfig`** — Grenzwerte für Tracking-Error (mm), Lag (ms), Jitter (ms); Korrektur-Limits; Offset-Modell (Basis + geschwindigkeits-/beschleunigungsabhängig)
- **`KlipperSourceConfig`** — TCP-Verbindung zu move_export.py (Host, Port, Reconnect)
- **`TelemetryConfig`** — Log-Verzeichnis, Level, CSV-Export, Metrik-Intervall

Die JSON-Config (`bridge_config.json`) wird mit `load_config()` geladen. `validate_config()` prüft vor Jobstart auf Plausibilität (Ranges, Monotonie der Grenzwerte, etc.) und gibt eine Liste von Fehlern zurück.

**Neuen Parameter hinzufügen:**
1. Dataclass in `config.py` erweitern
2. In `validate_config()` Range-Check hinzufügen
3. In `bridge_config.json` Default setzen
4. In der nutzenden Komponente verwenden

---

### 4. `data/egm_client.py` — EGM-Kommunikation (`EgmClient`)

Zwei getrennte UDP-Sockets (wie vom ABB EGM Standard gefordert):

- **TX-Socket:** Bindet auf `(robot_ip, local_send_port)` → sendet an `(robot_ip, send_port)`. Default: `127.0.0.1:6512 → 127.0.0.1:6599` (UCdevice in RobotStudio).
- **RX-Socket:** Bindet auf `(0.0.0.0, recv_port)` → empfängt Feedback. Default: Port `6510` (von ROB_1).

Das Protokoll ist **Protobuf** (wenn `egm_pb2` verfügbar) mit **JSON-Fallback**. Die Protobuf-Nachrichten nutzen `EgmSensor` (Senden) und `EgmRobot` (Empfangen) aus der ABB EGM Spezifikation.

Datenstrukturen:
- `EgmTarget` — Sollposition (x, y, z, Quaternion) die an den Roboter gesendet wird
- `EgmFeedback` — Istposition die vom Roboter zurückkommt
- `EgmConnectionStats` — TX/RX-Zähler, Timeouts, RTT

Der RX-Thread läuft permanent und ruft bei jedem empfangenen Paket den `on_feedback`-Callback auf. Ein Watchdog zählt Zyklen ohne Antwort und löst bei Überschreitung `on_timeout` aus.

**Orientierung:** Aktuell wird eine feste Quaternion-Orientierung gesendet (konfigurierbar in `EgmConnectionConfig.default_q0..q3`). Soll die Orientierung dynamisch sein, muss `EgmTarget` in `bridge.py` entsprechend befüllt werden.

---

### 5. `data/segment_source.py` — Segment-Empfang

#### `TrapezSegment` (Datenmodell)

Repräsentiert ein einzelnes Trapezgeschwindigkeitsprofil-Segment. Enthält Start-/Endposition, Geschwindigkeiten (start_v, cruise_v, end_v), Beschleunigung und die drei Phasenzeiten (accel_t, cruise_t, decel_t).

Wichtigste Methoden:
- `position_at(t)` — Berechnet Position zum Zeitpunkt t innerhalb des Segments. Wird vom TrajectoryPlanner in jedem EGM-Zyklus aufgerufen.
- `velocity_at(t)` — Skalare Geschwindigkeit zum Zeitpunkt t.
- `from_dict(d)` — Erzeugt Segment aus JSON-Dict (TCP-Empfang).
- `from_csv_row(row)` — Erzeugt Segment aus CSV-Zeile (Batch-Tests).

#### `TcpSegmentReceiver`

Verbindet sich per TCP mit dem `move_export.py` Server (default `127.0.0.1:7200`). Läuft in eigenem Thread. Empfängt JSON-Lines, parst sie zu `TrapezSegment`-Objekten und ruft den `on_segment`-Callback auf. Reconnect bei Verbindungsverlust.

#### `CsvSegmentSource`

Liest Segmente aus einer CSV-Datei (von `move_export.py` erzeugt). Für Offline-Tests und Replay. Kann mit `realtime=True` die originalen Zeitabstände einhalten.

---

### 6. `data/trajectory_planner.py` — Interpolation (`TrajectoryPlanner`)

Wandelt diskrete Trapez-Segmente in zeitkontinuierliche Sollwerte um.

**Funktionsweise:**
1. Segmente werden per `add_segment()` in eine FIFO-Queue (maximal `plan_queue_size`, default 2000) eingefügt
2. `next_sample(bridge_time)` wird im EGM-Takt aufgerufen
3. Innerhalb des aktuellen Segments wird per `TrapezSegment.position_at()` interpoliert
4. Wenn ein Segment abgearbeitet ist, wird automatisch das nächste geladen
5. Zeitüberschuss (Overshoot) wird auf das nächste Segment übertragen

**Rolling-Horizon-Korrektur:**
Per `apply_correction(dx, dy, dz)` kann eine Positions-Korrektur eingebracht werden (z.B. aus SyncMonitor-Feedback). Die Korrektur wird rate-limited und geglättet angewandt — keine Sprünge. Begrenzung durch `correction_max_mm` und `correction_rate_limit_mm_per_s`.

**Ausgabe:** `EgmSample` mit Timestamp, Position (x, y, z), Geschwindigkeit, Segment-Nr, Progress und Sequence-ID.

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
- **DEGRADED** — Sync-Problem erkannt (z.B. hoher Tracking-Error), System läuft weiter aber mit Warnung. Recovery nach RUN möglich wenn Sync wieder OK.
- **STOP** — Geordneter Halt (Job fertig, manueller Stop, Underflow)
- **FAULT** — Harter Fehler (Watchdog, Sync-Stop-Level), erfordert Reset

Thread-sicher. Listener-System für State-Changes. History der letzten 500 Übergänge.

---

### 8. `data/sync_monitor.py` — Synchronisationsüberwachung (`SyncMonitor`)

Wird in jedem EGM-Zyklus mit dem letzten gesendeten Sample und dem letzten Roboter-Feedback gefüttert. Berechnet:

- **Tracking-Error** (mm) — Euklidischer Abstand Soll/Ist, plus EMA-Durchschnitt und Maximum
- **Lag** (ms) — Zeitversatz zwischen Sample- und Feedback-Timestamp
- **Jitter** — p95/p99 Percentile des Lags aus der History

Bewertung erfolgt hierarchisch anhand konfigurierbarer Grenzwerte:

| Level | Tracking-Error | Lag | Jitter p99 | Reaktion |
|-------|---------------|-----|-----------|----------|
| OK | < warn | < warn | < warn | Normalbetrieb |
| WARN | > warn | > warn | > warn | Logging |
| DEGRADE | > degrade | > degrade | — | RUN → DEGRADED |
| STOP | > stop | > stop | > stop | → FAULT |

**Warmup:** Die ersten 100 Zyklen (~2s bei 50Hz) werden nicht bewertet, um Anlauftransienten zu ignorieren.

**Grenzwerte anpassen:** In `bridge_config.json` unter `sync`, oder zur Laufzeit per `bridge.set_param("sync", "tracking_warn_mm", 5.0)`.

---

### 9. `data/telemetry.py` — Telemetrie (`TelemetryWriter`)

Erzeugt pro Job ein Verzeichnis unter `logs/` mit fünf CSV-Streams:

| Stream | Datei | Inhalt | Frequenz |
|--------|-------|--------|----------|
| PLAN | `plan.csv` | Empfangene Segmente (Nr, Dauer, Position, Geschwindigkeit) | Pro Segment |
| TX | `tx.csv` | Gesendete Sollwerte (Seq, Position, Velocity, Segment-Info) | Jeder EGM-Zyklus |
| RX | `rx.csv` | Empfangene Istpositionen (Seq, Robot-Time, Position, Quaternion) | Jedes Feedback-Paket |
| SYNC | `sync.csv` | Tracking-Error, Lag, Jitter, Buffer-Tiefe, Sync-Level | Konfigurierbares Intervall (default 1s) |
| EVENT | `event.csv` | State-Changes, Warnungen, Fehler, Param-Änderungen | Bei Auftreten |

Zusätzlich wird ein `config_snapshot.json` gespeichert mit der zum Jobstart aktiven Konfiguration.

Low-Volume-Streams (PLAN, EVENT, SYNC) werden sofort geflusht. High-Volume-Streams (TX, RX) alle 50 Zeilen.

---

### 10. `klippy_extras/move_export.py` — Klipper-Seite

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

## Konfigurationsprofil (`bridge_config.json`)

Die wichtigsten Sektionen:

```json
{
  "profile_name": "robotstudio_dev",
  "connection": {
    "robot_ip": "127.0.0.1",
    "send_port": 6599,        // UCdevice Port in RobotStudio
    "recv_port": 6510,        // Feedback-Port von ROB_1
    "local_send_port": 6512,  // Source-Port des TX-Sockets
    "cycle_ms": 20,           // EGM-Zykluszeit (50Hz)
    "timeout_ms": 100,        // RX-Timeout
    "watchdog_cycles": 250,   // Zyklen ohne Antwort → FAULT
    "protocol": "protobuf",   // "protobuf" | "json" | "auto"
    "default_q0..q3": "..."   // Feste Orientierung (Quaternion)
  },
  "queues": {
    "plan_queue_size": 2000,
    "low_watermark": 5,       // Unter 5 Segmenten → DEGRADED
    "high_watermark": 1800,
    "underflow_action": "degrade"  // oder "stop"
  },
  "sync": {
    "tracking_warn_mm": 10.0,
    "tracking_degrade_mm": 25.0,
    "tracking_stop_mm": 50.0,
    "lag_warn_ms": 100.0,
    "lag_degrade_ms": 250.0,
    "lag_stop_ms": 500.0,
    "correction_enabled": true,
    "correction_max_mm": 3.0
  },
  "klipper": {
    "tcp_host": "127.0.0.1",
    "tcp_port": 7200
  },
  "telemetry": {
    "log_dir": "./logs",
    "csv_export": true
  }
}
```

**Hinweis:** Die aktuellen Default-Werte in `bridge_config.json` sind für RobotStudio-Simulation ausgelegt (großzügige Sync-Grenzen). Für echte Hardware sollten die Tracking- und Lag-Grenzen deutlich enger gesetzt werden (z.B. `tracking_warn_mm: 2.0`, `tracking_stop_mm: 10.0`).

---

## Häufige Änderungsszenarien

### Neuen Prozessparameter in die Segmente aufnehmen (z.B. Extrusionsmenge)

1. **`klippy_extras/move_export.py`** — In `_export_move()` den neuen Wert berechnen/auslesen, in CSV-Zeile und JSON-Dict (`segment`) aufnehmen
2. **`bridge/data/segment_source.py`** — Feld in `TrapezSegment` Dataclass hinzufügen, `from_dict()` und `from_csv_row()` anpassen
3. **`bridge/data/trajectory_planner.py`** — Falls der Wert interpoliert werden muss: in `EgmSample` aufnehmen und in `next_sample()` berechnen
4. **`bridge/data/telemetry.py`** — Falls geloggt werden soll: CSV-Header und `log_tx()`/`log_plan()` erweitern

### Orientierung dynamisch steuern (statt fixer Quaternion)

1. **`bridge/data/segment_source.py`** — Orientierungsdaten ins `TrapezSegment` aufnehmen
2. **`bridge/data/trajectory_planner.py`** — Orientierung interpolieren (Slerp), in `EgmSample` ausgeben
3. **`bridge/data/bridge.py`** — In `_egm_loop()` die Quaternion aus dem Sample statt aus der Config nehmen

### Andere Sync-Strategie / Neue Metrik

1. **`bridge/data/sync_monitor.py`** — Neue Berechnung in `update()`, neues Feld in `SyncMetrics`, Bewertung in `_evaluate_sync_level()` anpassen
2. **`bridge/data/config.py`** — Neue Grenzwerte in `SyncConfig`
3. **`bridge/data/telemetry.py`** — Neues Feld im SYNC-Stream

### Anderen Roboter / anderes Protokoll (kein EGM)

1. **`bridge/data/egm_client.py`** — Komplett ersetzen oder neuen Client schreiben mit gleichem Interface (`connect()`, `send_target()`, `disconnect()`, `on_feedback` Callback)
2. **`bridge/data/bridge.py`** — `EgmClient` durch neuen Client ersetzen
3. **`bridge/data/egm_pb2.py`** — Entfällt bzw. durch neuen Protobuf ersetzen

### Korrektur-Logik aktivieren / anpassen

1. **`bridge/data/trajectory_planner.py`** — `apply_correction()` wird aufgerufen, Korrektur-Parameter in `__init__()`. Die Korrektur wird aktuell nicht automatisch aus dem SyncMonitor gespeist — das muss in `bridge.py` verdrahtet werden.
2. **`bridge/data/config.py`** — `SyncConfig.correction_enabled`, `correction_max_mm`, `correction_rate_limit_mm_per_s`

---

## Docker-Infrastruktur (Kurzübersicht)

Die Docker-Umgebung ist nicht Teil der Bridge, aber relevant für den Entwicklungs-Workflow:

- **`Dockerfile`** — Baut Ubuntu 24.04 mit Klipper, Moonraker, Mainsail. Patcht die Linux-MCU via `gpio_patch.py` damit sie ohne echte Hardware läuft.
- **`start.sh`** — Startet die simulierte MCU per `socat`, verlinkt Custom Extras aus `klippy_extras/` nach Klipper, startet Supervisor.
- **`supervisord.conf`** — Managed Klipper, Moonraker und Nginx als Prozesse.
- **`gpio_patch.py`** — Ersetzt alle Hardware-Zugriffsdateien (GPIO, ADC, SPI, I2C, PWM) in Klippers Linux-MCU durch Stubs, die ohne `/dev/gpiochip` etc. funktionieren.
