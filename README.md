# Robotergestütztes FDM/FFF-Drucksystem — Klipper + ABB EGM Integration

Dieses Repository enthält die Software für ein robotergestütztes 3D-Druck-System, das einen **ABB IRB 1100** Industrieroboter mit einem **Klipper**-gesteuerten Extruder (Smart Orbiter SO3) verbindet. Der Kern ist eine **EGM-Bridge**, die Klippers Bahnplanung in Echtzeit-Sollwerte für den Roboter übersetzt und über das ABB Externally Guided Motion (EGM) Protokoll überträgt.

Das System läuft in einem **Docker-Container**, der Klipper, Moonraker und Mainsail als simulierte Umgebung bereitstellt — ohne echte Hardware. Der Container nutzt eine gepatchte Linux-MCU, sodass Klipper vollständig in Docker/WSL2 funktioniert.

---

## Projektstruktur

```
.
├── Dockerfile                  Docker-Image: Ubuntu + Klipper + Moonraker + Mainsail
├── docker-compose.yml          Container-Konfiguration (Ports, Volumes)
├── start.sh                    Startup-Script: MCU starten, Extras verlinken, Supervisor
├── supervisord.conf            Prozess-Management (Klipper, Moonraker, Nginx)
├── nginx.conf                  Reverse-Proxy für Mainsail Web-UI
├── gpio_patch.py               Patcht Klipper Linux-MCU für Docker (simulierte GPIO/ADC/SPI)
│
├── config/
│   ├── printer.cfg             Klipper-Konfiguration (Stepper, Extruder, move_export)
│   └── moonraker.conf          Moonraker-Konfiguration (API, Auth)
│
├── klippy_extras/
│   └── move_export.py          Klipper-Extra: Exportiert Trapez-Segmente per CSV + TCP
│
├── bridge/                     *** EGM-Bridge-Core (siehe bridge.md) ***
│   ├── run_bridge.py           Haupteinstiegspunkt / CLI
│   ├── bridge_config.json      Konfigurationsprofil (JSON)
│   ├── data/                   Kernmodule
│   │   ├── __init__.py
│   │   ├── bridge.py           Orchestrator (Hauptloop)
│   │   ├── config.py           Konfiguration / Validierung
│   │   ├── egm_client.py       UDP-Kommunikation mit ABB (Protobuf + JSON)
│   │   ├── egm_pb2.py          Generierter Protobuf-Code (ABB EGM)
│   │   ├── segment_source.py   TCP-Empfang der Trapez-Segmente + CSV-Replay
│   │   ├── state_machine.py    Zustandsmaschine (INIT→READY→RUN→STOP/FAULT)
│   │   ├── sync_monitor.py     Tracking-Error, Lag, Jitter Überwachung
│   │   ├── telemetry.py        CSV-Logging (TX/RX/PLAN/SYNC/EVENT Streams)
│   │   └── trajectory_planner.py  Trapez-Interpolation → EGM-Samples
│   └── tests/
│       ├── test_core.py        Smoke-Tests (State Machine, Planner, Config, Sync)
│       └── test_egm_direct.py  EGM-Kreisbahn-Test (Vergleich mit workingEGM.py)
│
├── gcodes/                     Test-G-Codes
│   ├── toolhead_test.gcode     XYZ-Bewegungen mit Extrusion
│   ├── extruder_test.gcode     Reine Extruder-Tests
│   └── extruder_batch_test.gcode  Batch-Modus-Test
│
└── logs/                       Log-Ausgaben (von Docker gemountet)
```

---

## Voraussetzungen

- Docker + Docker Compose
- (Optional) ABB RobotStudio mit EGM-Konfiguration für Live-Tests

## Quickstart

```bash
# 1. Container bauen und starten
docker-compose up --build -d

# 2. Mainsail Web-UI öffnen
#    http://localhost:8181

# 3. In Mainsail: G-Code hochladen und Print starten
#    (z.B. toolhead_test.gcode)

# 4. Bridge starten (in separatem Terminal)
cd bridge
python run_bridge.py --config bridge_config.json
```

## Ports

| Port | Dienst |
|------|--------|
| 8181 | Mainsail Web-UI (via Nginx) |
| 7125 | Moonraker API |
| 7200 | MoveExport TCP-Stream (Klipper → Bridge) |

## Konfiguration

Die Klipper-Konfiguration liegt in `config/printer.cfg`. Dort ist auch das `[move_export]`-Modul konfiguriert, das die Trapez-Segmente per TCP an Port 7200 streamt.

Die Bridge-Konfiguration liegt in `bridge/bridge_config.json` und enthält EGM-Verbindungsparameter, Queue-Einstellungen, Sync-Grenzwerte und Telemetrie-Optionen. Details dazu in `bridge.md`.

## Tests

```bash
cd bridge/tests
python test_core.py          # Offline Smoke-Tests (kein Netzwerk nötig)
python test_egm_direct.py    # EGM-Kreisbahn (braucht RobotStudio)
```

## Weiterführende Dokumentation

- **[bridge.md](bridge.md)** — Detaillierte Architektur und Modul-Referenz der EGM-Bridge
