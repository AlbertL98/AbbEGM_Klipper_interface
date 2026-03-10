# Robotergestütztes FDM/FFF-Drucksystem — Klipper + ABB EGM Integration

Dieses Repository enthält die Software für ein robotergestütztes 3D-Druck-System, das einen **ABB IRB 1100** Industrieroboter mit einem **Klipper**-gesteuerten Extruder (Smart Orbiter SO3) verbindet. Der Kern ist eine **EGM-Bridge**, die Klippers Bahnplanung in Echtzeit-Sollwerte für den Roboter übersetzt und über das ABB Externally Guided Motion (EGM) Protokoll überträgt.

Das System läuft in einem **Docker-Container**, der Klipper, Moonraker und Mainsail als simulierte Umgebung bereitstellt — ohne echte Hardware. Der Container nutzt eine gepatchte Linux-MCU, sodass Klipper vollständig in Docker/WSL2 funktioniert.

Zwischen Bridge und Klipper besteht ein **bidirektionaler Sicherheitsmechanismus**: Die Bridge sendet Heartbeats an Klipper und kann bei erkannten Problemen (Sync-Fehler, Watchdog) den Print stoppen. Umgekehrt erkennt Klipper wenn die Bridge ausfällt und pausiert eigenständig.

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
│   ├── printer.cfg             Klipper-Konfiguration (Stepper, Extruder, move_export, bridge_watchdog)
│   └── moonraker.conf          Moonraker-Konfiguration (API, Auth)
│
├── klippy_extras/
│   ├── move_export.py          Klipper-Extra: Exportiert Trapez-Segmente per CSV + TCP
│   └── bridge_watchdog.py      Klipper-Extra: Heartbeat-Überwachung + Stop-Empfang von Bridge
│
├── bridge/                     *** EGM-Bridge-Core (siehe BRIDGE.md) ***
│   ├── run_bridge.py           Haupteinstiegspunkt / CLI
│   ├── bridge_config.json      Konfigurationsprofil (JSON)
│   ├── data/                   Kernmodule
│   │   ├── __init__.py
│   │   ├── bridge.py           Orchestrator (Hauptloop)
│   │   ├── config.py           Konfiguration / Validierung
│   │   ├── egm_client.py       UDP-Kommunikation mit ABB (Protobuf + JSON)
│   │   ├── egm_pb2.py          Generierter Protobuf-Code (ABB EGM)
│   │   ├── klipper_command.py  Heartbeat + Stop/Pause-Befehle an Klipper
│   │   ├── moonraker_client.py Websocket-Client für Extruder-E-Wert
│   │   ├── segment_source.py   TCP-Empfang der Trapez-Segmente + CSV-Replay
│   │   ├── state_machine.py    Zustandsmaschine (INIT→READY→RUN→STOP/FAULT)
│   │   ├── sync_monitor.py     Tracking-Error, Lag, Jitter Überwachung
│   │   ├── telemetry.py        CSV-Logging (TX/RX/PLAN/SYNC/EVENT Streams)
│   │   └── trajectory_planner.py  Trapez-Interpolation → EGM-Samples
│   └── tests/
│       ├── test_core.py        Smoke-Tests (State Machine, Planner, Config, Sync)
│       ├── test_egm_direct.py  EGM-Kreisbahn-Test (Vergleich mit workingEGM.py)
│       └── test_watchdog.py    Watchdog-/Command-Tests
│
├── watchdog/
│   ├── egm_pb2.py              Generierter Protobuf-Code (ABB EGM)
│   ├── watchdog.md             Dokumentation
│   └── watchdog.py             Standalone-Watchdog (Legacy)
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
- Python 3.10+ (für die Bridge, läuft auf dem Host)
- (Optional) ABB RobotStudio mit EGM-Konfiguration für Live-Tests

## Quickstart

```bash
# 1. Container bauen und starten
docker-compose up --build -d

# 2. Mainsail Web-UI öffnen
#    http://localhost:8181

# 3. Bridge starten (in separatem Terminal)
cd bridge
python run_bridge.py --config bridge_config.json

# 4. In Mainsail: G-Code hochladen und Print starten
#    (z.B. toolhead_test.gcode)
#    Die Bridge verbindet sich automatisch und sendet Heartbeats.
#    Bei Sync-Problemen wird der Print automatisch pausiert.
```

## Ports

| Port | Dienst | Richtung |
|------|--------|----------|
| 8181 | Mainsail Web-UI (via Nginx) | Browser → Container |
| 7125 | Moonraker API | Bridge → Container |
| 7200 | MoveExport TCP-Stream (Klipper → Bridge) | Container → Bridge |
| 7201 | Bridge-Watchdog (Heartbeat + Stop) | Bridge → Container |

## Sicherheitsmechanismen

Das System hat zwei Schutzrichtungen die ohne G-Code-Änderungen funktionieren — der Slicer-Output bleibt unverändert:

**Bridge → Klipper (aktiv):** Wenn die Bridge einen Sync-Fehler erkennt (Tracking-Error zu hoch, Lag zu groß, EGM-Watchdog), sendet sie einen Stop-Befehl an Klipper. Primär über den dedizierten TCP-Kanal (Port 7201), Fallback über die Moonraker HTTP-API.

**Klipper → sich selbst (passiv):** Die Bridge sendet alle 1s einen Heartbeat. Wenn 5s lang kein Heartbeat kommt (Bridge crashed, Netzwerk unterbrochen), pausiert Klipper den Print eigenständig.

In der Mainsail-Konsole:
```
BRIDGE_STATUS              # Zeigt Bridge-Verbindungsstatus
BRIDGE_WATCHDOG_RESET      # Setzt Watchdog nach Trigger zurück
```

## Konfiguration

Die Klipper-Konfiguration liegt in `config/printer.cfg`. Dort sind das `[move_export]`-Modul (TCP-Stream an Port 7200) und das `[bridge_watchdog]`-Modul (Heartbeat-Empfang auf Port 7201) konfiguriert.

Die Bridge-Konfiguration liegt in `bridge/bridge_config.json` und enthält EGM-Verbindungsparameter, Queue-Einstellungen, Sync-Grenzwerte, Watchdog-Einstellungen und Telemetrie-Optionen. Details dazu in `BRIDGE.md`.

## Tests

```bash
cd bridge/tests
python test_core.py          # Offline Smoke-Tests (kein Netzwerk nötig)
python test_watchdog.py      # Watchdog/Command-Tests (kein Netzwerk nötig)
python test_egm_direct.py    # EGM-Kreisbahn (braucht RobotStudio)
```

## Bekannte Probleme

**Windows Timer-Auflösung:** Auf Windows hat `time.sleep()` per Default nur 15.625ms Auflösung. Die Bridge aktiviert `timeBeginPeriod(1)` automatisch, aber falls die Zykluszeiten im TX-Log bei ~31ms statt ~20ms liegen, ist dies die Ursache. Details und Diagnose-Anleitung in `BRIDGE.md` unter "Bekannte Probleme".

## Weiterführende Dokumentation

- **[BRIDGE.md](BRIDGE.md)** — Detaillierte Architektur und Modul-Referenz der EGM-Bridge
- **[WATCHDOG_INTEGRATION.md](WATCHDOG_INTEGRATION.md)** — Beschreibung der Watchdog-Integration (Heartbeat + Stop)
