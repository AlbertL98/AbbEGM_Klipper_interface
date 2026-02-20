# EGM-Bridge — Projektstruktur

## Gesamtübersicht

```
Docker/
├── bridge/                          ← EGM-Bridge (eigener Prozess)
│   ├── run_bridge.py                ← Hauptskript — startet Bridge + Control-Server
│   ├── bridge_config.json           ← Konfiguration (Profil, Ports, Sync-Limits)
│   ├── egm_pb2.py                   ← ABB Protobuf (kompiliert, von dir bereitgestellt)
│   │
│   ├── egm_bridge/                  ← Python-Package: Core-Module
│   │   ├── __init__.py              ← Package-Exports (v0.2.0)
│   │   ├── bridge.py                ← Orchestrator: EGM-Loop, Lifecycle, Starvation-Erkennung
│   │   ├── config.py                ← BridgeConfig Dataclasses + Validierung + JSON-Load/Save
│   │   ├── control_server.py        ← TCP Control-Server (Port 7201) für G-Code-Commands  ★ NEU
│   │   ├── egm_client.py            ← UDP/EGM-Kommunikation (Dual-Socket, Protobuf)
│   │   ├── segment_source.py        ← TCP-Empfänger für Klipper-Segmente + CSV-Replay
│   │   ├── state_machine.py         ← INIT→READY→RUN→DEGRADED→STOP/FAULT
│   │   ├── sync_monitor.py          ← Tracking-Error, Lag, Jitter → Sync-Level
│   │   ├── telemetry.py             ← CSV-Streams: plan, tx, rx, sync, event
│   │   └── trajectory_planner.py    ← Trapez-Interpolation → EGM-Samples (50Hz)
│   │
│   ├── tests/                       ← Test-Skripte
│   │   ├── test_core.py             ← Smoke-Tests (State Machine, Config, Planner, Sync)
│   │   └── sim_robot.py             ← Simulierter ABB-Roboter (UDP Echo)
│   │
│   ├── test_egm_direct.py           ← Standalone-Test: Kreisbahn wie workingEGM.py
│   │
│   └── logs/                        ← Telemetrie-Daten (pro Job ein Ordner)
│       └── job_XXXXXXXXXX/
│           ├── config_snapshot.json
│           ├── plan.csv              ← Empfangene Segmente von Klipper
│           ├── tx.csv                ← An Roboter gesendete Positionen
│           ├── rx.csv                ← Vom Roboter empfangenes Feedback
│           ├── sync.csv              ← Tracking-Error, Lag, Jitter über Zeit
│           └── event.csv             ← State-Wechsel, Warnungen, Job-Events
│
├── klippy_extras/                   ← Klipper-Module (→ klipper/klippy/extras/)
│   ├── move_export.py               ← Segment-Export (CSV + TCP Port 7200) — UNVERÄNDERT
│   └── egm_commands.py              ← G-Code-Commands: EGM_START/STOP/STATUS/SET_PARAM  ★ NEU
│
└── workingEGM.py                    ← Referenz-EGM-Code (dein bewährter Standalone-Test)
```

## Port-Belegung

| Port | Protokoll | Richtung | Zweck |
|------|-----------|----------|-------|
| 7200 | TCP | Klipper → Bridge | Segment-Stream (JSON-Lines) |
| 7201 | TCP | Klipper → Bridge | Control-Commands (JSON-RPC) |
| 6510 | UDP | ABB → Bridge | EGM-Feedback (Protobuf) |
| 6512 | UDP | Bridge (lokal) | EGM-Send-Socket (bind) |
| 6599 | UDP | Bridge → ABB | EGM-Sollwerte (Protobuf) |

## Datenfluss

```
Mainsail Terminal                    RobotStudio / ABB Controller
      │                                        ▲   │
      │ EGM_START / EGM_STOP                   │   │
      ▼                                        │   ▼
┌─────────────────────┐              ┌─────────────────────┐
│  egm_commands.py    │  TCP:7201    │                     │
│  (Klipper-Extra)    │─────────────►│    run_bridge.py    │
│                     │◄─────────────│    + ControlServer   │
│  Spawnt Prozess     │  Response    │                     │
└─────────────────────┘              │  EGM-Loop (50Hz):   │
                                     │  Interpolation →    │
┌─────────────────────┐  TCP:7200    │  UDP-Send → ABB     │──► UDP:6599
│  move_export.py     │─────────────►│  ← UDP-Feedback     │◄── UDP:6510
│  (Klipper-Extra)    │  Segmente    │  Sync-Monitoring    │
│                     │              │  Telemetrie-CSV     │
│  trapq_append Hook  │              └─────────────────────┘
└─────────────────────┘
```

## printer.cfg Konfiguration

```ini
[move_export]
output_path: /home/klippy/printer_data/logs/move_segments.csv
tcp_port: 7200
tcp_enabled: true
log_to_console: false

[egm_commands]
bridge_path: /home/klippy/egm_bridge/run_bridge.py
bridge_config: /home/klippy/egm_bridge/bridge_config.json
control_port: 7201
python_path: python3
log_file: /home/klippy/printer_data/logs/egm_bridge.log
```

## G-Code Commands

| Command | Beschreibung | Beispiel |
|---------|-------------|----------|
| `EGM_START` | Bridge starten + Job beginnen | `EGM_START JOB=my_print` |
| `EGM_STOP` | Laufenden Job stoppen | `EGM_STOP` |
| `EGM_STATUS` | Status im Terminal anzeigen | `EGM_STATUS` |
| `EGM_SET_PARAM` | Parameter zur Laufzeit ändern | `EGM_SET_PARAM SECTION=sync KEY=tracking_warn_mm VALUE=10.0` |

## Deployment

### Auf dem Pi5 (Produktion):
```bash
# 1. Bridge-Code kopieren
scp -r bridge/ pi@klipper:/home/klippy/egm_bridge/

# 2. Klipper-Extras kopieren
scp klippy_extras/egm_commands.py pi@klipper:/home/klippy/klipper/klippy/extras/
scp klippy_extras/move_export.py  pi@klipper:/home/klippy/klipper/klippy/extras/

# 3. printer.cfg anpassen (Sections oben hinzufügen)

# 4. Klipper neu starten → EGM_START im Mainsail-Terminal
```

### Auf Windows (Entwicklung mit RobotStudio):
```powershell
cd Docker\bridge
python run_bridge.py --config bridge_config.json --auto-start
# Oder mit Control-Port:
python run_bridge.py --config bridge_config.json --control-port 7201
```
