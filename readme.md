# Klipper Docker Setup – Kontext-Zusammenfassung

## Überblick
Ein Docker-Container auf einem **Windows-Host** (WSL2/Docker Desktop), der Klipper, Moonraker, Mainsail und einen virtuellen MCU in einem einzelnen Container betreibt. Enthält das **EGM Bridge Modul** für die spätere ABB-Roboter-Integration.

**Host-System:** Windows, Ryzen 7 4800HS, WSL2
**Projektpfad:** `E:\FH\Masterarbeit\Code\EgmControl\V1\Docker\`

## Architektur

```
┌──────────────────────────────────────────────────────────────┐
│  Docker Container (Ubuntu 24.04) - privileged mode           │
│                                                              │
│  ┌─────────────┐                                             │
│  │ klipper_mcu │  Virtueller MCU auf /tmp/klipper_host_mcu   │
│  └──────┬──────┘                                             │
│         │                                                    │
│  ┌──────▼──────┐  ┌───────────┐  ┌───────────┐              │
│  │   Klipper   │←→│ Moonraker │←→│   Nginx   │              │
│  │  (klippy)   │  │   (API)   │  │ (Mainsail)│              │
│  │             │  └───────────┘  └───────────┘              │
│  │ + egm_bridge│        ↑              ↑                     │
│  └─────────────┘        │              │                     │
│         ↑          Port 7125       Port 80                   │
│   /tmp/klippy_uds                                            │
│                                                              │
│  Supervisor verwaltet alle 4 Prozesse                        │
└──────────────────────────┬───────────────────────────────────┘
                           │
                  Ports nach außen:
                  - 8181:80   (Mainsail GUI)
                  - 7125:7125 (Moonraker API)
```

## Dateien im Docker-Ordner

| Datei | Funktion |
|---|---|
| `Dockerfile` | Baut Image mit Klipper, Moonraker, Mainsail, Nginx + **kompiliert Linux Host MCU** |
| `docker-compose.yml` | Container-Definition, Ports, Volumes, **privileged: true** |
| `start.sh` | Entrypoint: verlinkt Custom Extras, erstellt Default-Configs, startet Supervisor |
| `supervisord.conf` | Prozess-Manager für **klipper_mcu** + Klipper + Moonraker + Nginx |
| `nginx.conf` | Reverse Proxy: Mainsail static files + Moonraker API/Websocket |

## Volumes (persistent auf Host)

```
./config        → /home/klippy/printer_data/config   (printer.cfg, moonraker.conf)
./logs          → /home/klippy/printer_data/logs     (alle Log-Dateien)
./gcodes        → /home/klippy/printer_data/gcodes   (G-Code Dateien)
./klippy_extras → /home/klippy/custom_extras         (Custom Klipper Module)
```

## Custom Klipper Module

### EGM Bridge (`klippy_extras/egm_bridge.py`)

Registriert folgende G-Code Befehle:

| Command | Beschreibung |
|---------|--------------|
| `EGM_STATUS` | Zeigt aktuellen Status des Moduls |
| `EGM_ACTIVATE` | Aktiviert EGM-Modus |
| `EGM_DEACTIVATE` | Deaktiviert EGM-Modus |
| `EGM_TEST PARAM=x VALUE=y` | Test-Befehl mit Parametern |
| `EGM_MOVE X=.. Y=.. Z=.. E=.. F=..` | Simulierter Move |
| `EGM_MOVE_LOG` | Toggle Move-Logging an/aus |

Wird in `printer.cfg` geladen via:
```ini
[egm_bridge]
debug_mode: true
log_moves: false
```

## Wichtige Details

- **privileged: true** erforderlich damit `klipper_mcu` läuft (Realtime-Scheduling)
- **klipper_mcu** wird mit `-I /tmp/klipper_host_mcu` gestartet (ohne `-w` und `-r` wegen WSL2-Limitierungen)
- **Port 8080 ist auf dem Host belegt** → Mainsail läuft auf **Port 8181**
- **start.sh** verlinkt automatisch alle `.py` Dateien aus `./klippy_extras` nach Klipper
- **kinematics: none** - Klipper läuft im Simulator-Modus ohne echte Stepper

## Befehle

```powershell
# Starten
cd E:\FH\Masterarbeit\Code\EgmControl\V1\Docker
docker compose up -d

# Logs
docker logs klipper -f

# In Container
docker exec -it klipper bash

# Klipper Log checken
docker exec klipper tail -50 /home/klippy/printer_data/logs/klippy.log

# Nur Klipper neustarten (nach Code-Änderung in egm_bridge.py)
docker exec klipper supervisorctl restart klipper

# Alle Services Status
docker exec klipper supervisorctl status

# Stoppen
docker compose down

# Komplett neu bauen
docker compose down
Remove-Item -Recurse -Force config, logs -ErrorAction SilentlyContinue
docker compose up --build -d
```

## Entwicklungs-Workflow

1. **Code ändern:** `klippy_extras/egm_bridge.py` bearbeiten
2. **Klipper neustarten:** `docker exec klipper supervisorctl restart klipper`
3. **Testen:** In Mainsail Console (http://localhost:8181) Commands eingeben
4. **Logs prüfen:** `docker exec klipper tail -f /home/klippy/printer_data/logs/klippy.log`

## GUI
Mainsail erreichbar unter: **http://localhost:8181**

## printer.cfg (aktuell)

```ini
[mcu]
serial: /tmp/klipper_host_mcu

[printer]
kinematics: none
max_velocity: 300
max_accel: 3000

[virtual_sdcard]
path: /home/klippy/printer_data/gcodes

[display_status]
[pause_resume]

[egm_bridge]
debug_mode: true
log_moves: false
```

## Status (Stand: 12.02.2026)
- ✅ Container baut und startet sauber
- ✅ klipper_mcu (virtueller MCU) läuft
- ✅ Klipper, Moonraker, Nginx alle RUNNING
- ✅ Mainsail GUI erreichbar und funktional
- ✅ Moonraker verbindet sich mit Klipper
- ✅ Klipper "Ready" (kein Config-Error mehr)
- ✅ EGM Bridge Modul geladen
- ✅ Custom G-Code Commands funktionieren (EGM_STATUS, EGM_MOVE, etc.)
- 🔲 EGM UDP-Kommunikation zu ABB implementieren
- 🔲 Echte Move-Interception vom Toolhead
- 🔲 Pressure Advance Integration

## Nächste Schritte

1. Socket/UDP-Verbindung zu ABB OmniCore implementieren
2. EGM Protokoll (Protobuf) einbinden
3. Klipper Toolhead Moves abfangen und an Roboter streamen
4. Pressure Advance in der Extruder-Steuerung testen
