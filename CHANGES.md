# EGM Bridge Addon - Änderungen an deinem bestehenden Setup

## Was geändert wurde

### 1. docker-compose.yml
Neue Zeile bei volumes:
```yaml
- ./klippy_extras:/home/klippy/custom_extras
```

### 2. start.sh  
Neuer Block am Anfang (nach chmod):
```bash
CUSTOM_EXTRAS="/home/klippy/custom_extras"
KLIPPER_EXTRAS="/home/klippy/klipper/klippy/extras"
if [ -d "$CUSTOM_EXTRAS" ]; then
    for pyfile in "$CUSTOM_EXTRAS"/*.py; do
        ...symlink erstellen...
    done
fi
```

Außerdem: Default printer.cfg jetzt mit `kinematics: none` und `[egm_bridge]`

### 3. Neuer Ordner: klippy_extras/
```
klippy_extras/
└── egm_bridge.py    # Dein Custom Modul
```

## Installation

```powershell
cd D:\FH\Masterarbeit\Code\EgmControl\V1\Docker

# 1. Neuen Ordner erstellen
mkdir klippy_extras

# 2. egm_bridge.py reinkopieren

# 3. start.sh ersetzen

# 4. docker-compose.yml ersetzen (oder nur die Volume-Zeile hinzufügen)

# 5. Config löschen damit neue Default erstellt wird
Remove-Item -Recurse -Force config -ErrorAction SilentlyContinue

# 6. Neu bauen
docker compose down
docker compose up --build -d
```

## Testen

1. Öffne http://localhost:8181
2. Gehe zur Console
3. Tippe:
```
EGM_STATUS
EGM_TEST PARAM=hello VALUE=42
EGM_ACTIVATE
EGM_MOVE X=50 Y=50 Z=10
EGM_STATUS
```

## Nach Code-Änderungen

```powershell
# Schnell - nur Klipper neustarten (~2 Sek)
docker exec klipper supervisorctl restart klipper

# Logs checken
docker exec klipper tail -f /home/klippy/printer_data/logs/klippy.log
```

## Verfügbare Commands

| Command | Beschreibung |
|---------|-------------|
| `EGM_STATUS` | Status anzeigen |
| `EGM_ACTIVATE` | EGM aktivieren |
| `EGM_DEACTIVATE` | EGM deaktivieren |
| `EGM_TEST PARAM=x VALUE=y` | Test mit Parametern |
| `EGM_MOVE X=.. Y=.. Z=.. E=..` | Simulierter Move |
| `EGM_MOVE_LOG` | Toggle Logging |
