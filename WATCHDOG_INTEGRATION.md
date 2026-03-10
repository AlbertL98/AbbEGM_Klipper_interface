# Bridge ↔ Klipper Watchdog — Integrationsanleitung

## Überblick

Zwei neue Schutzmechanismen die verhindern, dass der Roboter weiterfährt wenn die Bridge Probleme hat — und umgekehrt, dass Klipper weiter extrudiert wenn die Bridge tot ist.

### Schutzrichtung 1: Bridge stirbt → Klipper stoppt (Heartbeat)

```
Bridge                          Klipper
  │                               │
  │──── {"type":"heartbeat"} ────→│  (alle 1s)
  │──── {"type":"heartbeat"} ────→│
  │──── {"type":"heartbeat"} ────→│
  │                               │
  ✗ Bridge crashed                │
  │                               │← 5s ohne Heartbeat
  │                               │← bridge_watchdog.py: PAUSE!
```

### Schutzrichtung 2: Bridge erkennt Problem → Klipper stoppt (Befehl)

```
Bridge                          Klipper
  │                               │
  │  SyncMonitor: Tracking 15mm!  │
  │  StateMachine → FAULT         │
  │                               │
  │──── {"type":"stop"} ─────────→│  (sofort)
  │                               │← bridge_watchdog.py: PAUSE!
  │                               │
  │  (Fallback bei TCP-Fehler:)   │
  │──── POST /printer/emergency──→│  (via Moonraker HTTP)
```

## Neue Dateien

| Datei | Zweck |
|-------|-------|
| `klippy_extras/bridge_watchdog.py` | Klipper-Extra: empfängt Heartbeats, stoppt bei Timeout/Befehl |
| `bridge/data/klipper_command.py` | Bridge-Modul: sendet Heartbeats + Stop/Pause-Befehle |
| `bridge/tests/test_watchdog.py` | Tests für die neuen Komponenten |

## Geänderte Dateien

| Datei | Änderung |
|-------|----------|
| `bridge/data/config.py` | Neue `WatchdogConfig` Dataclass + Validierung |
| `bridge/data/bridge.py` | `KlipperCommandClient` Integration + `_on_state_change` Callback |
| `bridge/data/__init__.py` | `KlipperCommandClient` Export |
| `bridge/bridge_config.json` | Neue `"watchdog"` Sektion |
| `config/printer.cfg` | Neue `[bridge_watchdog]` Sektion |
| `docker-compose.yml` | Port 7201 exponieren |
| `Dockerfile` | `EXPOSE 7201` hinzufügen |

## Einrichtung

### 1. printer.cfg ergänzen

Am Ende von `config/printer.cfg` hinzufügen:

```ini
[bridge_watchdog]
tcp_port: 7201
heartbeat_timeout: 5.0
action_on_timeout: pause
action_on_bridge_stop: pause
enabled: true
```

### 2. bridge_config.json ergänzen

Die neue `"watchdog"` Sektion ist bereits in der aktualisierten `bridge_config.json`:

```json
"watchdog": {
    "enabled": true,
    "tcp_host": "127.0.0.1",
    "tcp_port": 7201,
    "heartbeat_interval_s": 1.0,
    "reconnect_interval_s": 2.0,
    "stop_on_fault": true,
    "pause_on_degrade": false,
    "moonraker_fallback": true
}
```

### 3. Docker-Ports

In `docker-compose.yml` den neuen Port hinzufügen:

```yaml
ports:
  - "7201:7201"     # Bridge-Watchdog
```

In `Dockerfile` die EXPOSE-Zeile erweitern:

```dockerfile
EXPOSE 7125 80 7200 7201
```

### 4. Testen

```bash
# Watchdog-Tests (kein Netzwerk nötig)
cd bridge/tests
python test_watchdog.py

# Bestehende Tests (sollten weiterhin bestehen)
python test_core.py
```

## Konfigurationsoptionen

### Klipper-Seite (printer.cfg)

| Parameter | Default | Beschreibung |
|-----------|---------|-------------|
| `tcp_port` | 7201 | Port auf dem der Watchdog lauscht |
| `heartbeat_timeout` | 5.0 | Sekunden ohne Heartbeat → Aktion auslösen |
| `action_on_timeout` | pause | Was tun bei Heartbeat-Timeout: `pause` oder `emergency_stop` |
| `action_on_bridge_stop` | pause | Was tun bei explizitem Stop von Bridge: `pause` oder `emergency_stop` |
| `enabled` | true | Watchdog komplett deaktivieren |

### Bridge-Seite (bridge_config.json)

| Parameter | Default | Beschreibung |
|-----------|---------|-------------|
| `enabled` | true | Watchdog-Client aktivieren |
| `tcp_host` | 127.0.0.1 | Host des Klipper-Watchdog |
| `tcp_port` | 7201 | Port des Klipper-Watchdog |
| `heartbeat_interval_s` | 1.0 | Heartbeat-Frequenz |
| `stop_on_fault` | true | Bei FAULT → Stop an Klipper senden |
| `pause_on_degrade` | false | Bei DEGRADE → Pause an Klipper senden |
| `moonraker_fallback` | true | Bei TCP-Fehler über Moonraker stoppen |

## Wichtige Design-Entscheidungen

**Kein G-Code nötig:** Alles läuft über `printer.cfg` Konfiguration. Der Slicer-Output bleibt unverändert.

**PAUSE statt EMERGENCY_STOP als Default:** `PAUSE` ist sicherer weil der Print danach fortgesetzt werden kann. `emergency_stop` (M112) legt Klipper komplett lahm und erfordert Neustart.

**Heartbeat-Timeout nur bei aktivem Print:** Der Watchdog löst NUR aus wenn ein Print läuft UND die Bridge während dieses Prints verbunden war. Ohne Bridge kann normal gedruckt werden (z.B. Extruder-Tests).

**Zwei Kanäle für Stop-Befehle:** Primär über dedizierten TCP, Fallback über Moonraker HTTP-API. Bei einem echten Fehler will man auf keinen Fall, dass der Befehl verloren geht.

**Bridge-Verbindung ist optional:** Wenn die Bridge nie verbindet, passiert nichts. Der Watchdog wird erst aktiv wenn die Bridge sich verbindet UND ein Print läuft.

## GCode-Befehle

In der Mainsail-Konsole:

```
BRIDGE_STATUS              # Zeigt Bridge-Verbindungsstatus
BRIDGE_WATCHDOG_RESET      # Setzt Watchdog zurück (nach Trigger)
```

## Sequenzdiagramm: Normaler Print

```
Klipper          bridge_watchdog     Bridge            ABB Robot
  │                    │                │                  │
  │ Print Start        │                │                  │
  ├───────────────────→│                │                  │
  │                    │                │                  │
  │                    │←── heartbeat ──┤                  │
  │                    │←── heartbeat ──┤                  │
  │                    │                │                  │
  │── Segmente ──────────────────────→│                  │
  │                    │                ├── EGM Samples ──→│
  │                    │←── heartbeat ──┤←── Feedback ─────┤
  │                    │                │                  │
  │                    │                │ Sync OK ✓        │
  │                    │←── heartbeat ──┤                  │
  │                    │                │                  │
  │ Print Ende         │                │                  │
```

## Sequenzdiagramm: Bridge-Crash

```
Klipper          bridge_watchdog     Bridge            ABB Robot
  │                    │                │                  │
  │                    │←── heartbeat ──┤                  │
  │                    │←── heartbeat ──┤                  │
  │                    │                │                  │
  │                    │                ✗ CRASH            │
  │                    │                                   │
  │                    │  (5s Timeout)                     │
  │                    │                                   │
  │←── PAUSE ──────────┤                                   │
  │                    │                                   │
  │ Print pausiert ✓   │                                   │
```

## Sequenzdiagramm: Sync-Fehler

```
Klipper          bridge_watchdog     Bridge            ABB Robot
  │                    │                │                  │
  │                    │←── heartbeat ──┤←── Feedback ─────┤
  │                    │                │                  │
  │                    │                │ Tracking: 15mm!  │
  │                    │                │ → FAULT          │
  │                    │                │                  │
  │                    │←── STOP ───────┤                  │
  │←── PAUSE ──────────┤                │                  │
  │                    │                │                  │
  │ Print pausiert ✓   │                │                  │
```
