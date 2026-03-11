# EGM-Bridge v0.3.0 — Änderungen

## Neue Dateien

### `bridge/data/clock.py` — Einheitliche Zeitquelle
Zentrale `bridge_now()` Funktion die `time.perf_counter()` kapselt.
Alle Komponenten importieren diese Funktion statt direkte Clock-Aufrufe.
Verhindert Clock-Mixing (was auf Windows zu falschen RTT/Lag-Werten führte).

## Bug-Fixes

### 1. Clock-Mixing in `egm_client.py` (KRITISCH)
**Problem:** `send_target()` nutzte `time.monotonic()` für TX-Timestamps,
`_receive_loop()` nutzte `time.perf_counter()` für RX-Timestamps.
RTT wurde als Differenz berechnet → auf Windows falsche Werte weil
die Clocks unterschiedliche Epochen haben.

**Fix:** Alle Timestamps nutzen jetzt `bridge_now()` (= `time.perf_counter()`).

### 2. Clock-Inkonsistenz in `telemetry.py`
**Problem:** `log_event()` nutzte `time.monotonic()`, alle anderen Streams
(`log_tx`, `log_rx`, `log_plan`) nutzten `time.perf_counter()`.
Event-Timestamps waren nicht mit TX/RX vergleichbar.

**Fix:** `log_event()` nutzt jetzt `bridge_now()`.

### 3. Reconnect-Zähler in `klipper_command.py`
**Problem:** `self._reconnect_count += 1 if self._reconnect_count > 0 else 0`
tat beim ersten Connect nichts (Zähler startet bei 0). Log-Throttling
funktionierte dadurch nicht wie beabsichtigt.

**Fix:** Getrennte Zähler `_connect_count` (alle Verbindungen) und
`_reconnect_count` (nur Reconnects nach der ersten Verbindung).

### 4. Distanz-Overshoot in `segment_source.py`
**Problem:** `position_at()` konnte in der Decel-Phase durch Rundungsfehler
eine Distanz > `self.distance` berechnen → Position überschießt Segment-Ende.

**Fix:** Distanz wird auf `[0, self.distance]` geclampt.

### 5. Dockerfile EXPOSE
**Problem:** `EXPOSE 7125 80 7200` — Port 7201 fehlt obwohl er in
docker-compose.yml gemappt wird.

**Fix:** `EXPOSE 7125 80 7200 7201`

## Neue Features

### 1. Workspace Envelope (`config.py`, `bridge.py`)
**Begrenzungsbox für Roboter-Zielpositionen (Kollisionsschutz).**

Jede Zielposition wird VOR dem Senden an den Roboter geprüft.
Liegt sie außerhalb der erlaubten Box:
- Position wird NICHT gesendet
- Bridge geht in FAULT
- Klipper wird gestoppt
- Telemetrie-Event "WORKSPACE_VIOLATION" wird geloggt

Konfiguration in `bridge_config.json`:
```json
"workspace": {
    "enabled": true,
    "min_x": -500.0, "max_x": 500.0,
    "min_y": -500.0, "max_y": 500.0,
    "min_z": -10.0,  "max_z": 500.0
}
```

**Für echte Hardware:** Werte an den tatsächlichen Arbeitsraum des
ABB IRB 1100 anpassen — konservativ, lieber etwas kleiner.

### 2. Segment-Validierung (`segment_source.py`)
Neue `validate()` Methode auf `TrapezSegment`:
- Prüft duration > 0
- Prüft distance ≥ 0
- Prüft keine negativen Geschwindigkeiten
- Prüft keine negativen Phasenzeiten
- Prüft Phasensumme = duration
- Prüft Richtungsvektor normiert

Wird automatisch in `from_dict()` und `from_csv_row()` aufgerufen.
Ungültige Segmente werden mit `SegmentValidationError` abgelehnt
und gezählt (`segments_rejected` im Snapshot).

### 3. Konfigurierbarer Starvation-Timeout (`config.py`, `bridge.py`)
**Vorher:** `_starvation_timeout_s = 3.0` hardcoded in `bridge.py`.
**Jetzt:** `queues.starvation_timeout_s` in Config, Default 3.0s.
Validiert auf [0.5, 60.0].

### 4. Config-Validierung erweitert
- Workspace-Envelope: min < max pro Achse, Volumen-Warnung
- starvation_timeout_s: Range-Check [0.5, 60.0]

## Geänderte Dateien (Zusammenfassung)

| Datei | Änderung |
|-------|----------|
| `bridge/data/clock.py` | **NEU** — Einheitliche Zeitquelle |
| `bridge/data/config.py` | + WorkspaceEnvelopeConfig, + starvation_timeout_s, erweiterte Validierung |
| `bridge/data/bridge.py` | + Workspace-Check vor send_target(), bridge_now(), starvation aus Config |
| `bridge/data/egm_client.py` | Clock-Fix: bridge_now() statt gemischter Clocks |
| `bridge/data/sync_monitor.py` | Clock-Fix: bridge_now(), Kommentar korrigiert |
| `bridge/data/telemetry.py` | Clock-Fix: bridge_now() in log_event() |
| `bridge/data/klipper_command.py` | Reconnect-Zähler-Fix, bridge_now() |
| `bridge/data/segment_source.py` | + validate(), + Distanz-Clamp, + SegmentValidationError |
| `bridge/data/trajectory_planner.py` | bridge_now() für Korrektur |
| `bridge/data/moonraker_client.py` | bridge_now() |
| `bridge/data/__init__.py` | + clock, WorkspaceEnvelopeConfig, __all__ Export |
| `bridge/bridge_config.json` | + workspace Sektion, + starvation_timeout_s, v1.4 |
| `bridge/tests/test_core.py` | + 3 neue Tests (Clock, Envelope, Segment-Validierung) |
| `Dockerfile` | EXPOSE 7201 hinzufügen |
