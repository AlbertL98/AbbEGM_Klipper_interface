from __future__ import annotations
# config.py — Konfiguration und Parameter-Profile
#
# CodingPlan §B6: Extern konfigurierbare Parameterprofile
#   - EGM-Regelparameter/Filter/Gains
#   - Pfad-/Orientierungstoleranzen
#   - Max speed/accel/Abweichung
#   - Timeout-/Watchdog-Grenzen
#   - Plausibilitätsprüfung vor Jobstart (Range Checks)
#
# ÄNDERUNGEN:
#   - WorkspaceEnvelopeConfig: Begrenzungsbox für Zielpositionen
#   - starvation_timeout_s: Konfigurierbar statt hardcoded

import copy
import json
import time
import logging
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

from .clock import bridge_now

logger = logging.getLogger("egm.config")


@dataclass
class EgmConnectionConfig:
    """UDP-Verbindungsparameter zum ABB-Controller."""
    robot_ip: str = "127.0.0.1"
    send_port: int = 6599           # Zielport UCdevice
    recv_port: int = 6510           # Feedback von ROB_1
    local_send_port: int = 6512     # Fester Source-Port fürs Senden
    cycle_ms: int = 20              # 50 Hz wie workingEGM.py
    timeout_ms: int = 100           # rx.settimeout(0.1) wie workingEGM.py
    watchdog_cycles: int = 50
    protocol: str = "auto"          # "auto" | "protobuf" | "json"
    # Default-Orientierung (Quaternion aus workingEGM.py)
    default_q0: float = 0.0
    default_q1: float = -0.707106
    default_q2: float = 0.707106
    default_q3: float = 0.0


@dataclass
class WorkspaceEnvelopeConfig:
    """
    Begrenzungsbox für Roboter-Zielpositionen (Kollisionsschutz).

    Jede Zielposition wird VOR dem Senden an den Roboter geprüft.
    Liegt sie außerhalb der Box, wird NICHT gesendet und die
    Bridge geht in FAULT → Klipper wird gestoppt.

    Die Werte sollten konservativ gesetzt werden — lieber etwas
    kleiner als der tatsächliche Arbeitsraum des Roboters.

    Einheit: mm (wie alle Positionen in der Bridge).
    """
    enabled: bool = True
    min_x: float = -500.0
    max_x: float = 500.0
    min_y: float = -500.0
    max_y: float = 500.0
    min_z: float = -10.0       # Leicht unter 0 für Toleranz
    max_z: float = 500.0

    def contains(self, x: float, y: float, z: float) -> bool:
        """Prüft ob Position innerhalb der erlaubten Box liegt."""
        return (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y
                and self.min_z <= z <= self.max_z)

    def violation_reason(self, x: float, y: float, z: float) -> str:
        """Beschreibt welche Achse(n) außerhalb liegen."""
        parts = []
        if x < self.min_x:
            parts.append(f"X={x:.2f} < min_x={self.min_x:.2f}")
        if x > self.max_x:
            parts.append(f"X={x:.2f} > max_x={self.max_x:.2f}")
        if y < self.min_y:
            parts.append(f"Y={y:.2f} < min_y={self.min_y:.2f}")
        if y > self.max_y:
            parts.append(f"Y={y:.2f} > max_y={self.max_y:.2f}")
        if z < self.min_z:
            parts.append(f"Z={z:.2f} < min_z={self.min_z:.2f}")
        if z > self.max_z:
            parts.append(f"Z={z:.2f} > max_z={self.max_z:.2f}")
        return "; ".join(parts) if parts else "OK"


@dataclass
class QueueConfig:
    """Buffer- und Queue-Parameter (§B2)."""
    plan_queue_size: int = 2000      # Max Segmente in der Plan-Queue
    send_queue_lookahead_ms: float = 200.0  # Vorlaufzeit in ms
    feedback_queue_size: int = 500
    low_watermark: int = 20          # Segmente — darunter DEGRADED
    high_watermark: int = 1800       # Segmente — darüber throttle
    underflow_action: str = "degrade"  # "degrade" | "stop"
    starvation_timeout_s: float = 3.0  # Sekunden ohne Segmente → Job beendet


@dataclass
class SyncConfig:
    """Synchronisations- und Korrekturparameter (§B3–B5, §E)."""
    # Tracking-Error-Grenzen (mm)
    tracking_warn_mm: float = 2.0
    tracking_degrade_mm: float = 5.0
    tracking_stop_mm: float = 10.0

    # Lag-Grenzen (ms)
    lag_warn_ms: float = 20.0
    lag_degrade_ms: float = 50.0
    lag_stop_ms: float = 100.0

    # Jitter-Grenzen (ms)
    jitter_p99_warn_ms: float = 5.0
    jitter_p99_stop_ms: float = 15.0

    # Lag-Messung: Ringbuffer-Größe für positionsbasiertes Matching
    lag_buffer_size: int = 200       # ~4s bei 50Hz, ~1s bei 250Hz

    # Korrektur-Limits (§B4)
    correction_max_mm: float = 3.0          # Max Korrekturbetrag
    correction_rate_limit_mm_per_s: float = 10.0  # Max Änderung/s
    correction_enabled: bool = True

    # Offset-Modell (§B5)
    time_offset_ms: float = 0.0       # Δt0 — Basis-Offset
    offset_kv: float = 0.0            # Geschwindigkeitsabhängig
    offset_ka: float = 0.0            # Beschleunigungsabhängig


@dataclass
class KlipperSourceConfig:
    """Verbindung zur Klipper move_export.py."""
    tcp_host: str = "127.0.0.1"
    tcp_port: int = 7200
    reconnect_interval_s: float = 2.0
    receive_timeout_s: float = 5.0


@dataclass
class MoonrakerConfig:
    """Moonraker-Websocket-Verbindung für Extruder-E-Wert."""
    enabled: bool = False           # Explizit aktivieren
    host: str = "127.0.0.1"
    port: int = 7125                # Moonraker Default-Port
    reconnect_interval_s: float = 3.0
    poll_interval_ms: float = 0.0      # 0 = nur Subscribe (~4 Hz, Moonraker-Limit)
    stale_threshold_ms: float = 500.0  # E-Wert-Alter ab dem gewarnt wird


@dataclass
class WatchdogConfig:
    """
    Klipper-Watchdog: Heartbeat + Stop-Befehle an Klipper.

    Verbindet sich mit dem bridge_watchdog.py Klipper-Extra
    und sendet periodische Heartbeats. Bei Problemen (FAULT,
    DEGRADE, Watchdog-Timeout) wird Klipper angewiesen den
    Print zu pausieren oder zu stoppen.

    Zwei Schutzrichtungen:
      Bridge → Klipper: "Ich habe ein Problem, stopp den Print!"
      Klipper → sich selbst: "Bridge meldet sich nicht mehr, ich pausiere."
    """
    enabled: bool = True
    tcp_host: str = "127.0.0.1"
    tcp_port: int = 7201                # Port des bridge_watchdog Klipper-Extra
    heartbeat_interval_s: float = 1.0   # Heartbeat alle N Sekunden
    reconnect_interval_s: float = 2.0   # Reconnect-Interval bei Verbindungsverlust
    stop_on_fault: bool = True          # Bei FAULT → Stop-Befehl an Klipper
    pause_on_degrade: bool = False      # Bei DEGRADE → Pause-Befehl an Klipper
    moonraker_fallback: bool = True     # Zusätzlich über Moonraker-API stoppen


@dataclass
class TelemetryConfig:
    """Logging und Telemetrie (§B7)."""
    log_dir: str = "./logs"
    log_level: str = "INFO"
    csv_export: bool = True
    metric_interval_s: float = 1.0     # Sync-Metriken alle N Sekunden
    max_log_size_mb: int = 100


@dataclass
class BridgeConfig:
    """Gesamtkonfiguration des EGM-Bridge-Core."""
    connection: EgmConnectionConfig = field(default_factory=EgmConnectionConfig)
    workspace: WorkspaceEnvelopeConfig = field(default_factory=WorkspaceEnvelopeConfig)
    queues: QueueConfig = field(default_factory=QueueConfig)
    sync: SyncConfig = field(default_factory=SyncConfig)
    klipper: KlipperSourceConfig = field(default_factory=KlipperSourceConfig)
    moonraker: MoonrakerConfig = field(default_factory=MoonrakerConfig)
    watchdog: WatchdogConfig = field(default_factory=WatchdogConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)

    # Profil-Metadaten
    profile_name: str = "default"
    profile_version: str = "1.0"

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self, indent=2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def snapshot(self) -> dict:
        """Snapshot für Job-Protokollierung (§B6)."""
        return {
            "profile_name": self.profile_name,
            "profile_version": self.profile_version,
            "timestamp": bridge_now(),
            "config": self.to_dict(),
        }


# ── Range-Checks / Plausibilitätsprüfung (§B6) ──────────────

class ConfigValidationError(Exception):
    pass


def validate_config(cfg: BridgeConfig) -> list[str]:
    """
    Plausibilitätsprüfung vor Jobstart.
    Gibt Liste von Warnungen/Fehlern zurück.
    Leere Liste = alles OK.
    """
    errors = []

    # Connection
    c = cfg.connection
    if c.cycle_ms < 2 or c.cycle_ms > 100:
        errors.append(f"cycle_ms={c.cycle_ms} außerhalb [2, 100]")
    if c.timeout_ms < c.cycle_ms:
        errors.append(f"timeout_ms={c.timeout_ms} zu klein (< cycle_ms)")
    if c.watchdog_cycles < 5:
        errors.append(f"watchdog_cycles={c.watchdog_cycles} zu klein (< 5)")

    # Workspace Envelope
    w = cfg.workspace
    if w.enabled:
        if w.min_x >= w.max_x:
            errors.append(f"workspace: min_x={w.min_x} >= max_x={w.max_x}")
        if w.min_y >= w.max_y:
            errors.append(f"workspace: min_y={w.min_y} >= max_y={w.max_y}")
        if w.min_z >= w.max_z:
            errors.append(f"workspace: min_z={w.min_z} >= max_z={w.max_z}")
        # Warnung bei sehr großem Arbeitsraum
        vol = ((w.max_x - w.min_x) * (w.max_y - w.min_y)
               * (w.max_z - w.min_z))
        if vol > 1e9:  # > 1000mm³ pro Achse im Schnitt
            errors.append(
                f"workspace: Volumen={vol:.0f}mm³ sehr groß — "
                "Begrenzung prüfen")

    # Queues
    q = cfg.queues
    if q.low_watermark >= q.high_watermark:
        errors.append("low_watermark >= high_watermark")
    if q.send_queue_lookahead_ms < c.cycle_ms * 5:
        errors.append("lookahead zu klein (< 5 Zyklen)")
    if q.starvation_timeout_s < 0.5:
        errors.append(f"starvation_timeout_s={q.starvation_timeout_s} "
                      "zu klein (< 0.5s)")
    if q.starvation_timeout_s > 60.0:
        errors.append(f"starvation_timeout_s={q.starvation_timeout_s} "
                      "zu groß (> 60s)")

    # Sync
    s = cfg.sync
    if not (s.tracking_warn_mm < s.tracking_degrade_mm < s.tracking_stop_mm):
        errors.append("Tracking-Grenzen nicht monoton steigend")
    if not (s.lag_warn_ms < s.lag_degrade_ms < s.lag_stop_ms):
        errors.append("Lag-Grenzen nicht monoton steigend")
    if s.correction_max_mm <= 0:
        errors.append("correction_max_mm muss > 0 sein")
    if s.time_offset_ms < -500 or s.time_offset_ms > 500:
        errors.append(f"time_offset_ms={s.time_offset_ms} unrealistisch")

    # Moonraker
    m = cfg.moonraker
    if m.enabled:
        if m.port < 1 or m.port > 65535:
            errors.append(f"moonraker.port={m.port} ungültig")
        if m.poll_interval_ms < 0:
            errors.append(f"moonraker.poll_interval_ms={m.poll_interval_ms} "
                          "ungültig (< 0)")
        if 0 < m.poll_interval_ms < 10:
            errors.append(f"moonraker.poll_interval_ms={m.poll_interval_ms} "
                          "zu klein (< 10ms, Moonraker-Überlastung)")
        if m.stale_threshold_ms < 50:
            errors.append(f"moonraker.stale_threshold_ms="
                          f"{m.stale_threshold_ms} zu klein (< 50)")

    # Watchdog
    wd = cfg.watchdog
    if wd.enabled:
        if wd.tcp_port < 1 or wd.tcp_port > 65535:
            errors.append(f"watchdog.tcp_port={wd.tcp_port} ungültig")
        if wd.heartbeat_interval_s < 0.1:
            errors.append(f"watchdog.heartbeat_interval_s="
                          f"{wd.heartbeat_interval_s} zu klein (< 0.1s)")
        if wd.heartbeat_interval_s > 30.0:
            errors.append(f"watchdog.heartbeat_interval_s="
                          f"{wd.heartbeat_interval_s} zu groß (> 30s)")
        # Port-Kollision mit move_export prüfen
        if wd.tcp_port == cfg.klipper.tcp_port:
            errors.append(f"watchdog.tcp_port={wd.tcp_port} kollidiert "
                          f"mit klipper.tcp_port")

    return errors


# ── Laden / Speichern ────────────────────────────────────────

def load_config(path: str) -> BridgeConfig:
    """Lädt Konfiguration aus JSON-Datei."""
    p = Path(path)
    if not p.exists():
        logger.warning("Config nicht gefunden: %s — nutze Defaults", path)
        return BridgeConfig()

    with open(p) as f:
        data = json.load(f)

    cfg = BridgeConfig()

    # Flache Zuordnung zu Dataclasses
    for section_name, section_cls in [
        ("connection", EgmConnectionConfig),
        ("workspace", WorkspaceEnvelopeConfig),
        ("queues", QueueConfig),
        ("sync", SyncConfig),
        ("klipper", KlipperSourceConfig),
        ("moonraker", MoonrakerConfig),
        ("watchdog", WatchdogConfig),
        ("telemetry", TelemetryConfig),
    ]:
        if section_name in data:
            section_obj = getattr(cfg, section_name)
            for key, val in data[section_name].items():
                if hasattr(section_obj, key):
                    setattr(section_obj, key, val)
                else:
                    logger.warning("Unbekannter Config-Key: %s.%s",
                                   section_name, key)

    # Top-Level-Felder
    if "profile_name" in data:
        cfg.profile_name = data["profile_name"]
    if "profile_version" in data:
        cfg.profile_version = data["profile_version"]

    logger.info("Config geladen: %s (Profil: %s v%s)",
                path, cfg.profile_name, cfg.profile_version)
    return cfg


def save_config(cfg: BridgeConfig, path: str):
    """Speichert Konfiguration als JSON."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w") as f:
        f.write(cfg.to_json())
    logger.info("Config gespeichert: %s", path)
