from __future__ import annotations
# config.py — Konfiguration und Parameter-Profile
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
    send_port: int = 6599
    recv_port: int = 6510
    local_send_port: int = 6512
    cycle_ms: int = 20
    timeout_ms: int = 100
    watchdog_cycles: int = 50
    protocol: str = "auto"
    default_q0: float = 0.0
    default_q1: float = -0.707106
    default_q2: float = 0.707106
    default_q3: float = 0.0


@dataclass
class WorkspaceEnvelopeConfig:
    """Begrenzungsbox für Roboter-Zielpositionen (Kollisionsschutz)."""
    enabled: bool = True
    min_x: float = -500.0
    max_x: float = 500.0
    min_y: float = -500.0
    max_y: float = 500.0
    min_z: float = -10.0
    max_z: float = 500.0

    def contains(self, x: float, y: float, z: float) -> bool:
        return (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y
                and self.min_z <= z <= self.max_z)

    def violation_reason(self, x: float, y: float, z: float) -> str:
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
    """Buffer- und Queue-Parameter."""
    plan_queue_size: int = 2000
    send_queue_lookahead_ms: float = 200.0
    feedback_queue_size: int = 500
    low_watermark: int = 20
    high_watermark: int = 1800
    underflow_action: str = "degrade"
    starvation_timeout_s: float = 3.0


@dataclass
class SyncConfig:
    """Synchronisations- und Korrekturparameter."""
    delay_backend_ms: float         = 4.0   # Zeitversoegerung bei egm IST-Position-Kommunikation
    delay_frontend_init_ms: float   = 50.0  # Zeitverzögerung egm SOLL-Position-Kommunikation Initialisierungswert
    evaluate_direction: bool        = True  # Wenn aktiv die soll/ist error wir in tangential und normal richtung bewertet
    ema_slow: float                 = 0.04  # gewichtung bei niedriger geschwindigkeit
    ema_fast: float                 = 0.25  # gewichtung bei hoher geschwindigkeit
    ema_output: float               = 0.08  # gewichtung des outputs -> kleiner wert = keine ruckartigen offset änderungen
    alpha_weight: float             = 10.0  # gewichtung von normal error
    tx_buffer_size: int             = 500


@dataclass
class KlipperClientConfig:
    """Verbindung zur Klipper move_export.py."""
    tcp_host: str = "127.0.0.1"
    tcp_port_segment: int = 7200
    tcp_port_watchdog: int = 7201
    reconnect_interval_s: float = 2.0
    receive_timeout_s: float = 5.0
    watchdog_heartbeat_s: float = 1.0
    watchdog_stop_on_fault: bool = True
    watchdog_pause_on_degrade: bool = False


@dataclass
class MoonrakerConfig:
    """Moonraker-Websocket-Verbindung für Extruder-E-Wert."""
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 7125
    reconnect_interval_s: float = 3.0
    poll_interval_ms: float = 0.0
    stale_threshold_ms: float = 500.0


@dataclass
class WatchdogConfig:
    """Klipper-Watchdog: Heartbeat + Stop-Befehle an Klipper."""
    enabled: bool = True
    tcp_host: str = "127.0.0.1"
    tcp_port: int = 7201
    heartbeat_interval_s: float = 1.0
    reconnect_interval_s: float = 2.0
    stop_on_fault: bool = True
    pause_on_degrade: bool = False
    moonraker_fallback: bool = True


@dataclass
class TelemetryConfig:
    """Logging und Telemetrie."""
    log_dir: str = "./logs"
    log_level: str = "INFO"
    csv_export: bool = True
    metric_interval_s: float = 1.0
    max_log_size_mb: int = 100


@dataclass
class BridgeConfig:
    """Gesamtkonfiguration des EGM-Bridge-Core."""
    connection: EgmConnectionConfig = field(default_factory=EgmConnectionConfig)
    workspace: WorkspaceEnvelopeConfig = field(default_factory=WorkspaceEnvelopeConfig)
    queues: QueueConfig = field(default_factory=QueueConfig)
    sync: SyncConfig = field(default_factory=SyncConfig)
    klipper: KlipperClientConfig = field(default_factory=KlipperClientConfig)
    moonraker: MoonrakerConfig = field(default_factory=MoonrakerConfig)
    watchdog: WatchdogConfig = field(default_factory=WatchdogConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    estimator: LatencyEstimatorConfig = field(default_factory=LatencyEstimatorConfig)

    profile_name: str = "default"
    profile_version: str = "1.0"

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self, indent=2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def snapshot(self) -> dict:
        return {
            "profile_name": self.profile_name,
            "profile_version": self.profile_version,
            "timestamp": bridge_now(),
            "config": self.to_dict(),
        }


# ── Validierung ──────────────────────────────────────────────

class ConfigValidationError(Exception):
    pass


def validate_config(cfg: BridgeConfig) -> list[str]:
    """Plausibilitätsprüfung vor Jobstart. Leere Liste = alles OK."""
    errors = []

    c = cfg.connection
    if c.cycle_ms < 2 or c.cycle_ms > 100:
        errors.append(f"cycle_ms={c.cycle_ms} außerhalb [2, 100]")
    if c.timeout_ms < c.cycle_ms:
        errors.append(f"timeout_ms={c.timeout_ms} zu klein (< cycle_ms)")
    if c.watchdog_cycles < 5:
        errors.append(f"watchdog_cycles={c.watchdog_cycles} zu klein (< 5)")

    w = cfg.workspace
    if w.enabled:
        if w.min_x >= w.max_x:
            errors.append(f"workspace: min_x={w.min_x} >= max_x={w.max_x}")
        if w.min_y >= w.max_y:
            errors.append(f"workspace: min_y={w.min_y} >= max_y={w.max_y}")
        if w.min_z >= w.max_z:
            errors.append(f"workspace: min_z={w.min_z} >= max_z={w.max_z}")
        vol = ((w.max_x - w.min_x) * (w.max_y - w.min_y)
               * (w.max_z - w.min_z))
        vol_m3 = vol / 1e9
        if vol_m3 > 2.0:   # > 2 m³ ist unrealistisch für einen ABB IRB 1100
            logger.warning(
                "WorkspaceEnvelope: Volumen %.0f mm³ (%.2f m³) "
                "erscheint unrealistisch groß", vol, vol_m3)

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

    s = cfg.sync
    if not (s.tracking_warn_mm < s.tracking_degrade_mm < s.tracking_stop_mm):
        errors.append("Tracking-Grenzen nicht monoton steigend")
    if not (s.lag_warn_ms < s.lag_degrade_ms < s.lag_stop_ms):
        errors.append("Lag-Grenzen nicht monoton steigend")
    if s.correction_max_mm <= 0:
        errors.append("correction_max_mm muss > 0 sein")
    if s.time_offset_ms < -500 or s.time_offset_ms > 500:
        errors.append(f"time_offset_ms={s.time_offset_ms} unrealistisch")

    if not (s.offset_warn_ms < s.offset_degrade_ms < s.offset_stop_ms):
        errors.append("Offset-Grenzen nicht monoton steigend "
                      f"(warn={s.offset_warn_ms}, "
                      f"degrade={s.offset_degrade_ms}, "
                      f"stop={s.offset_stop_ms})")
    if s.offset_rate_warn_ms_per_s <= 0:
        errors.append(f"offset_rate_warn_ms_per_s="
                      f"{s.offset_rate_warn_ms_per_s} muss > 0 sein")
    if not (s.norm_error_warn_mm < s.norm_error_degrade_mm):
        errors.append("Norm-Error-Grenzen nicht monoton steigend")
    if s.norm_error_warn_mm <= 0:
        errors.append(f"norm_error_warn_mm={s.norm_error_warn_mm} "
                      "muss > 0 sein")

    m = cfg.moonraker
    if m.enabled:
        if m.port < 1 or m.port > 65535:
            errors.append(f"moonraker.port={m.port} ungültig")
        if m.poll_interval_ms < 0:
            errors.append(f"moonraker.poll_interval_ms="
                          f"{m.poll_interval_ms} ungültig (< 0)")
        if 0 < m.poll_interval_ms < 10:
            errors.append(f"moonraker.poll_interval_ms="
                          f"{m.poll_interval_ms} zu klein")
        if m.stale_threshold_ms < 50:
            errors.append(f"moonraker.stale_threshold_ms="
                          f"{m.stale_threshold_ms} zu klein (< 50)")

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
        if wd.tcp_port == cfg.klipper.tcp_port:
            errors.append(f"watchdog.tcp_port={wd.tcp_port} kollidiert "
                          f"mit klipper.tcp_port")

    est = cfg.estimator
    if est.enabled:
        if est.t_delay_init_ms < 1.0 or est.t_delay_init_ms > 1000.0:
            errors.append(f"estimator.t_delay_init_ms="
                          f"{est.t_delay_init_ms} außerhalb [1, 1000]ms")
        if not (0.001 <= est.ema_slow <= 0.5):
            errors.append(f"estimator.ema_slow={est.ema_slow} "
                          "außerhalb [0.001, 0.5]")
        if not (0.001 <= est.ema_fast <= 1.0):
            errors.append(f"estimator.ema_fast={est.ema_fast} "
                          "außerhalb [0.001, 1.0]")
        if est.ema_slow >= est.ema_fast:
            errors.append("estimator: ema_slow muss < ema_fast sein")
        if est.alpha_weight <= 0:
            errors.append(f"estimator.alpha_weight={est.alpha_weight} "
                          "muss > 0 sein")
        if est.tx_buffer_size < 50:
            errors.append(f"estimator.tx_buffer_size="
                          f"{est.tx_buffer_size} zu klein (< 50)")

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

    for section_name, section_cls in [
        ("connection", EgmConnectionConfig),
        ("workspace", WorkspaceEnvelopeConfig),
        ("queues", QueueConfig),
        ("sync", SyncConfig),
        ("klipper", KlipperClientConfig),
        ("moonraker", MoonrakerConfig),
        ("watchdog", WatchdogConfig),
        ("telemetry", TelemetryConfig),
        ("estimator", LatencyEstimatorConfig),
    ]:
        if section_name in data:
            section_obj = getattr(cfg, section_name)
            for key, val in data[section_name].items():
                if hasattr(section_obj, key):
                    setattr(section_obj, key, val)
                else:
                    logger.warning("Unbekannter Config-Key: %s.%s",
                                   section_name, key)

    if "profile_name" in data:
        cfg.profile_name = data["profile_name"]
    if "profile_version" in data:
        cfg.profile_version = data["profile_version"]

    logger.info("CONFIG: Geladen aus %s (Profil: %s v%s)",
                path, cfg.profile_name, cfg.profile_version)
    return cfg


def save_config(cfg: BridgeConfig, path: str):
    """Speichert Konfiguration als JSON."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w") as f:
        f.write(cfg.to_json())
    logger.info("Config gespeichert: %s", path)
