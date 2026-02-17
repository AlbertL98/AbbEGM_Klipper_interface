from __future__ import annotations
# config.py — Konfiguration und Parameter-Profile
#
# CodingPlan §B6: Extern konfigurierbare Parameterprofile
#   - EGM-Regelparameter/Filter/Gains
#   - Pfad-/Orientierungstoleranzen
#   - Max speed/accel/Abweichung
#   - Timeout-/Watchdog-Grenzen
#   - Plausibilitätsprüfung vor Jobstart (Range Checks)

import copy
import json
import time
import logging
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

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
class QueueConfig:
    """Buffer- und Queue-Parameter (§B2)."""
    plan_queue_size: int = 2000      # Max Segmente in der Plan-Queue
    send_queue_lookahead_ms: float = 200.0  # Vorlaufzeit in ms
    feedback_queue_size: int = 500
    low_watermark: int = 20          # Segmente — darunter DEGRADED
    high_watermark: int = 1800       # Segmente — darüber throttle
    underflow_action: str = "degrade"  # "degrade" | "stop"


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
    queues: QueueConfig = field(default_factory=QueueConfig)
    sync: SyncConfig = field(default_factory=SyncConfig)
    klipper: KlipperSourceConfig = field(default_factory=KlipperSourceConfig)
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
            "timestamp": time.time(),
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

    # Queues
    q = cfg.queues
    if q.low_watermark >= q.high_watermark:
        errors.append("low_watermark >= high_watermark")
    if q.send_queue_lookahead_ms < c.cycle_ms * 5:
        errors.append("lookahead zu klein (< 5 Zyklen)")

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
        ("queues", QueueConfig),
        ("sync", SyncConfig),
        ("klipper", KlipperSourceConfig),
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
