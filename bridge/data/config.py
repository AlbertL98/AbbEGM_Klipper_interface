# config.py — Zentrale Konfiguration für den EGM-Bridge
#
# Alle Konfigurationsklassen sind dataclasses mit sinnvollen Defaults.
# Die Bridge lädt config.yaml via BridgeConfig.from_yaml().
#
# Abschnitte:
#   EgmConfig              — Netzwerk (Robot-IP, Ports, Timeout)
#   PlannerConfig          — Trajektorien-Buffer, Starvation-Hold, Hz
#   SyncConfig             — Sync-Schwellenwerte (WARN/DEGRADE/STOP)
#   LatencyEstimatorConfig — Closed-Loop Offset-Estimator (adaptiv)
#   WorkspaceEnvelopeConfig— Workspace-Grenzwerte (X/Y/Z in mm)
#   TelemetryConfig        — Log-Verzeichnis, Flush-Intervall
#   BridgeConfig           — Dachkonfiguration + from_yaml()

from __future__ import annotations
import os
import yaml
import logging
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger("egm.config")


# ═══════════════════════════════════════════════════════════════════════
# Einzelne Konfigurations-Abschnitte
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class EgmConfig:
    """EGM-Netzwerk-Konfiguration."""
    robot_ip: str = "192.168.125.1"
    egm_port: int = 6510
    feedback_port: int = 6511
    timeout_ms: int = 500
    max_send_rate_hz: float = 250.0

    def validate(self):
        if not self.robot_ip:
            raise ValueError("EgmConfig: robot_ip darf nicht leer sein")
        if not (1 <= self.egm_port <= 65535):
            raise ValueError(f"EgmConfig: egm_port ungültig: {self.egm_port}")
        if not (1 <= self.feedback_port <= 65535):
            raise ValueError(f"EgmConfig: feedback_port ungültig: {self.feedback_port}")
        if self.egm_port == self.feedback_port:
            raise ValueError("EgmConfig: egm_port und feedback_port dürfen nicht gleich sein")


@dataclass
class PlannerConfig:
    """Trajektorien-Planner-Konfiguration."""
    target_hz: float = 50.0
    buffer_depth: int = 60
    starvation_hold_ms: float = 200.0
    segment_lookahead: int = 3
    interpolation_mode: str = "linear"   # "linear" | "cubic"
    velocity_smoothing: float = 0.0      # 0 = aus, 0..1 = EMA

    def validate(self):
        if self.target_hz <= 0:
            raise ValueError(f"PlannerConfig: target_hz muss > 0 sein: {self.target_hz}")
        if self.buffer_depth < 1:
            raise ValueError(f"PlannerConfig: buffer_depth muss >= 1 sein: {self.buffer_depth}")
        if self.interpolation_mode not in ("linear", "cubic"):
            raise ValueError(f"PlannerConfig: unbekannter interpolation_mode: {self.interpolation_mode}")


@dataclass
class SyncConfig:
    """Schwellenwerte für die Synchronisations-Überwachung."""
    # Positionsfehler
    warn_threshold_mm: float = 5.0
    degrade_threshold_mm: float = 15.0
    stop_threshold_mm: float = 50.0

    # Offset-Schwellenwerte
    offset_warn_ms: float = 350.0
    offset_degrade_ms: float = 500.0
    offset_stop_ms: float = 800.0
    offset_rate_warn_ms_per_s: float = 200.0

    # Normalfehler (Abweichung quer zur Fahrtrichtung)
    norm_error_warn_mm: float = 5.0
    norm_error_degrade_mm: float = 15.0

    # Bestätigungs-Zyklen vor Level-Wechsel
    warn_confirm_cycles: int = 5
    degrade_confirm_cycles: int = 3
    stop_confirm_cycles: int = 1

    # Telemetrie-Logging
    log_every_n_cycles: int = 1

    def validate(self):
        if self.warn_threshold_mm >= self.degrade_threshold_mm:
            raise ValueError("SyncConfig: warn_threshold muss < degrade_threshold sein")
        if self.degrade_threshold_mm >= self.stop_threshold_mm:
            raise ValueError("SyncConfig: degrade_threshold muss < stop_threshold sein")
        if self.offset_warn_ms >= self.offset_degrade_ms:
            raise ValueError("SyncConfig: offset_warn muss < offset_degrade sein")


@dataclass
class LatencyEstimatorConfig:
    """Konfiguration für den adaptiven Closed-Loop Latenz-Estimator."""
    enabled: bool = True

    # Initialer Offset (ms) — Startwert vor dem ersten Feedback
    t_delay_init_ms: float = 100.0

    # EMA-Raten
    # ema_slow: langsame Rate für stabile Geraden
    # ema_fast: schnelle Rate bei Beschleunigungsphasen
    ema_slow: float = 0.008
    ema_fast: float = 0.04
    # ema_output: Glättung des Ausgabe-Lookahead (für Planner)
    ema_output: float = 0.1

    # Gewichtung: Tangential- vs. Normalfehler (0 = nur tangential)
    alpha_weight: float = 0.3

    # Offset zurück von t_rx (Kompensation für Verarbeitungslatenz)
    offset_back_ms: float = 20.0

    # TX-Ringbuffer-Größe (Anzahl Samples)
    tx_buffer_size: int = 500

    def validate(self):
        if self.t_delay_init_ms <= 0:
            raise ValueError("LatencyEstimatorConfig: t_delay_init_ms muss > 0 sein")
        if self.ema_slow <= 0 or self.ema_fast <= 0:
            raise ValueError("LatencyEstimatorConfig: ema_slow/ema_fast muessen > 0 sein")
        if self.tx_buffer_size < 10:
            raise ValueError("LatencyEstimatorConfig: tx_buffer_size muss >= 10 sein")


@dataclass
class WorkspaceEnvelopeConfig:
    """
    Workspace-Grenzen in mm (Roboter-Koordinatensystem).
    Die Bridge stoppt den Job wenn die geplante Trajektorie die
    Grenzen verlässt.

    Typische Werte für ABB IRB 1100:
      X: -600..+600 mm
      Y: -600..+600 mm
      Z:    0..+800 mm
    """
    x_min: float = -600.0
    x_max: float = 600.0
    y_min: float = -600.0
    y_max: float = 600.0
    z_min: float = 0.0
    z_max: float = 800.0
    enabled: bool = True

    def validate(self):
        if self.x_min >= self.x_max:
            raise ValueError(f"WorkspaceEnvelopeConfig: x_min >= x_max ({self.x_min} >= {self.x_max})")
        if self.y_min >= self.y_max:
            raise ValueError(f"WorkspaceEnvelopeConfig: y_min >= y_max ({self.y_min} >= {self.y_max})")
        if self.z_min >= self.z_max:
            raise ValueError(f"WorkspaceEnvelopeConfig: z_min >= z_max ({self.z_min} >= {self.z_max})")

        vol = (
            (self.x_max - self.x_min) *
            (self.y_max - self.y_min) *
            (self.z_max - self.z_min)
        )

        # FIX: Schwellenwert und Kommentar korrigiert.
        #   Vorher: if vol > 1e9:  # > 1000mm³ pro Achse
        #   1e9 mm³ = 1 km³ — weit außerhalb jedes sinnvollen Roboter-Workspace.
        #   Korrekt: 1e6 mm³ = 1m³ (entspricht ~100cm × 100cm × 100cm),
        #   das ist bereits größer als der gesamte Arbeitsraum des IRB 1100.
        if vol > 2e6:  # > 2 m³ (2'000'000 mm³) — Warnung bei sehr großem Workspace
            logger.warning(
                "WorkspaceEnvelope: Volumen %.0f mm³ (%.2f m³) erscheint unrealistisch groß",
                vol, vol / 1e9
            )

    def contains(self, x: float, y: float, z: float) -> bool:
        """Prüft ob eine Position innerhalb des Workspace liegt."""
        if not self.enabled:
            return True
        return (
            self.x_min <= x <= self.x_max and
            self.y_min <= y <= self.y_max and
            self.z_min <= z <= self.z_max
        )

    def clamp(self, x: float, y: float, z: float) -> tuple:
        """Klemmt eine Position an die Workspace-Grenzen."""
        return (
            max(self.x_min, min(self.x_max, x)),
            max(self.y_min, min(self.y_max, y)),
            max(self.z_min, min(self.z_max, z)),
        )


@dataclass
class TelemetryConfig:
    """Telemetrie-Konfiguration."""
    enabled: bool = True
    log_dir: str = "logs"
    flush_interval_s: float = 1.0
    estimator_log_every_n: int = 1   # 1 = jeden RX-Packet loggen (~250Hz)
    sync_log_every_n: int = 1        # 1 = jeden update()-Zyklus loggen (~50Hz)

    def validate(self):
        if self.enabled and not self.log_dir:
            raise ValueError("TelemetryConfig: log_dir darf nicht leer sein wenn enabled=True")


# ═══════════════════════════════════════════════════════════════════════
# Dach-Konfiguration
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class BridgeConfig:
    """Dach-Konfiguration für die gesamte Bridge."""
    egm: EgmConfig = field(default_factory=EgmConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    sync: SyncConfig = field(default_factory=SyncConfig)
    estimator: LatencyEstimatorConfig = field(default_factory=LatencyEstimatorConfig)
    workspace: WorkspaceEnvelopeConfig = field(default_factory=WorkspaceEnvelopeConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)

    # Bridge-weite Einstellungen
    klipper_host: str = "localhost"
    klipper_port: int = 7200
    api_port: int = 7201
    log_level: str = "INFO"
    profile_name: str = "default"
    profile_version: str = "0.0.0"

    def validate(self):
        """Validiert alle Unter-Configs. Wirft ValueError bei Fehler."""
        self.egm.validate()
        self.planner.validate()
        self.sync.validate()
        self.estimator.validate()
        self.workspace.validate()
        self.telemetry.validate()

    @classmethod
    def from_yaml(cls, path: str) -> "BridgeConfig":
        """Lädt Konfiguration aus einer YAML-Datei."""
        if not os.path.exists(path):
            raise FileNotFoundError(f"Config-Datei nicht gefunden: {path}")

        with open(path, "r") as f:
            raw = yaml.safe_load(f) or {}

        cfg = cls()

        # Direkte Bridge-Parameter
        for key in ("klipper_host", "klipper_port", "api_port", "log_level", "profile_name", "profile_version"):
            if key in raw:
                setattr(cfg, key, raw[key])

        # Unter-Configs
        if "egm" in raw:
            cfg.egm = _load_dataclass(EgmConfig, raw["egm"])
        if "planner" in raw:
            cfg.planner = _load_dataclass(PlannerConfig, raw["planner"])
        if "sync" in raw:
            cfg.sync = _load_dataclass(SyncConfig, raw["sync"])
        if "estimator" in raw:
            cfg.estimator = _load_dataclass(LatencyEstimatorConfig, raw["estimator"])
        if "workspace" in raw:
            cfg.workspace = _load_dataclass(WorkspaceEnvelopeConfig, raw["workspace"])
        if "telemetry" in raw:
            cfg.telemetry = _load_dataclass(TelemetryConfig, raw["telemetry"])

        cfg.validate()
        logger.info("CONFIG: Geladen aus %s", path)
        return cfg

    def to_dict(self) -> dict:
        """Serialisiert Config für Config-Snapshot (Telemetrie §B6)."""
        import dataclasses
        return dataclasses.asdict(self)


def save_config(cfg: "BridgeConfig", path: str) -> None:
    """Speichert eine BridgeConfig als YAML-Datei."""
    import dataclasses
    with open(path, "w") as f:
        yaml.dump(dataclasses.asdict(cfg), f, default_flow_style=False, allow_unicode=True)
    logger.info("CONFIG: Gespeichert nach %s", path)


def load_config(path: str) -> "BridgeConfig":
    """Standalone-Wrapper für BridgeConfig.from_yaml().

    Für Rückwärtskompatibilität mit Imports der Form:
        from .config import load_config
    """
    return BridgeConfig.from_yaml(path)


def validate_config(cfg: "BridgeConfig") -> None:
    """Standalone-Wrapper für BridgeConfig.validate().

    Für Rückwärtskompatibilität mit Imports der Form:
        from .config import BridgeConfig, validate_config
    """
    cfg.validate()


def _load_dataclass(cls, data: dict):
    """Lädt ein Dataclass aus einem Dict, ignoriert unbekannte Keys."""
    import dataclasses
    known = {f.name for f in dataclasses.fields(cls)}
    filtered = {k: v for k, v in data.items() if k in known}
    return cls(**filtered)
