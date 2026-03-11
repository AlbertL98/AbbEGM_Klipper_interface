# egm_bridge — EGM-Bridge-Core für Klipper↔ABB Integration
#
# Hauptkomponenten:
#   EgmBridge           — Orchestrator
#   BridgeStateMachine  — Zustandsmaschine
#   BridgeConfig        — Konfiguration
#   TrajectoryPlanner   — Segment-Interpolation
#   EgmClient           — UDP/EGM-Kommunikation
#   SyncMonitor         — Synchronisationsüberwachung
#   TelemetryWriter     — Logging
#   KlipperCommandClient — Heartbeat + Stop/Pause an Klipper
#   bridge_now          — Einheitliche Zeitquelle

from .clock import bridge_now
from .bridge import EgmBridge
from .state_machine import BridgeStateMachine, State
from .config import (BridgeConfig, WorkspaceEnvelopeConfig,
                     load_config, save_config, validate_config)
from .trajectory_planner import TrajectoryPlanner
from .egm_client import EgmClient
from .sync_monitor import SyncMonitor, SyncLevel
from .telemetry import TelemetryWriter
from .segment_source import (TrapezSegment, TcpSegmentReceiver,
                             CsvSegmentSource, SegmentValidationError)
from .klipper_command import KlipperCommandClient

__all__ = [
    "bridge_now",
    "EgmBridge",
    "BridgeStateMachine", "State",
    "BridgeConfig", "WorkspaceEnvelopeConfig",
    "load_config", "save_config", "validate_config",
    "TrajectoryPlanner",
    "EgmClient",
    "SyncMonitor", "SyncLevel",
    "TelemetryWriter",
    "TrapezSegment", "TcpSegmentReceiver", "CsvSegmentSource",
    "SegmentValidationError",
    "KlipperCommandClient",
]

__version__ = "0.3.0"
