# tests/test_core.py — Unit-Tests: TrajectoryPlanner, SyncMonitor, Buffer
#
# FIX: SyncMonitor braucht zwei Argumente: SyncConfig + LatencyEstimatorConfig.
#   Vorher: mon = SyncMonitor(cfg)  → TypeError beim Ausführen
#   Jetzt:  mon = SyncMonitor(cfg, est_cfg)

import pytest
import sys
from pathlib import Path

# Projekt imports
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from data.sync_monitor import SyncMonitor, SyncLevel
from data.trajectory_planner import TrajectoryPlanner, EgmSample
from data.config import (
    SyncConfig,
    LatencyEstimatorConfig,
    PlannerConfig,
    WorkspaceEnvelopeConfig,
)


# ═══════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture
def sync_cfg():
    return SyncConfig(
        warn_threshold_mm=5.0,
        degrade_threshold_mm=15.0,
        stop_threshold_mm=50.0,
        warn_confirm_cycles=5,
        degrade_confirm_cycles=3,
        stop_confirm_cycles=1,
        offset_warn_ms=350.0,
        offset_degrade_ms=500.0,
        offset_stop_ms=800.0,
        offset_rate_warn_ms_per_s=200.0,
    )


@pytest.fixture
def est_cfg():
    return LatencyEstimatorConfig(
        enabled=True,
        t_delay_init_ms=100.0,
        ema_slow=0.01,
        ema_fast=0.05,
        ema_output=0.1,
        alpha_weight=0.3,
        offset_back_ms=20.0,
        tx_buffer_size=500,
    )


@pytest.fixture
def planner_cfg():
    return PlannerConfig(
        target_hz=50.0,
        buffer_depth=60,
        starvation_hold_ms=200.0,
    )


# ═══════════════════════════════════════════════════════════════════════
# SyncMonitor Tests
# ═══════════════════════════════════════════════════════════════════════

class TestSyncMonitor:

    def test_instantiation(self, sync_cfg, est_cfg):
        """SyncMonitor lässt sich mit beiden Config-Objekten erstellen."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        assert mon is not None

    def test_initial_state(self, sync_cfg, est_cfg):
        """Frischer Monitor ist im OK-Level."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        assert mon.sync_level == SyncLevel.OK
        assert mon.is_ok

    def test_initial_lookahead(self, sync_cfg, est_cfg):
        """Initialer Lookahead entspricht t_delay_init_ms."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        lookahead = mon.get_lookahead()
        assert abs(lookahead - est_cfg.t_delay_init_ms / 1000.0) < 0.001

    def test_reset_clears_consecutive_counters(self, sync_cfg, est_cfg):
        """reset() setzt alle consecutive-Counter auf 0."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        # Direkt auf die privaten Counter zugreifen
        mon._consecutive_warn = 3
        mon._consecutive_degrade = 2
        mon._consecutive_stop = 1
        mon.reset()
        assert mon._consecutive_warn == 0
        assert mon._consecutive_degrade == 0
        assert mon._consecutive_stop == 0

    def test_reset_clears_metrics(self, sync_cfg, est_cfg):
        """reset() setzt Metriken zurück."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        mon._total_updates = 500
        mon.reset()
        assert mon._total_updates == 0
        assert mon.sync_level == SyncLevel.OK

    def test_warmup_period(self, sync_cfg, est_cfg):
        """Während Warmup bleibt Level OK, egal welche Metriken."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        mon._total_updates = 5  # weit unter warmup_cycles=100
        mon._metrics.error_tcp_pos = 999.0
        mon.update(None, None)
        assert mon.sync_level == SyncLevel.OK

    def test_record_sent_sample(self, sync_cfg, est_cfg):
        """record_sent_sample() befüllt den TX-Buffer."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        sample = _make_sample(0.0, 100.0, 200.0, 0.0)
        mon.record_sent_sample(sample)
        assert len(mon._tx_times) == 1

    def test_tx_buffer_max_size(self, sync_cfg, est_cfg):
        """TX-Buffer überschreitet nie _TX_BUFFER_SIZE."""
        mon = SyncMonitor(sync_cfg, est_cfg)
        for i in range(mon._TX_BUFFER_SIZE + 50):
            mon.record_sent_sample(_make_sample(float(i), 0, 0, 1.0))
        assert len(mon._tx_times) <= mon._TX_BUFFER_SIZE

    def test_reset_after_job_no_degrade_bleedover(self, sync_cfg, est_cfg):
        """
        Regression: Consecutive-Counter aus vorherigem Job bleiben nicht
        stehen und verursachen beim nächsten Job sofortiges DEGRADE.
        """
        mon = SyncMonitor(sync_cfg, est_cfg)
        # Simuliere: Job-Ende mit erhöhtem Degrade-Counter
        mon._consecutive_degrade = est_cfg.t_delay_init_ms  # irgendein Wert > 0
        mon.reset()
        # Nächster Job: ein einzelnes update() direkt nach Warmup
        mon._total_updates = 200
        mon._metrics.error_tcp_pos = 1.0  # unauffällig
        mon._metrics.t_delay_ms = 100.0   # im grünen Bereich
        mon.update(None, None)
        assert mon.sync_level != SyncLevel.DEGRADE


# ═══════════════════════════════════════════════════════════════════════
# TrajectoryPlanner Tests
# ═══════════════════════════════════════════════════════════════════════

class TestTrajectoryPlanner:

    def test_instantiation(self, planner_cfg):
        planner = TrajectoryPlanner(planner_cfg)
        assert planner is not None

    def test_empty_buffer_returns_none(self, planner_cfg):
        planner = TrajectoryPlanner(planner_cfg)
        result = planner.get_next_sample(elapsed=0.0, lookahead=0.1)
        assert result is None

    def test_sample_interpolation(self, planner_cfg):
        """Zwischen zwei Segmenten wird korrekt interpoliert."""
        from data.trajectory_planner import TrajectorySegment
        planner = TrajectoryPlanner(planner_cfg)
        seg = TrajectorySegment(
            nr=1,
            print_time=0.0,
            duration=1.0,
            start_x=0.0, start_y=0.0, start_z=0.0,
            end_x=100.0, end_y=0.0, end_z=0.0,
            start_v=100.0, cruise_v=100.0, end_v=100.0,
            distance=100.0,
        )
        planner.add_segment(seg)
        sample = planner.get_next_sample(elapsed=0.0, lookahead=0.5)
        assert sample is not None
        # Bei t=0.5 sollte x ≈ 50mm sein
        assert 40.0 < sample.x < 60.0


# ═══════════════════════════════════════════════════════════════════════
# Config-Tests
# ═══════════════════════════════════════════════════════════════════════

class TestConfig:

    def test_workspace_volume_positive(self):
        """WorkspaceEnvelopeConfig akzeptiert sinnvolle Werte."""
        cfg = WorkspaceEnvelopeConfig(
            x_min=-600, x_max=600,
            y_min=-600, y_max=600,
            z_min=0, z_max=800,
        )
        assert cfg.x_max > cfg.x_min
        assert cfg.y_max > cfg.y_min
        assert cfg.z_max > cfg.z_min

    def test_latency_estimator_config_defaults(self):
        """LatencyEstimatorConfig hat sinnvolle Default-Werte."""
        cfg = LatencyEstimatorConfig()
        assert 20 <= cfg.t_delay_init_ms <= 500
        assert 0 < cfg.ema_slow < cfg.ema_fast <= 1.0


# ═══════════════════════════════════════════════════════════════════════
# Hilfsfunktionen
# ═══════════════════════════════════════════════════════════════════════

def _make_sample(t: float, x: float, y: float, v: float) -> EgmSample:
    """Erstellt ein minimales EgmSample für Tests."""
    s = EgmSample.__new__(EgmSample)
    s.timestamp = t
    s.x = x
    s.y = y
    s.z = 0.0
    s.velocity = v
    s.sequence_id = 0
    s.segment_nr = 0
    s.segment_progress = 0.0
    s.t_klipper = t
    return s
