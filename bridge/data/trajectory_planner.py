from __future__ import annotations
# trajectory_planner.py — Plan-Queue und Sample-Interpolation
#
# CLOCK-FIX: Nutzt bridge_now() statt time.monotonic()
# ESTIMATOR: klipper_time_now() nutzt adaptiven Lookahead aus
#   LatencyEstimator statt fixem offset_s (wenn Estimator gesetzt).

import math
import logging
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Optional, TYPE_CHECKING

from .clock import bridge_now
from .segment_source import TrapezSegment

if TYPE_CHECKING:
    from .latency_estimator import LatencyEstimator

logger = logging.getLogger("egm.planner")


@dataclass
class EgmSample:
    """Ein einzelner Sollwert für einen EGM-Zyklus."""
    timestamp: float        # Monotone Bridge-Zeit (s)
    x: float
    y: float
    z: float
    velocity: float         # Skalare Geschwindigkeit (mm/s)
    segment_nr: int         # Zugehöriges Segment
    segment_progress: float  # 0.0 – 1.0 innerhalb des Segments
    sequence_id: int        # Laufende Nummer für EGM
    t_klipper: float = 0.0  # Klipper-Zeitlinie (für Telemetrie/Debug)


@dataclass
class CorrectionState:
    """Rolling-Horizon-Korrektur (§B4)."""
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    rate_x: float = 0.0
    rate_y: float = 0.0
    rate_z: float = 0.0
    applied_at: float = 0.0


class TrajectoryPlanner:
    """
    Verwaltet die Plan-Queue und erzeugt EGM-Samples.

    TIME-INDEXED PLAYBACK:
      t_klipper_now = (bridge_time - t0_bridge) + t0_klipper + offset_s
    """

    def __init__(self, cycle_s: float,
                 max_queue_size: int = 2000,
                 correction_max_mm: float = 3.0,
                 correction_rate_limit: float = 10.0,
                 offset_s: float = 0.0,
                 estimator: Optional["LatencyEstimator"] = None):
        self.cycle_s = cycle_s
        self.max_queue_size = max_queue_size
        self.correction_max_mm = correction_max_mm
        self.correction_rate_limit = correction_rate_limit
        self.offset_s = offset_s        # Fallback wenn Estimator disabled/None
        self._estimator = estimator     # Adaptiver Latenz-Estimator (optional)

        self._queue: deque[TrapezSegment] = deque(maxlen=max_queue_size)
        self._lock = threading.Lock()

        self._t0_bridge: float = 0.0
        self._t0_klipper: float = 0.0
        self._time_synced: bool = False

        self._current_segment: Optional[TrapezSegment] = None
        self._sequence_id: int = 0

        self._last_position: Optional[tuple[float, float, float]] = None
        self._last_velocity: float = 0.0
        self._last_segment_nr: int = 0

        self._correction = CorrectionState()
        self._target_correction = CorrectionState()

        self._segments_consumed: int = 0
        self._samples_generated: int = 0
        self._underflows: int = 0
        self._gap_holds: int = 0

    # ── Zeitkopplung ─────────────────────────────────────────

    def init_time_sync(self, bridge_time: float, klipper_time: float):
        """Koppelt die Bridge-Zeitlinie mit Klippers print_time."""
        self._t0_bridge = bridge_time
        self._t0_klipper = klipper_time
        self._time_synced = True
        logger.info("PLANNER: Zeitkopplung hergestellt — "
                     "t0_bridge=%.3f t0_klipper=%.3f offset=%.3fs",
                     bridge_time, klipper_time, self.offset_s)

    def klipper_time_now(self, bridge_time: float) -> float:
        """
        Berechnet die aktuelle Klipper-Zeit aus der Bridge-Zeit.

        Lookahead-Quelle:
          - Estimator aktiv + enabled: LatencyEstimator.get_lookahead()
            (adaptiv, lernt tatsächliche Controller-Latenz)
          - Sonst: fixer offset_s aus Config (SyncConfig.time_offset_ms)
        """
        if (self._estimator is not None
                and self._estimator.enabled):
            lookahead = self._estimator.get_lookahead()
        else:
            lookahead = self.offset_s
        return (bridge_time - self._t0_bridge) + self._t0_klipper + lookahead

    @property
    def time_synced(self) -> bool:
        return self._time_synced

    # ── Queue-Management ─────────────────────────────────────

    def add_segment(self, seg: TrapezSegment):
        """Segment zur Plan-Queue hinzufügen (thread-safe)."""
        with self._lock:
            if len(self._queue) >= self.max_queue_size:
                logger.warning("PLANNER: Queue voll (%d), Segment #%d "
                               "verworfen", self.max_queue_size, seg.nr)
                return False
            self._queue.append(seg)
        return True

    @property
    def queue_depth(self) -> int:
        return len(self._queue)

    @property
    def queue_time_s(self) -> float:
        with self._lock:
            total = sum(s.duration for s in self._queue)
            if self._current_segment:
                total += self._current_segment.duration
            return total

    @property
    def has_data(self) -> bool:
        return self._current_segment is not None or len(self._queue) > 0

    @property
    def is_starved(self) -> bool:
        return self._current_segment is None and len(self._queue) == 0

    # ── Sample-Erzeugung (TIME-INDEXED) ──────────────────────

    def next_sample(self, bridge_time: float) -> Optional[EgmSample]:
        """Erzeugt den nächsten EGM-Sollwert."""
        if not self._time_synced:
            self._underflows += 1
            return None

        t_klipper_now = self.klipper_time_now(bridge_time)
        seg = self._find_segment_for_time(t_klipper_now)

        if seg is None:
            self._underflows += 1
            if self._last_position is not None:
                self._gap_holds += 1
                self._sequence_id += 1
                return EgmSample(
                    timestamp=bridge_time,
                    x=self._last_position[0],
                    y=self._last_position[1],
                    z=self._last_position[2],
                    velocity=0.0,
                    segment_nr=self._last_segment_nr,
                    segment_progress=1.0,
                    sequence_id=self._sequence_id,
                    t_klipper=t_klipper_now,
                )
            return None

        local_t = t_klipper_now - seg.print_time
        x, y, z = seg.position_at(local_t)
        velocity = seg.velocity_at(local_t)
        progress = local_t / seg.duration if seg.duration > 0 else 1.0

        cx, cy, cz = self._get_smoothed_correction()
        x += cx
        y += cy
        z += cz

        self._sequence_id += 1
        self._samples_generated += 1

        self._last_position = (x, y, z)
        self._last_velocity = velocity
        self._last_segment_nr = seg.nr

        return EgmSample(
            timestamp=bridge_time,
            x=x, y=y, z=z,
            velocity=velocity,
            segment_nr=seg.nr,
            segment_progress=progress,
            sequence_id=self._sequence_id,
            t_klipper=t_klipper_now,
        )

    def _find_segment_for_time(
            self, t_klipper: float) -> Optional[TrapezSegment]:
        """Findet das Segment das zum Zeitpunkt t_klipper aktiv ist."""
        if self._current_segment is not None:
            seg = self._current_segment
            seg_end = seg.print_time + seg.duration
            if seg.print_time <= t_klipper < seg_end:
                return seg
            if t_klipper >= seg_end:
                end_x, end_y, end_z = seg.position_at(seg.duration)
                self._last_position = (end_x, end_y, end_z)
                self._last_velocity = seg.velocity_at(seg.duration)
                self._last_segment_nr = seg.nr
                self._current_segment = None
                self._segments_consumed += 1

        with self._lock:
            while self._queue:
                candidate = self._queue[0]
                cand_end = candidate.print_time + candidate.duration
                if t_klipper >= cand_end:
                    discarded = self._queue.popleft()
                    self._segments_consumed += 1
                    end_x, end_y, end_z = discarded.position_at(
                        discarded.duration)
                    self._last_position = (end_x, end_y, end_z)
                    self._last_velocity = discarded.velocity_at(
                        discarded.duration)
                    self._last_segment_nr = discarded.nr
                    continue
                if candidate.print_time <= t_klipper < cand_end:
                    self._current_segment = self._queue.popleft()
                    return self._current_segment
                if t_klipper < candidate.print_time:
                    if self._gap_holds % 500 == 0 and self._gap_holds > 0:
                        logger.debug(
                            "PLANNER: Zeitlücke — t_klipper=%.3f, "
                            "nächstes Segment bei %.3f (Δ%.3fs)",
                            t_klipper, candidate.print_time,
                            candidate.print_time - t_klipper)
                    return None
                break
        return None

    def peek_ahead(self, n: int = 5) -> list[TrapezSegment]:
        with self._lock:
            return list(self._queue)[:n]

    # ── Rolling-Horizon-Korrektur (§B4) ──────────────────────

    def apply_correction(self, dx: float, dy: float, dz: float):
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag > self.correction_max_mm:
            scale = self.correction_max_mm / mag
            dx *= scale
            dy *= scale
            dz *= scale
            logger.warning("PLANNER: Korrektur geclampt: "
                           "%.2f mm → %.2f mm", mag,
                           self.correction_max_mm)
        self._target_correction = CorrectionState(
            offset_x=dx, offset_y=dy, offset_z=dz,
            applied_at=bridge_now()
        )

    def _get_smoothed_correction(self) -> tuple[float, float, float]:
        max_step = self.correction_rate_limit * self.cycle_s
        target = self._target_correction
        current = self._correction

        def step_towards(current_val, target_val, max_delta):
            diff = target_val - current_val
            if abs(diff) <= max_delta:
                return target_val
            return current_val + math.copysign(max_delta, diff)

        self._correction.offset_x = step_towards(
            current.offset_x, target.offset_x, max_step)
        self._correction.offset_y = step_towards(
            current.offset_y, target.offset_y, max_step)
        self._correction.offset_z = step_towards(
            current.offset_z, target.offset_z, max_step)

        return (self._correction.offset_x,
                self._correction.offset_y,
                self._correction.offset_z)

    def reset_correction(self):
        self._correction = CorrectionState()
        self._target_correction = CorrectionState()

    # ── Reset ────────────────────────────────────────────────

    def clear(self):
        with self._lock:
            self._queue.clear()
        self._current_segment = None
        self._time_synced = False
        self._last_position = None
        self._last_velocity = 0.0
        self._last_segment_nr = 0
        self._gap_holds = 0
        self.reset_correction()
        logger.info("PLANNER: Queue geleert, Zeitkopplung zurückgesetzt")

    # ── Status ───────────────────────────────────────────────

    def snapshot(self) -> dict:
        seg = self._current_segment
        est = self._estimator
        return {
            "queue_depth": len(self._queue),
            "queue_time_s": round(self.queue_time_s, 3),
            "current_segment": seg.nr if seg else None,
            "time_synced": self._time_synced,
            "offset_s": self.offset_s,
            "lookahead_ms": round(
                (est.get_lookahead() if est and est.enabled
                 else self.offset_s) * 1000.0, 2),
            "estimator_active": est is not None and est.enabled,
            "t0_bridge": (round(self._t0_bridge, 3)
                          if self._time_synced else None),
            "t0_klipper": (round(self._t0_klipper, 3)
                           if self._time_synced else None),
            "segments_consumed": self._segments_consumed,
            "samples_generated": self._samples_generated,
            "underflows": self._underflows,
            "gap_holds": self._gap_holds,
            "correction": {
                "x": round(self._correction.offset_x, 3),
                "y": round(self._correction.offset_y, 3),
                "z": round(self._correction.offset_z, 3),
            },
        }
