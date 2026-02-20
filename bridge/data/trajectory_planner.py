from __future__ import annotations
# trajectory_planner.py — Plan-Queue und Sample-Interpolation
#
# CodingPlan §B2: Plan-Queue für zeitparametrierte Solltrajektorie
# CodingPlan §B4: Rolling-Horizon-Korrektur
#
# Kernaufgabe:
#   Trapez-Segmente (von Klipper) → zeitdiskrete Samples (für EGM)
#
# Jeder EGM-Zyklus (z.B. 4ms) braucht eine Sollposition.
# Der Planner interpoliert innerhalb des aktuellen Segments
# und wechselt automatisch zum nächsten.

import math
import time
import logging
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Optional

from .segment_source import TrapezSegment

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


@dataclass
class CorrectionState:
    """Rolling-Horizon-Korrektur (§B4)."""
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    rate_x: float = 0.0     # Aktuelle Korrekturrate (mm/s)
    rate_y: float = 0.0
    rate_z: float = 0.0
    applied_at: float = 0.0


class TrajectoryPlanner:
    """
    Verwaltet die Plan-Queue und erzeugt EGM-Samples.

    Arbeitsweise:
    1. Segmente werden von außen via add_segment() eingefügt
    2. next_sample() wird im EGM-Takt aufgerufen
    3. Der Planner interpoliert innerhalb des aktuellen Segments
    4. Wenn ein Segment abgearbeitet ist, wechselt er zum nächsten
    5. Korrekturen werden per apply_correction() eingebracht
       und nur auf zukünftige Samples angewandt (§B4)
    """

    def __init__(self, cycle_s: float,
                 max_queue_size: int = 2000,
                 correction_max_mm: float = 3.0,
                 correction_rate_limit: float = 10.0):
        self.cycle_s = cycle_s
        self.max_queue_size = max_queue_size
        self.correction_max_mm = correction_max_mm
        self.correction_rate_limit = correction_rate_limit  # mm/s

        # Plan-Queue: FIFO von Segmenten
        self._queue: deque[TrapezSegment] = deque(maxlen=max_queue_size)
        self._lock = threading.Lock()

        # Aktueller Zustand
        self._current_segment: Optional[TrapezSegment] = None
        self._segment_elapsed: float = 0.0  # Zeit im aktuellen Segment
        self._sequence_id: int = 0
        self._started = False

        # Korrektur (§B4)
        self._correction = CorrectionState()
        self._target_correction = CorrectionState()

        # Statistik
        self._segments_consumed: int = 0
        self._samples_generated: int = 0
        self._underflows: int = 0

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
        """Geschätzte Zeit in der Queue (Summe aller Segment-Dauern)."""
        with self._lock:
            total = sum(s.duration for s in self._queue)
            if self._current_segment:
                total += self._current_segment.duration - self._segment_elapsed
            return total

    @property
    def has_data(self) -> bool:
        return self._current_segment is not None or len(self._queue) > 0

    @property
    def is_starved(self) -> bool:
        """Queue leer und kein aktives Segment."""
        return self._current_segment is None and len(self._queue) == 0

    # ── Sample-Erzeugung ─────────────────────────────────────

    def next_sample(self, bridge_time: float) -> Optional[EgmSample]:
        """
        Erzeugt den nächsten EGM-Sollwert.

        Aufgerufen im EGM-Zyklustakt. Gibt None zurück wenn
        keine Daten verfügbar (Underflow).
        """
        # Neues Segment laden wenn nötig
        if self._current_segment is None:
            if not self._advance_segment():
                self._underflows += 1
                return None

        seg = self._current_segment

        # Position im aktuellen Segment interpolieren
        x, y, z = seg.position_at(self._segment_elapsed)
        velocity = seg.velocity_at(self._segment_elapsed)
        progress = (self._segment_elapsed / seg.duration
                    if seg.duration > 0 else 1.0)

        # Korrektur anwenden (nur auf Soll, nicht rückwirkend)
        cx, cy, cz = self._get_smoothed_correction()
        x += cx
        y += cy
        z += cz

        self._sequence_id += 1
        self._samples_generated += 1

        sample = EgmSample(
            timestamp=bridge_time,
            x=x, y=y, z=z,
            velocity=velocity,
            segment_nr=seg.nr,
            segment_progress=progress,
            sequence_id=self._sequence_id,
        )

        # Zeit im Segment voranschreiten
        self._segment_elapsed += self.cycle_s

        # Segment abgearbeitet?
        if self._segment_elapsed >= seg.duration:
            overshoot = self._segment_elapsed - seg.duration
            self._advance_segment()
            # Überschuss auf nächstes Segment übertragen
            if self._current_segment and overshoot > 0:
                self._segment_elapsed = overshoot

        return sample

    def _advance_segment(self) -> bool:
        """Nächstes Segment aus der Queue laden."""
        with self._lock:
            if not self._queue:
                self._current_segment = None
                self._segment_elapsed = 0.0
                return False
            self._current_segment = self._queue.popleft()

        self._segment_elapsed = 0.0
        self._segments_consumed += 1
        return True

    def peek_ahead(self, n: int = 5) -> list[TrapezSegment]:
        """Die nächsten n Segmente ansehen (ohne zu konsumieren)."""
        with self._lock:
            return list(self._queue)[:n]

    # ── Rolling-Horizon-Korrektur (§B4) ──────────────────────

    def apply_correction(self, dx: float, dy: float, dz: float):
        """
        Setzt neue Ziel-Korrektur.

        WICHTIG (§B4):
        - Korrektur wird NUR auf zukünftige Samples angewandt
        - Wird geglättet/rate-limited, nicht sprunghaft
        - Begrenzt auf correction_max_mm
        """
        # Clamp auf Maximum
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag > self.correction_max_mm:
            scale = self.correction_max_mm / mag
            dx *= scale
            dy *= scale
            dz *= scale
            logger.warning("PLANNER: Korrektur geclampt: %.2f mm → %.2f mm",
                           mag, self.correction_max_mm)

        self._target_correction = CorrectionState(
            offset_x=dx, offset_y=dy, offset_z=dz,
            applied_at=time.monotonic()
        )

    def _get_smoothed_correction(self) -> tuple[float, float, float]:
        """
        Interpoliert sanft zur Ziel-Korrektur.
        Rate-Limited per cycle (§B4: max Änderung pro Zyklus).
        """
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
        """Korrektur zurücksetzen."""
        self._correction = CorrectionState()
        self._target_correction = CorrectionState()

    # ── Reset ────────────────────────────────────────────────

    def clear(self):
        """Queue leeren und Zustand zurücksetzen."""
        with self._lock:
            self._queue.clear()
        self._current_segment = None
        self._segment_elapsed = 0.0
        self.reset_correction()
        logger.info("PLANNER: Queue geleert")

    # ── Status ───────────────────────────────────────────────

    def snapshot(self) -> dict:
        seg = self._current_segment
        return {
            "queue_depth": len(self._queue),
            "queue_time_s": round(self.queue_time_s, 3),
            "current_segment": seg.nr if seg else None,
            "segment_elapsed": round(self._segment_elapsed, 4),
            "segments_consumed": self._segments_consumed,
            "samples_generated": self._samples_generated,
            "underflows": self._underflows,
            "correction": {
                "x": round(self._correction.offset_x, 3),
                "y": round(self._correction.offset_y, 3),
                "z": round(self._correction.offset_z, 3),
            },
        }
