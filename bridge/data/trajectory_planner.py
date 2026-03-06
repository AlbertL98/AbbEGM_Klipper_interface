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
#
# TIME-INDEXED PLAYBACK (v2):
#   Statt FIFO + lokaler Elapsed-Time wird jetzt Klippers
#   print_time-Zeitlinie als Referenz genutzt:
#
#     t_klipper_now = (bridge_time - t0_bridge) + t0_klipper + offset_s
#
#   Damit ist die Ausführung zeitlich konsistent mit Klippers
#   Extruder-Steuerung. offset_s modelliert die gewünschte
#   Differenz zwischen Extruder (Klipper) und Roboter (ABB).

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
    t_klipper: float = 0.0  # Klipper-Zeitlinie (für Telemetrie/Debug)


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

    TIME-INDEXED PLAYBACK:
      Statt _segment_elapsed hochzuzählen wird in jedem Zyklus
      die Klipper-Zeit t_klipper_now berechnet und das passende
      Segment per print_time ausgewählt.

      t_klipper_now = (bridge_time - t0_bridge) + t0_klipper + offset_s

      t0_bridge / t0_klipper werden beim ersten Segment gekoppelt
      (via init_time_sync). offset_s ist konfigurierbar und
      modelliert die zeitliche Differenz Extruder ↔ Roboter.
    """

    def __init__(self, cycle_s: float,
                 max_queue_size: int = 2000,
                 correction_max_mm: float = 3.0,
                 correction_rate_limit: float = 10.0,
                 offset_s: float = 0.0):
        self.cycle_s = cycle_s
        self.max_queue_size = max_queue_size
        self.correction_max_mm = correction_max_mm
        self.correction_rate_limit = correction_rate_limit  # mm/s
        self.offset_s = offset_s  # Konfigurierbar: Roboter-Vorlauf/Nachlauf

        # Plan-Queue: FIFO von Segmenten
        self._queue: deque[TrapezSegment] = deque(maxlen=max_queue_size)
        self._lock = threading.Lock()

        # Zeitkopplung Klipper ↔ Bridge
        self._t0_bridge: float = 0.0       # bridge_time beim ersten Segment
        self._t0_klipper: float = 0.0      # print_time des ersten Segments
        self._time_synced: bool = False     # Wurde die Kopplung hergestellt?

        # Aktuelles Segment (Cache — das Segment in dem wir uns befinden)
        self._current_segment: Optional[TrapezSegment] = None
        self._sequence_id: int = 0

        # Letzte bekannte Position (für Hold bei Lücken)
        self._last_position: Optional[tuple[float, float, float]] = None
        self._last_velocity: float = 0.0
        self._last_segment_nr: int = 0

        # Korrektur (§B4)
        self._correction = CorrectionState()
        self._target_correction = CorrectionState()

        # Statistik
        self._segments_consumed: int = 0
        self._samples_generated: int = 0
        self._underflows: int = 0
        self._gap_holds: int = 0           # Zyklen in print_time-Lücken

    # ── Zeitkopplung ─────────────────────────────────────────

    def init_time_sync(self, bridge_time: float, klipper_time: float):
        """
        Koppelt die Bridge-Zeitlinie mit Klippers print_time.

        Wird typischerweise beim ersten empfangenen Segment
        aufgerufen: init_time_sync(time.monotonic(), seg.print_time)
        """
        self._t0_bridge = bridge_time
        self._t0_klipper = klipper_time
        self._time_synced = True
        logger.info("PLANNER: Zeitkopplung hergestellt — "
                     "t0_bridge=%.3f t0_klipper=%.3f offset=%.3fs",
                     bridge_time, klipper_time, self.offset_s)

    def klipper_time_now(self, bridge_time: float) -> float:
        """Berechnet die aktuelle Klipper-Zeit aus der Bridge-Zeit."""
        return (bridge_time - self._t0_bridge) + self._t0_klipper + self.offset_s

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
        """Geschätzte Zeit in der Queue (Summe aller Segment-Dauern)."""
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
        """Queue leer und kein aktives Segment."""
        return self._current_segment is None and len(self._queue) == 0

    # ── Sample-Erzeugung (TIME-INDEXED) ──────────────────────

    def next_sample(self, bridge_time: float) -> Optional[EgmSample]:
        """
        Erzeugt den nächsten EGM-Sollwert basierend auf der
        Klipper-Zeitlinie.

        Ablauf:
          1. t_klipper_now berechnen
          2. Vergangene Segmente verwerfen
          3. Passendes Segment finden (print_time ≤ t_klipper_now < print_time + duration)
          4. Lokale Zeit berechnen und interpolieren
          5. Korrektur anwenden

        Gibt None zurück wenn Zeitkopplung noch nicht hergestellt
        oder kein Segment verfügbar (Underflow).
        """
        if not self._time_synced:
            self._underflows += 1
            return None

        t_klipper_now = self.klipper_time_now(bridge_time)

        # Passendes Segment finden
        seg = self._find_segment_for_time(t_klipper_now)

        if seg is None:
            # Kein Segment für diesen Zeitpunkt
            self._underflows += 1

            # Letzte Position halten wenn möglich
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

        # Lokale Zeit innerhalb des Segments
        local_t = t_klipper_now - seg.print_time

        # Position interpolieren
        x, y, z = seg.position_at(local_t)
        velocity = seg.velocity_at(local_t)
        progress = local_t / seg.duration if seg.duration > 0 else 1.0

        # Korrektur anwenden (nur auf Soll, nicht rückwirkend)
        cx, cy, cz = self._get_smoothed_correction()
        x += cx
        y += cy
        z += cz

        self._sequence_id += 1
        self._samples_generated += 1

        # Letzte Position merken (für Hold bei Lücken)
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

    def _find_segment_for_time(self, t_klipper: float) -> Optional[TrapezSegment]:
        """
        Findet das Segment das zum Zeitpunkt t_klipper aktiv ist.

        Strategie:
          1. Prüfe ob _current_segment noch passt
          2. Verwerfe vergangene Segmente aus der Queue
          3. Lade nächstes passendes Segment

        Ein Segment ist aktiv wenn:
          seg.print_time ≤ t_klipper < seg.print_time + seg.duration
        """
        # 1. Prüfe aktuelles Segment
        if self._current_segment is not None:
            seg = self._current_segment
            seg_end = seg.print_time + seg.duration

            if seg.print_time <= t_klipper < seg_end:
                # Noch im aktuellen Segment
                return seg

            if t_klipper >= seg_end:
                # Segment abgelaufen — Position am Ende merken
                end_x, end_y, end_z = seg.position_at(seg.duration)
                self._last_position = (end_x, end_y, end_z)
                self._last_velocity = seg.velocity_at(seg.duration)
                self._last_segment_nr = seg.nr
                self._current_segment = None
                self._segments_consumed += 1

        # 2. Vergangene Segmente verwerfen, passendes laden
        with self._lock:
            while self._queue:
                candidate = self._queue[0]
                cand_end = candidate.print_time + candidate.duration

                if t_klipper >= cand_end:
                    # Komplett vergangen — verwerfen
                    discarded = self._queue.popleft()
                    self._segments_consumed += 1
                    # Position am Ende merken
                    end_x, end_y, end_z = discarded.position_at(
                        discarded.duration)
                    self._last_position = (end_x, end_y, end_z)
                    self._last_velocity = discarded.velocity_at(
                        discarded.duration)
                    self._last_segment_nr = discarded.nr
                    continue

                if candidate.print_time <= t_klipper < cand_end:
                    # Passt — als aktuelles Segment setzen
                    self._current_segment = self._queue.popleft()
                    return self._current_segment

                if t_klipper < candidate.print_time:
                    # Wir sind in einer Lücke vor dem nächsten Segment
                    # → Position halten (Gap)
                    if self._gap_holds % 500 == 0 and self._gap_holds > 0:
                        logger.debug(
                            "PLANNER: Zeitlücke — t_klipper=%.3f, "
                            "nächstes Segment bei %.3f (Δ%.3fs)",
                            t_klipper, candidate.print_time,
                            candidate.print_time - t_klipper)
                    return None

                break  # Sollte nicht erreicht werden

        return None

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
        return {
            "queue_depth": len(self._queue),
            "queue_time_s": round(self.queue_time_s, 3),
            "current_segment": seg.nr if seg else None,
            "time_synced": self._time_synced,
            "offset_s": self.offset_s,
            "t0_bridge": round(self._t0_bridge, 3) if self._time_synced else None,
            "t0_klipper": round(self._t0_klipper, 3) if self._time_synced else None,
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
