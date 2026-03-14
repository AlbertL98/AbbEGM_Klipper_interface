from __future__ import annotations

import math
import logging
from dataclasses import dataclass

logger = logging.getLogger("egm.source")


# ── Segment-Validierungsfehler ───────────────────────────────

class SegmentValidationError(ValueError):
    """Wird bei ungültigen Segmenten geworfen."""
    pass


# ── Datenmodell ──────────────────────────────────────────────

@dataclass
class TrapezSegment:
    """Ein Trapez-Geschwindigkeitsprofil-Segment aus Klipper."""
    nr: int
    print_time: float       # Klipper print_time (s)
    duration: float         # Gesamtdauer (s)

    start_x: float
    start_y: float
    start_z: float
    end_x: float
    end_y: float
    end_z: float
    distance: float

    start_v: float          # Startgeschwindigkeit (mm/s)
    cruise_v: float         # Maximalgeschwindigkeit (mm/s)
    end_v: float            # Endgeschwindigkeit (mm/s)
    accel: float            # Beschleunigung (mm/s²)

    accel_t: float          # Beschleunigungszeit (s)
    cruise_t: float         # Konstantfahrzeit (s)
    decel_t: float          # Bremszeit (s)

    axes_r_x: float         # Richtungsvektor (normiert)
    axes_r_y: float
    axes_r_z: float

    def validate(self):
        """
        Plausibilitätsprüfung — wirft SegmentValidationError
        wenn das Segment ungültige Werte enthält.

        Wird nach from_dict() / from_csv_row() aufgerufen.
        Fängt Fehler ab, die zu Division-by-Zero oder falscher
        Interpolation führen würden.
        """
        if self.duration <= 0:
            raise SegmentValidationError(
                f"Segment #{self.nr}: duration={self.duration} <= 0")
        if self.distance < 0:
            raise SegmentValidationError(
                f"Segment #{self.nr}: distance={self.distance} < 0")
        if self.accel_t < 0 or self.cruise_t < 0 or self.decel_t < 0:
            raise SegmentValidationError(
                f"Segment #{self.nr}: negative Phasenzeit "
                f"(accel_t={self.accel_t}, cruise_t={self.cruise_t}, "
                f"decel_t={self.decel_t})")
        phase_sum = self.accel_t + self.cruise_t + self.decel_t
        if abs(phase_sum - self.duration) > 0.001:
            raise SegmentValidationError(
                f"Segment #{self.nr}: Phasensumme "
                f"{phase_sum:.6f} != duration {self.duration:.6f}")
        if self.start_v < 0 or self.cruise_v < 0 or self.end_v < 0:
            raise SegmentValidationError(
                f"Segment #{self.nr}: negative Geschwindigkeit "
                f"(start_v={self.start_v}, cruise_v={self.cruise_v}, "
                f"end_v={self.end_v})")
        # Richtungsvektor: Länge sollte ~1.0 sein (oder 0 bei Stillstand)
        r_len = math.sqrt(self.axes_r_x**2 + self.axes_r_y**2
                          + self.axes_r_z**2)
        if self.distance > 0.001 and abs(r_len - 1.0) > 0.01:
            raise SegmentValidationError(
                f"Segment #{self.nr}: Richtungsvektor nicht normiert "
                f"(Länge={r_len:.4f})")

    @classmethod
    def from_dict(cls, d: dict) -> "TrapezSegment":
        """Erzeugt Segment aus dem JSON-Dict von move_export.py."""
        start = d["start"]
        end = d["end"]
        axes_r = d["axes_r"]
        seg = cls(
            nr=d["nr"],
            print_time=d["print_time"],
            duration=d["duration"],
            start_x=start[0], start_y=start[1], start_z=start[2],
            end_x=end[0], end_y=end[1], end_z=end[2],
            distance=d["distance"],
            start_v=d["start_v"],
            cruise_v=d["cruise_v"],
            end_v=d["end_v"],
            accel=d["accel"],
            accel_t=d["accel_t"],
            cruise_t=d["cruise_t"],
            decel_t=d["decel_t"],
            axes_r_x=axes_r[0], axes_r_y=axes_r[1], axes_r_z=axes_r[2],
        )
        seg.validate()
        return seg

    @classmethod
    def from_csv_row(cls, row: dict) -> "TrapezSegment":
        """Erzeugt Segment aus einer CSV-Zeile (für Batch-Tests)."""
        seg = cls(
            nr=int(row["move_nr"]),
            print_time=float(row["print_time"]),
            duration=float(row["move_duration"]),
            start_x=float(row["start_x"]),
            start_y=float(row["start_y"]),
            start_z=float(row["start_z"]),
            end_x=float(row["end_x"]),
            end_y=float(row["end_y"]),
            end_z=float(row["end_z"]),
            distance=float(row["distance"]),
            start_v=float(row["start_v"]),
            cruise_v=float(row["cruise_v"]),
            end_v=float(row["end_v"]),
            accel=float(row["accel"]),
            accel_t=float(row["accel_time"]),
            cruise_t=float(row["cruise_time"]),
            decel_t=float(row["decel_time"]),
            axes_r_x=float(row["axes_r_x"]),
            axes_r_y=float(row["axes_r_y"]),
            axes_r_z=float(row["axes_r_z"]),
        )
        seg.validate()
        return seg

    @property
    def end_time(self) -> float:
        """print_time + duration — Zeitpunkt an dem dieses Segment endet."""
        return self.print_time + self.duration

    def position_at(self, t: float) -> tuple[float, float, float]:
        """
        Berechnet Position zum Zeitpunkt t (0 ≤ t ≤ duration)
        innerhalb des Trapezprofils.

        Kernstück der Interpolation — wird vom Trajectory Planner
        in jedem EGM-Zyklus aufgerufen.

        FIX: Distanz wird auf [0, self.distance] geclampt, damit
        Rundungsfehler in der Decel-Phase die Position nicht
        über das Segment-Ende hinausschießen lassen.
        """
        t = max(0.0, min(t, self.duration))

        # Phase 1: Beschleunigung
        if t <= self.accel_t:
            d = self.start_v * t + 0.5 * self.accel * t * t

        # Phase 2: Konstantfahrt
        elif t <= self.accel_t + self.cruise_t:
            dt = t - self.accel_t
            d_accel = (self.start_v * self.accel_t
                       + 0.5 * self.accel * self.accel_t ** 2)
            d = d_accel + self.cruise_v * dt

        # Phase 3: Bremsen
        else:
            dt = t - self.accel_t - self.cruise_t
            d_accel = (self.start_v * self.accel_t
                       + 0.5 * self.accel * self.accel_t ** 2)
            d_cruise = self.cruise_v * self.cruise_t
            d = (d_accel + d_cruise
                 + self.cruise_v * dt - 0.5 * self.accel * dt * dt)

        # FIX: Distanz auf [0, distance] clampen
        d = max(0.0, min(d, self.distance))

        x = self.start_x + self.axes_r_x * d
        y = self.start_y + self.axes_r_y * d
        z = self.start_z + self.axes_r_z * d
        return x, y, z

    def velocity_at(self, t: float) -> float:
        """Skalare Geschwindigkeit zum Zeitpunkt t."""
        t = max(0.0, min(t, self.duration))
        if t <= self.accel_t:
            return self.start_v + self.accel * t
        elif t <= self.accel_t + self.cruise_t:
            return self.cruise_v
        else:
            dt = t - self.accel_t - self.cruise_t
            return max(0.0, self.cruise_v - self.accel * dt)

    def to_dict(self) -> dict:
        return {
            "nr": self.nr,
            "print_time": self.print_time,
            "duration": self.duration,
            "start": [self.start_x, self.start_y, self.start_z],
            "end": [self.end_x, self.end_y, self.end_z],
            "distance": self.distance,
            "start_v": self.start_v,
            "cruise_v": self.cruise_v,
            "end_v": self.end_v,
            "accel": self.accel,
            "accel_t": self.accel_t,
            "cruise_t": self.cruise_t,
            "decel_t": self.decel_t,
            "axes_r": [self.axes_r_x, self.axes_r_y, self.axes_r_z],
        }