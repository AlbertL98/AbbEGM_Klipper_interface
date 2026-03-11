from __future__ import annotations
# segment_source.py — Empfang von Trapez-Segmenten aus Klipper
#
# Empfängt die JSON-Lines von move_export.py über TCP.
# Identisches Format wie TrapezSegment in move_export.py:
#   {type: "segment", nr, print_time, duration, start, end,
#    distance, start_v, cruise_v, end_v, accel,
#    accel_t, cruise_t, decel_t, axes_r}
#
# Stellt Segmente in die Plan-Queue des Bridge-Core.
#
# ÄNDERUNGEN:
#   - validate(): Plausibilitätsprüfung für Segmente
#   - position_at(): Distanz-Clamp auf [0, distance]
#   - Einheitliche Clock via bridge_now()

import json
import math
import socket
import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable
from collections import deque

from .clock import bridge_now

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


# ── TCP Segment-Empfänger ────────────────────────────────────

class TcpSegmentReceiver:
    """
    Verbindet sich mit dem TCP-Server von move_export.py
    und schiebt empfangene Segmente in eine Queue.

    Thread-sicher: Läuft in eigenem Thread, füllt die Queue,
    die vom Bridge-Core konsumiert wird.

    Validiert monoton steigende print_time (Voraussetzung
    für time-indexed Playback).
    """

    def __init__(self, host: str, port: int,
                 on_segment: Callable[[TrapezSegment], None],
                 reconnect_interval: float = 2.0,
                 receive_timeout: float = 5.0):
        self.host = host
        self.port = port
        self.on_segment = on_segment
        self.reconnect_interval = reconnect_interval
        self.receive_timeout = receive_timeout

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None
        self._connected = False
        self._segments_received = 0
        self._segments_rejected = 0
        self._last_error: Optional[str] = None

        # print_time-Monotonie-Tracking
        self._last_print_time: float = -1.0
        self._monotone_violations: int = 0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def segments_received(self) -> int:
        return self._segments_received

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._receive_loop, daemon=True, name="seg-receiver"
        )
        self._thread.start()
        logger.info("SOURCE: Segment-Empfänger gestartet → %s:%d",
                     self.host, self.port)

    def stop(self):
        self._running = False
        self._disconnect()
        if self._thread:
            self._thread.join(timeout=5.0)
        logger.info("SOURCE: Segment-Empfänger gestoppt "
                     "(%d empfangen, %d abgelehnt, "
                     "%d Monotonie-Verstöße)",
                     self._segments_received, self._segments_rejected,
                     self._monotone_violations)

    def _connect(self) -> bool:
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self.receive_timeout)
            self._socket.connect((self.host, self.port))
            self._connected = True
            self._last_error = None
            # Monotonie-Tracking bei Reconnect zurücksetzen
            self._last_print_time = -1.0
            logger.info("SOURCE: Verbunden mit Klipper @ %s:%d",
                         self.host, self.port)
            return True
        except (ConnectionRefusedError, OSError) as e:
            self._last_error = str(e)
            self._connected = False
            return False

    def _disconnect(self):
        self._connected = False
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

    def _receive_loop(self):
        buffer = ""
        while self._running:
            # Verbinden (mit Retry)
            if not self._connected:
                if not self._connect():
                    time.sleep(self.reconnect_interval)
                    continue
                buffer = ""

            # Empfangen
            try:
                data = self._socket.recv(65536)
                if not data:
                    logger.warning("SOURCE: Verbindung geschlossen")
                    self._disconnect()
                    continue

                buffer += data.decode("utf-8")

                # JSON-Lines parsen
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    self._process_line(line)

            except socket.timeout:
                # Kein Daten — OK, weiter warten
                continue
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                logger.warning("SOURCE: Verbindungsfehler: %s", e)
                self._disconnect()
                time.sleep(self.reconnect_interval)

    def _process_line(self, line: str):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as e:
            logger.warning("SOURCE: JSON-Fehler: %s | %s", e, line[:80])
            return

        msg_type = msg.get("type", "")

        if msg_type == "hello":
            logger.info("SOURCE: Server-Hello: %s", msg.get("msg", ""))
            return

        if msg_type == "segment":
            try:
                seg = TrapezSegment.from_dict(msg)

                # print_time-Monotonie prüfen
                if seg.print_time < self._last_print_time:
                    self._monotone_violations += 1
                    logger.warning(
                        "SOURCE: print_time nicht monoton! "
                        "Segment #%d: %.6f < vorheriges %.6f "
                        "(Verstoß #%d)",
                        seg.nr, seg.print_time, self._last_print_time,
                        self._monotone_violations)
                self._last_print_time = seg.print_time

                self._segments_received += 1
                self.on_segment(seg)

            except SegmentValidationError as e:
                self._segments_rejected += 1
                logger.error("SOURCE: Segment abgelehnt: %s", e)
            except (KeyError, ValueError) as e:
                self._segments_rejected += 1
                logger.error("SOURCE: Segment-Parse-Fehler: %s", e)
            return

        logger.debug("SOURCE: Unbekannter Message-Typ: %s", msg_type)

    def snapshot(self) -> dict:
        return {
            "connected": self._connected,
            "segments_received": self._segments_received,
            "segments_rejected": self._segments_rejected,
            "monotone_violations": self._monotone_violations,
            "last_error": self._last_error,
            "target": f"{self.host}:{self.port}",
        }


# ── CSV Segment-Quelle (für Batch-Tests) ─────────────────────

class CsvSegmentSource:
    """
    Liest Segmente aus einer CSV-Datei (von move_export.py).
    Für Offline-Tests und Replay.
    """

    def __init__(self, csv_path: str):
        self.csv_path = csv_path
        self.segments: list[TrapezSegment] = []
        self._rejected: int = 0
        self._load()

    def _load(self):
        import csv
        with open(self.csv_path, "r") as f:
            reader = csv.DictReader(f)
            prev_time = -1.0
            for row in reader:
                try:
                    seg = TrapezSegment.from_csv_row(row)
                    if seg.print_time < prev_time:
                        logger.warning("CSV: print_time nicht monoton "
                                       "bei Segment #%d", seg.nr)
                    prev_time = seg.print_time
                    self.segments.append(seg)
                except SegmentValidationError as e:
                    self._rejected += 1
                    logger.warning("CSV: Segment abgelehnt: %s", e)
                except (KeyError, ValueError) as e:
                    self._rejected += 1
                    logger.warning("CSV: Zeile übersprungen: %s", e)

        logger.info("CSV: %d Segmente geladen, %d abgelehnt aus %s",
                     len(self.segments), self._rejected, self.csv_path)

    def replay(self, on_segment: Callable[[TrapezSegment], None],
               realtime: bool = False):
        """
        Gibt Segmente an Callback weiter.
        Bei realtime=True werden die print_time-Abstände eingehalten.
        """
        prev_time = None
        for seg in self.segments:
            if realtime and prev_time is not None:
                delay = seg.print_time - prev_time
                if delay > 0:
                    time.sleep(delay)
            on_segment(seg)
            prev_time = seg.print_time
