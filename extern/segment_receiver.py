#!/usr/bin/env python3
"""
segment_receiver.py — Klipper → EGM Bridge

Empfängt Trapezsegmente von Klipper (CSV oder TCP), interpoliert sie
in 4ms-Ticks und streamt die Positionen per EGM-UDP an einen ABB-Roboter.

Architektur (3 Threads):
    ┌─────────────┐     ┌──────────────────┐     ┌─────────────┐
    │  Receiver   │-───▶│  SegmentTimeline │────▶│  EGM Loop   │
    │  (TCP/CSV)  │     │  (Thread-safe)   │     │  (250 Hz)   │
    └─────────────┘     └──────────────────┘     └──────┬──────┘
                                                        │ UDP
                                                        ▼
                                                   ┌─────────┐
                                                   │   ABB   │
                                                   │  Robot  │
                                                   └────┬────┘
                                                        │ UDP Feedback
                                                        ▼
                                                   ┌──────────────┐
                                                   │ FeedbackStore│
                                                   │ (für Klipper)│
                                                   └──────────────┘

Verwendung:
    # CSV-Modus (Offline-Test):
    python segment_receiver.py --csv move_segments.csv

    # TCP-Modus (Live mit Klipper):
    python segment_receiver.py --tcp --host localhost --port 7200

    # Ohne echten Roboter (Dry-Run, kein UDP):
    python segment_receiver.py --csv move_segments.csv --dry-run

    # EGM-Ziel konfigurieren:
    python segment_receiver.py --tcp --egm-ip 192.168.1.10 --egm-send-port 6599 --egm-recv-port 6510
"""

import argparse
import collections
import logging
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, List, Deque

from segment_source import (
    CsvSegmentSource,
    TcpSegmentSource,
    TrapezSegment,
)

# ─────────────────────────────────────────────────────────────────────
# Versuche egm_pb2 zu importieren — falls nicht vorhanden, Dummy
# ─────────────────────────────────────────────────────────────────────
try:
    import egm_pb2
    HAS_EGM_PB2 = True
except Exception as e:
    egm_pb2 = None
    HAS_EGM_PB2 = False
    print("DEBUG egm_pb2 import error:", repr(e))  # optional


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger('egm_bridge')


# =====================================================================
#  Konfiguration
# =====================================================================

@dataclass
class BridgeConfig:
    """Zentrale Konfiguration für die gesamte Bridge."""

    # EGM Netzwerk
    egm_ip: str = "127.0.0.1"
    egm_send_port: int = 6599       # Ziel-Port (UCdevice / RobotStudio)
    egm_local_send_port: int = 6512 # Lokaler Source-Port fürs Senden
    egm_recv_port: int = 6510       # Feedback vom Roboter

    # EGM Timing
    egm_rate_hz: float = 250.0      # 250 Hz = 4ms Ticks
    egm_dt: float = 0.004           # wird aus egm_rate_hz berechnet

    # Orientierung (fix)
    orient_q0: float = 0.0
    orient_q1: float = -0.707106
    orient_q2: float = 0.707106
    orient_q3: float = 0.0

    # Buffer / Timing
    lookahead_s: float = 0.080      # Φ: 80ms Lookahead in die Zukunft
    buffer_warn_ms: float = 50.0    # Warnung wenn Buffer < 50ms
    buffer_critical_ms: float = 20.0  # Kritisch wenn Buffer < 20ms

    # Koordinaten-Offset (Klipper → Roboter)
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0

    # Velocity limit (Sicherheit, mm/s)
    max_velocity_xy: float = 300.0
    max_velocity_z: float = 50.0

    # Modi
    dry_run: bool = False           # Kein UDP senden
    log_interval: int = 250         # Status alle N Ticks (= 1s bei 250Hz)

    def __post_init__(self):
        self.egm_dt = 1.0 / self.egm_rate_hz


# =====================================================================
#  Trapez-Interpolator
# =====================================================================

def interpolate_segment(seg: TrapezSegment, t_local: float) -> List[float]:
    """
    Berechnet die XYZ-Position innerhalb eines Trapezsegments.

    t_local: Zeit seit Segment-Start (0 .. segment.duration)
    Rückgabe: [x, y, z]

    Trapezprofil:
        Phase 1 (accel):  s = start_v * t + 0.5 * a * t²
        Phase 2 (cruise): s = s_accel + cruise_v * (t - accel_t)
        Phase 3 (decel):  s = s_accel + s_cruise + cruise_v * dt - 0.5 * a * dt²
    """
    t_local = max(0.0, min(t_local, seg.duration))

    if seg.distance < 1e-9:
        # Null-Move (z.B. reiner Extruder-Move)
        return list(seg.start)

    # Distanz entlang der Bahn berechnen
    if t_local <= seg.accel_t:
        # Beschleunigungsphase
        s = seg.start_v * t_local + 0.5 * seg.accel * t_local * t_local
    elif t_local <= seg.accel_t + seg.cruise_t:
        # Cruisephase
        s_accel = seg.start_v * seg.accel_t + 0.5 * seg.accel * seg.accel_t * seg.accel_t
        dt = t_local - seg.accel_t
        s = s_accel + seg.cruise_v * dt
    else:
        # Verzögerungsphase
        s_accel = seg.start_v * seg.accel_t + 0.5 * seg.accel * seg.accel_t * seg.accel_t
        s_cruise = seg.cruise_v * seg.cruise_t
        dt = t_local - seg.accel_t - seg.cruise_t
        s = s_accel + s_cruise + seg.cruise_v * dt - 0.5 * seg.accel * dt * dt

    # Distanz auf Segment-Distanz clampen (numerische Sicherheit)
    s = max(0.0, min(s, seg.distance))

    # Position entlang axes_r Richtungsvektor
    x = seg.start[0] + seg.axes_r[0] * s
    y = seg.start[1] + seg.axes_r[1] * s
    z = seg.start[2] + seg.axes_r[2] * s

    return [x, y, z]


# =====================================================================
#  SegmentTimeline — Thread-sicherer Segment-Buffer
# =====================================================================

class SegmentTimeline:
    """
    Thread-sicherer Buffer für Trapezsegmente.

    Segmente werden vom Receiver-Thread hinzugefügt und vom EGM-Loop
    abgefragt. Die Timeline weiß, welches Segment zu welcher print_time
    gehört, und kann für eine gegebene Zeit die interpolierte Position
    berechnen.

    Die Timeline entfernt automatisch alte Segmente, die vollständig
    abgespielt wurden.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._segments: Deque[TrapezSegment] = collections.deque()
        self._total_received = 0
        self._total_consumed = 0
        self._finished = False   # Quelle hat keine Segmente mehr

        # Timing: wird beim ersten Segment gesetzt
        self._t_offset: Optional[float] = None  # wall_clock - print_time

        # Statistik
        self._underrun_count = 0
        self._last_valid_pos: Optional[List[float]] = None

    def add_segment(self, seg: TrapezSegment):
        """Fügt ein neues Segment hinzu (Receiver-Thread)."""
        with self._lock:
            self._segments.append(seg)
            self._total_received += 1

    def mark_finished(self):
        """Signalisiert, dass keine weiteren Segmente kommen."""
        with self._lock:
            self._finished = True

    def is_finished(self) -> bool:
        with self._lock:
            return self._finished and len(self._segments) == 0

    def sync_clock(self, wall_time: float, print_time: float):
        """
        Synchronisiert die Klipper-print_time mit der Wall-Clock.
        Wird einmalig beim ersten Segment aufgerufen.
        """
        with self._lock:
            if self._t_offset is None:
                self._t_offset = wall_time - print_time
                logger.info(
                    "Timeline: Clock synchronisiert — "
                    "t_offset=%.4fs (wall=%.4f, print=%.4f)",
                    self._t_offset, wall_time, print_time
                )

    def klipper_time_to_wall(self, print_time: float) -> float:
        """Konvertiert Klipper print_time zu Wall-Clock-Zeit."""
        if self._t_offset is None:
            return 0.0
        return print_time + self._t_offset

    def get_position(self, wall_time: float) -> Optional[List[float]]:
        """
        Berechnet die interpolierte Position für die gegebene Wall-Clock-Zeit.

        Rückgabe: [x, y, z] oder None wenn kein Segment verfügbar.
        Entfernt automatisch abgelaufene Segmente.
        """
        with self._lock:
            if self._t_offset is None:
                return self._last_valid_pos

            # Klipper-Zeit für diesen Moment
            t_klipper = wall_time - self._t_offset

            # Abgelaufene Segmente entfernen
            while self._segments:
                seg = self._segments[0]
                seg_end_time = seg.print_time + seg.duration
                if t_klipper > seg_end_time + 0.001:  # 1ms Toleranz
                    self._segments.popleft()
                    self._total_consumed += 1
                else:
                    break

            # Aktuelles Segment finden und interpolieren
            for seg in self._segments:
                seg_end_time = seg.print_time + seg.duration
                if t_klipper >= seg.print_time - 0.001 and t_klipper <= seg_end_time + 0.001:
                    t_local = t_klipper - seg.print_time
                    pos = interpolate_segment(seg, t_local)
                    self._last_valid_pos = pos
                    return pos

            # Kein passendes Segment — Underrun oder noch nicht gestartet
            if self._segments:
                # Noch nicht beim ersten Segment angekommen → Startposition
                first = self._segments[0]
                if t_klipper < first.print_time:
                    return list(first.start)

            # Buffer leer → Underrun
            if self._last_valid_pos is not None:
                self._underrun_count += 1
            return self._last_valid_pos

    def get_stats(self) -> dict:
        """Gibt aktuelle Buffer-Statistiken zurück."""
        with self._lock:
            buffered = len(self._segments)
            buffer_ms = 0.0
            if self._segments:
                first = self._segments[0]
                last = self._segments[-1]
                buffer_ms = (
                    (last.print_time + last.duration) - first.print_time
                ) * 1000.0

            return {
                'buffered_segments': buffered,
                'buffer_ms': buffer_ms,
                'total_received': self._total_received,
                'total_consumed': self._total_consumed,
                'underrun_count': self._underrun_count,
                'clock_synced': self._t_offset is not None,
                'finished': self._finished,
            }


# =====================================================================
#  EGM Feedback Store — für späteres Weiterreichen an Klipper
# =====================================================================

@dataclass
class RobotFeedback:
    """Letzte bekannte Ist-Position vom Roboter."""
    timestamp: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    seq: int = 0
    valid: bool = False


class FeedbackStore:
    """
    Thread-sicherer Speicher für das EGM-Feedback vom Roboter.

    Wird vom EGM-Loop geschrieben und kann von einem
    zukünftigen Klipper-Rückkanal gelesen werden.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._feedback = RobotFeedback()
        self._tracking_errors: Deque[float] = collections.deque(maxlen=100)

    def update(self, fb: RobotFeedback):
        with self._lock:
            self._feedback = fb

    def update_tracking_error(self, error_mm: float):
        with self._lock:
            self._tracking_errors.append(error_mm)

    def get_latest(self) -> RobotFeedback:
        with self._lock:
            return self._feedback

    def get_max_tracking_error(self) -> float:
        with self._lock:
            if self._tracking_errors:
                return max(self._tracking_errors)
            return 0.0

    def get_avg_tracking_error(self) -> float:
        with self._lock:
            if self._tracking_errors:
                return sum(self._tracking_errors) / len(self._tracking_errors)
            return 0.0


# =====================================================================
#  EGM Protocol Helper
# =====================================================================

def build_egm_message(seq: int, t_ms: int,
                      x: float, y: float, z: float,
                      q0: float, q1: float, q2: float, q3: float) -> bytes:
    """Baut eine EgmSensor Protobuf-Nachricht."""
    if not HAS_EGM_PB2:
        raise RuntimeError("egm_pb2 nicht verfügbar — bitte generieren oder --dry-run verwenden")

    msg = egm_pb2.EgmSensor()
    msg.header.seqno = seq
    msg.header.tm = t_ms
    msg.header.mtype = egm_pb2.EgmHeader.MSGTYPE_CORRECTION

    msg.planned.cartesian.pos.x = x
    msg.planned.cartesian.pos.y = y
    msg.planned.cartesian.pos.z = z

    msg.planned.cartesian.orient.u0 = q0
    msg.planned.cartesian.orient.u1 = q1
    msg.planned.cartesian.orient.u2 = q2
    msg.planned.cartesian.orient.u3 = q3

    return msg.SerializeToString()


def parse_egm_feedback(data: bytes) -> Optional[RobotFeedback]:
    """Parst eine EgmRobot Protobuf-Nachricht."""
    if not HAS_EGM_PB2:
        return None
    try:
        fb_msg = egm_pb2.EgmRobot()
        fb_msg.ParseFromString(data)

        fb = RobotFeedback(
            timestamp=time.time(),
            x=fb_msg.feedBack.cartesian.pos.x,
            y=fb_msg.feedBack.cartesian.pos.y,
            z=fb_msg.feedBack.cartesian.pos.z,
            q0=fb_msg.feedBack.cartesian.orient.u0,
            q1=fb_msg.feedBack.cartesian.orient.u1,
            q2=fb_msg.feedBack.cartesian.orient.u2,
            q3=fb_msg.feedBack.cartesian.orient.u3,
            seq=fb_msg.header.seqno,
            valid=True,
        )
        return fb
    except Exception as e:
        logger.warning("EGM Feedback Parse-Fehler: %s", e)
        return None


# =====================================================================
#  Receiver Thread — empfängt Segmente und füllt die Timeline
# =====================================================================

class ReceiverThread(threading.Thread):
    """
    Empfängt Segmente von einer SegmentSource und fügt sie
    in die SegmentTimeline ein.
    """

    def __init__(self, source, timeline: SegmentTimeline, config: BridgeConfig):
        super().__init__(name="Receiver", daemon=True)
        self._source = source
        self._timeline = timeline
        self._config = config
        self._running = True
        self._error: Optional[str] = None

    def stop(self):
        self._running = False

    @property
    def error(self):
        return self._error

    def run(self):
        try:
            logger.info("Receiver: Starte Segment-Empfang...")
            first = True

            for seg in self._source.segments():
                if not self._running:
                    break

                # Clock-Sync beim ersten Segment
                if first:
                    self._timeline.sync_clock(
                        wall_time=time.time(),
                        print_time=seg.print_time
                    )
                    first = False
                    logger.info(
                        "Receiver: Erstes Segment #%d bei t=%.3fs",
                        seg.nr, seg.print_time
                    )

                self._timeline.add_segment(seg)

            logger.info("Receiver: Quelle erschöpft — alle Segmente empfangen")
            self._timeline.mark_finished()

        except Exception as e:
            self._error = str(e)
            logger.error("Receiver: Fehler — %s", e)
            self._timeline.mark_finished()


# =====================================================================
#  EGM Loop Thread — 250Hz Senden + Feedback Empfangen
# =====================================================================

class EgmLoopThread(threading.Thread):
    """
    250Hz Loop der:
      1. Position aus SegmentTimeline interpoliert (mit Lookahead Φ)
      2. EGM-Nachricht per UDP sendet
      3. Feedback vom Roboter empfängt
      4. Tracking-Error berechnet
    """

    def __init__(self, timeline: SegmentTimeline,
                 feedback_store: FeedbackStore,
                 config: BridgeConfig):
        super().__init__(name="EGM-Loop", daemon=True)
        self._timeline = timeline
        self._feedback = feedback_store
        self._config = config
        self._running = True

        # Sockets
        self._tx_sock: Optional[socket.socket] = None
        self._rx_sock: Optional[socket.socket] = None

        # State
        self._seq = 0
        self._tick_count = 0
        self._t0 = 0.0
        self._last_sent_pos = [0.0, 0.0, 0.0]

        # Statistik
        self._late_count = 0
        self._max_jitter_ms = 0.0

    def stop(self):
        self._running = False

    def _setup_sockets(self):
        """Erstellt die UDP-Sockets für EGM."""
        if self._config.dry_run:
            logger.info("EGM-Loop: Dry-Run — kein UDP")
            return

        try:
            self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._tx_sock.bind(("0.0.0.0", self._config.egm_local_send_port))
            self._tx_sock.setblocking(False)

            self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx_sock.bind(("0.0.0.0", self._config.egm_recv_port))
            self._rx_sock.settimeout(0.001)  # 1ms Timeout für nicht-blockierendes Lesen

            logger.info(
                "EGM-Loop: UDP bereit — Senden an %s:%d, Empfang auf :%d",
                self._config.egm_ip, self._config.egm_send_port,
                self._config.egm_recv_port
            )
        except OSError as e:
            logger.error("EGM-Loop: Socket-Fehler: %s", e)
            raise

    def _close_sockets(self):
        for s in (self._tx_sock, self._rx_sock):
            if s:
                try:
                    s.close()
                except OSError:
                    pass

    def _send_egm(self, x: float, y: float, z: float):
        """Sendet eine EGM-Position per UDP."""
        cfg = self._config

        # Koordinaten-Offset anwenden
        x_robot = x + cfg.offset_x
        y_robot = y + cfg.offset_y
        z_robot = z + cfg.offset_z

        t_ms = int((time.time() - self._t0) * 1000)

        if not cfg.dry_run and self._tx_sock:
            try:
                raw = build_egm_message(
                    seq=self._seq, t_ms=t_ms,
                    x=x_robot, y=y_robot, z=z_robot,
                    q0=cfg.orient_q0, q1=cfg.orient_q1,
                    q2=cfg.orient_q2, q3=cfg.orient_q3,
                )
                self._tx_sock.sendto(
                    raw, (cfg.egm_ip, cfg.egm_send_port)
                )
            except OSError as e:
                if self._seq % 1000 == 0:
                    logger.warning("EGM Send-Fehler: %s", e)

        self._seq += 1
        self._last_sent_pos = [x_robot, y_robot, z_robot]

    def _recv_feedback(self):
        """Versucht Feedback vom Roboter zu lesen (nicht-blockierend)."""
        if self._config.dry_run or not self._rx_sock:
            return

        try:
            data, _ = self._rx_sock.recvfrom(4096)
            fb = parse_egm_feedback(data)
            if fb and fb.valid:
                self._feedback.update(fb)

                # Tracking-Error berechnen (Soll vs. Ist)
                dx = self._last_sent_pos[0] - fb.x
                dy = self._last_sent_pos[1] - fb.y
                dz = self._last_sent_pos[2] - fb.z
                error = math.sqrt(dx*dx + dy*dy + dz*dz)
                self._feedback.update_tracking_error(error)

        except socket.timeout:
            pass
        except OSError:
            pass

    def run(self):
        cfg = self._config
        dt = cfg.egm_dt

        self._setup_sockets()
        self._t0 = time.time()

        logger.info(
            "EGM-Loop: Gestartet — %d Hz, dt=%.1fms, Lookahead=%.0fms",
            int(cfg.egm_rate_hz), dt * 1000, cfg.lookahead_s * 1000
        )

        # Warte bis Timeline synchronisiert ist
        logger.info("EGM-Loop: Warte auf erstes Segment...")
        while self._running:
            stats = self._timeline.get_stats()
            if stats['clock_synced']:
                break
            if self._timeline.is_finished():
                logger.warning("EGM-Loop: Quelle beendet bevor Segmente ankamen")
                return
            time.sleep(0.01)

        logger.info("EGM-Loop: Timeline synchronisiert — starte Streaming")

        # ─── Hauptschleife ───
        next_tick = time.perf_counter()

        try:
            while self._running:
                tick_start = time.perf_counter()
                self._tick_count += 1

                # Wall-Clock mit Lookahead
                wall_now = time.time()
                t_query = wall_now + cfg.lookahead_s

                # Position interpolieren
                pos = self._timeline.get_position(t_query)

                if pos is not None:
                    self._send_egm(pos[0], pos[1], pos[2])
                else:
                    # Underrun: letzte Position halten
                    if self._last_sent_pos:
                        self._send_egm(
                            self._last_sent_pos[0],
                            self._last_sent_pos[1],
                            self._last_sent_pos[2],
                        )

                # Feedback lesen
                self._recv_feedback()

                # Periodische Status-Ausgabe
                if self._tick_count % cfg.log_interval == 0:
                    self._print_status()

                # Ende-Bedingung
                if self._timeline.is_finished():
                    logger.info("EGM-Loop: Alle Segmente abgespielt")
                    # Noch ein paar Hold-Ticks senden
                    for _ in range(50):
                        if self._last_sent_pos:
                            self._send_egm(*self._last_sent_pos)
                        self._recv_feedback()
                        time.sleep(dt)
                    break

                # Timing: nächsten Tick abwarten
                next_tick += dt
                sleep_time = next_tick - time.perf_counter()

                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Tick verpasst
                    self._late_count += 1
                    jitter = -sleep_time * 1000
                    if jitter > self._max_jitter_ms:
                        self._max_jitter_ms = jitter
                    # Timer resetten wenn wir zu weit hinterher sind
                    if sleep_time < -dt * 3:
                        next_tick = time.perf_counter()

        finally:
            self._close_sockets()
            logger.info(
                "EGM-Loop: Beendet — %d Ticks, %d late, max_jitter=%.1fms",
                self._tick_count, self._late_count, self._max_jitter_ms
            )

    def _print_status(self):
        """Gibt periodischen Status aus."""
        stats = self._timeline.get_stats()
        fb = self._feedback.get_latest()
        avg_err = self._feedback.get_avg_tracking_error()

        buf_ms = stats['buffer_ms']
        buf_icon = "●" if buf_ms > self._config.buffer_warn_ms else (
            "◐" if buf_ms > self._config.buffer_critical_ms else "○"
        )

        status = (
            f"  {buf_icon} Tick #{self._tick_count:>7d} | "
            f"Buf: {stats['buffered_segments']:>4d} seg "
            f"({buf_ms:>6.1f}ms) | "
            f"Sent: ({self._last_sent_pos[0]:>7.2f}, "
            f"{self._last_sent_pos[1]:>7.2f}, "
            f"{self._last_sent_pos[2]:>6.2f}) | "
            f"Underruns: {stats['underrun_count']} | "
            f"Late: {self._late_count}"
        )

        if fb.valid:
            status += (
                f" | FB: ({fb.x:>7.2f}, {fb.y:>7.2f}, {fb.z:>6.2f}) "
                f"err={avg_err:.2f}mm"
            )

        print(status)

        # Warnungen
        if buf_ms < self._config.buffer_critical_ms and stats['buffered_segments'] > 0:
            logger.warning("BUFFER CRITICAL: nur %.1fms verbleibend!", buf_ms)
        elif buf_ms < self._config.buffer_warn_ms and stats['buffered_segments'] > 0:
            logger.warning("Buffer niedrig: %.1fms", buf_ms)


# =====================================================================
#  Hauptprogramm
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Klipper → EGM Bridge: Trapezsegmente zu ABB Roboter'
    )

    # ─── Quellen-Auswahl ───
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument(
        '--csv', metavar='DATEI',
        help='CSV-Datei als Quelle (Batch/Offline-Test)'
    )
    source_group.add_argument(
        '--tcp', action='store_true',
        help='TCP-Verbindung zu Klipper (Live-Modus)'
    )

    # ─── Klipper TCP ───
    parser.add_argument('--host', default='localhost', help='Klipper Host (default: localhost)')
    parser.add_argument('--port', type=int, default=7200, help='Klipper TCP Port (default: 7200)')

    # ─── EGM Netzwerk ───
    parser.add_argument('--egm-ip', default='127.0.0.1', help='EGM Ziel-IP (default: 127.0.0.1)')
    parser.add_argument('--egm-send-port', type=int, default=6599, help='EGM Ziel-Port (default: 6599)')
    parser.add_argument('--egm-recv-port', type=int, default=6510, help='EGM Feedback-Port (default: 6510)')
    parser.add_argument('--egm-local-port', type=int, default=6512, help='Lokaler Send-Port (default: 6512)')

    # ─── Timing ───
    parser.add_argument('--lookahead', type=float, default=0.080, help='Lookahead Φ in Sekunden (default: 0.08)')
    parser.add_argument('--rate', type=float, default=250.0, help='EGM Rate in Hz (default: 250)')

    # ─── Koordinaten-Offset ───
    parser.add_argument('--offset-x', type=float, default=0.0, help='X-Offset Klipper→Robot (mm)')
    parser.add_argument('--offset-y', type=float, default=0.0, help='Y-Offset Klipper→Robot (mm)')
    parser.add_argument('--offset-z', type=float, default=0.0, help='Z-Offset Klipper→Robot (mm)')

    # ─── Modi ───
    parser.add_argument('--dry-run', action='store_true', help='Kein UDP — nur Interpolation + Log')
    parser.add_argument('--log-interval', type=int, default=250, help='Status alle N Ticks (default: 250 = 1s)')

    args = parser.parse_args()

    # ─── Config bauen ───
    config = BridgeConfig(
        egm_ip=args.egm_ip,
        egm_send_port=args.egm_send_port,
        egm_recv_port=args.egm_recv_port,
        egm_local_send_port=args.egm_local_port,
        egm_rate_hz=args.rate,
        lookahead_s=args.lookahead,
        offset_x=args.offset_x,
        offset_y=args.offset_y,
        offset_z=args.offset_z,
        dry_run=args.dry_run,
        log_interval=args.log_interval,
    )

    # ─── EGM Protobuf Check ───
    if not config.dry_run and not HAS_EGM_PB2:
        logger.error(
            "egm_pb2 nicht gefunden! Entweder:\n"
            "  1. egm_pb2.py generieren (protoc --python_out=. egm.proto)\n"
            "  2. --dry-run verwenden für Tests ohne Roboter"
        )
        sys.exit(1)

    # ─── Source erstellen ───
    if not args.csv and not args.tcp:
        args.csv = 'move_segments.csv'

    if args.csv:
        source = CsvSegmentSource(csv_path=args.csv, realtime=True, speed_factor=1.0)
        source_name = f"CSV: {args.csv}"
    else:
        source = TcpSegmentSource(host=args.host, port=args.port)
        source_name = f"TCP: {args.host}:{args.port}"

    # ─── Header ───
    print()
    print("=" * 70)
    print("  Klipper → EGM Bridge")
    print(f"  Quelle:     {source_name}")
    print(f"  EGM Ziel:   {config.egm_ip}:{config.egm_send_port}")
    print(f"  Rate:       {config.egm_rate_hz:.0f} Hz ({config.egm_dt*1000:.1f}ms)")
    print(f"  Lookahead:  {config.lookahead_s*1000:.0f}ms")
    print(f"  Offset:     ({config.offset_x}, {config.offset_y}, {config.offset_z})")
    if config.dry_run:
        print("  Modus:      DRY-RUN (kein UDP)")
    print("=" * 70)
    print()

    # ─── Pipeline aufbauen ───
    timeline = SegmentTimeline()
    feedback_store = FeedbackStore()

    # Source öffnen
    try:
        source.connect()
    except (FileNotFoundError, ConnectionError) as e:
        logger.error("Quellfehler: %s", e)
        sys.exit(1)

    # Threads starten
    receiver = ReceiverThread(source, timeline, config)
    egm_loop = EgmLoopThread(timeline, feedback_store, config)

    receiver.start()

    # Kurz warten bis erste Segmente da sind (Buffer aufbauen)
    logger.info("Warte auf initialen Buffer...")
    pre_buffer_start = time.time()
    while time.time() - pre_buffer_start < 5.0:
        stats = timeline.get_stats()
        if stats['buffer_ms'] >= config.lookahead_s * 1000 * 2:
            break
        if stats['total_received'] > 0 and not receiver.is_alive():
            break  # CSV komplett geladen
        time.sleep(0.01)

    stats = timeline.get_stats()
    logger.info(
        "Buffer: %d Segmente (%.1fms) — starte EGM Loop",
        stats['buffered_segments'], stats['buffer_ms']
    )

    egm_loop.start()

    # ─── Main Thread: Warten + Monitoring ───
    try:
        while egm_loop.is_alive():
            egm_loop.join(timeout=0.5)
    except KeyboardInterrupt:
        print("\n  Ctrl+C — stoppe...")
        egm_loop.stop()
        receiver.stop()

    # ─── Aufräumen ───
    receiver.stop()
    egm_loop.stop()
    egm_loop.join(timeout=2.0)
    receiver.join(timeout=2.0)
    source.disconnect()

    # ─── Zusammenfassung ───
    stats = timeline.get_stats()
    fb = feedback_store.get_latest()
    max_err = feedback_store.get_max_tracking_error()

    print()
    print("=" * 70)
    print("  Zusammenfassung")
    print(f"  Segmente empfangen: {stats['total_received']}")
    print(f"  Segmente abgespielt: {stats['total_consumed']}")
    print(f"  Buffer-Underruns:   {stats['underrun_count']}")
    if fb.valid:
        print(f"  Max Tracking-Error: {max_err:.2f}mm")
        print(f"  Avg Tracking-Error: {feedback_store.get_avg_tracking_error():.2f}mm")
    if receiver.error:
        print(f"  Receiver-Fehler:    {receiver.error}")
    print("=" * 70)
    print()


if __name__ == '__main__':
    main()
