from __future__ import annotations
# egm_client.py — UDP/EGM-Kommunikation mit ABB RobotStudio
#
# Basiert auf workingEGM.py — bewährtes Setup:
#   TX-Socket (127.0.0.1:6512) → sendet an UCdevice (127.0.0.1:6599)
#   RX-Socket (0.0.0.0:6510)  → empfängt Feedback von ROB_1
#
# Zwei getrennte Sockets wie im ABB-EGM-Standard.
#
# CLOCK-FIX: Alle internen Timestamps nutzen jetzt bridge_now()
#   (= time.perf_counter()). Vorher wurde RTT mit gemischten
#   Clocks berechnet (monotonic TX vs. perf_counter RX), was
#   auf Windows zu falschen Werten führte.

import socket
import time
import logging
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable

from .clock import bridge_now

logger = logging.getLogger("egm.client")


# ── Datenstrukturen ──────────────────────────────────────────

@dataclass
class EgmTarget:
    """Sollposition für den Roboter (Sensor → Robot)."""
    sequence_id: int
    timestamp: float        # Bridge-Zeit (s), via bridge_now()
    x: float
    y: float
    z: float
    # Quaternion-Orientierung (Default aus workingEGM.py)
    q0: float = 0.0
    q1: float = -0.707106
    q2: float = 0.707106
    q3: float = 0.0


@dataclass
class EgmFeedback:
    """Istposition vom Roboter (Robot → Sensor)."""
    sequence_id: int
    timestamp: float        # Empfangszeit (Bridge-Zeit), via bridge_now()
    robot_time: float       # Roboter-Zeitstempel (ms)
    x: float
    y: float
    z: float
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0


@dataclass
class EgmConnectionStats:
    """Verbindungsstatistik."""
    tx_count: int = 0
    rx_count: int = 0
    tx_errors: int = 0
    rx_timeouts: int = 0
    last_tx_time: float = 0.0
    last_rx_time: float = 0.0
    cycles_without_response: int = 0
    rtt_last_ms: float = 0.0
    rtt_avg_ms: float = 0.0


class EgmClient:
    """
    EGM UDP-Client — exakt wie workingEGM.py aufgebaut.

    Zwei getrennte Sockets:
      tx: bind(robot_ip, local_send_port) → sendto(robot_ip, send_port)
      rx: bind(0.0.0.0, recv_port)        → recvfrom()
    """

    def __init__(self,
                 robot_ip: str = "127.0.0.1",
                 send_port: int = 6599,
                 recv_port: int = 6510,
                 local_send_port: int = 6512,
                 timeout_ms: int = 100,
                 watchdog_cycles: int = 50,
                 protocol: str = "auto",
                 default_q0: float = 0.0,
                 default_q1: float = -0.707106,
                 default_q2: float = 0.707106,
                 default_q3: float = 0.0,
                 on_feedback: Optional[Callable[[EgmFeedback], None]] = None,
                 on_timeout: Optional[Callable[[], None]] = None):
        self.robot_ip = robot_ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.local_send_port = local_send_port
        self.timeout_ms = timeout_ms
        self.watchdog_cycles = watchdog_cycles
        self.on_feedback = on_feedback
        self.on_timeout = on_timeout

        self.default_q0 = default_q0
        self.default_q1 = default_q1
        self.default_q2 = default_q2
        self.default_q3 = default_q3

        self._tx_socket: Optional[socket.socket] = None
        self._rx_socket: Optional[socket.socket] = None
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._stats = EgmConnectionStats()
        self._connected = False
        self._t0 = 0.0

        # Ringbuffer: sequence_id → send_time (bridge_now())
        # Für korrektes RTT-Matching bei Feedback-Empfang
        self._send_times: dict[int, float] = {}
        self._send_times_max = 200  # Älteste Einträge aufräumen

        self._pb2 = None
        self._use_protobuf = False
        self._try_load_protobuf(protocol)

    def _try_load_protobuf(self, protocol: str = "auto"):
        if protocol == "json":
            self._use_protobuf = False
            logger.info("EGM: JSON-Modus erzwungen")
            return

        try:
            try:
                from . import egm_pb2
            except ImportError:
                import sys
                from pathlib import Path

                package_root = str(Path(__file__).resolve().parent.parent)
                if package_root not in sys.path:
                    sys.path.insert(0, package_root)

                from data import egm_pb2

            self._pb2 = egm_pb2
            self._use_protobuf = True
            logger.info("EGM: Protobuf geladen")

        except ImportError as e:
            if protocol == "protobuf":
                raise RuntimeError(f"egm_pb2 nicht gefunden: {e}")
            self._use_protobuf = False
            logger.info("EGM: Kein Protobuf — JSON-Fallback")

    # ── Lifecycle ────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Zwei Sockets öffnen:
          tx.bind(("127.0.0.1", 6512))
          rx.bind(("0.0.0.0", 6510))
          rx.settimeout(0.1)
        """
        try:
            # TX-Socket
            self._tx_socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM
            )
            self._tx_socket.bind(
                (self.robot_ip, self.local_send_port)
            )
            logger.info("EGM TX: %s:%d → %s:%d",
                        self.robot_ip, self.local_send_port,
                        self.robot_ip, self.send_port)

            # RX-Socket
            self._rx_socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM
            )
            self._rx_socket.bind(("0.0.0.0", self.recv_port))
            self._rx_socket.settimeout(self.timeout_ms / 1000.0)
            logger.info("EGM RX: 0.0.0.0:%d", self.recv_port)

            self._t0 = time.time()
            self._connected = True
            self._running = True

            # Empfangs-Thread
            self._rx_thread = threading.Thread(
                target=self._receive_loop, daemon=True, name="egm-rx"
            )
            self._rx_thread.start()

            logger.info("EGM: Verbunden (proto: %s)",
                        "protobuf" if self._use_protobuf else "json")
            return True

        except OSError as e:
            logger.error("EGM: Verbindungsfehler: %s", e)
            self._connected = False
            return False

    def disconnect(self):
        self._running = False
        self._connected = False
        for sock in (self._tx_socket, self._rx_socket):
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
        self._tx_socket = None
        self._rx_socket = None
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        logger.info("EGM: Geschlossen (TX:%d RX:%d Err:%d)",
                     self._stats.tx_count, self._stats.rx_count,
                     self._stats.tx_errors)

    # ── Senden ───────────────────────────────────────────────

    def send_target(self, target: EgmTarget) -> bool:
        if not self._connected or not self._tx_socket:
            return False
        try:
            data = self._encode_target(target)
            self._tx_socket.sendto(
                data, (self.robot_ip, self.send_port)
            )
            # FIX: Einheitlich bridge_now() statt time.monotonic()
            now = bridge_now()
            self._stats.tx_count += 1
            self._stats.last_tx_time = now
            self._stats.cycles_without_response += 1

            # Send-Zeit für RTT-Matching speichern (gleiche Clock!)
            self._send_times[target.sequence_id] = now
            if len(self._send_times) > self._send_times_max:
                # Älteste Hälfte aufräumen
                sorted_ids = sorted(self._send_times)
                for old_id in sorted_ids[:len(sorted_ids) // 2]:
                    del self._send_times[old_id]

            if self._stats.cycles_without_response > self.watchdog_cycles:
                logger.error("EGM: Watchdog! %d Zyklen ohne Antwort",
                             self._stats.cycles_without_response)
                if self.on_timeout:
                    self.on_timeout()
            return True
        except OSError as e:
            self._stats.tx_errors += 1
            logger.error("EGM: Sendefehler: %s", e)
            return False

    # ── Empfangen ────────────────────────────────────────────

    def _receive_loop(self):
        while self._running:
            try:
                data, addr = self._rx_socket.recvfrom(4096)
                # FIX: Einheitlich bridge_now() — gleiche Clock wie TX
                rx_time = bridge_now()

                feedback = self._decode_feedback(data, rx_time)
                if feedback:
                    self._stats.rx_count += 1
                    self._stats.last_rx_time = rx_time
                    self._stats.cycles_without_response = 0

                    if self._stats.last_tx_time > 0:
                        # RTT per sequence_id-Matching
                        # Jetzt korrekt: beide Zeiten von bridge_now()
                        tx_time = self._send_times.get(
                            feedback.sequence_id)
                        if tx_time is not None:
                            rtt = (rx_time - tx_time) * 1000
                            del self._send_times[feedback.sequence_id]
                        else:
                            # Fallback: letztes TX (ungenau)
                            rtt = (rx_time
                                   - self._stats.last_tx_time) * 1000
                        self._stats.rtt_last_ms = rtt
                        alpha = 0.1
                        self._stats.rtt_avg_ms = (
                            alpha * rtt +
                            (1 - alpha) * self._stats.rtt_avg_ms
                        )

                    if self.on_feedback:
                        self.on_feedback(feedback)

            except socket.timeout:
                self._stats.rx_timeouts += 1
                continue
            except OSError:
                if self._running:
                    logger.warning("EGM: Empfangsfehler")
                break

    # ── Protobuf (exakt wie workingEGM.py) ───────────────────

    def _encode_target(self, target: EgmTarget) -> bytes:
        if self._use_protobuf and self._pb2:
            return self._encode_protobuf(target)
        return self._encode_json(target)

    def _decode_feedback(self, data: bytes,
                         rx_time: float) -> Optional[EgmFeedback]:
        if self._use_protobuf and self._pb2:
            return self._decode_protobuf(data, rx_time)
        return self._decode_json(data, rx_time)

    def _encode_protobuf(self, target: EgmTarget) -> bytes:
        """
        Exakt wie workingEGM.py:
          msg = egm_pb2.EgmSensor()
          msg.header.seqno / .tm / .mtype
          msg.planned.cartesian.pos.x/y/z
          msg.planned.cartesian.orient.u0/u1/u2/u3
        """
        pb2 = self._pb2
        msg = pb2.EgmSensor()

        msg.header.seqno = target.sequence_id
        msg.header.tm = int((time.time() - self._t0) * 1000)
        msg.header.mtype = pb2.EgmHeader.MSGTYPE_CORRECTION

        msg.planned.cartesian.pos.x = target.x
        msg.planned.cartesian.pos.y = target.y
        msg.planned.cartesian.pos.z = target.z

        msg.planned.cartesian.orient.u0 = target.q0
        msg.planned.cartesian.orient.u1 = target.q1
        msg.planned.cartesian.orient.u2 = target.q2
        msg.planned.cartesian.orient.u3 = target.q3

        return msg.SerializeToString()

    def _decode_protobuf(self, data: bytes,
                         rx_time: float) -> Optional[EgmFeedback]:
        try:
            pb2 = self._pb2
            robot = pb2.EgmRobot()
            robot.ParseFromString(data)

            pos = robot.feedBack.cartesian.pos
            orient = robot.feedBack.cartesian.orient

            return EgmFeedback(
                sequence_id=robot.header.seqno,
                timestamp=rx_time,
                robot_time=robot.header.tm,
                x=pos.x,
                y=pos.y,
                z=pos.z,
                q0=orient.u0,
                q1=orient.u1,
                q2=orient.u2,
                q3=orient.u3,
            )
        except Exception as e:
            logger.warning("EGM: Decode-Fehler: %s", e)
            return None

    # ── JSON Fallback ────────────────────────────────────────

    def _encode_json(self, target: EgmTarget) -> bytes:
        import json
        msg = {
            "type": "egm_target",
            "seq": target.sequence_id,
            "ts": round(time.time() - self._t0, 6),
            "pos": [round(target.x, 3), round(target.y, 3),
                    round(target.z, 3)],
            "orient": [target.q0, target.q1, target.q2, target.q3],
        }
        return json.dumps(msg).encode("utf-8")

    def _decode_json(self, data: bytes,
                     rx_time: float) -> Optional[EgmFeedback]:
        try:
            import json
            msg = json.loads(data.decode("utf-8"))
            pos = msg.get("pos", [0, 0, 0])
            orient = msg.get("orient", [0, 0, 0, 0])
            return EgmFeedback(
                sequence_id=msg.get("seq", 0),
                timestamp=rx_time,
                robot_time=msg.get("ts", 0),
                x=pos[0], y=pos[1], z=pos[2],
                q0=orient[0], q1=orient[1],
                q2=orient[2], q3=orient[3],
            )
        except Exception as e:
            logger.warning("EGM: JSON-Decode-Fehler: %s", e)
            return None

    # ── Status ───────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def stats(self) -> EgmConnectionStats:
        return self._stats

    def snapshot(self) -> dict:
        s = self._stats
        return {
            "connected": self._connected,
            "protocol": "protobuf" if self._use_protobuf else "json",
            "tx_target": f"{self.robot_ip}:{self.send_port}",
            "tx_source": f"{self.robot_ip}:{self.local_send_port}",
            "rx_listen": f"0.0.0.0:{self.recv_port}",
            "tx_count": s.tx_count,
            "rx_count": s.rx_count,
            "tx_errors": s.tx_errors,
            "rx_timeouts": s.rx_timeouts,
            "cycles_without_response": s.cycles_without_response,
            "rtt_avg_ms": round(s.rtt_avg_ms, 2),
        }
