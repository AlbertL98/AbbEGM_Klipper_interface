#!/usr/bin/env python3
# sim_robot.py — Simulierter ABB-Roboter für EGM-Tests
#
# CodingPlan §F1, Stufe 1+2:
#   Stufe 1: Nur Connectivity + Logging (kein Material)
#   Stufe 2: Bewegungsstream trocken (ohne Extrusion)
#
# Empfängt UDP-Sollwerte vom Bridge und antwortet mit
# simuliertem Feedback (Position = Sollposition + konfigurierbarem Offset).
#
# Nutzung:
#   python sim_robot.py                    # Defaults
#   python sim_robot.py --lag 8 --jitter 2 # 8ms Lag + 2ms Jitter
#   python sim_robot.py --tracking-error 1.5  # 1.5mm konstanter Offset

import argparse
import json
import random
import socket
import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d │ SIM │ %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("sim")


class RobotSimulator:
    def __init__(self, listen_port: int = 6511,
                 lag_ms: float = 4.0,
                 jitter_ms: float = 1.0,
                 tracking_error_mm: float = 0.0):
        self.listen_port = listen_port
        self.lag_ms = lag_ms
        self.jitter_ms = jitter_ms
        self.tracking_error_mm = tracking_error_mm

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.settimeout(1.0)

        self._rx_count = 0
        self._tx_count = 0
        self._current_pos = [0.0, 0.0, 0.0]

    def run(self):
        logger.info("Robot-Simulator lauscht auf UDP Port %d", self.listen_port)
        logger.info("  Lag: %.1fms | Jitter: %.1fms | Tracking-Error: %.1fmm",
                     self.lag_ms, self.jitter_ms, self.tracking_error_mm)

        while True:
            try:
                data, addr = self.sock.recvfrom(65536)
                self._rx_count += 1

                # Sollwert dekodieren (JSON-Modus)
                try:
                    msg = json.loads(data.decode("utf-8"))
                except json.JSONDecodeError:
                    logger.warning("Ungültiges Paket von %s", addr)
                    continue

                target_pos = msg.get("pos", [0, 0, 0])

                # Simulierte Verzögerung
                delay = self.lag_ms / 1000.0
                if self.jitter_ms > 0:
                    delay += random.gauss(0, self.jitter_ms / 1000.0)
                    delay = max(0, delay)
                time.sleep(delay)

                # Simulierte Ist-Position (mit konfiguriertem Tracking-Error)
                offset = self.tracking_error_mm
                self._current_pos = [
                    target_pos[0] + random.gauss(0, offset * 0.3),
                    target_pos[1] + random.gauss(0, offset * 0.3),
                    target_pos[2] + random.gauss(0, offset * 0.3),
                ]

                # Feedback senden
                feedback = {
                    "type": "egm_feedback",
                    "seq": msg.get("seq", 0),
                    "ts": time.monotonic(),
                    "pos": [round(p, 3) for p in self._current_pos],
                    "rot": [0.0, 0.0, 0.0],
                    "rapid_running": True,
                    "motors_on": True,
                }
                self.sock.sendto(
                    json.dumps(feedback).encode("utf-8"), addr
                )
                self._tx_count += 1

                if self._rx_count % 250 == 0:
                    logger.info(
                        "Pakete RX:%d TX:%d | Pos: (%.1f, %.1f, %.1f)",
                        self._rx_count, self._tx_count,
                        *self._current_pos
                    )

            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break

        logger.info("Simulator beendet. RX:%d TX:%d",
                     self._rx_count, self._tx_count)
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description="ABB Robot EGM Simulator")
    parser.add_argument("--port", type=int, default=6511)
    parser.add_argument("--lag", type=float, default=4.0,
                        help="Simulierte Latenz in ms")
    parser.add_argument("--jitter", type=float, default=1.0,
                        help="Jitter (Standardabweichung) in ms")
    parser.add_argument("--tracking-error", type=float, default=0.3,
                        help="Tracking-Error-Offset in mm")
    args = parser.parse_args()

    sim = RobotSimulator(
        listen_port=args.port,
        lag_ms=args.lag,
        jitter_ms=args.jitter,
        tracking_error_mm=args.tracking_error,
    )
    sim.run()


if __name__ == "__main__":
    main()
