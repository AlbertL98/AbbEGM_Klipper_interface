#!/usr/bin/env python3
# fake_klipper.py — Simuliert move_export.py TCP-Output
#
# Sendet Trapez-Segmente über TCP, genau wie move_export.py
# Für Stufe-1-Tests ohne echten Klipper.
#
# Nutzung:
#   python fake_klipper.py                     # Default: Quadrat 100x100mm
#   python fake_klipper.py --speed 20          # 20 mm/s
#   python fake_klipper.py --pattern line      # Gerade Linie statt Quadrat

import argparse
import json
import math
import socket
import time
import logging
import threading

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d │ FAKE-KLIPPER │ %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("fake_klipper")


def make_line_segments(speed=10.0, length=100.0, accel=500.0):
    """Einfache Linie in X-Richtung."""
    segments = []

    # Beschleunigung
    accel_t = speed / accel
    accel_d = 0.5 * accel * accel_t ** 2
    # Konstantfahrt
    cruise_d = length - 2 * accel_d
    cruise_t = cruise_d / speed if cruise_d > 0 else 0.0
    # Bremsen
    decel_t = accel_t

    total_d = accel_d + cruise_d + accel_d
    total_t = accel_t + cruise_t + decel_t

    segments.append({
        "type": "segment",
        "nr": 1,
        "print_time": 0.0,
        "duration": round(total_t, 6),
        "start": [0.0, 0.0, 50.0],
        "end": [round(total_d, 4), 0.0, 50.0],
        "distance": round(total_d, 4),
        "start_v": 0.0,
        "cruise_v": speed,
        "end_v": 0.0,
        "accel": accel,
        "accel_t": round(accel_t, 6),
        "cruise_t": round(cruise_t, 6),
        "decel_t": round(decel_t, 6),
        "axes_r": [1.0, 0.0, 0.0],
    })
    return segments


def make_square_segments(speed=10.0, side=100.0, accel=500.0, z=50.0):
    """Quadrat: 4 Seiten mit Beschleunigung/Bremsen."""
    segments = []
    # Eckpunkte
    corners = [
        ([0.0, 0.0, z], [side, 0.0, z], [1.0, 0.0, 0.0]),     # → rechts
        ([side, 0.0, z], [side, side, z], [0.0, 1.0, 0.0]),     # ↑ hoch
        ([side, side, z], [0.0, side, z], [-1.0, 0.0, 0.0]),    # ← links
        ([0.0, side, z], [0.0, 0.0, z], [0.0, -1.0, 0.0]),     # ↓ runter
    ]

    print_time = 0.0
    for i, (start, end, axes_r) in enumerate(corners):
        accel_t = speed / accel
        accel_d = 0.5 * accel * accel_t ** 2
        cruise_d = side - 2 * accel_d
        cruise_t = cruise_d / speed if cruise_d > 0 else 0.0
        decel_t = accel_t

        total_t = accel_t + cruise_t + decel_t

        segments.append({
            "type": "segment",
            "nr": i + 1,
            "print_time": round(print_time, 6),
            "duration": round(total_t, 6),
            "start": [round(v, 4) for v in start],
            "end": [round(v, 4) for v in end],
            "distance": round(side, 4),
            "start_v": 0.0,
            "cruise_v": speed,
            "end_v": 0.0,
            "accel": accel,
            "accel_t": round(accel_t, 6),
            "cruise_t": round(cruise_t, 6),
            "decel_t": round(decel_t, 6),
            "axes_r": axes_r,
        })
        print_time += total_t

    return segments


def make_zigzag_segments(speed=10.0, width=80.0, step=10.0,
                         layers=5, accel=500.0, z=50.0):
    """Zickzack-Muster (typisch für FDM-Infill)."""
    segments = []
    print_time = 0.0
    nr = 0

    for layer in range(layers):
        y = layer * step

        # Hin (→)
        nr += 1
        t = width / speed  # Vereinfacht: konstante Geschwindigkeit
        segments.append({
            "type": "segment",
            "nr": nr,
            "print_time": round(print_time, 6),
            "duration": round(t, 6),
            "start": [0.0, round(y, 4), z],
            "end": [round(width, 4), round(y, 4), z],
            "distance": round(width, 4),
            "start_v": speed,
            "cruise_v": speed,
            "end_v": speed,
            "accel": 0.0,
            "accel_t": 0.0,
            "cruise_t": round(t, 6),
            "decel_t": 0.0,
            "axes_r": [1.0, 0.0, 0.0],
        })
        print_time += t

        if layer < layers - 1:
            # Step nach oben (↑)
            nr += 1
            t_step = step / speed
            segments.append({
                "type": "segment",
                "nr": nr,
                "print_time": round(print_time, 6),
                "duration": round(t_step, 6),
                "start": [round(width, 4), round(y, 4), z],
                "end": [round(width, 4), round(y + step, 4), z],
                "distance": round(step, 4),
                "start_v": speed * 0.5,
                "cruise_v": speed * 0.5,
                "end_v": speed * 0.5,
                "accel": 0.0,
                "accel_t": 0.0,
                "cruise_t": round(t_step, 6),
                "decel_t": 0.0,
                "axes_r": [0.0, 1.0, 0.0],
            })
            print_time += t_step

            # Zurück (←)
            nr += 1
            segments.append({
                "type": "segment",
                "nr": nr,
                "print_time": round(print_time, 6),
                "duration": round(t, 6),
                "start": [round(width, 4), round(y + step, 4), z],
                "end": [0.0, round(y + step, 4), z],
                "distance": round(width, 4),
                "start_v": speed,
                "cruise_v": speed,
                "end_v": speed,
                "accel": 0.0,
                "accel_t": 0.0,
                "cruise_t": round(t, 6),
                "decel_t": 0.0,
                "axes_r": [-1.0, 0.0, 0.0],
            })
            print_time += t

            if layer < layers - 2:
                # Step nach oben (↑)
                nr += 1
                segments.append({
                    "type": "segment",
                    "nr": nr,
                    "print_time": round(print_time, 6),
                    "duration": round(t_step, 6),
                    "start": [0.0, round(y + step, 4), z],
                    "end": [0.0, round(y + 2 * step, 4), z],
                    "distance": round(step, 4),
                    "start_v": speed * 0.5,
                    "cruise_v": speed * 0.5,
                    "end_v": speed * 0.5,
                    "accel": 0.0,
                    "accel_t": 0.0,
                    "cruise_t": round(t_step, 6),
                    "decel_t": 0.0,
                    "axes_r": [0.0, 1.0, 0.0],
                })
                print_time += t_step

    return segments


class FakeKlipperServer:
    """TCP-Server der Segmente wie move_export.py sendet."""

    def __init__(self, port: int = 7200):
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", port))
        self.server.listen(5)
        self.server.settimeout(1.0)
        self.clients = []

    def wait_for_client(self, timeout=60):
        """Wartet auf eine Verbindung von der Bridge."""
        logger.info("Warte auf Bridge-Verbindung auf Port %d...", self.port)
        start = time.time()
        while time.time() - start < timeout:
            try:
                client, addr = self.server.accept()
                logger.info("Bridge verbunden: %s", addr)

                # Welcome (wie move_export.py)
                hello = json.dumps({
                    "type": "hello",
                    "version": 1,
                    "msg": "Fake Klipper MoveExport Stream"
                }) + "\n"
                client.sendall(hello.encode("utf-8"))

                self.clients.append(client)
                return True
            except socket.timeout:
                continue
        logger.error("Timeout — keine Bridge verbunden")
        return False

    def send_segments(self, segments, realtime=True, speed_factor=1.0):
        """Sendet Segmente an alle verbundenen Clients."""
        if not self.clients:
            logger.error("Keine Clients verbunden!")
            return

        prev_time = None
        for seg in segments:
            # Realtime-Delay
            if realtime and prev_time is not None:
                delay = (seg["print_time"] - prev_time) / speed_factor
                if delay > 0:
                    time.sleep(delay)

            line = json.dumps(seg) + "\n"
            raw = line.encode("utf-8")

            dead = []
            for c in self.clients:
                try:
                    c.sendall(raw)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(c)

            for c in dead:
                self.clients.remove(c)
                logger.warning("Client getrennt")

            prev_time = seg["print_time"]

            # Log
            start = seg["start"]
            end = seg["end"]
            logger.info(
                "Segment #%d: (%.1f,%.1f,%.1f)→(%.1f,%.1f,%.1f) "
                "v=%.1f d=%.1f t=%.3fs",
                seg["nr"],
                start[0], start[1], start[2],
                end[0], end[1], end[2],
                seg["cruise_v"], seg["distance"], seg["duration"],
            )

        logger.info("Alle %d Segmente gesendet", len(segments))

    def close(self):
        for c in self.clients:
            try:
                c.close()
            except Exception:
                pass
        self.server.close()


def main():
    parser = argparse.ArgumentParser(
        description="Fake Klipper — simuliert move_export.py TCP-Output"
    )
    parser.add_argument("--port", type=int, default=7200)
    parser.add_argument("--speed", type=float, default=10.0,
                        help="Druckgeschwindigkeit mm/s")
    parser.add_argument("--accel", type=float, default=500.0,
                        help="Beschleunigung mm/s²")
    parser.add_argument("--pattern", default="square",
                        choices=["line", "square", "zigzag"],
                        help="Bewegungsmuster")
    parser.add_argument("--repeat", type=int, default=1,
                        help="Muster N-mal wiederholen")
    parser.add_argument("--speed-factor", type=float, default=1.0,
                        help="Zeitfaktor (2.0 = doppelt so schnell)")
    parser.add_argument("--no-realtime", action="store_true",
                        help="Alle Segmente sofort senden (kein Delay)")
    args = parser.parse_args()

    # Segmente erzeugen
    if args.pattern == "line":
        segments = make_line_segments(speed=args.speed, accel=args.accel)
    elif args.pattern == "square":
        segments = make_square_segments(speed=args.speed, accel=args.accel)
    elif args.pattern == "zigzag":
        segments = make_zigzag_segments(speed=args.speed, accel=args.accel)

    # Wiederholen
    if args.repeat > 1:
        all_segs = []
        time_offset = 0.0
        nr_offset = 0
        for r in range(args.repeat):
            for seg in segments:
                s = dict(seg)
                s["nr"] = nr_offset + seg["nr"]
                s["print_time"] = seg["print_time"] + time_offset
                all_segs.append(s)
            if segments:
                last = segments[-1]
                time_offset += last["print_time"] + last["duration"]
                nr_offset += len(segments)
        segments = all_segs

    logger.info("Pattern: %s | %d Segmente | Speed: %.1f mm/s | Accel: %.1f",
                args.pattern, len(segments), args.speed, args.accel)

    # Gesamtdauer
    total_time = sum(s["duration"] for s in segments)
    total_dist = sum(s["distance"] for s in segments)
    logger.info("Gesamtdauer: %.2fs | Gesamtdistanz: %.1fmm", total_time, total_dist)

    # Server starten
    server = FakeKlipperServer(port=args.port)

    try:
        if not server.wait_for_client(timeout=30):
            return

        logger.info("═══ Starte Segment-Stream ═══")
        time.sleep(0.5)  # Kurz warten damit Bridge bereit ist

        server.send_segments(
            segments,
            realtime=not args.no_realtime,
            speed_factor=args.speed_factor,
        )

        # Kurz warten damit Bridge letzte Segmente verarbeiten kann
        logger.info("Warte 3s auf Verarbeitung...")
        time.sleep(3.0)

    except KeyboardInterrupt:
        logger.info("Abgebrochen")
    finally:
        server.close()
        logger.info("Server geschlossen")


if __name__ == "__main__":
    main()
