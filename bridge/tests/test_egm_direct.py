#!/usr/bin/env python3
# test_egm_direct.py — Vergleichstest: Unser EgmClient vs. workingEGM.py
#
# Macht EXAKT das was workingEGM.py macht (Kreisbahn),
# aber über unseren EgmClient. Wenn das funktioniert,
# wissen wir dass der Client korrekt ist.

import sys
import os
import time
import math
import logging

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d │ %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("test_egm")

from egm_bridge.egm_client import EgmClient, EgmTarget, EgmFeedback

# ── Parameter exakt wie workingEGM.py ────────────────────────
RADIUS_MM = 80.0
rad_s = 0.5
OMEGA = 2.0 * math.pi * rad_s
V_MAX_XY = 100.0
DT = 0.02  # 50 Hz

Q0 = 0.0
Q1 = -0.707106
Q2 = 0.707106
Q3 = 0.0

x_cmd = 0.0
y_cmd = 0.0
z_cmd = 10.0

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

# ── Feedback-Speicher ────────────────────────────────────────
last_feedback = None

def on_feedback(fb: EgmFeedback):
    global last_feedback
    last_feedback = fb

# ── Client starten ───────────────────────────────────────────
client = EgmClient(
    robot_ip="127.0.0.1",
    send_port=6599,
    recv_port=6510,
    local_send_port=6512,
    timeout_ms=100,
    watchdog_cycles=500,  # Kein schneller Timeout für Test
    protocol="auto",
    on_feedback=on_feedback,
)

if not client.connect():
    print("FEHLER: EGM-Verbindung fehlgeschlagen!")
    sys.exit(1)

print("EGM-Verbindung OK. Starte Kreisbahn wie workingEGM.py...")
print("Ctrl+C zum Stoppen.\n")

seq = 0
t0 = time.time()
print_counter = 0

try:
    while True:
        loop_start = time.perf_counter()
        t = time.time() - t0

        # 1) Zielpunkt (exakt wie workingEGM.py)
        x_target = RADIUS_MM * math.cos(OMEGA * t)
        y_target = RADIUS_MM * math.sin(OMEGA * t)
        z_target = 10.0

        x_target = clamp(x_target, -100.0, 100.0)
        y_target = clamp(y_target, -100.0, 100.0)
        z_target = max(0.0, z_target)

        # 2) Geschwindigkeitsbegrenzung
        dx = x_target - x_cmd
        dy = y_target - y_cmd
        dmax = V_MAX_XY * DT
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > dmax and dist > 1e-9:
            scale = dmax / dist
            dx *= scale
            dy *= scale
        x_cmd += dx
        y_cmd += dy
        z_cmd = max(0.0, z_target)

        x_cmd = clamp(x_cmd, -100.0, 100.0)
        y_cmd = clamp(y_cmd, -100.0, 100.0)

        # 3) Senden über unseren Client
        seq += 1
        target = EgmTarget(
            sequence_id=seq,
            timestamp=time.monotonic(),
            x=x_cmd,
            y=y_cmd,
            z=z_cmd,
            q0=Q0, q1=Q1, q2=Q2, q3=Q3,
        )
        client.send_target(target)

        # 4) Feedback ausgeben (wie workingEGM.py)
        print_counter += 1
        if print_counter % 50 == 0:
            if last_feedback:
                fb = last_feedback
                print(
                    f"SOLL  x={x_cmd:7.2f} y={y_cmd:7.2f} z={z_cmd:6.2f} "
                    f"| IST  x={fb.x:7.2f} y={fb.y:7.2f} z={fb.z:6.2f} "
                    f"| RX:{client.stats.rx_count} "
                    f"RTT:{client.stats.rtt_avg_ms:.1f}ms"
                )
            else:
                print(
                    f"SOLL  x={x_cmd:7.2f} y={y_cmd:7.2f} z={z_cmd:6.2f} "
                    f"| IST  (kein Feedback) "
                    f"| TX:{client.stats.tx_count} "
                    f"Timeouts:{client.stats.rx_timeouts}"
                )

        # 5) Timing
        elapsed = time.perf_counter() - loop_start
        sleep_t = DT - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

except KeyboardInterrupt:
    print("\nGestoppt.")
finally:
    client.disconnect()
    print(f"TX:{client.stats.tx_count} RX:{client.stats.rx_count} "
          f"Errors:{client.stats.tx_errors}")
