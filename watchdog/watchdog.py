# TODO: nich ausreichend getetstet, extruder verküpung myb fehlerhaft, log file wird einfach abgelgegt
import csv
import json
import os
import socket
import threading
import time

import egm_pb2


LOCAL_RECV_PORT = 6510
KLIPPY_SOCKET = "/tmp/klippy_uds"
LOG_PATH = "egm_extruder_log.csv"

# Shared state for latest extruder value from Klipper
state_lock = threading.Lock()
latest_e = None
latest_e_eventtime = None      # Klipper eventtime
latest_e_recv_mono = None      # local monotonic receive time


def klipper_subscriber():
    """
    Subscribes to Klipper toolhead.position and keeps the latest E component.
    """
    global latest_e, latest_e_eventtime, latest_e_recv_mono

    if not os.path.exists(KLIPPY_SOCKET):
        print(f"[Klipper] Socket not found: {KLIPPY_SOCKET}")
        print("[Klipper] Make sure Klipper API server is enabled.")
        return

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(KLIPPY_SOCKET)
    sock.settimeout(1.0)

    def send_msg(obj):
        payload = json.dumps(obj).encode("utf-8") + b"\x03"
        sock.sendall(payload)

    # Subscribe to toolhead.position
    send_msg({
        "id": 1,
        "method": "objects/subscribe",
        "params": {
            "objects": {
                "toolhead": ["position"]
            },
            "response_template": {}
        }
    })

    buffer = b""

    while True:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                print("[Klipper] Connection closed.")
                break
            buffer += chunk

            while b"\x03" in buffer:
                raw, buffer = buffer.split(b"\x03", 1)
                if not raw:
                    continue

                msg = json.loads(raw.decode("utf-8", errors="replace"))

                # Initial reply:
                # {"id":1, "result":{"status":{"toolhead":{"position":[...]}},"eventtime":...}}
                # Async updates:
                # {"params":{"status":{"toolhead":{"position":[...]}},"eventtime":...}}

                if "result" in msg:
                    status = msg["result"].get("status", {})
                    eventtime = msg["result"].get("eventtime")
                elif "params" in msg:
                    status = msg["params"].get("status", {})
                    eventtime = msg["params"].get("eventtime")
                else:
                    continue

                toolhead = status.get("toolhead", {})
                pos = toolhead.get("position")
                if not pos:
                    continue

                # In Klipper API, toolhead.position is a list.
                # X,Y,Z are first three components. E is also present as extra axis.
                # In a standard XYZ+E setup, E is usually index 3.
                if len(pos) >= 4:
                    e_val = float(pos[3])
                    recv_mono = time.monotonic()

                    with state_lock:
                        latest_e = e_val
                        latest_e_eventtime = eventtime
                        latest_e_recv_mono = recv_mono

        except socket.timeout:
            continue
        except Exception as exc:
            print(f"[Klipper] Error: {exc}")
            break


def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", LOCAL_RECV_PORT))

    print(f"Listening for incoming EGM data on UDP :{LOCAL_RECV_PORT} (Ctrl+C to stop)")
    print(f"Logging to: {LOG_PATH}")

    t = threading.Thread(target=klipper_subscriber, daemon=True)
    t.start()

    last_log_mono = None

    with open(LOG_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "host_mono_s",
            "dt_ms",
            "egm_seq",
            "egm_tm_ms",
            "robot_x",
            "robot_y",
            "robot_z",
            "extruder_e",
            "extruder_age_ms",
            "klipper_eventtime"
        ])

        try:
            while True:
                data, addr = rx.recvfrom(4096)
                now_mono = time.monotonic()

                fb = egm_pb2.EgmRobot()
                fb.ParseFromString(data)

                x = fb.feedBack.cartesian.pos.x
                y = fb.feedBack.cartesian.pos.y
                z = fb.feedBack.cartesian.pos.z
                seq = fb.header.seqno
                tm = fb.header.tm

                with state_lock:
                    e_val = latest_e
                    e_evt = latest_e_eventtime
                    e_recv = latest_e_recv_mono

                dt_ms = None if last_log_mono is None else (now_mono - last_log_mono) * 1000.0
                e_age_ms = None if e_recv is None else (now_mono - e_recv) * 1000.0
                last_log_mono = now_mono

                writer.writerow([
                    f"{now_mono:.6f}",
                    "" if dt_ms is None else f"{dt_ms:.3f}",
                    seq,
                    tm,
                    f"{x:.4f}",
                    f"{y:.4f}",
                    f"{z:.4f}",
                    "" if e_val is None else f"{e_val:.6f}",
                    "" if e_age_ms is None else f"{e_age_ms:.3f}",
                    "" if e_evt is None else f"{e_evt:.6f}",
                ])
                f.flush()

                print(
                    f"{addr[0]}:{addr[1]} | seq={seq} tm={tm}ms | "
                    f"x={x:8.2f} y={y:8.2f} z={z:7.2f} | "
                    f"e={e_val if e_val is not None else 'n/a'} | "
                    f"e_age_ms={e_age_ms:.2f}" if e_age_ms is not None else
                    f"{addr[0]}:{addr[1]} | seq={seq} tm={tm}ms | "
                    f"x={x:8.2f} y={y:8.2f} z={z:7.2f} | e=n/a"
                )

        except KeyboardInterrupt:
            print("Stopped.")


if __name__ == "__main__":
    main()