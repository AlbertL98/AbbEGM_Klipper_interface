# move_export.py — Klipper Extra: Trapez-Segment-Export
#
# Zwei Ausgabekanäle:
#   1. CSV-Datei  → für Batch-Tests, wird von CsvSegmentSource gelesen
#   2. TCP-Socket → für Live-Betrieb, wird von TcpSegmentSource empfangen
#
# Beide Kanäle erzeugen dasselbe TrapezSegment-Format, sodass der
# segment_receiver.py identisch funktioniert — egal ob CSV oder TCP.
#
# Config:
#   [move_export]
#   output_path: /home/klippy/printer_data/logs/move_segments.csv
#   tcp_port: 7200
#   tcp_enabled: true
#   log_to_console: true

import os
import json
import socket
import logging
import threading


# CSV-Header — muss exakt zu TrapezSegment.from_csv_row() passen
CSV_HEADER = (
    'move_nr,'
    'print_time,'
    'move_duration,'
    'start_x,start_y,start_z,'
    'end_x,end_y,end_z,'
    'distance,'
    'start_v,cruise_v,end_v,'
    'accel,'
    'accel_time,cruise_time,decel_time,'
    'axes_r_x,axes_r_y,axes_r_z\n'
)


class MoveExport:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.output_path = config.get(
            'output_path',
            '/home/klippy/printer_data/logs/move_segments.csv'
        )
        self.log_to_console = config.getboolean('log_to_console', False)
        self.tcp_enabled = config.getboolean('tcp_enabled', True)
        self.tcp_port = config.getint('tcp_port', 7200)

        self.move_count = 0
        self.csv_file = None
        self.logger = logging.getLogger('move_export')

        # TCP State
        self.server_socket = None
        self.clients = []
        self.clients_lock = threading.Lock()

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )

    # ────────────────────────────────────────────────────────────────
    # Lifecycle
    # ────────────────────────────────────────────────────────────────

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self._open_csv()

        if self.tcp_enabled:
            self._start_tcp_server()

        # Monkey-Patch trapq_append
        self._orig_trapq_append = self.toolhead.trapq_append
        self.toolhead.trapq_append = self._capturing_trapq_append

    def _handle_disconnect(self):
        self._close_csv()
        self._stop_tcp_server()
        if hasattr(self, '_orig_trapq_append'):
            self.toolhead.trapq_append = self._orig_trapq_append

    # ────────────────────────────────────────────────────────────────
    # CSV
    # ────────────────────────────────────────────────────────────────

    def _open_csv(self):
        try:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            self.csv_file = open(self.output_path, 'w')
            self.csv_file.write(CSV_HEADER)
            self.csv_file.flush()
            self.logger.info("MoveExport: CSV geöffnet: %s", self.output_path)
        except Exception as e:
            self.logger.error("MoveExport: Kann CSV nicht öffnen: %s", e)
            self.csv_file = None

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.logger.info(
                "MoveExport: %d Moves exportiert nach %s",
                self.move_count, self.output_path
            )

    def _write_csv(self, nr, print_time, move_t,
                   start_x, start_y, start_z,
                   end_x, end_y, end_z, total_d,
                   start_v, cruise_v, end_v, accel,
                   accel_t, cruise_t, decel_t,
                   axes_r_x, axes_r_y, axes_r_z):
        if not self.csv_file:
            return
        self.csv_file.write(
            f'{nr},'
            f'{print_time:.6f},'
            f'{move_t:.6f},'
            f'{start_x:.4f},{start_y:.4f},{start_z:.4f},'
            f'{end_x:.4f},{end_y:.4f},{end_z:.4f},'
            f'{total_d:.4f},'
            f'{start_v:.4f},{cruise_v:.4f},{end_v:.4f},'
            f'{accel:.1f},'
            f'{accel_t:.6f},{cruise_t:.6f},{decel_t:.6f},'
            f'{axes_r_x:.6f},{axes_r_y:.6f},{axes_r_z:.6f}\n'
        )
        self.csv_file.flush()

    # ────────────────────────────────────────────────────────────────
    # TCP Server
    # ────────────────────────────────────────────────────────────────

    def _start_tcp_server(self):
        try:
            self.server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM
            )
            self.server_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(5)
            self.server_socket.settimeout(1.0)

            self._accept_thread = threading.Thread(
                target=self._accept_loop, daemon=True
            )
            self._accept_thread.start()

            self.logger.info(
                "MoveExport: TCP Server auf Port %d", self.tcp_port
            )
        except Exception as e:
            self.logger.error("MoveExport: TCP Server Fehler: %s", e)
            self.server_socket = None

    def _accept_loop(self):
        while self.server_socket:
            try:
                client, addr = self.server_socket.accept()
                with self.clients_lock:
                    self.clients.append(client)
                self.logger.info("MoveExport: Client verbunden: %s", addr)

                welcome = json.dumps({
                    'type': 'hello',
                    'version': 1,
                    'msg': 'Klipper MoveExport Stream'
                }) + '\n'
                try:
                    client.sendall(welcome.encode('utf-8'))
                except Exception:
                    pass
            except socket.timeout:
                continue
            except OSError:
                break

    def _stop_tcp_server(self):
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None

        with self.clients_lock:
            for c in self.clients:
                try:
                    c.close()
                except Exception:
                    pass
            self.clients.clear()

    def _send_to_clients(self, data_dict):
        if not self.clients:
            return
        line = json.dumps(data_dict) + '\n'
        raw = line.encode('utf-8')
        dead = []
        with self.clients_lock:
            for c in self.clients:
                try:
                    c.sendall(raw)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)
                try:
                    c.close()
                except Exception:
                    pass

    # ────────────────────────────────────────────────────────────────
    # trapq_append Wrapper
    # ────────────────────────────────────────────────────────────────

    def _capturing_trapq_append(self, trapq, print_time,
                                accel_t, cruise_t, decel_t,
                                start_x, start_y, start_z,
                                axes_r_x, axes_r_y, axes_r_z,
                                start_v, cruise_v, accel):
        self._export_move(
            print_time, accel_t, cruise_t, decel_t,
            start_x, start_y, start_z,
            axes_r_x, axes_r_y, axes_r_z,
            start_v, cruise_v, accel
        )
        return self._orig_trapq_append(
            trapq, print_time,
            accel_t, cruise_t, decel_t,
            start_x, start_y, start_z,
            axes_r_x, axes_r_y, axes_r_z,
            start_v, cruise_v, accel
        )

    # ────────────────────────────────────────────────────────────────
    # Export (gemeinsame Logik für CSV + TCP)
    # ────────────────────────────────────────────────────────────────

    def _export_move(self, print_time, accel_t, cruise_t, decel_t,
                     start_x, start_y, start_z,
                     axes_r_x, axes_r_y, axes_r_z,
                     start_v, cruise_v, accel):
        self.move_count += 1
        move_t = accel_t + cruise_t + decel_t
        end_v = max(0.0, cruise_v - accel * decel_t)

        # Distanz berechnen
        accel_d = start_v * accel_t + 0.5 * accel * accel_t ** 2
        cruise_d = cruise_v * cruise_t
        decel_d = cruise_v * decel_t - 0.5 * accel * decel_t ** 2
        total_d = accel_d + cruise_d + decel_d

        # Endposition
        end_x = start_x + axes_r_x * total_d
        end_y = start_y + axes_r_y * total_d
        end_z = start_z + axes_r_z * total_d

        # CSV schreiben
        self._write_csv(
            self.move_count, print_time, move_t,
            start_x, start_y, start_z,
            end_x, end_y, end_z, total_d,
            start_v, cruise_v, end_v, accel,
            accel_t, cruise_t, decel_t,
            axes_r_x, axes_r_y, axes_r_z
        )

        # TCP senden — gleiches Format wie TrapezSegment.to_dict()
        segment = {
            'type': 'segment',
            'nr': self.move_count,
            'print_time': round(print_time, 6),
            'duration': round(move_t, 6),
            'start': [round(start_x, 4), round(start_y, 4),
                      round(start_z, 4)],
            'end': [round(end_x, 4), round(end_y, 4), round(end_z, 4)],
            'distance': round(total_d, 4),
            'start_v': round(start_v, 4),
            'cruise_v': round(cruise_v, 4),
            'end_v': round(end_v, 4),
            'accel': round(accel, 1),
            'accel_t': round(accel_t, 6),
            'cruise_t': round(cruise_t, 6),
            'decel_t': round(decel_t, 6),
            'axes_r': [round(axes_r_x, 6), round(axes_r_y, 6),
                       round(axes_r_z, 6)]
        }
        self._send_to_clients(segment)

        if self.log_to_console:
            self.logger.info(
                "Move #%d: t=%.3f dur=%.4f "
                "(%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f) "
                "v=%.1f/%.1f/%.1f d=%.3f",
                self.move_count, print_time, move_t,
                start_x, start_y, start_z,
                end_x, end_y, end_z,
                start_v, cruise_v, end_v, total_d
            )


def load_config(config):
    return MoveExport(config)
