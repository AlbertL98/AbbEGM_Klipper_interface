# move_export.py — Klipper Extra: Trapez-Segment-Export
# Patcht trapq_append() um finalisierte Moves als CSV zu exportieren.
# Kompatibel mit Klipper v0.13 (klippy/toolhead.py)
#
# Installation: In klippy/extras/ oder custom_extras/ ablegen
# Config:       [move_export] in printer.cfg
#
# Ausgabe: CSV mit allen finalisierten Move-Segmenten (Trapezprofile)

import os
import logging

class MoveExport:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.output_path = config.get(
            'output_path',
            '/home/klippy/printer_data/logs/move_segments.csv'
        )
        self.log_to_console = config.getboolean('log_to_console', False)
        self.move_count = 0
        self.csv_file = None
        self.logger = logging.getLogger('move_export')

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        # CSV öffnen
        try:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            self.csv_file = open(self.output_path, 'w')
            self.csv_file.write(
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
            self.csv_file.flush()
            self.logger.info("MoveExport: CSV geöffnet: %s", self.output_path)
        except Exception as e:
            self.logger.error("MoveExport: Kann CSV nicht öffnen: %s", e)
            self.csv_file = None

        # --- KERN: Monkey-Patch auf trapq_append ---
        # trapq_append wird in _process_lookahead() für jeden kinematischen
        # Move aufgerufen. Wir wrappen es, um die Argumente mitzuschreiben.
        self._orig_trapq_append = self.toolhead.trapq_append
        self.toolhead.trapq_append = self._capturing_trapq_append

    def _handle_disconnect(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.logger.info(
                "MoveExport: %d Moves exportiert nach %s",
                self.move_count, self.output_path
            )
        # Restore original
        if hasattr(self, '_orig_trapq_append'):
            self.toolhead.trapq_append = self._orig_trapq_append

    def _capturing_trapq_append(self, trapq, print_time,
                                accel_t, cruise_t, decel_t,
                                start_x, start_y, start_z,
                                axes_r_x, axes_r_y, axes_r_z,
                                start_v, cruise_v, accel):
        """Wrapper um trapq_append: Daten exportieren, dann Original aufrufen."""

        # Move exportieren
        self._export_move(
            print_time, accel_t, cruise_t, decel_t,
            start_x, start_y, start_z,
            axes_r_x, axes_r_y, axes_r_z,
            start_v, cruise_v, accel
        )

        # Original aufrufen — Klipper-Logik bleibt komplett unverändert
        return self._orig_trapq_append(
            trapq, print_time,
            accel_t, cruise_t, decel_t,
            start_x, start_y, start_z,
            axes_r_x, axes_r_y, axes_r_z,
            start_v, cruise_v, accel
        )

    def _export_move(self, print_time, accel_t, cruise_t, decel_t,
                     start_x, start_y, start_z,
                     axes_r_x, axes_r_y, axes_r_z,
                     start_v, cruise_v, accel):
        """Einen Move als CSV-Zeile schreiben."""
        self.move_count += 1

        move_t = accel_t + cruise_t + decel_t

        # end_v berechnen
        end_v = max(0.0, cruise_v - accel * decel_t)

        # Endposition berechnen
        accel_d = start_v * accel_t + 0.5 * accel * accel_t**2
        cruise_d = cruise_v * cruise_t
        decel_d = cruise_v * decel_t - 0.5 * accel * decel_t**2
        total_d = accel_d + cruise_d + decel_d

        end_x = start_x + axes_r_x * total_d
        end_y = start_y + axes_r_y * total_d
        end_z = start_z + axes_r_z * total_d

        if self.csv_file:
            self.csv_file.write(
                f'{self.move_count},'
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
