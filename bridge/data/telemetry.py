from __future__ import annotations
# telemetry.py — Einheitliches Logging und Telemetrie
#
# CodingPlan §B7: Telemetrie und Logging (Pflicht)
#
# CLOCK-FIX: Alle Streams nutzen bridge_now() als Zeitquelle.
#
# FIX: TX- und RX-CSV-Header auf tatsächlich geschriebene Felder
#   reduziert. e_value, extruder_age_ms, lookahead_ms sind in TODO.py
#   vorbereitet, aber noch nicht in log_tx()/log_rx() integriert —
#   daher aus dem Header entfernt um malformed CSVs zu verhindern.

import os
import csv
import json
import logging
from datetime import datetime
from typing import Optional, IO

from .clock import bridge_now

logger = logging.getLogger("egm.telemetry")


class TelemetryWriter:
    """
    Schreibt Telemetrie-Streams in CSV-Dateien.

    Streams:
      PLAN      — Geplante Trajektorie (Segmente aus Klipper)
      TX        — Gesendete EGM-Sollwerte (seq, pos, velocity, seg-info, t_klipper)
      RX        — Empfangene Ist-Werte vom Roboter (seq, robot_time, pos, quaternion)
      SYNC      — Offset, Normal/Tangential-Error, TCP-Error, Buffer-Metriken
      EVENT     — Zustandswechsel, Warnungen, Fehler
      ESTIMATOR — Closed-Loop-Latenz-Debug (pro RX-Packet, ~250Hz)

    Noch nicht integriert (vorbereitet in TODO.py):
      TX/RX e_value, extruder_age_ms, TX lookahead_ms
    """

    def __init__(self, log_dir: str, job_id: Optional[str] = None):
        self.log_dir = log_dir
        self.job_id = job_id or f"job_{datetime.now().strftime('%y%m%d_%H%M%S')}"
        self._job_dir = os.path.join(log_dir, self.job_id)

        self._files: dict[str, IO] = {}
        self._writers: dict[str, csv.writer] = {}
        self._counts: dict[str, int] = {}
        self._active = False

    def start(self):
        """Öffnet alle Telemetrie-Streams."""
        os.makedirs(self._job_dir, exist_ok=True)

        # PLAN Stream
        self._open_stream("plan", [
            "timestamp", "seg_nr", "print_time", "duration",
            "start_x", "start_y", "start_z",
            "end_x", "end_y", "end_z",
            "distance", "start_v", "cruise_v", "end_v",
        ])

        # TX Stream — gesendete Sollwerte
        # Felder die noch fehlen (TODO.py): e_value, extruder_age_ms, lookahead_ms
        self._open_stream("tx", [
            "timestamp", "seq_id", "x", "y", "z",
            "velocity", "seg_nr", "seg_progress", "t_klipper",
        ])

        # RX Stream — empfangene Ist-Werte
        # Felder die noch fehlen (TODO.py): e_value, extruder_age_ms
        self._open_stream("rx", [
            "timestamp", "seq_id", "robot_time",
            "x", "y", "z", "q0", "q1", "q2", "q3",
            "ext_value", "ext_age"
        ])

        # SYNC Stream
        self._open_stream("sync", [
            "timestamp", "error_tcp_pos",
            "error_tcp_speed", "tang_error", "norm_error",
            "t_delay_ms", "t_delay_output_ms",
            "buffer_depth", "buffer_time_s",
            "sync_level"
        ])

        # EVENT Stream
        self._open_stream("event", [
            "timestamp", "event_type", "severity", "message", "details",
        ])

        # ESTIMATOR Stream — Closed-Loop-Latenz-Debug (pro RX-Packet)
        self._open_stream("estimator", [
            "timestamp",
            "t_delay_ms",
            "t_delay_output_ms",
            "tang_err_mm",
            "norm_err_mm",
            "weight",
            "accel_detected",
            "ema_rate_used",
            "correction_ms"
        ])

        self._active = True

        self.log_event("JOB_START", "INFO",
                       f"Job {self.job_id} gestartet")
        logger.info("TELEMETRY: Streams geöffnet in %s", self._job_dir)

    def stop(self):
        """Schließt alle Streams."""
        if not self._active:
            return

        self.log_event("JOB_STOP", "INFO",
                       f"Job {self.job_id} beendet",
                       {"counts": dict(self._counts)})
        self._active = False

        for name, f in self._files.items():
            try:
                f.flush()
                f.close()
            except Exception:
                pass

        self._files.clear()
        self._writers.clear()

        logger.info("TELEMETRY: Streams geschlossen. Zeilen: %s",
                     self._counts)

    def _open_stream(self, name: str, headers: list[str]):
        path = os.path.join(self._job_dir, f"{name}.csv")
        f = open(path, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(headers)
        f.flush()
        self._files[name] = f
        self._writers[name] = writer
        self._counts[name] = 0

    # Low-volume Streams: sofort flushen.
    # ESTIMATOR ist NICHT hier drin — bei ~250Hz wären das 250 flushes/sec.
    _FLUSH_ALWAYS = {"plan", "event", "sync"}

    def _write(self, stream: str, row: list):
        if not self._active or stream not in self._writers:
            return
        self._writers[stream].writerow(row)
        self._counts[stream] = self._counts.get(stream, 0) + 1

        if stream in self._FLUSH_ALWAYS:
            self._files[stream].flush()
        elif self._counts[stream] % 50 == 0:
            self._files[stream].flush()

    # ── Stream-Methoden ──────────────────────────────────────

    def log_plan(self, seg):
        """Loggt ein geplantes Segment (PLAN Stream)."""
        self._write("plan", [
            f"{bridge_now():.6f}",
            seg.nr, f"{seg.print_time:.6f}", f"{seg.duration:.6f}",
            f"{seg.start_x:.4f}", f"{seg.start_y:.4f}",
            f"{seg.start_z:.4f}",
            f"{seg.end_x:.4f}", f"{seg.end_y:.4f}", f"{seg.end_z:.4f}",
            f"{seg.distance:.4f}",
            f"{seg.start_v:.4f}", f"{seg.cruise_v:.4f}",
            f"{seg.end_v:.4f}",
        ])

    def log_tx(self, sample):
        """Loggt einen gesendeten Sollwert (TX Stream).

        9 Felder — passend zum TX-Header.
        Für e_value / extruder_age_ms / lookahead_ms: siehe TODO.py.
        """
        self._write("tx", [
            f"{sample.timestamp:.6f}",
            sample.sequence_id,
            f"{sample.x:.3f}", f"{sample.y:.3f}", f"{sample.z:.3f}",
            f"{sample.velocity:.3f}",
            sample.segment_nr,
            f"{sample.segment_progress:.4f}",
            f"{sample.t_klipper:.6f}",
        ])

    def log_rx(self, feedback):
        """Loggt empfangenes Feedback (RX Stream).

        10 Felder — passend zum RX-Header.
        Für e_value / extruder_age_ms: siehe TODO.py.
        """
        self._write("rx", [
            f"{feedback.timestamp:.6f}",
            feedback.sequence_id,
            f"{feedback.robot_time:.3f}",
            f"{feedback.x:.3f}", f"{feedback.y:.3f}",
            f"{feedback.z:.3f}",
            f"{feedback.q0:.6f}", f"{feedback.q1:.6f}",
            f"{feedback.q2:.6f}", f"{feedback.q3:.6f}",
            f"{feedback.e:.3f}",f"{feedback.e_age:.3f}"
        ])

    def log_sync(self, metrics):
        """Loggt Sync-Metriken (SYNC Stream)."""
        self._write("sync", [
            f"{metrics.timestamp:.6f}",
            f"{metrics.error_tcp_pos:.3f}",
            f"{metrics.error_tcp_speed:.3f}",
            f"{metrics.tang_error:.3f}",
            f"{metrics.norm_error:.3f}",
            f"{metrics.t_delay_ms:.2f}",
            f"{metrics.t_delay_output_ms:.2f}",
            metrics.buffer_depth,
            f"{metrics.buffer_time_s:.3f}",
            metrics.sync_level.value
        ])

    def log_estimator(self, debug) -> None:
        """Loggt einen Estimator-Debug-Snapshot (ESTIMATOR Stream)."""
        self._write("estimator", [
            f"{debug.timestamp:.6f}",
            f"{debug.t_delay_ms:.3f}",
            f"{debug.t_delay_output_ms:.3f}",
            f"{debug.tang_err_mm:.3f}",
            f"{debug.norm_err_mm:.3f}",
            f"{debug.weight:.4f}",
            debug.accel_detected,
            f"{debug.ema_used:.3f}",
            f"{debug.correction_ms:.2f}"
        ])

    def log_event(self, event_type: str, severity: str,
                  message: str, details: Optional[dict] = None):
        """Loggt ein Event (EVENT Stream).

        Nutzt bridge_now() damit Event-Timestamps mit TX/RX/SYNC vergleichbar sind.
        """
        self._write("event", [
            f"{bridge_now():.6f}",
            event_type, severity, message,
            json.dumps(details) if details else "",
        ])

    def save_config_snapshot(self, config_dict: dict):
        """Speichert den Config-Snapshot für diesen Job (§B6)."""
        path = os.path.join(self._job_dir, "config_snapshot.json")
        with open(path, "w") as f:
            json.dump(config_dict, f, indent=2)
        logger.info("TELEMETRY: Config-Snapshot gespeichert")

    # ── Status ───────────────────────────────────────────────

    def snapshot(self) -> dict:
        return {
            "active": self._active,
            "job_id": self.job_id,
            "job_dir": self._job_dir,
            "row_counts": dict(self._counts),
        }
