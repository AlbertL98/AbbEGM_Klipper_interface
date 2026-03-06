from __future__ import annotations
# telemetry.py — Einheitliches Logging und Telemetrie
#
# CodingPlan §B7: Telemetrie und Logging (Pflicht)
#   - Einheitliches Logschema mit Job-ID und Move-ID
#   - Streams: Planned, TX, RX, Extruder, Sync, Fault
#   - Präfixe für schnelle Filterung
#   - Export in CSV

import os
import csv
import time
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, IO

logger = logging.getLogger("egm.telemetry")


class TelemetryWriter:
    """
    Schreibt Telemetrie-Streams in CSV-Dateien.

    Streams (§B7):
      PLAN  — Geplante Trajektorie (Segmente aus Klipper)
      TX    — Gesendete EGM-Sollwerte (inkl. t_klipper für Debug)
      RX    — Empfangene Ist-Werte vom Roboter
      SYNC  — Tracking-Error, Lag, Jitter, Buffer-Metriken
      EVENT — Zustandswechsel, Warnungen, Fehler
    """

    def __init__(self, log_dir: str, job_id: Optional[str] = None):
        self.log_dir = log_dir
        self.job_id = job_id or f"job_{int(time.time())}"
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

        # TX Stream (an Roboter gesendete Samples)
        # t_klipper: die berechnete Klipper-Zeit für diesen Sample
        self._open_stream("tx", [
            "timestamp", "seq_id", "x", "y", "z",
            "velocity", "seg_nr", "seg_progress", "t_klipper",
        ])

        # RX Stream (vom Roboter empfangene Werte)
        self._open_stream("rx", [
            "timestamp", "seq_id", "robot_time",
            "x", "y", "z", "q0", "q1", "q2", "q3",
        ])

        # SYNC Stream
        self._open_stream("sync", [
            "timestamp", "tracking_error_mm",
            "tracking_x", "tracking_y", "tracking_z",
            "lag_ms", "jitter_p99_ms",
            "buffer_depth", "buffer_time_s",
            "sync_level",
        ])

        # EVENT Stream
        self._open_stream("event", [
            "timestamp", "event_type", "severity", "message", "details",
        ])

        self._active = True

        # Konfig-Snapshot speichern
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
        f.flush()  # Headers sofort auf Disk
        self._files[name] = f
        self._writers[name] = writer
        self._counts[name] = 0

    # Low-volume Streams die sofort geflusht werden
    _FLUSH_ALWAYS = {"plan", "event", "sync"}

    def _write(self, stream: str, row: list):
        if not self._active or stream not in self._writers:
            return
        self._writers[stream].writerow(row)
        self._counts[stream] = self._counts.get(stream, 0) + 1

        # Low-volume Streams: sofort flushen
        # High-volume (TX/RX): alle 50 Zeilen
        if stream in self._FLUSH_ALWAYS:
            self._files[stream].flush()
        elif self._counts[stream] % 50 == 0:
            self._files[stream].flush()

    # ── Stream-Methoden ──────────────────────────────────────

    def log_plan(self, seg):
        """Loggt ein geplantes Segment (PLAN Stream)."""
        self._write("plan", [
            f"{time.monotonic():.6f}",
            seg.nr, f"{seg.print_time:.6f}", f"{seg.duration:.6f}",
            f"{seg.start_x:.4f}", f"{seg.start_y:.4f}", f"{seg.start_z:.4f}",
            f"{seg.end_x:.4f}", f"{seg.end_y:.4f}", f"{seg.end_z:.4f}",
            f"{seg.distance:.4f}",
            f"{seg.start_v:.4f}", f"{seg.cruise_v:.4f}", f"{seg.end_v:.4f}",
        ])

    def log_tx(self, sample):
        """Loggt einen gesendeten Sollwert (TX Stream)."""
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
        """Loggt empfangenes Feedback (RX Stream)."""
        self._write("rx", [
            f"{feedback.timestamp:.6f}",
            feedback.sequence_id,
            f"{feedback.robot_time:.3f}",
            f"{feedback.x:.3f}", f"{feedback.y:.3f}", f"{feedback.z:.3f}",
            f"{feedback.q0:.6f}", f"{feedback.q1:.6f}",
            f"{feedback.q2:.6f}", f"{feedback.q3:.6f}",
        ])

    def log_sync(self, metrics):
        """Loggt Sync-Metriken (SYNC Stream)."""
        self._write("sync", [
            f"{metrics.timestamp:.6f}",
            f"{metrics.tracking_error_mm:.3f}",
            f"{metrics.tracking_error_x:.3f}",
            f"{metrics.tracking_error_y:.3f}",
            f"{metrics.tracking_error_z:.3f}",
            f"{metrics.lag_ms:.2f}",
            f"{metrics.jitter_p99_ms:.2f}",
            metrics.buffer_depth,
            f"{metrics.buffer_time_s:.3f}",
            metrics.sync_level.value,
        ])

    def log_event(self, event_type: str, severity: str,
                  message: str, details: Optional[dict] = None):
        """Loggt ein Event (EVENT Stream)."""
        self._write("event", [
            f"{time.monotonic():.6f}",
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
