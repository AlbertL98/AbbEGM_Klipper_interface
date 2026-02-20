# bridge.py — EGM-Bridge-Core Orchestrator
#
# Zentrale Steuerung: Verbindet alle Komponenten und
# betreibt den EGM-Zyklusloop.
#
# Architektur (CodingPlan §A):
#   Klipper ist Motion-Master → Bridge ist Übersetzer → ABB führt aus
#
# Datenfluss:
#   Klipper (move_export.py)
#     ↓ TCP (TrapezSegmente)
#   SegmentReceiver
#     ↓ Plan-Queue
#   TrajectoryPlanner (Interpolation)
#     ↓ EGM-Samples (4ms Takt)
#   EgmClient (UDP)
#     ↓ Sollwerte
#   ABB Controller
#     ↓ Feedback (UDP)
#   SyncMonitor (Bewertung)
#     ↓ Korrektur / State-Änderung
#   StateMachine

import time
import logging
import threading
import signal
from typing import Optional

from .state_machine import BridgeStateMachine, State
from .config import BridgeConfig, validate_config
from .segment_source import TrapezSegment, TcpSegmentReceiver
from .trajectory_planner import TrajectoryPlanner, EgmSample
from .egm_client import EgmClient, EgmTarget, EgmFeedback
from .sync_monitor import SyncMonitor, SyncLevel
from .telemetry import TelemetryWriter

logger = logging.getLogger("egm.bridge")


class EgmBridge:
    """
    Haupt-Orchestrator des EGM-Bridge-Core.

    Lifecycle:
      1. __init__() → Konfiguration, Komponenten erzeugen
      2. start()    → Verbindungen herstellen, INIT → READY
      3. run_job()  → READY → RUN, EGM-Loop starten
      4. stop()     → RUN → STOP, alles herunterfahren
    """

    def __init__(self, config: BridgeConfig):
        self.cfg = config
        self.sm = BridgeStateMachine()

        # Zykluszeit
        self._cycle_s = config.connection.cycle_ms / 1000.0

        # Komponenten
        self.planner = TrajectoryPlanner(
            cycle_s=self._cycle_s,
            max_queue_size=config.queues.plan_queue_size,
            correction_max_mm=config.sync.correction_max_mm,
            correction_rate_limit=config.sync.correction_rate_limit_mm_per_s,
        )

        self.egm = EgmClient(
            robot_ip=config.connection.robot_ip,
            send_port=config.connection.send_port,
            recv_port=config.connection.recv_port,
            local_send_port=config.connection.local_send_port,
            timeout_ms=config.connection.timeout_ms,
            watchdog_cycles=config.connection.watchdog_cycles,
            protocol=config.connection.protocol,
            default_q0=config.connection.default_q0,
            default_q1=config.connection.default_q1,
            default_q2=config.connection.default_q2,
            default_q3=config.connection.default_q3,
            on_feedback=self._on_feedback,
            on_timeout=self._on_egm_timeout,
        )

        self.sync = SyncMonitor(config=config.sync)
        self.sync.set_level_change_callback(self._on_sync_level_change)

        self.telemetry = TelemetryWriter(
            log_dir=config.telemetry.log_dir,
        )

        self.receiver: Optional[TcpSegmentReceiver] = None

        # Interner Zustand
        self._loop_thread: Optional[threading.Thread] = None
        self._running = False
        self._last_sample: Optional[EgmSample] = None
        self._last_feedback: Optional[EgmFeedback] = None
        self._feedback_lock = threading.Lock()

        # End-of-Job-Erkennung
        self._last_segment_time: float = 0.0
        self._starvation_timeout_s: float = 3.0  # Job fertig nach 3s ohne neue Segmente

        # Metriken
        self._loop_count = 0
        self._loop_overruns = 0
        self._shutdown_done = False

    # ── Lifecycle ────────────────────────────────────────────

    def start(self) -> bool:
        """
        Initialisiert Verbindungen.
        INIT → READY (oder FAULT bei Fehlern).
        """
        logger.info("BRIDGE: Starte...")

        # Config validieren (§B6)
        errors = validate_config(self.cfg)
        if errors:
            for e in errors:
                logger.error("BRIDGE: Config-Fehler: %s", e)
            self.sm.to_fault("Config-Validierung fehlgeschlagen",
                             {"errors": errors})
            return False

        # EGM-Verbindung
        if not self.egm.connect():
            self.sm.to_fault("EGM-Verbindung fehlgeschlagen")
            return False

        # Segment-Empfänger starten
        self.receiver = TcpSegmentReceiver(
            host=self.cfg.klipper.tcp_host,
            port=self.cfg.klipper.tcp_port,
            on_segment=self._on_segment_received,
            reconnect_interval=self.cfg.klipper.reconnect_interval_s,
            receive_timeout=self.cfg.klipper.receive_timeout_s,
        )
        self.receiver.start()

        # Telemetrie starten
        self.telemetry.start()
        self.telemetry.save_config_snapshot(self.cfg.snapshot())

        self.sm.to_ready("Alle Verbindungen hergestellt")
        logger.info("BRIDGE: Bereit (Zyklus: %.1fms, Profil: %s)",
                     self.cfg.connection.cycle_ms, self.cfg.profile_name)
        return True

    def run_job(self, job_id: Optional[str] = None):
        """
        Startet den EGM-Zyklusloop.
        READY → RUN
        """
        if self.sm.state != State.READY:
            logger.error("BRIDGE: Kann Job nur aus READY starten "
                         "(aktuell: %s)", self.sm.state.value)
            return False

        if job_id:
            self.telemetry.log_event("JOB_START", "INFO",
                                     f"Job {job_id} gestartet")

        self.sync.reset()
        self._last_segment_time = time.monotonic()  # Startzeitpunkt
        self.sm.to_run(f"Job gestartet: {job_id or 'default'}")

        self._running = True
        self._loop_thread = threading.Thread(
            target=self._egm_loop, daemon=True, name="egm-loop"
        )
        self._loop_thread.start()

        logger.info("BRIDGE: Job läuft")
        return True

    def stop(self, reason: str = "Manueller Stop"):
        """
        Stoppt den laufenden Job, behält aber Verbindungen offen.
        Für reset_to_ready() → nächster Job kann sofort starten.
        """
        logger.info("BRIDGE: Stoppe Job... (%s)", reason)
        self._running = False

        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)
            self._loop_thread = None

        self.sm.to_stop(reason)
        self.planner.clear()

        logger.info("BRIDGE: Job gestoppt (Zyklen: %d, Overruns: %d)",
                     self._loop_count, self._loop_overruns)

    def reset_to_ready(self):
        """
        Setzt die Bridge nach einem Job zurück in den READY-Zustand.
        Verbindungen bleiben aktiv — wartet auf nächsten Druckjob.

        Ablauf: STOP → READY (Planner, Metriken, Sync werden resettet)
        """
        if self.sm.state == State.READY:
            logger.debug("BRIDGE: Bereits in READY")
            return True

        if self.sm.state not in (State.STOP,):
            logger.warning("BRIDGE: reset_to_ready() nur aus STOP möglich "
                           "(aktuell: %s)", self.sm.state.value)
            return False

        # Planner komplett zurücksetzen (Queue + Zähler)
        self.planner.reset()

        # Sync-Monitor zurücksetzen
        self.sync.reset()

        # Metriken zurücksetzen
        self._loop_count = 0
        self._loop_overruns = 0
        self._last_sample = None
        self._last_feedback = None
        self._last_segment_time = 0.0

        # Telemetrie: Neuen Job vorbereiten
        self.telemetry.new_session()

        # State Machine: STOP → READY
        if not self.sm.to_ready("Reset für nächsten Job"):
            logger.error("BRIDGE: Transition STOP → READY fehlgeschlagen")
            return False

        logger.info("BRIDGE: Zurück in READY — warte auf nächsten Druckjob")
        return True

    def shutdown(self):
        """Komplettes Herunterfahren inkl. Cleanup — aus JEDEM Zustand."""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        logger.info("BRIDGE: Shutdown aus Zustand %s", self.sm.state.value)
        self._running = False

        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)

        # Verbindungen schließen (unabhängig vom State)
        if self.receiver:
            self.receiver.stop()
        self.egm.disconnect()
        self.telemetry.stop()
        self.planner.clear()

        logger.info("BRIDGE: Shutdown abgeschlossen (Zyklen: %d, Overruns: %d)",
                     self._loop_count, self._loop_overruns)

    # ── EGM Cycle Loop ───────────────────────────────────────

    def _egm_loop(self):
        """
        Hauptloop: Läuft im festen EGM-Zyklustakt.

        Jeder Zyklus:
          1. Sample aus Planner holen (Interpolation)
          2. An EGM-Client senden
          3. Sync-Monitor updaten
          4. Telemetrie schreiben
          5. Bei Bedarf: Korrektur, State-Change
        """
        logger.info("BRIDGE: EGM-Loop gestartet (Zyklus: %.1fms)",
                     self._cycle_s * 1000)

        cycle_s = self._cycle_s
        metric_interval = self.cfg.telemetry.metric_interval_s
        last_metric_time = time.monotonic()
        starvation_logged = False

        while self._running and self.sm.is_running:
            loop_start = time.perf_counter()
            bridge_time = time.monotonic()
            self._loop_count += 1

            try:
                # 1. Sample erzeugen
                sample = self.planner.next_sample(bridge_time)

                if sample:
                    # ── Normaler Betrieb ─────────────────────────
                    starvation_logged = False
                    self._last_sample = sample

                    # 2. An Roboter senden
                    target = EgmTarget(
                        sequence_id=sample.sequence_id,
                        timestamp=sample.timestamp,
                        x=sample.x,
                        y=sample.y,
                        z=sample.z,
                        q0=self.cfg.connection.default_q0,
                        q1=self.cfg.connection.default_q1,
                        q2=self.cfg.connection.default_q2,
                        q3=self.cfg.connection.default_q3,
                    )
                    self.egm.send_target(target)
                    self.telemetry.log_tx(sample)

                    # 3. Sync updaten (NUR bei echten Samples)
                    with self._feedback_lock:
                        fb = self._last_feedback

                    if fb:
                        self.telemetry.log_rx(fb)
                        self.sync.update(
                            sample=sample,
                            feedback=fb,
                            buffer_depth=self.planner.queue_depth,
                            buffer_time_s=self.planner.queue_time_s,
                        )

                    # 4. Periodische Metriken
                    if bridge_time - last_metric_time >= metric_interval:
                        self.telemetry.log_sync(self.sync.metrics)
                        last_metric_time = bridge_time

                    # 5. Sync-Level-Reaktion
                    self._handle_sync_level()

                else:
                    # ── Buffer leer — Starvation ─────────────────
                    if not starvation_logged:
                        logger.info("BRIDGE: Buffer leer — halte Position")
                        starvation_logged = True

                    # Job-Ende prüfen
                    time_since_seg = (
                        bridge_time - self._last_segment_time
                        if self._last_segment_time > 0 else 0
                    )
                    if time_since_seg > self._starvation_timeout_s:
                        logger.info(
                            "BRIDGE: Print abgeschlossen "
                            "(keine neuen Segmente seit %.1fs)",
                            time_since_seg)
                        self.telemetry.log_event(
                            "JOB_COMPLETE", "INFO",
                            f"Abgeschlossen: {self._loop_count} Zyklen, "
                            f"{self.egm.stats.tx_count} TX")
                        self._running = False
                        break

                    # Letzte Position halten (wichtig für Roboter!)
                    if self._last_sample:
                        hold = EgmTarget(
                            sequence_id=self._loop_count,
                            timestamp=bridge_time,
                            x=self._last_sample.x,
                            y=self._last_sample.y,
                            z=self._last_sample.z,
                            q0=self.cfg.connection.default_q0,
                            q1=self.cfg.connection.default_q1,
                            q2=self.cfg.connection.default_q2,
                            q3=self.cfg.connection.default_q3,
                        )
                        self.egm.send_target(hold)

                    # KEIN Sync-Update während Starvation!
                    # (verhindert falschen Lag-Aufbau → FAULT)

            except Exception as e:
                logger.error("BRIDGE: Loop-Fehler: %s", e, exc_info=True)
                self.telemetry.log_event("LOOP_ERROR", "ERROR", str(e))
                self.sm.to_fault(f"Loop-Fehler: {e}")
                break

            # Zykluszeit einhalten
            elapsed = time.perf_counter() - loop_start
            sleep_time = cycle_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self._loop_overruns += 1
                if self._loop_overruns % 100 == 1:
                    logger.warning(
                        "BRIDGE: Zyklus-Overrun #%d "
                        "(%.2fms > %.2fms)",
                        self._loop_overruns,
                        elapsed * 1000,
                        cycle_s * 1000
                    )

        logger.info("BRIDGE: EGM-Loop beendet")

        # Sauberer Zustandswechsel wenn Job fertig (nicht FAULT)
        if self.sm.state in (State.RUN, State.DEGRADED):
            self.sm.to_stop("Job abgeschlossen")

    # ── Callbacks ────────────────────────────────────────────

    def _on_segment_received(self, seg: TrapezSegment):
        """Callback: Neues Segment von Klipper empfangen."""
        self._last_segment_time = time.monotonic()
        self.planner.add_segment(seg)
        self.telemetry.log_plan(seg)

    def _on_feedback(self, feedback: EgmFeedback):
        """Callback: Feedback vom Roboter empfangen."""
        with self._feedback_lock:
            self._last_feedback = feedback
        self.telemetry.log_rx(feedback)

    def _on_egm_timeout(self):
        """Callback: EGM-Watchdog ausgelöst."""
        self.telemetry.log_event("EGM_TIMEOUT", "CRITICAL",
                                 "Watchdog — keine Antwort vom Roboter")
        self.sm.to_fault("EGM-Watchdog-Timeout")

    def _on_sync_level_change(self, level: SyncLevel, reason: str):
        """Callback: Sync-Level hat sich geändert."""
        severity = {
            SyncLevel.OK: "INFO",
            SyncLevel.WARN: "WARNING",
            SyncLevel.DEGRADE: "WARNING",
            SyncLevel.STOP: "CRITICAL",
        }[level]
        self.telemetry.log_event(
            f"SYNC_{level.value}", severity, reason
        )

    # ── Reaktionen ───────────────────────────────────────────

    def _handle_buffer_underflow(self):
        """Reagiert auf leeren Buffer (§B2)."""
        now = time.monotonic()

        # Job-Ende erkennen: Buffer leer + keine neuen Segmente
        # seit starvation_timeout Sekunden
        if (self._last_segment_time > 0
                and self.planner.is_starved
                and (now - self._last_segment_time) > self._starvation_timeout_s):
            logger.info("BRIDGE: Print abgeschlossen "
                        "(keine neuen Segmente seit %.1fs)",
                        now - self._last_segment_time)
            self.telemetry.log_event("JOB_COMPLETE", "INFO",
                                     "Print abgeschlossen — Buffer leer")
            self._running = False  # Loop sauber beenden
            return

        if self.sm.state == State.RUN:
            action = self.cfg.queues.underflow_action
            if action == "degrade":
                self.sm.to_degraded("Buffer-Underflow")
            elif action == "stop":
                self.sm.to_stop("Buffer-Underflow — Stop konfiguriert")

    def _handle_sync_level(self):
        """Reagiert auf Sync-Level-Änderungen (§E2)."""
        level = self.sync.sync_level

        if level == SyncLevel.STOP and self.sm.state != State.FAULT:
            self.sm.to_fault(f"Sync-Stop: {self.sync.metrics.sync_reason}")

        elif level == SyncLevel.DEGRADE and self.sm.state == State.RUN:
            self.sm.to_degraded(
                f"Sync-Degrade: {self.sync.metrics.sync_reason}"
            )

        elif level == SyncLevel.OK and self.sm.state == State.DEGRADED:
            # Recovery: DEGRADED → RUN
            self.sm.to_run("Sync wieder OK — Recovery")

    # ── Status / API ─────────────────────────────────────────

    def snapshot(self) -> dict:
        """Vollständiger Status-Snapshot (für EGM_STATUS)."""
        return {
            "state": self.sm.snapshot(),
            "planner": self.planner.snapshot(),
            "egm": self.egm.snapshot(),
            "sync": self.sync.snapshot(),
            "telemetry": self.telemetry.snapshot(),
            "receiver": self.receiver.snapshot() if self.receiver else None,
            "loop_count": self._loop_count,
            "loop_overruns": self._loop_overruns,
            "config_profile": self.cfg.profile_name,
        }

    def set_param(self, section: str, key: str, value) -> bool:
        """
        Setzt einen Parameter zur Laufzeit (für EGM_SET_PARAM).
        Loggt die Änderung.
        """
        section_obj = getattr(self.cfg, section, None)
        if section_obj is None:
            logger.error("BRIDGE: Unbekannte Config-Section: %s", section)
            return False

        if not hasattr(section_obj, key):
            logger.error("BRIDGE: Unbekannter Parameter: %s.%s", section, key)
            return False

        old_value = getattr(section_obj, key)
        setattr(section_obj, key, value)

        self.telemetry.log_event(
            "PARAM_CHANGE", "INFO",
            f"{section}.{key}: {old_value} → {value}"
        )
        logger.info("BRIDGE: Parameter geändert: %s.%s = %s → %s",
                     section, key, old_value, value)
        return True
