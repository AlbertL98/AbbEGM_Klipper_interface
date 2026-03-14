# bridge.py — EGM-Bridge-Core Orchestrator

import time
import logging
import threading
from typing import Optional

from .clock import bridge_now
from .state_machine import BridgeStateMachine, State, StateChangeEvent
from .config import BridgeConfig, validate_config
from .path_segment import TrapezSegment
from .trajectory_planner import TrajectoryPlanner, EgmSample
from .egm_client import EgmClient, EgmTarget, EgmFeedback
from .sync_monitor import SyncMonitor, SyncLevel
from .telemetry import TelemetryWriter
from .klipper_client import KlipperClient, MoonrakerEmergencyStop

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

        #State Machine
        self.state_cont = BridgeStateMachine()

        #Sync Monitor
        self.sync = SyncMonitor(
            config=config.sync
        )

        # Trajectory Planner
        self.planner = TrajectoryPlanner(
            max_queue_size=config.queues.plan_queue_size
        )

        # EGM Client
        self.egm = EgmClient(
            robot_ip=config.connection.robot_ip,
            send_port=config.connection.send_port,
            recv_port=config.connection.recv_port,
            local_send_port=config.connection.local_send_port,
            timeout_ms=config.connection.timeout_ms,
            watchdog_cycles=config.connection.watchdog_cycles,
            default_q0=config.connection.default_q0,
            default_q1=config.connection.default_q1,
            default_q2=config.connection.default_q2,
            default_q3=config.connection.default_q3,
            on_feedback=self._on_feedback,
            on_timeout=self._on_egm_timeout
        )

        # Telemetry
        self.telemetry = TelemetryWriter(
            log_dir=config.telemetry.log_dir
        )

        # Klipper Communication
        self.klipper_client = KlipperClient(config=config.klipper)
        self.MoonrakerEmergencyStop = MoonrakerEmergencyStop()

        # State-Machine-Listener für Watchdog
        self.state_cont.add_listener(self._on_state_change)

        # Interner Zustand
        self._loop_thread: Optional[threading.Thread] = None
        self._running = False
        self._last_sample: Optional[EgmSample] = None
        self._last_feedback: Optional[EgmFeedback] = None
        self._feedback_lock = threading.Lock()
        self._cycle_s = self.cfg.connection.cycle_ms

        # End-of-Job-Erkennung (jetzt konfigurierbar)
        self._last_segment_time: float = 0.0
        self._starvation_timeout_s: float = config.queues.starvation_timeout_s

        # Zeitkopplung
        self._first_segment_received: bool = False

        # Workspace-Envelope-Statistik
        self._envelope_violations: int = 0

        # Metriken
        self._loop_count = 0
        self._loop_overruns = 0

    def start(self) -> bool:
        """
        Initialisiert Verbindungen.
        INIT → READY (oder FAULT bei Fehlern).
        """
        logger.info("BRIDGE: Starte...")

        # Config validieren
        errors = validate_config(self.cfg)
        if errors:
            for e in errors:
                logger.error("BRIDGE: Config-Fehler: %s", e)
            self.state_cont.to_fault("Config-Validierung fehlgeschlagen",
                                     {"errors": errors})
            return False

        # EGM-Verbindung
        if not self.egm.connect():
            self.state_cont.to_fault("EGM-Verbindung fehlgeschlagen")
            return False

        # Klipper client starten
        self.klipper_client = KlipperClient(config=self.cfg.klipper, on_segment=self._on_segment_received)
        self.klipper_client.start()

        # Telemetrie starten
        self.telemetry.start()
        self.telemetry.save_config_snapshot(self.cfg.snapshot())

        self.state_cont.to_ready("Alle Verbindungen hergestellt")

        ws = self.cfg.workspace
        ws_status = (f"aktiv [{ws.min_x},{ws.max_x}]x"
                     f"[{ws.min_y},{ws.max_y}]x"
                     f"[{ws.min_z},{ws.max_z}]mm"
                     if ws.enabled else "AUS")

        logger.info("BRIDGE: Bereit (Zyklus: %.1fms, Profil: %s, "
                     "time_offset_backend: %.1fms, Workspace Status: )",
                     self.cfg.connection.cycle_ms, self.cfg.profile_name,
                     self.cfg.sync.delay_backend_ms,
                     ws_status)
        return True

    def run_job(self, job_id: Optional[str] = None):
        """
        Startet den EGM-Zyklusloop.
        READY → RUN
        """
        if self.state_cont.state != State.READY:
            logger.error("BRIDGE: Kann Job nur aus READY starten "
                         "(aktuell: %s)", self.state_cont.state.value)
            return False

        if job_id:
            self.telemetry.log_event("JOB_START", "INFO",f"Job {job_id} gestartet")

        self.sync.reset()
        self._last_segment_time = bridge_now()
        self.state_cont.to_run(f"Job gestartet: {job_id or 'default'}")

        self._running = True
        self._loop_thread = threading.Thread(
            target=self._egm_loop, daemon=True, name="egm-loop"
        )
        self._loop_thread.start()

        logger.info("BRIDGE: Job läuft")
        return True

    def stop(self, reason: str = "Manueller Stop"):
        """Ordentliches Herunterfahren."""
        logger.info("BRIDGE: Stoppe... (%s)", reason)
        self._running = False

        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)

        self.state_cont.to_stop(reason)

        if self.klipper_client:
            self.klipper_client.stop()
        self.egm.disconnect()
        self.telemetry.stop()
        self.planner.clear()

        logger.info("BRIDGE: Gestoppt (Zyklen: %d, Overruns: %d, "
                     "Envelope-Verletzungen: %d)",
                     self._loop_count, self._loop_overruns,
                     self._envelope_violations)

    def shutdown(self):
        """Komplettes Herunterfahren inkl. Cleanup — aus JEDEM Zustand."""
        logger.info("BRIDGE: Shutdown aus Zustand %s", self.state_cont.state.value)
        self._running = False

        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)

        if self.klipper_client:
            self.klipper_client.stop()
        self.egm.disconnect()
        self.telemetry.stop()
        self.planner.clear()

        logger.info("BRIDGE: Shutdown abgeschlossen (Zyklen: %d, "
                     "Overruns: %d, Envelope-Verletzungen: %d)",
                     self._loop_count, self._loop_overruns,
                     self._envelope_violations)

    # ── EGM Cycle Loop ───────────────────────────────────────

    def _egm_loop(self):
        """Hauptloop: Läuft im festen EGM-Zyklustakt."""
        logger.info("BRIDGE: EGM-Loop gestartet (Zyklus: %.1fms)",self._cycle_s * 1000)

        cycle_s = self._cycle_s
        metric_interval = self.cfg.telemetry.metric_interval_s
        last_metric_time = bridge_now()
        starvation_logged = False

        use_spin_only = (cycle_s < 0.015)
        if use_spin_only:
            logger.info("BRIDGE: Spin-Wait aktiv (cycle < 15ms)")

        while self._running and self.state_cont.is_running and self.klipper_client.klipper_is_printing:
            loop_start = bridge_now()
            bt = loop_start  # bridge_time
            self._loop_count += 1

            try:
                # 1. Sample erzeugen
                if self.sync.enabled:
                    lookahead = self.sync.get_lookahead()
                else:
                    lookahead = self.cfg.sync.delay_frontend_init_ms
                sample = self.planner.next_sample(bt, lookahead)

                if sample and sample.velocity > 0:
                    # ── Normaler Betrieb ─────────────────────────
                    starvation_logged = False
                    self._last_sample = sample

                    # 2. An Roboter senden
                    target = EgmTarget(
                        sequence_id=sample.sequence_id,
                        timestamp=sample.timestamp,
                        x=sample.x, y=sample.y, z=sample.z,
                        q0=self.cfg.connection.default_q0,
                        q1=self.cfg.connection.default_q1,
                        q2=self.cfg.connection.default_q2,
                        q3=self.cfg.connection.default_q3,
                    )
                    self.egm.send_target(target)

                    # Sync-Monitor: TX aufzeichnen (gleich nach send,
                    # damit RX-Callback sofort matchen kann)
                    self.sync.record_sent_sample(sample)

                    self.telemetry.log_tx(
                        sample
                    )

                    # 3. Sync updaten (Bewertung + Cycle-Log, 50Hz)
                    with self._feedback_lock:
                        fb = self._last_feedback

                    if fb:
                        self.sync.update(
                            sample=sample,
                            feedback=fb,
                            buffer_depth=self.planner.queue_depth,
                            buffer_time_s=self.planner.queue_time_s,
                        )

                    # 4. Periodische Metriken
                    if bt - last_metric_time >= metric_interval:
                        self.telemetry.log_sync(self.sync.metrics)
                        last_metric_time = bt

                    # 5. Sync-Level-Reaktion
                    self._handle_sync_level()

                elif sample and sample.velocity == 0:
                    # ── Position-Hold ────────────────────────────
                    self._last_sample = sample

                    target = EgmTarget(
                        sequence_id=sample.sequence_id,
                        timestamp=sample.timestamp,
                        x=sample.x, y=sample.y, z=sample.z,
                        q0=self.cfg.connection.default_q0,
                        q1=self.cfg.connection.default_q1,
                        q2=self.cfg.connection.default_q2,
                        q3=self.cfg.connection.default_q3,
                    )
                    self.egm.send_target(target)

                    # TX auch bei velocity=0 aufzeichnen —
                    # sonst bleibt der Buffer leer während Holds/Gaps
                    self.sync.record_sent_sample(sample)

                else:
                    # ── Starvation ───────────────────────────────
                    if not starvation_logged:
                        logger.info("BRIDGE: Kein Sample — "
                                    "warte auf Zeitkopplung oder "
                                    "Segmente")
                        starvation_logged = True

                    time_since_seg = (
                        bt - self._last_segment_time
                        if self._last_segment_time > 0 else 0
                    )
                    if (time_since_seg > self._starvation_timeout_s
                            and self._first_segment_received):
                        logger.error("BRIDGE: Samples fehlen")
                        self.telemetry.log_event(
                            "SAMPLES FEHLEN", "ERROR",
                            f"Keine Samples seit {time_since_seg}s, Bridge wird gestopt")
                        self._running = False
                        break

                    if self._last_sample:
                        hold = EgmTarget(
                            sequence_id=self._loop_count,
                            timestamp=bt,
                            x=self._last_sample.x,
                            y=self._last_sample.y,
                            z=self._last_sample.z,
                            q0=self.cfg.connection.default_q0,
                            q1=self.cfg.connection.default_q1,
                            q2=self.cfg.connection.default_q2,
                            q3=self.cfg.connection.default_q3,
                        )
                        self.egm.send_target(hold)

                        # TX auch bei Starvation-Hold aufzeichnen
                        self.sync.record_sent_sample(self._last_sample)

            except Exception as e:
                logger.error("BRIDGE: Loop-Fehler: %s", e, exc_info=True)
                self.telemetry.log_event("LOOP_ERROR", "ERROR", str(e))
                self.state_cont.to_fault(f"Loop-Fehler: {e}")
                break

            # Zykluszeit einhalten
            if use_spin_only:
                deadline = loop_start + cycle_s
                while bridge_now() < deadline:
                    pass
            else:
                elapsed = bridge_now() - loop_start
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

        if self.state_cont.state in (State.RUN, State.DEGRADED):
            self.state_cont.to_stop("Job abgeschlossen")

    # ── Callbacks ────────────────────────────────────────────

    def _on_segment_received(self, seg: TrapezSegment):
        """Callback: Neues Segment von Klipper empfangen."""
        now = bridge_now()
        self._last_segment_time = now

        if not self._first_segment_received:
            self._first_segment_received = True
            self.planner.init_time_sync(
                bridge_time=now,
                klipper_time=seg.print_time,
            )
            self.telemetry.log_event(
                "TIME_SYNC", "INFO",
                f"Zeitkopplung: bridge={now:.3f} ↔ "
                f"klipper={seg.print_time:.3f} "
            )

        self.planner.add_segment(seg)
        self.telemetry.log_plan(seg)

    def _on_feedback(self, feedback: EgmFeedback):
        """Callback: Feedback vom Roboter empfangen (250Hz)."""
        with self._feedback_lock:
            self._last_feedback = feedback

        # Offset-Estimator aktualisieren (im RX-Thread, ~250Hz)
        # on_feedback_received() ist threadsicher (Lock auf T_delay)
        debug = self.sync.on_feedback_received(feedback)
        if debug is not None:
            self.telemetry.log_estimator(debug)

        self.telemetry.log_rx(feedback)

    def _on_egm_timeout(self):
        """Callback: EGM-Watchdog ausgelöst."""
        self.telemetry.log_event("EGM_TIMEOUT", "CRITICAL",
                                 "Watchdog — keine Antwort vom Roboter")
        self.state_cont.to_fault("EGM-Watchdog-Timeout")

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

    def _on_state_change(self, event: StateChangeEvent):
        """Callback: State-Machine-Übergang."""
        if self.klipper_client:
            self.klipper_client.set_bridge_state(event.to_state.value)

        if event.to_state == State.FAULT:
            self._notify_klipper_stop(
                f"Bridge-FAULT: {event.reason}")
        elif (event.to_state == State.DEGRADED
              and self.cfg.watchdog.pause_on_degrade):
            self._notify_klipper_pause(
                f"Bridge-DEGRADED: {event.reason}")

    # ── Klipper benachrichtigen ──────────────────────────────

    def _notify_klipper_stop(self, reason: str):
        """Sendet Stop-Befehl an Klipper."""
        if not self.cfg.watchdog.stop_on_fault:
            return

        sent = False
        if self.klipper_client:
            sent = self.klipper_client.send_stop(reason)
            if sent:
                logger.info("BRIDGE: Stop an Klipper gesendet (TCP)")

        if not sent and self.cfg.watchdog.moonraker_fallback:
            logger.warning("BRIDGE: TCP-Stop fehlgeschlagen, "
                            "versuche Moonraker-Fallback...")
            sent = MoonrakerEmergencyStop.send_stop(
                self.cfg.moonraker.host, self.cfg.moonraker.port)

        self.telemetry.log_event(
            "KLIPPER_STOP_SENT", "CRITICAL" if sent else "ERROR",
            f"Klipper-Stop: {reason} (gesendet: {sent})")

    def _notify_klipper_pause(self, reason: str):
        """Sendet Pause-Befehl an Klipper."""
        sent = False
        if self.klipper_client:
            sent = self.klipper_client.send_pause(reason)
        if not sent and self.cfg.watchdog.moonraker_fallback:
            sent = MoonrakerEmergencyStop.send_pause(
                self.cfg.moonraker.host, self.cfg.moonraker.port)
        self.telemetry.log_event(
            "KLIPPER_PAUSE_SENT", "WARNING",
            f"Klipper-Pause: {reason} (gesendet: {sent})")

    # ── Reaktionen ───────────────────────────────────────────

    def _handle_sync_level(self):
        """Reagiert auf Sync-Level-Änderungen."""
        level = self.sync.sync_level

        if level == SyncLevel.STOP and self.state_cont.state != State.FAULT:
            self.state_cont.to_fault(
                f"Sync-Stop: {self.sync.metrics.sync_reason}")
        elif level == SyncLevel.DEGRADE and self.state_cont.state == State.RUN:
            self.state_cont.to_degraded(
                f"Sync-Degrade: {self.sync.metrics.sync_reason}")
        elif level == SyncLevel.OK and self.state_cont.state == State.DEGRADED:
            self.state_cont.to_run("Sync wieder OK — Recovery")

    # ── Status / API ─────────────────────────────────────────

    # def snapshot(self) -> dict:
    #     """Vollständiger Status-Snapshot."""
    #     return {
    #         "state": self.state_cont.snapshot(),
    #         "planner": self.planner.snapshot(),
    #         "egm": self.egm.snapshot(),
    #         "sync": self.sync.snapshot(),
    #         "telemetry": self.telemetry.snapshot(),
    #         "receiver": (self.receiver.snapshot()
    #                      if self.receiver else None),
    #         "moonraker": (self.moonraker.snapshot()
    #                       if self.moonraker else None),
    #         "watchdog": (self.klipper_cmd.snapshot()
    #                      if self.klipper_cmd else None),
    #         "loop_count": self._loop_count,
    #         "loop_overruns": self._loop_overruns,
    #         "envelope_violations": self._envelope_violations,
    #         "config_profile": self.cfg.profile_name,
    #     }
