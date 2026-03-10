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
#   TrajectoryPlanner (Interpolation, time-indexed)
#     ↓ EGM-Samples (Zyklustakt)
#   EgmClient (UDP)
#     ↓ Sollwerte
#   ABB Controller
#     ↓ Feedback (UDP)
#   SyncMonitor (Bewertung)
#     ↓ Korrektur / State-Änderung
#   StateMachine
#     ↓ Stop/Pause-Befehle
#   KlipperCommandClient (TCP → bridge_watchdog.py)

import time
import logging
import threading
import signal
import sys
from typing import Optional

from .state_machine import BridgeStateMachine, State, StateChangeEvent
from .config import BridgeConfig, validate_config
from .segment_source import TrapezSegment, TcpSegmentReceiver
from .trajectory_planner import TrajectoryPlanner, EgmSample
from .egm_client import EgmClient, EgmTarget, EgmFeedback
from .sync_monitor import SyncMonitor, SyncLevel
from .telemetry import TelemetryWriter
from .moonraker_client import MoonrakerClient
from .klipper_command import KlipperCommandClient, MoonrakerEmergencyStop

logger = logging.getLogger("egm.bridge")


# ── Windows High-Resolution Timer ───────────────────────────
# Windows time.sleep() hat ~15ms Auflösung per Default.
# Für cycle_ms < 15 braucht man timeBeginPeriod(1) aus winmm.dll.

_win_timer_active = False


def _enable_win_hires_timer():
    """Aktiviert 1ms Timer-Auflösung auf Windows."""
    global _win_timer_active
    if sys.platform != "win32" or _win_timer_active:
        return
    try:
        import ctypes
        winmm = ctypes.windll.winmm
        winmm.timeBeginPeriod(1)
        _win_timer_active = True
        logger.info("BRIDGE: Windows High-Res Timer aktiviert (1ms)")
    except Exception as e:
        logger.warning("BRIDGE: Windows Timer-Setup fehlgeschlagen: %s "
                        "— sleep-Auflösung bleibt ~15ms", e)


def _disable_win_hires_timer():
    """Gibt die Timer-Auflösung wieder frei."""
    global _win_timer_active
    if sys.platform != "win32" or not _win_timer_active:
        return
    try:
        import ctypes
        winmm = ctypes.windll.winmm
        winmm.timeEndPeriod(1)
        _win_timer_active = False
        logger.info("BRIDGE: Windows High-Res Timer deaktiviert")
    except Exception:
        pass


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

        # time_offset_ms aus SyncConfig → Sekunden für den Planner
        offset_s = config.sync.time_offset_ms / 1000.0

        # Komponenten
        self.planner = TrajectoryPlanner(
            cycle_s=self._cycle_s,
            max_queue_size=config.queues.plan_queue_size,
            correction_max_mm=config.sync.correction_max_mm,
            correction_rate_limit=config.sync.correction_rate_limit_mm_per_s,
            offset_s=offset_s,
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

        # Moonraker-Client für Extruder-E-Wert (optional)
        self.moonraker: Optional[MoonrakerClient] = None
        if config.moonraker.enabled:
            self.moonraker = MoonrakerClient(
                host=config.moonraker.host,
                port=config.moonraker.port,
                reconnect_interval_s=config.moonraker.reconnect_interval_s,
                poll_interval_ms=config.moonraker.poll_interval_ms,
            )

        # ── Klipper-Watchdog-Client (Heartbeat + Stop-Befehle) ──
        self.klipper_cmd: Optional[KlipperCommandClient] = None
        if config.watchdog.enabled:
            self.klipper_cmd = KlipperCommandClient(
                host=config.watchdog.tcp_host,
                port=config.watchdog.tcp_port,
                heartbeat_interval_s=config.watchdog.heartbeat_interval_s,
                reconnect_interval_s=config.watchdog.reconnect_interval_s,
            )

        # State-Machine-Listener für Watchdog registrieren
        self.sm.add_listener(self._on_state_change)

        # Interner Zustand
        self._loop_thread: Optional[threading.Thread] = None
        self._running = False
        self._last_sample: Optional[EgmSample] = None
        self._last_feedback: Optional[EgmFeedback] = None
        self._feedback_lock = threading.Lock()

        # End-of-Job-Erkennung
        self._last_segment_time: float = 0.0
        self._starvation_timeout_s: float = 3.0  # Job fertig nach 3s ohne neue Segmente

        # Zeitkopplung: Wurde das erste Segment empfangen?
        self._first_segment_received: bool = False

        # Metriken
        self._loop_count = 0
        self._loop_overruns = 0

    # ── Lifecycle ────────────────────────────────────────────

    def start(self) -> bool:
        """
        Initialisiert Verbindungen.
        INIT → READY (oder FAULT bei Fehlern).
        """
        logger.info("BRIDGE: Starte...")

        # Windows: Timer-Auflösung auf 1ms setzen (nötig für cycle_ms < 15)
        if sys.platform == "win32":
            _enable_win_hires_timer()

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

        # Moonraker-Client starten (optional, für E-Wert-Logging)
        if self.moonraker:
            self.moonraker.start()
            logger.info("BRIDGE: Moonraker E-Wert-Subscriber aktiv "
                         "→ ws://%s:%d",
                         self.cfg.moonraker.host, self.cfg.moonraker.port)

        # Klipper-Watchdog-Client starten (Heartbeat + Stop-Befehle)
        if self.klipper_cmd:
            self.klipper_cmd.start()
            logger.info("BRIDGE: Klipper-Watchdog-Client aktiv "
                         "→ %s:%d (Heartbeat: %.1fs)",
                         self.cfg.watchdog.tcp_host,
                         self.cfg.watchdog.tcp_port,
                         self.cfg.watchdog.heartbeat_interval_s)

        # Telemetrie starten
        self.telemetry.start()
        self.telemetry.save_config_snapshot(self.cfg.snapshot())

        self.sm.to_ready("Alle Verbindungen hergestellt")
        logger.info("BRIDGE: Bereit (Zyklus: %.1fms, Profil: %s, "
                     "time_offset: %.1fms, Watchdog: %s)",
                     self.cfg.connection.cycle_ms, self.cfg.profile_name,
                     self.cfg.sync.time_offset_ms,
                     "aktiv" if self.klipper_cmd else "aus")
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
        self._last_segment_time = time.perf_counter()  # Startzeitpunkt
        self.sm.to_run(f"Job gestartet: {job_id or 'default'}")

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

        self.sm.to_stop(reason)

        if self.receiver:
            self.receiver.stop()
        if self.moonraker:
            self.moonraker.stop()
        if self.klipper_cmd:
            self.klipper_cmd.stop()
        self.egm.disconnect()
        self.telemetry.stop()
        self.planner.clear()

        logger.info("BRIDGE: Gestoppt (Zyklen: %d, Overruns: %d)",
                     self._loop_count, self._loop_overruns)
        _disable_win_hires_timer()

    def shutdown(self):
        """Komplettes Herunterfahren inkl. Cleanup — aus JEDEM Zustand."""
        logger.info("BRIDGE: Shutdown aus Zustand %s", self.sm.state.value)
        self._running = False

        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)

        # Verbindungen schließen (unabhängig vom State)
        if self.receiver:
            self.receiver.stop()
        if self.moonraker:
            self.moonraker.stop()
        if self.klipper_cmd:
            self.klipper_cmd.stop()
        self.egm.disconnect()
        self.telemetry.stop()
        self.planner.clear()

        logger.info("BRIDGE: Shutdown abgeschlossen (Zyklen: %d, Overruns: %d)",
                     self._loop_count, self._loop_overruns)
        _disable_win_hires_timer()

    # ── EGM Cycle Loop ───────────────────────────────────────

    def _egm_loop(self):
        """
        Hauptloop: Läuft im festen EGM-Zyklustakt.

        Jeder Zyklus:
          1. Sample aus Planner holen (time-indexed Interpolation)
          2. An EGM-Client senden
          3. Sync-Monitor updaten
          4. Telemetrie schreiben
          5. Bei Bedarf: Korrektur, State-Change
        """
        logger.info("BRIDGE: EGM-Loop gestartet (Zyklus: %.1fms)",
                     self._cycle_s * 1000)

        cycle_s = self._cycle_s
        metric_interval = self.cfg.telemetry.metric_interval_s
        last_metric_time = time.perf_counter()
        starvation_logged = False

        # Auf Windows hat time.monotonic() nur ~15ms Auflösung.
        # time.perf_counter() hat µs-Auflösung — wir nutzen es für alles.
        # Für Zyklen < 15ms: reiner Spin-Wait (kein time.sleep).
        use_spin_only = (cycle_s < 0.015)
        if use_spin_only:
            logger.info("BRIDGE: Spin-Wait aktiv (cycle < 15ms, "
                         "1 CPU-Kern @ 100%%)")

        while self._running and self.sm.is_running:
            loop_start = time.perf_counter()
            bridge_time = loop_start  # perf_counter für µs-Auflösung
            self._loop_count += 1

            try:
                # 1. Sample erzeugen (time-indexed)
                sample = self.planner.next_sample(bridge_time)

                if sample and sample.velocity > 0:
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

                    # Sample im Sync-Monitor für Lag-Matching aufzeichnen
                    self.sync.record_sent_sample(sample)

                    # Extruder-E-Wert für Telemetrie abfragen
                    e_val = 0.0
                    e_age = -1.0
                    if self.moonraker:
                        ext = self.moonraker.get_extruder_state()
                        e_val = ext.e_value
                        e_age = ext.age_ms
                        # Warnung bei veraltetem E-Wert
                        if (e_age > self.cfg.moonraker.stale_threshold_ms
                                and e_age > 0
                                and self._loop_count % 250 == 0):
                            logger.warning(
                                "BRIDGE: Extruder-E-Wert veraltet "
                                "(age=%.0fms > threshold=%.0fms)",
                                e_age,
                                self.cfg.moonraker.stale_threshold_ms)

                    self.telemetry.log_tx(sample, e_value=e_val,
                                          extruder_age_ms=e_age)

                    # 3. Sync updaten (NUR bei echten Samples)
                    with self._feedback_lock:
                        fb = self._last_feedback

                    if fb:
                        # RX wird NICHT hier geloggt — das passiert
                        # bereits in _on_feedback() bei voller ABB-Rate.
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

                elif sample and sample.velocity == 0:
                    # ── Position-Hold (Lücke / Gap) ──────────────
                    # Sample kam zurück mit velocity=0 → Planner
                    # hält die letzte Position (Zeitlücke in print_time)
                    self._last_sample = sample
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
                    # Kein Sync-Update während Gap-Hold

                else:
                    # ── Kein Sample — Starvation ─────────────────
                    if not starvation_logged:
                        logger.info("BRIDGE: Kein Sample — "
                                    "warte auf Zeitkopplung oder Segmente")
                        starvation_logged = True

                    # Job-Ende prüfen
                    time_since_seg = (
                        bridge_time - self._last_segment_time
                        if self._last_segment_time > 0 else 0
                    )
                    if (time_since_seg > self._starvation_timeout_s
                            and self._first_segment_received):
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

            except Exception as e:
                logger.error("BRIDGE: Loop-Fehler: %s", e, exc_info=True)
                self.telemetry.log_event("LOOP_ERROR", "ERROR", str(e))
                self.sm.to_fault(f"Loop-Fehler: {e}")
                break

            # Zykluszeit einhalten
            if use_spin_only:
                # Kurze Zyklen (< 15ms): reiner Spin-Wait
                # time.sleep() ist auf Windows unbrauchbar unter 15ms
                deadline = loop_start + cycle_s
                while time.perf_counter() < deadline:
                    pass
            else:
                # Längere Zyklen: normal schlafen
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
        now = time.perf_counter()
        self._last_segment_time = now

        # Zeitkopplung beim ersten Segment herstellen
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
                f"offset={self.planner.offset_s:.3f}s"
            )

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

    def _on_state_change(self, event: StateChangeEvent):
        """
        Callback: State-Machine-Übergang.

        Aktualisiert den Bridge-State im Watchdog-Heartbeat
        und sendet bei kritischen Übergängen Befehle an Klipper.
        """
        # Watchdog-Heartbeat-State aktualisieren
        if self.klipper_cmd:
            self.klipper_cmd.set_bridge_state(event.to_state.value)

        # ── Bei FAULT: Klipper stoppen ──────────────────────
        if event.to_state == State.FAULT:
            self._notify_klipper_stop(
                f"Bridge-FAULT: {event.reason}")

        # ── Bei DEGRADED: Optional Klipper pausieren ────────
        elif (event.to_state == State.DEGRADED
              and self.cfg.watchdog.pause_on_degrade):
            self._notify_klipper_pause(
                f"Bridge-DEGRADED: {event.reason}")

    # ── Klipper benachrichtigen ──────────────────────────────

    def _notify_klipper_stop(self, reason: str):
        """
        Sendet Stop-Befehl an Klipper über alle verfügbaren Kanäle.

        1. Primär: Dedizierter TCP (bridge_watchdog.py)
        2. Fallback: Moonraker HTTP-API
        """
        if not self.cfg.watchdog.stop_on_fault:
            logger.info("BRIDGE: Klipper-Stop unterdrückt "
                         "(stop_on_fault=false)")
            return

        sent = False

        # Kanal 1: Dedizierter TCP
        if self.klipper_cmd:
            sent = self.klipper_cmd.send_stop(reason)
            if sent:
                logger.info("BRIDGE: Stop an Klipper gesendet (TCP)")

        # Kanal 2: Moonraker HTTP-API als Fallback
        if not sent and self.cfg.watchdog.moonraker_fallback:
            logger.warning("BRIDGE: TCP-Stop fehlgeschlagen, "
                            "versuche Moonraker-Fallback...")
            mr_host = self.cfg.moonraker.host
            mr_port = self.cfg.moonraker.port
            sent = MoonrakerEmergencyStop.send_pause_via_http(
                mr_host, mr_port)
            if sent:
                logger.info("BRIDGE: Stop an Klipper gesendet "
                             "(Moonraker HTTP)")
            else:
                logger.error("BRIDGE: KONNTE KLIPPER NICHT STOPPEN! "
                              "Weder TCP noch Moonraker verfügbar.")

        self.telemetry.log_event(
            "KLIPPER_STOP_SENT", "CRITICAL" if sent else "ERROR",
            f"Klipper-Stop: {reason} (gesendet: {sent})")

    def _notify_klipper_pause(self, reason: str):
        """Sendet Pause-Befehl an Klipper."""
        sent = False

        if self.klipper_cmd:
            sent = self.klipper_cmd.send_pause(reason)
            if sent:
                logger.info("BRIDGE: Pause an Klipper gesendet (TCP)")

        if not sent and self.cfg.watchdog.moonraker_fallback:
            mr_host = self.cfg.moonraker.host
            mr_port = self.cfg.moonraker.port
            sent = MoonrakerEmergencyStop.send_pause_via_http(
                mr_host, mr_port)

        self.telemetry.log_event(
            "KLIPPER_PAUSE_SENT", "WARNING",
            f"Klipper-Pause: {reason} (gesendet: {sent})")

    # ── Reaktionen ───────────────────────────────────────────

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
            "moonraker": self.moonraker.snapshot() if self.moonraker else None,
            "watchdog": self.klipper_cmd.snapshot() if self.klipper_cmd else None,
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

        # offset_s live aktualisieren wenn time_offset_ms geändert wird
        if section == "sync" and key == "time_offset_ms":
            self.planner.offset_s = value / 1000.0
            logger.info("BRIDGE: Planner offset_s aktualisiert: %.3fs",
                         self.planner.offset_s)

        self.telemetry.log_event(
            "PARAM_CHANGE", "INFO",
            f"{section}.{key}: {old_value} → {value}"
        )
        logger.info("BRIDGE: Parameter geändert: %s.%s = %s → %s",
                     section, key, old_value, value)
        return True
