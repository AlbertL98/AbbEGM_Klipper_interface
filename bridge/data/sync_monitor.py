# sync_monitor.py — Closed-Loop Synchronisationsüberwachung
#
# Konsolidiert den bisherigen SyncMonitor und LatencyEstimator in eine Klasse.
# Portiert die Matching-Logik aus EGM_Error_Test.py.
#
# Zwei Update-Pfade:
#   - on_feedback_received() → 250Hz (RX-Thread), Offset-Estimator
#   - update()               → 50Hz  (EGM-Loop), Sync-Bewertung + Logging
#
# Closed-Loop-Prinzip:
#   1. Bridge sendet Sample → record_sent_sample()
#   2. RX-Thread empfängt Feedback → on_feedback_received()
#      → Position-Matching im Suchfenster → T_delay EMA-Update
#   3. EGM-Loop holt Lookahead → get_lookahead()
#      → TrajectoryPlanner nutzt Lookahead für nächstes Sample
#   4. EGM-Loop ruft update() → Sync-Level + Cycle-Log
#
# FIX: Consecutive-Counter (_consecutive_warn/degrade/stop) starten
#   jetzt bei 0 statt bei den Schwellenwerten. Vorher konnte ein
#   DEGRADE kurz vor Job-Ende den Zähler erhöht stehen lassen, sodass
#   der nächste Job sofort in DEGRADE startete. reset() setzt die
#   Zähler jetzt explizit zurück.
#
# FIX: update()-Parameter sample ist reserviert (Matching passiert
#   in on_feedback_received() bei 250Hz), jetzt entsprechend dokumentiert.
#
# CLOCK-FIX: Alle Timestamps nutzen bridge_now() aus clock.py.

import math
import bisect
import logging
import enum
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Optional, Callable, List, NamedTuple

from .clock import bridge_now
from .egm_client import EgmFeedback
from .trajectory_planner import EgmSample
from .config import SyncConfig, LatencyEstimatorConfig

logger = logging.getLogger("egm.sync")


# ═══════════════════════════════════════════════════════════════════════
# Datenstrukturen
# ═══════════════════════════════════════════════════════════════════════

class SyncLevel(enum.Enum):
    OK = "OK"
    WARN = "WARN"
    DEGRADE = "DEGRADE"
    STOP = "STOP"


@dataclass
class SyncMetrics:
    """Aktueller Sync-Zustand (Snapshot pro Zyklus)."""
    timestamp: float = 0.0
    robot_time: float = 0.0

    # Ist-Position (letztes Feedback)
    ist_x: float = 0.0
    ist_y: float = 0.0
    ist_z: float = 0.0

    # Soll-Position (best-match aus Ringbuffer)
    soll_x: float = 0.0
    soll_y: float = 0.0
    soll_z: float = 0.0
    soll_v: float = 0.0          # Soll-Geschwindigkeit am Matchpunkt (mm/s)

    # TCP-Fehler
    error_tcp_pos: float = 0.0   # Euklidischer Positionsfehler (mm)
    error_tcp_speed: float = 0.0 # Geschwindigkeitsfehler Soll-Ist (mm/s)

    # Richtungsaufgelöster Fehler
    tang_error: float = 0.0      # Tangentialfehler (mm, vorzeichenbehaftet)
    norm_error: float = 0.0      # Normalfehler (mm, immer >= 0)

    # Offset (Kern des Closed Loop)
    t_delay_ms: float = 0.0          # Aktueller Roh-Offset (ms)
    t_delay_output_ms: float = 0.0   # Geglätteter Output-Offset (ms)
    offset_rate_ms_per_s: float = 0.0  # Änderungsrate des Offsets (ms/s)

    # Estimator-Internals
    estimator_weight: float = 0.0
    accel_detected: bool = False

    # Buffer
    buffer_depth: int = 0
    buffer_time_s: float = 0.0

    # Gesamtbewertung
    sync_level: SyncLevel = SyncLevel.OK
    sync_reason: str = ""

    # Zähler
    warn_count: int = 0
    degrade_count: int = 0
    total_updates: int = 0


class CycleLogEntry(NamedTuple):
    """Ein Log-Eintrag pro update()-Aufruf → Telemetry."""
    timestamp: float
    robot_time: float
    ist_x: float
    ist_y: float
    ist_z: float
    soll_x: float
    soll_y: float
    soll_z: float
    soll_v: float
    error_tcp_speed: float
    error_tcp_pos: float
    offset_ms: float
    sync_level: str
    tang_error: float
    norm_error: float


class EstimatorDebug(NamedTuple):
    """Debug-Info pro Estimator-Update (250Hz) → Telemetry."""
    timestamp: float
    t_delay_ms: float
    t_delay_output_ms: float
    tang_err_mm: float
    norm_err_mm: float
    weight: float
    accel_detected: bool
    ema_used: float
    correction_ms: float


# ═══════════════════════════════════════════════════════════════════════
# SyncMonitor — Closed-Loop Offset-Estimator + Sync-Bewertung
# ═══════════════════════════════════════════════════════════════════════

class SyncMonitor:
    """
    Überwacht die Synchronität zwischen geplanter Trajektorie und
    Roboter-Feedback. Berechnet einen adaptiven Zeitoffset (T_delay),
    der als Lookahead für das EGM-Senden verwendet wird.

    Thread-Sicherheit:
      - on_feedback_received() wird im RX-Thread aufgerufen (250Hz)
      - get_lookahead() wird im EGM-Loop aufgerufen (50Hz)
      - update() wird im EGM-Loop aufgerufen (50Hz)
      Alle drei greifen auf _t_delay/_t_delay_output zu → Lock.
    """

    def __init__(self, config: SyncConfig,
                 estimator_config: LatencyEstimatorConfig,
                 evaluate_direction: bool = True):
        self.cfg = config
        self.est_cfg = estimator_config
        self.evaluate_direction = evaluate_direction

        # ── Estimator-Parameter aus Config ───────────────────────────
        self._OFFSET_BACK_S = estimator_config.offset_back_ms / 1000.0
        self._EMA_SLOW = estimator_config.ema_slow
        self._EMA_FAST = estimator_config.ema_fast
        self._EMA_OUTPUT = estimator_config.ema_output
        self._ALPHA_NORMAL = estimator_config.alpha_weight
        self._TX_BUFFER_SIZE = estimator_config.tx_buffer_size

        # Konstanten
        self._EPS: float = 1e-9
        self._SEARCH_WIN_S: float = 0.120        # ±60ms Suchfenster
        self._ACCEL_THR_MMS2: float = 8.0        # mm/s² → Fast-Modus
        self._MIN_TANG_MM: float = 0.5            # Min. Tangentialfehler
        self._TANG_SPAN: int = 3                  # ±3 Punkte für Tangente
        self._T_DELAY_MIN: float = 0.020          # 20ms Untergrenze
        self._T_DELAY_MAX: float = 0.800          # 800ms Obergrenze

        # ── Offset-Estimator-State ───────────────────────────────────
        init_delay = estimator_config.t_delay_init_ms / 1000.0
        self._t_delay = init_delay
        self._t_delay_output = init_delay
        self._lock = threading.Lock()

        # ── TX-Ringbuffer (parallele Listen für bisect) ──────────────
        self._tx_times: List[float] = []
        self._tx_x: List[float] = []
        self._tx_y: List[float] = []
        self._tx_z: List[float] = []
        self._tx_vel: List[float] = []

        # ── Metriken ─────────────────────────────────────────────────
        self._metrics = SyncMetrics()
        self._error_history: deque[float] = deque(maxlen=1000)

        # ── Estimator-Tracking (RX-Thread) ───────────────────────────
        self._prev_speed: Optional[float] = None
        self._prev_t_rx: Optional[float] = None
        self._prev_t_delay: float = init_delay
        self._prev_t_delay_time: float = 0.0
        self._n_estimator_updates: int = 0

        # ── Ist-Geschwindigkeit ──────────────────────────────────────
        self._prev_fb_pos: Optional[tuple] = None
        self._prev_fb_time: Optional[float] = None
        self._ist_velocity: float = 0.0

        # ── Letztes Estimator-Debug ──────────────────────────────────
        self._last_est_debug: Optional[EstimatorDebug] = None

        # ── Zähler ───────────────────────────────────────────────────
        self._total_updates = 0
        self._warn_count = 0
        self._degrade_count = 0
        self._warmup_cycles = 100  # ~2s bei 50Hz

        # FIX: Consecutive-Counter starten bei 0, nicht bei den
        # Schwellenwerten. Vorher konnte ein DEGRADE kurz vor Job-Ende
        # den Zähler erhöht stehen lassen → nächster Job startete
        # sofort in DEGRADE. reset() setzt sie ebenfalls auf 0.
        self._consecutive_warn: int = 0
        self._consecutive_degrade: int = 0
        self._consecutive_stop: int = 0

        # ── Callbacks ────────────────────────────────────────────────
        self._on_level_change: Optional[Callable] = None
        self._on_cycle_log: Optional[Callable] = None

    # ── Callbacks ────────────────────────────────────────────────────

    def set_level_change_callback(self, callback: Callable):
        """Callback wenn sich das Sync-Level ändert: fn(level, reason)"""
        self._on_level_change = callback

    def set_cycle_log_callback(self, callback: Callable):
        """Callback für jeden update()-Log-Eintrag: fn(CycleLogEntry)"""
        self._on_cycle_log = callback

    # ═════════════════════════════════════════════════════════════════
    # Lookahead-API (Thread-Safe, von Planner/Bridge aufgerufen)
    # ═════════════════════════════════════════════════════════════════

    def get_lookahead(self) -> float:
        """
        Gibt den aktuellen geglätteten Offset zurück (positiv, Sekunden).

        Wird vom TrajectoryPlanner verwendet:
            query_time = elapsed + sync_monitor.get_lookahead()
        """
        with self._lock:
            self._t_delay_output += self._EMA_OUTPUT * (
                self._t_delay - self._t_delay_output
            )
            return self._t_delay_output

    @property
    def T_delay_output_ms(self) -> float:
        """Aktueller geglätteter Offset in ms."""
        with self._lock:
            return self._t_delay_output * 1000.0

    @property
    def enabled(self) -> bool:
        """Ist der Estimator aktiv?"""
        return self.est_cfg.enabled

    # ═════════════════════════════════════════════════════════════════
    # TX-Aufzeichnung (EGM-Loop, 50Hz)
    # ═════════════════════════════════════════════════════════════════

    def record_sent_sample(self, sample: EgmSample):
        """
        Speichert ein gesendetes Sample im TX-Ringbuffer.
        Wird im EGM-Loop NACH dem Senden aufgerufen.
        """
        t = bridge_now()
        self._tx_times.append(t)
        self._tx_x.append(sample.x)
        self._tx_y.append(sample.y)
        self._tx_z.append(sample.z)
        vel = getattr(sample, 'velocity', 0.0) or 0.0
        self._tx_vel.append(vel)

        if len(self._tx_times) > self._TX_BUFFER_SIZE:
            excess = len(self._tx_times) - self._TX_BUFFER_SIZE
            del self._tx_times[:excess]
            del self._tx_x[:excess]
            del self._tx_y[:excess]
            del self._tx_z[:excess]
            del self._tx_vel[:excess]

    # ═════════════════════════════════════════════════════════════════
    # Feedback-Pfad: 250Hz (RX-Thread) — Offset-Estimator
    # ═════════════════════════════════════════════════════════════════

    def on_feedback_received(self, feedback: EgmFeedback) -> Optional[EstimatorDebug]:
        """
        Wird im RX-Thread bei jedem empfangenen Feedback aufgerufen (~250Hz).
        Macht das Position-Matching und aktualisiert T_delay.
        """
        if not self.est_cfg.enabled:
            return None

        n = len(self._tx_times)
        if n < 6:
            return None

        t_rx = feedback.timestamp
        ix, iy, iz = feedback.x, feedback.y, feedback.z

        # ── Ist-Geschwindigkeit ──────────────────────────────────────
        pos = (ix, iy, iz)
        if self._prev_fb_pos is not None and self._prev_fb_time is not None:
            dt = t_rx - self._prev_fb_time
            if dt > self._EPS:
                px, py, pz = self._prev_fb_pos
                dist = math.sqrt((ix-px)**2 + (iy-py)**2 + (iz-pz)**2)
                self._ist_velocity = dist / dt
        self._prev_fb_pos = pos
        self._prev_fb_time = t_rx

        # ── Erwartete TX-Sendezeit ───────────────────────────────────
        with self._lock:
            T_d = self._t_delay

        t_actual = t_rx - self._OFFSET_BACK_S
        t_tx_guess = t_actual - T_d

        # ── Suchfenster im TX-Ringbuffer (bisect) ────────────────────
        t_lo = t_tx_guess - self._SEARCH_WIN_S / 2
        t_hi = t_tx_guess + self._SEARCH_WIN_S / 2
        idx_lo = max(0,   bisect.bisect_left(self._tx_times, t_lo, 0, n))
        idx_hi = min(n-1, bisect.bisect_right(self._tx_times, t_hi, 0, n))

        if idx_hi - idx_lo < 2:
            return None

        # ── Nächsten TX-Punkt suchen (XY-Abstand) ────────────────────
        best_idx = idx_lo
        best_dist = float('inf')
        for i in range(idx_lo, idx_hi + 1):
            dx = self._tx_x[i] - ix
            dy = self._tx_y[i] - iy
            d = dx*dx + dy*dy
            if d < best_dist:
                best_dist = d
                best_idx = i

        # ── Match-Ergebnisse in Metriken schreiben ───────────────────
        m = self._metrics
        m.ist_x = ix
        m.ist_y = iy
        m.ist_z = iz
        m.robot_time = getattr(feedback, 'robot_time',
                               getattr(feedback, 'timestamp', 0.0))
        m.soll_x = self._tx_x[best_idx]
        m.soll_y = self._tx_y[best_idx]
        m.soll_z = self._tx_z[best_idx]
        m.soll_v = self._tx_vel[best_idx]

        err_x = ix - m.soll_x
        err_y = iy - m.soll_y
        err_z = iz - m.soll_z
        m.error_tcp_pos = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        m.error_tcp_speed = m.soll_v - self._ist_velocity

        self._error_history.append(m.error_tcp_pos)

        # ── Tangential/Normal-Zerlegung ──────────────────────────────
        tang_err = 0.0
        norm_err = m.error_tcp_pos
        weight = 1.0
        speed = max(m.soll_v, self._EPS)

        if self.evaluate_direction:
            lo_t = max(idx_lo, best_idx - self._TANG_SPAN)
            hi_t = min(idx_hi, best_idx + self._TANG_SPAN)
            dtx = self._tx_x[hi_t] - self._tx_x[lo_t]
            dty = self._tx_y[hi_t] - self._tx_y[lo_t]
            tang_len = math.sqrt(dtx*dtx + dty*dty)

            if tang_len > self._EPS:
                tang = (dtx / tang_len, dty / tang_len)
                tang_err = err_x * tang[0] + err_y * tang[1]
                norm_sq = max(0.0, err_x**2 + err_y**2 - tang_err**2)
                norm_err = math.sqrt(norm_sq)
                tang_mag = abs(tang_err)
                weight = tang_mag / (
                    tang_mag + self._ALPHA_NORMAL * norm_err + self._EPS
                )
                if tang_mag < self._MIN_TANG_MM:
                    weight = 0.0
            else:
                weight = 0.0

        m.tang_error = tang_err
        m.norm_error = norm_err
        m.estimator_weight = weight

        # ── Delay-Schätzung ──────────────────────────────────────────
        if self.evaluate_direction and abs(tang_err) >= self._MIN_TANG_MM:
            tang_time = tang_err / speed
            T_d_raw = t_actual - self._tx_times[best_idx]
            T_d_refined = T_d_raw - tang_time
        else:
            T_d_raw = t_actual - self._tx_times[best_idx]
            T_d_refined = T_d_raw

        T_d_refined = max(self._T_DELAY_MIN,
                          min(self._T_DELAY_MAX, T_d_refined))

        # ── Beschleunigungs-Erkennung ────────────────────────────────
        accel_detected = False
        if self._prev_speed is not None and self._prev_t_rx is not None:
            dt_rx = t_rx - self._prev_t_rx
            if dt_rx > self._EPS:
                accel = abs(speed - self._prev_speed) / dt_rx
                if accel > self._ACCEL_THR_MMS2:
                    accel_detected = True
        self._prev_speed = speed
        self._prev_t_rx = t_rx
        m.accel_detected = accel_detected

        ema = self._EMA_FAST if accel_detected else self._EMA_SLOW

        # ── EMA-Update ───────────────────────────────────────────────
        with self._lock:
            correction = ema * weight * (T_d_refined - self._t_delay)
            self._t_delay += correction
            self._n_estimator_updates += 1
            m.t_delay_ms = self._t_delay * 1000.0
            m.t_delay_output_ms = self._t_delay_output * 1000.0

        # ── Offset-Änderungsrate ─────────────────────────────────────
        now = t_rx
        if self._prev_t_delay_time > 0:
            dt_offset = now - self._prev_t_delay_time
            if dt_offset > self._EPS:
                m.offset_rate_ms_per_s = (
                    (self._t_delay - self._prev_t_delay) * 1000.0 / dt_offset
                )
        self._prev_t_delay = self._t_delay
        self._prev_t_delay_time = now

        # ── Debug-Info ───────────────────────────────────────────────
        debug = EstimatorDebug(
            timestamp=t_rx,
            t_delay_ms=self._t_delay * 1000.0,
            t_delay_output_ms=self._t_delay_output * 1000.0,
            tang_err_mm=tang_err,
            norm_err_mm=norm_err,
            weight=weight,
            accel_detected=accel_detected,
            ema_used=ema,
            correction_ms=correction * 1000.0,
        )
        self._last_est_debug = debug
        return debug

    # ═════════════════════════════════════════════════════════════════
    # Sync-Bewertung + Logging: 50Hz (EGM-Loop)
    # ═════════════════════════════════════════════════════════════════

    def update(self, sample: Optional[EgmSample],
               feedback: Optional[EgmFeedback],
               buffer_depth: int = 0,
               buffer_time_s: float = 0.0):
        """
        Bewertet den Sync-Zustand und erzeugt Cycle-Log.
        Wird im EGM-Loop aufgerufen (~50Hz).

        Parameter sample ist reserviert für zukünftige direkte
        Nutzung. Das eigentliche Position-Matching passiert bereits
        in on_feedback_received() bei 250Hz — hier wird nur bewertet
        und geloggt.
        """
        now = bridge_now()
        self._total_updates += 1

        m = self._metrics
        m.timestamp = now
        m.buffer_depth = buffer_depth
        m.buffer_time_s = buffer_time_s
        m.total_updates = self._total_updates

        with self._lock:
            m.t_delay_ms = self._t_delay * 1000.0
            m.t_delay_output_ms = self._t_delay_output * 1000.0

        old_level = m.sync_level
        self._evaluate_sync_level()

        if m.sync_level != old_level:
            logger.info("SYNC: Level %s → %s | %s",
                        old_level.value, m.sync_level.value, m.sync_reason)
            if self._on_level_change:
                self._on_level_change(m.sync_level, m.sync_reason)

        if self._on_cycle_log:
            self._emit_cycle_log()

    # ── Sync-Level-Bewertung ──────────────────────────────────────────────────

    def _evaluate_sync_level(self):
        if self._total_updates < self._warmup_cycles:
            self._metrics.sync_level = SyncLevel.OK
            self._metrics.sync_reason = (
                f"Warmup ({self._total_updates}/{self._warmup_cycles})"
            )
            return

        m = self._metrics
        c = self.cfg
        reasons = []

        offset_warn_ms = getattr(c, 'offset_warn_ms', 350.0)
        offset_degrade_ms = getattr(c, 'offset_degrade_ms', 500.0)
        offset_stop_ms = getattr(c, 'offset_stop_ms', 800.0)
        offset_rate_warn = getattr(c, 'offset_rate_warn_ms_per_s', 200.0)
        norm_error_warn_mm = getattr(c, 'norm_error_warn_mm', 5.0)
        norm_error_degrade_mm = getattr(c, 'norm_error_degrade_mm', 15.0)
        tracking_stop_mm = getattr(c, 'tracking_stop_mm', 50.0)

        stop_confirm = getattr(c, 'stop_confirm_cycles', 1)
        degrade_confirm = getattr(c, 'degrade_confirm_cycles', 3)
        warn_confirm = getattr(c, 'warn_confirm_cycles', 5)

        # ── STOP ─────────────────────────────────────────────────────
        if m.t_delay_ms > offset_stop_ms:
            reasons.append(f"Offset {m.t_delay_ms:.0f}ms > Stop {offset_stop_ms:.0f}ms")
        if m.error_tcp_pos > tracking_stop_mm:
            reasons.append(f"TCP-Error {m.error_tcp_pos:.1f}mm > Stop {tracking_stop_mm:.0f}mm")

        if reasons:
            self._consecutive_stop += 1
            self._consecutive_degrade = 0
            self._consecutive_warn = 0
            if self._consecutive_stop >= stop_confirm:
                m.sync_level = SyncLevel.STOP
                m.sync_reason = "; ".join(reasons)
                return
        else:
            self._consecutive_stop = 0

        # ── DEGRADE ──────────────────────────────────────────────────
        if m.t_delay_ms > offset_degrade_ms:
            reasons.append(f"Offset {m.t_delay_ms:.0f}ms > Degrade {offset_degrade_ms:.0f}ms")
        if m.norm_error > norm_error_degrade_mm:
            reasons.append(f"Norm-Error {m.norm_error:.1f}mm > Degrade {norm_error_degrade_mm:.0f}mm")

        if reasons:
            self._consecutive_degrade += 1
            self._consecutive_warn = 0
            if self._consecutive_degrade >= degrade_confirm:
                m.sync_level = SyncLevel.DEGRADE
                m.sync_reason = "; ".join(reasons)
                self._degrade_count += 1
                m.degrade_count = self._degrade_count
                return
        else:
            self._consecutive_degrade = 0

        # ── WARN ─────────────────────────────────────────────────────
        if m.t_delay_ms > offset_warn_ms:
            reasons.append(f"Offset {m.t_delay_ms:.0f}ms > Warn {offset_warn_ms:.0f}ms")
        if abs(m.offset_rate_ms_per_s) > offset_rate_warn:
            reasons.append(f"Offset-Rate {m.offset_rate_ms_per_s:.0f}ms/s > Warn {offset_rate_warn:.0f}ms/s")
        if m.norm_error > norm_error_warn_mm:
            reasons.append(f"Norm-Error {m.norm_error:.1f}mm > Warn {norm_error_warn_mm:.0f}ms")

        if reasons:
            self._consecutive_warn += 1
            if self._consecutive_warn >= warn_confirm:
                m.sync_level = SyncLevel.WARN
                m.sync_reason = "; ".join(reasons)
                self._warn_count += 1
                m.warn_count = self._warn_count
                return
        else:
            self._consecutive_warn = 0

        # ── OK ───────────────────────────────────────────────────────
        m.sync_level = SyncLevel.OK
        m.sync_reason = ""

    # ── Cycle-Log ────────────────────────────────────────────────────

    def _emit_cycle_log(self):
        m = self._metrics
        entry = CycleLogEntry(
            timestamp=m.timestamp,
            robot_time=m.robot_time,
            ist_x=m.ist_x,
            ist_y=m.ist_y,
            ist_z=m.ist_z,
            soll_x=m.soll_x,
            soll_y=m.soll_y,
            soll_z=m.soll_z,
            soll_v=m.soll_v,
            error_tcp_speed=m.error_tcp_speed,
            error_tcp_pos=m.error_tcp_pos,
            offset_ms=m.t_delay_output_ms,
            sync_level=m.sync_level.value,
            tang_error=m.tang_error,
            norm_error=m.norm_error,
        )
        self._on_cycle_log(entry)

    # ═════════════════════════════════════════════════════════════════
    # Kompatibilität + Properties
    # ═════════════════════════════════════════════════════════════════

    @property
    def metrics(self) -> SyncMetrics:
        return self._metrics

    @property
    def sync_level(self) -> SyncLevel:
        return self._metrics.sync_level

    @property
    def is_ok(self) -> bool:
        return self._metrics.sync_level == SyncLevel.OK

    @property
    def needs_degrade(self) -> bool:
        return self._metrics.sync_level in (SyncLevel.DEGRADE, SyncLevel.STOP)

    @property
    def needs_stop(self) -> bool:
        return self._metrics.sync_level == SyncLevel.STOP

    @property
    def last_debug(self) -> Optional[EstimatorDebug]:
        """Letztes Estimator-Debug (für Telemetry)."""
        return self._last_est_debug

    def estimate_clock_offset(self, bridge_time: float,
                              robot_time_ms: float) -> float:
        """DEPRECATED: Offset wird jetzt positionsbasiert berechnet."""
        return bridge_time * 1000 - robot_time_ms

    # ── Reset ────────────────────────────────────────────────────────

    def reset(self):
        """Metriken und Estimator zurücksetzen (z.B. bei Jobstart)."""
        init_delay = self.est_cfg.t_delay_init_ms / 1000.0

        self._metrics = SyncMetrics()
        self._error_history.clear()
        self._tx_times.clear()
        self._tx_x.clear()
        self._tx_y.clear()
        self._tx_z.clear()
        self._tx_vel.clear()

        with self._lock:
            self._t_delay = init_delay
            self._t_delay_output = init_delay

        self._prev_speed = None
        self._prev_t_rx = None
        self._prev_t_delay = init_delay
        self._prev_t_delay_time = 0.0
        self._prev_fb_pos = None
        self._prev_fb_time = None
        self._ist_velocity = 0.0
        self._total_updates = 0
        self._warn_count = 0
        self._degrade_count = 0
        self._n_estimator_updates = 0
        self._last_est_debug = None

        # FIX: Consecutive-Counter explizit auf 0 zurücksetzen —
        # sie könnten vom vorherigen Job erhöht sein.
        self._consecutive_warn = 0
        self._consecutive_degrade = 0
        self._consecutive_stop = 0

        logger.info("SYNC: Reset (T_delay_init=%.0fms, Estimator=%s)",
                     init_delay * 1000,
                     "aktiv" if self.est_cfg.enabled else "aus")

    # ── Snapshot ─────────────────────────────────────────────────────

    def snapshot(self) -> dict:
        m = self._metrics
        return {
            "sync_level": m.sync_level.value,
            "sync_reason": m.sync_reason,
            "t_delay_ms": round(m.t_delay_ms, 2),
            "t_delay_output_ms": round(m.t_delay_output_ms, 2),
            "offset_rate_ms_per_s": round(m.offset_rate_ms_per_s, 2),
            "error_tcp_pos": round(m.error_tcp_pos, 3),
            "error_tcp_speed": round(m.error_tcp_speed, 2),
            "tang_error": round(m.tang_error, 3),
            "norm_error": round(m.norm_error, 3),
            "soll_v": round(m.soll_v, 2),
            "estimator_weight": round(m.estimator_weight, 3),
            "accel_detected": m.accel_detected,
            "buffer_depth": m.buffer_depth,
            "buffer_time_s": round(m.buffer_time_s, 3),
            "warn_count": m.warn_count,
            "degrade_count": m.degrade_count,
            "total_updates": m.total_updates,
            "estimator_updates": self._n_estimator_updates,
            "estimator_enabled": self.est_cfg.enabled,
        }
