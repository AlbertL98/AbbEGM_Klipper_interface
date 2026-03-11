# sync_monitor.py — Synchronisationsüberwachung
#
# CodingPlan §B3: Clock-Mapping, Lag, Jitter, Tracking-Error
# CodingPlan §E:  Messbare Sync-Kriterien, Reaktion bei Unsynchronität
#
# Berechnet laufend:
#   - Tracking-Error (Soll/Ist Position)
#   - Lag (Zeitversatz Soll/Ist)
#   - Jitter (p95/p99 des Lag)
#   - Buffer-Gesundheit
#   - Sync-Level: OK → WARN → DEGRADE → STOP
#
# CLOCK-FIX: Alle Timestamps nutzen jetzt bridge_now() aus clock.py.
#   Vorher gemischte Nutzung von time.monotonic() und time.perf_counter()
#   die auf Windows unterschiedliche Epochen haben.

import math
import logging
import enum
from dataclasses import dataclass, field
from collections import deque
from typing import Optional, NamedTuple

from .clock import bridge_now
from .egm_client import EgmFeedback
from .trajectory_planner import EgmSample
from .config import SyncConfig

logger = logging.getLogger("egm.sync")


class SentSample(NamedTuple):
    """Gesendetes Sample für positionsbasiertes Lag-Matching."""
    bridge_time: float   # bridge_now() beim Senden
    x: float
    y: float
    z: float


class SyncLevel(enum.Enum):
    OK = "OK"
    WARN = "WARN"
    DEGRADE = "DEGRADE"
    STOP = "STOP"


@dataclass
class SyncMetrics:
    """Aktuelle Sync-Metriken (Snapshot)."""
    timestamp: float = 0.0

    # Tracking-Error (mm)
    tracking_error_mm: float = 0.0
    tracking_error_x: float = 0.0
    tracking_error_y: float = 0.0
    tracking_error_z: float = 0.0
    tracking_error_max: float = 0.0     # Historisches Maximum
    tracking_error_avg: float = 0.0     # Gleitender Durchschnitt

    # Lag (ms)
    lag_ms: float = 0.0
    lag_avg_ms: float = 0.0

    # Jitter (ms)
    jitter_p95_ms: float = 0.0
    jitter_p99_ms: float = 0.0

    # Buffer
    buffer_depth: int = 0
    buffer_time_s: float = 0.0

    # Gesamtbewertung
    sync_level: SyncLevel = SyncLevel.OK
    sync_reason: str = ""

    # Zähler
    warn_count: int = 0
    degrade_count: int = 0
    clamp_count: int = 0
    underrun_count: int = 0

    # Latenz-Estimator (wird von EgmBridge nach jedem RX-Update gesetzt)
    t_delay_ms: float = 0.0          # Aktuelles T_delay (geglättet, ms)
    estimator_weight: float = 0.0    # Letztes Update-Gewicht [0..1]


class SyncMonitor:
    """
    Überwacht die Synchronität zwischen geplanter
    Trajektorie und Roboter-Feedback.

    Wird in jedem EGM-Zyklus mit dem letzten gesendeten
    Sample und dem letzten empfangenen Feedback gefüttert.
    """

    def __init__(self, config: SyncConfig,
                 history_size: int = 1000):
        self.cfg = config
        self._history_size = history_size

        # Metriken-Zustand
        self._metrics = SyncMetrics()

        # Histogramm-Daten für Jitter-Berechnung
        self._lag_history: deque[float] = deque(maxlen=history_size)
        self._error_history: deque[float] = deque(maxlen=history_size)

        # Ringbuffer gesendeter Samples für positionsbasiertes Lag-Matching
        lag_buf_size = getattr(config, 'lag_buffer_size', 200)
        self._sent_samples: deque[SentSample] = deque(maxlen=lag_buf_size)

        # Gleitender Durchschnitt (EMA)
        self._ema_alpha = 0.05  # ~20 Samples Halbwertszeit
        self._tracking_ema = 0.0
        self._lag_ema = 0.0

        # Zähler
        self._total_updates = 0
        self._warn_count = 0
        self._degrade_count = 0
        self._warmup_cycles = 100  # ~2s bei 50Hz — keine Bewertung davor

        # Callbacks
        self._on_level_change: Optional[callable] = None

    def set_level_change_callback(self, callback):
        """Callback wenn sich das Sync-Level ändert."""
        self._on_level_change = callback

    # ── Sent-Sample-Aufzeichnung (für Lag-Matching) ──────────

    def record_sent_sample(self, sample: EgmSample):
        """
        Speichert ein gesendetes Sample im Ringbuffer.
        Muss in jedem EGM-Zyklus NACH dem Senden aufgerufen werden.

        Verwendet bridge_now() — gleiche Clock wie feedback.timestamp
        (wird in egm_client._receive_loop ebenfalls mit bridge_now() gesetzt).
        """
        self._sent_samples.append(SentSample(
            bridge_time=bridge_now(),
            x=sample.x, y=sample.y, z=sample.z,
        ))

    # ── Update (wird jeden Zyklus aufgerufen) ────────────────

    def update(self, sample: Optional[EgmSample],
               feedback: Optional[EgmFeedback],
               buffer_depth: int = 0,
               buffer_time_s: float = 0.0):
        """
        Aktualisiert alle Metriken.

        Parameters:
            sample:       Letzter gesendeter Sollwert
            feedback:     Letztes empfangenes Feedback
            buffer_depth: Aktuelle Queue-Tiefe
            buffer_time_s: Geschätzte Queue-Zeit
        """
        now = bridge_now()
        self._total_updates += 1
        self._metrics.timestamp = now
        self._metrics.buffer_depth = buffer_depth
        self._metrics.buffer_time_s = buffer_time_s

        in_warmup = self._total_updates <= self._warmup_cycles

        if sample and feedback:
            self._update_tracking_error(sample, feedback)

            # Lag erst NACH Warmup aufzeichnen — davor ist der
            # sent_samples-Buffer zu dünn für sinnvolles Matching
            # und Ausreißer vergiften die Jitter-Berechnung.
            if not in_warmup:
                self._update_lag(sample, feedback)

        # Lag-History beim Warmup-Ende leeren (falls doch was
        # reingerutscht ist, z.B. durch Timing-Varianz)
        if self._total_updates == self._warmup_cycles:
            self._lag_history.clear()
            self._lag_ema = 0.0

        # Jitter berechnen
        self._update_jitter()

        # Bewertung
        old_level = self._metrics.sync_level
        self._evaluate_sync_level()

        if self._metrics.sync_level != old_level:
            logger.info("SYNC: Level %s → %s | %s",
                         old_level.value,
                         self._metrics.sync_level.value,
                         self._metrics.sync_reason)
            if self._on_level_change:
                self._on_level_change(self._metrics.sync_level,
                                      self._metrics.sync_reason)

    def _update_tracking_error(self, sample: EgmSample,
                                feedback: EgmFeedback):
        """Berechnet den Tracking-Error (Soll-Ist)."""
        dx = sample.x - feedback.x
        dy = sample.y - feedback.y
        dz = sample.z - feedback.z
        error = math.sqrt(dx*dx + dy*dy + dz*dz)

        self._metrics.tracking_error_x = dx
        self._metrics.tracking_error_y = dy
        self._metrics.tracking_error_z = dz
        self._metrics.tracking_error_mm = error

        # EMA
        self._tracking_ema = (
            self._ema_alpha * error +
            (1 - self._ema_alpha) * self._tracking_ema
        )
        self._metrics.tracking_error_avg = self._tracking_ema

        # Maximum
        self._metrics.tracking_error_max = max(
            self._metrics.tracking_error_max, error
        )

        self._error_history.append(error)

    def _update_lag(self, sample: EgmSample, feedback: EgmFeedback):
        """
        Berechnet den Lag positionsbasiert (nur Bridge-Clock).

        Sucht im Ringbuffer das gesendete Sample dessen Position
        am besten zur Feedback-Position passt. Der Lag ist die
        Zeitdifferenz — alles via bridge_now().

        RÜCKWÄRTS-SUCHE (neueste zuerst):
          Bei Stillstand oder langsamer Fahrt haben viele Samples
          fast identische Positionen. Vorwärts-Suche würde das
          älteste matchen → falscher Lag. Rückwärts-Suche findet
          das jüngste passende Sample → stabiler Lag und Jitter.

          Abbruch sobald die Distanz wieder deutlich steigt
          (wir sind zeitlich am Match vorbei).
        """
        if len(self._sent_samples) < 2:
            return

        fx, fy, fz = feedback.x, feedback.y, feedback.z

        best_dist_sq = float('inf')
        best_time = 0.0
        rising_count = 0

        for sent in reversed(self._sent_samples):
            dx = sent.x - fx
            dy = sent.y - fy
            dz = sent.z - fz
            dist_sq = dx*dx + dy*dy + dz*dz

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_time = sent.bridge_time
                rising_count = 0
            else:
                rising_count += 1
                # Wenn die Distanz 5 Samples in Folge steigt
                # sind wir am Match-Punkt vorbei → Abbruch
                if rising_count >= 5:
                    break

        lag = (feedback.timestamp - best_time) * 1000  # → ms

        # Plausibilitätscheck
        if lag < 0 or best_dist_sq > 2500:  # 50mm = 2500mm²
            return

        self._metrics.lag_ms = lag

        self._lag_ema = (
            self._ema_alpha * lag +
            (1 - self._ema_alpha) * self._lag_ema
        )
        self._metrics.lag_avg_ms = self._lag_ema
        self._lag_history.append(lag)

    def _update_jitter(self):
        """Berechnet p95/p99 Jitter aus der Lag-History."""
        if len(self._lag_history) < 10:
            return

        sorted_lags = sorted(self._lag_history)
        n = len(sorted_lags)
        self._metrics.jitter_p95_ms = sorted_lags[int(n * 0.95)]
        self._metrics.jitter_p99_ms = sorted_lags[int(min(n * 0.99, n - 1))]

    # ── Sync-Level-Bewertung (§E) ────────────────────────────

    def _evaluate_sync_level(self):
        """
        Bewertet den Sync-Zustand anhand aller Metriken.
        Hierarchisch: STOP > DEGRADE > WARN > OK
        """
        # Warmup: Keine Bewertung in den ersten Zyklen
        if self._total_updates < self._warmup_cycles:
            self._metrics.sync_level = SyncLevel.OK
            self._metrics.sync_reason = (
                f"Warmup ({self._total_updates}/{self._warmup_cycles})")
            return

        m = self._metrics
        c = self.cfg
        reasons = []

        # ── STOP-Bedingungen ──
        if m.tracking_error_mm > c.tracking_stop_mm:
            reasons.append(f"Tracking {m.tracking_error_mm:.1f}mm > "
                           f"Stop-Limit {c.tracking_stop_mm}mm")
        if m.lag_ms > c.lag_stop_ms:
            reasons.append(f"Lag {m.lag_ms:.1f}ms > "
                           f"Stop-Limit {c.lag_stop_ms}ms")
        if m.jitter_p99_ms > c.jitter_p99_stop_ms:
            reasons.append(f"Jitter p99 {m.jitter_p99_ms:.1f}ms > "
                           f"Stop-Limit {c.jitter_p99_stop_ms}ms")

        if reasons:
            m.sync_level = SyncLevel.STOP
            m.sync_reason = "; ".join(reasons)
            return

        # ── DEGRADE-Bedingungen ──
        if m.tracking_error_mm > c.tracking_degrade_mm:
            reasons.append(f"Tracking {m.tracking_error_mm:.1f}mm > "
                           f"Degrade-Limit {c.tracking_degrade_mm}mm")
        if m.lag_ms > c.lag_degrade_ms:
            reasons.append(f"Lag {m.lag_ms:.1f}ms > "
                           f"Degrade-Limit {c.lag_degrade_ms}ms")

        if reasons:
            m.sync_level = SyncLevel.DEGRADE
            m.sync_reason = "; ".join(reasons)
            self._degrade_count += 1
            m.degrade_count = self._degrade_count
            return

        # ── WARN-Bedingungen ──
        if m.tracking_error_mm > c.tracking_warn_mm:
            reasons.append(f"Tracking {m.tracking_error_mm:.1f}mm > "
                           f"Warn-Limit {c.tracking_warn_mm}mm")
        if m.lag_ms > c.lag_warn_ms:
            reasons.append(f"Lag {m.lag_ms:.1f}ms > "
                           f"Warn-Limit {c.lag_warn_ms}ms")
        if m.jitter_p99_ms > c.jitter_p99_warn_ms:
            reasons.append(f"Jitter p99 {m.jitter_p99_ms:.1f}ms > "
                           f"Warn-Limit {c.jitter_p99_warn_ms}ms")

        if reasons:
            m.sync_level = SyncLevel.WARN
            m.sync_reason = "; ".join(reasons)
            self._warn_count += 1
            m.warn_count = self._warn_count
            return

        # ── OK ──
        m.sync_level = SyncLevel.OK
        m.sync_reason = ""

    # ── Clock-Mapping (§B3) ──────────────────────────────────

    def estimate_clock_offset(self, bridge_time: float,
                              robot_time_ms: float) -> float:
        """
        DEPRECATED: Lag wird jetzt positionsbasiert berechnet
        (siehe _update_lag). Clock-Offset ist nicht mehr nötig.

        Beibehalten für API-Kompatibilität — gibt weiterhin
        die rohe Differenz zurück, wird aber nicht mehr
        für die Sync-Bewertung verwendet.
        """
        bridge_ms = bridge_time * 1000
        return bridge_ms - robot_time_ms

    # ── Zugriff ──────────────────────────────────────────────

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
        return self._metrics.sync_level in (SyncLevel.DEGRADE,
                                            SyncLevel.STOP)

    @property
    def needs_stop(self) -> bool:
        return self._metrics.sync_level == SyncLevel.STOP

    def reset(self):
        """Metriken zurücksetzen (z.B. bei Jobstart)."""
        self._metrics = SyncMetrics()
        self._lag_history.clear()
        self._error_history.clear()
        self._sent_samples.clear()
        self._tracking_ema = 0.0
        self._lag_ema = 0.0
        self._total_updates = 0
        logger.info("SYNC: Metriken zurückgesetzt")

    def snapshot(self) -> dict:
        m = self._metrics
        return {
            "sync_level": m.sync_level.value,
            "sync_reason": m.sync_reason,
            "tracking_error_mm": round(m.tracking_error_mm, 3),
            "tracking_error_avg": round(m.tracking_error_avg, 3),
            "tracking_error_max": round(m.tracking_error_max, 3),
            "lag_ms": round(m.lag_ms, 2),
            "lag_avg_ms": round(m.lag_avg_ms, 2),
            "jitter_p95_ms": round(m.jitter_p95_ms, 2),
            "jitter_p99_ms": round(m.jitter_p99_ms, 2),
            "buffer_depth": m.buffer_depth,
            "buffer_time_s": round(m.buffer_time_s, 3),
            "warn_count": m.warn_count,
            "degrade_count": m.degrade_count,
            "total_updates": self._total_updates,
            "t_delay_ms": round(m.t_delay_ms, 2),
            "estimator_weight": round(m.estimator_weight, 3),
        }
