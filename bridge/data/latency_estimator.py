from __future__ import annotations
# latency_estimator.py — Adaptiver Latenz-Estimator (v5 — absolute Delay-Messung)
#
# METHODIK (wie in EGM_Error_Test.py bewährt):
#   1. Suchfenster ±40ms um erwarteten Match-Zeitpunkt
#   2. XY-Matching: nächster TX-Punkt im Zeitfenster
#   3. Tangentialvektor via asymmetrischer Zentraldifferenz
#   4. Fehlerzerlegung: tang_err → Zeitversatz, norm_err → Pfadabw.
#   5. ABSOLUTE Delay-Messung: T_d_raw = t_actual - tx_time
#      Verfeinert: T_d_refined = T_d_raw - (tang_err / speed)
#   6. Gewichtung: weight = tang_mag / (tang_mag + alpha * norm_err)
#   7. EMA konvergiert auf T_d_refined (Zielwert), nicht blinde Integration
#   8. Output-Fading
#
# FIXES (v5, gegenüber v4):
#   - KERNFIX: EMA-Update konvergiert auf absoluten Messwert T_d_refined
#     statt blinder Integration von tang_err/velocity. Ohne diesen Anker
#     driftet T_delay in einen Positive-Feedback-Loop (→ MIN oder MAX).
#   - alpha_weight: 10 → 5 (konsistent mit EGM_Error_Test._ALPHA_NORMAL)
#   - MIN_TANG_MM: Neuer Filter — tang_err < 0.5mm wird übersprungen
#     (Rauschen bei Stillstand/Ecken, kein brauchbares Timing-Signal)
#   - Plausibilitäts-Obergrenze: T_DELAY_MAX_S statt T_DELAY_MAX_S * 1.5
#   - norm_err: Pythagoras statt Kreuzprodukt (immer positiv, wie EGM_Error_Test)
#   - Neue Debug-Felder: t_d_raw_ms, t_d_refined_ms für Diagnose
#   - Boundary-Bug-Fix aus v4 beibehalten

import math
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, NamedTuple

from .clock import bridge_now

logger = logging.getLogger("egm.latency")

# Skip-Gründe für CSV-Logging
SKIP_NONE          = ""           # Update durchgeführt
SKIP_DISABLED      = "disabled"
SKIP_TX_EMPTY      = "tx_empty"   # TX-Buffer noch zu dünn
SKIP_NO_MATCH      = "no_match"   # Kein TX-Punkt im Zeitfenster
SKIP_TANGENT_SHORT = "tang_short" # Tangentialvektor < 0.1mm (Stillstand/Kurve)
SKIP_TANG_SMALL    = "tang_small" # |tang_err| < MIN_TANG_MM (Rauschen)
SKIP_VELOCITY_LOW  = "vel_low"    # velocity am Matchpunkt < MIN_VELOCITY
SKIP_PLAUSIBILITY  = "implausibel"# Gemessener Delay ausserhalb [MIN,MAX]


class TxEntry(NamedTuple):
    bridge_time: float
    x: float
    y: float
    z: float
    velocity: float


@dataclass
class EstimatorDebug:
    """
    Pro-RX-Packet Debug-Snapshot → estimator.csv

    Lese-Anleitung:
      skip_reason != ""   → warum kein EMA-Update (kein match, tang_short, etc.)
      match_found = 0     → kein TX-Punkt im Suchfenster
      match_found = 1     → Match OK; wenn weight=0: skip_reason erklärt warum
      match_dist_mm       → XY-Abstand zum Matchpunkt (gut: < 5mm)
      match_age_ms        → Zeit seit dem TX dieses Punkts = grobe Delay-Messung
      t_d_raw_ms          → Absolut gemessener Delay (rx_time - offset_back - tx_time)
      t_d_refined_ms      → Verfeinert um Tangential-Zeitoffset
      tang_err_mm         → Tangentialfehler (Zeitversatz-Signal)
      norm_err_mm         → Normalfehler (Pfadabweichung, kein Delay-Signal)
      weight              → Einfluss auf EMA [0..1] (0 = eingefroren)
      correction_ms       → Korrektur die auf T_delay angewandt wurde
      t_delay_raw_ms      → T_delay nach EMA (vor Fading)
      t_delay_output_ms   → aktiver Lookahead im Planner
    """
    timestamp: float = 0.0
    t_delay_raw_ms: float = 0.0
    t_delay_output_ms: float = 0.0
    t_d_raw_ms: float = 0.0         # NEU: absolute Messung
    t_d_refined_ms: float = 0.0     # NEU: verfeinerte Messung
    tang_err_mm: float = 0.0
    norm_err_mm: float = 0.0
    weight: float = 0.0
    correction_ms: float = 0.0
    ema_rate_used: float = 0.0
    match_found: bool = False
    match_dist_mm: float = 0.0
    match_age_ms: float = 0.0
    skip_reason: str = SKIP_NONE   # Leerstring = Update wurde durchgeführt


class LatencyEstimator:
    """
    Adaptiver Latenz-Estimator — absolute Delay-Messung + tang/norm-Gewichtung.

    Konvergiert auf den absolut gemessenen Delay T_d_refined (wie EGM_Error_Test.py),
    NICHT auf blinde Integration von Tangentialfehlern.

    Thread-Sicherheit:
        record_tx()     → egm-loop Thread (bei JEDEM send_target)
        update()        → egm-rx Thread
        get_lookahead() → egm-loop Thread
    """

    SEARCH_WINDOW_S:   float = 0.040   # ±40ms Suchfenster
    TANGENT_HALF_SPAN: int   = 3       # Zentraldifferenz ±3 Punkte
    MIN_VELOCITY_MM_S: float = 1.0     # Kein Update bei Stillstand
    MIN_TANG_MM:       float = 0.5     # Mindest-Tangentialfehler für Update
    T_DELAY_MIN_S:     float = 0.005
    T_DELAY_MAX_S:     float = 0.500
    FAST_TRIGGER_MM:   float = 5.0     # Tang-Fehler-Schwelle für EMA_FAST

    def __init__(self,
                 t_delay_init_ms: float = 50.0,
                 ema_slow: float = 0.04,
                 ema_fast: float = 0.25,
                 ema_output: float = 0.08,
                 offset_back_ms: float = 5.0,
                 alpha_weight: float = 5.0,
                 tx_buffer_size: int = 500,
                 enabled: bool = True):
        self.enabled = enabled
        self.ema_slow = ema_slow
        self.ema_fast = ema_fast
        self.ema_output = ema_output
        self.offset_back_s = offset_back_ms / 1000.0
        self.alpha_weight = alpha_weight

        self._T_delay: float = t_delay_init_ms / 1000.0
        self._T_delay_output: float = t_delay_init_ms / 1000.0

        self._tx_log: deque[TxEntry] = deque(maxlen=tx_buffer_size)

        self._last_debug = EstimatorDebug(
            t_delay_raw_ms=t_delay_init_ms,
            t_delay_output_ms=t_delay_init_ms,
            skip_reason=SKIP_TX_EMPTY,
        )

        self._update_count: int = 0
        self._match_count: int = 0
        self._skip_count: int = 0
        self._skip_reasons: dict[str, int] = {}

        logger.info(
            "LATENCY-EST: Init — T_delay_init=%.0fms "
            "ema slow=%.3f fast=%.3f output=%.3f "
            "offset_back=%.0fms alpha=%.1f buf=%d enabled=%s",
            t_delay_init_ms, ema_slow, ema_fast, ema_output,
            offset_back_ms, alpha_weight, tx_buffer_size, enabled)

    # ── TX-Aufzeichnung ──────────────────────────────────────

    def record_tx(self, timestamp: float, x: float, y: float,
                  z: float, velocity: float):
        """
        Im egm-loop-Thread bei JEDEM send_target() aufrufen —
        auch bei velocity=0 (Hold/Gap). Sonst bleibt der TX-Buffer
        während Pausen leer und das Suchfenster findet keinen Match.
        """
        self._tx_log.append(TxEntry(
            bridge_time=timestamp, x=x, y=y, z=z, velocity=velocity))

    # ── Update ───────────────────────────────────────────────

    def update(self, rx_x: float, rx_y: float, rx_z: float,
               rx_timestamp: float) -> EstimatorDebug:
        """
        Verarbeitet einen RX-Feedback-Wert.
        Gibt immer einen vollständigen EstimatorDebug zurück —
        skip_reason erklärt warum kein Update stattfand.
        """
        self._update_count += 1
        debug = EstimatorDebug(timestamp=rx_timestamp)
        debug.t_delay_raw_ms = self._T_delay * 1000.0
        debug.t_delay_output_ms = self._T_delay_output * 1000.0

        def _skip(reason: str) -> EstimatorDebug:
            debug.skip_reason = reason
            self._skip_count += 1
            self._skip_reasons[reason] = self._skip_reasons.get(reason, 0) + 1
            # Output-Fading läuft weiter
            self._T_delay_output += self.ema_output * (
                self._T_delay - self._T_delay_output)
            debug.t_delay_output_ms = self._T_delay_output * 1000.0
            self._last_debug = debug
            return debug

        if not self.enabled:
            return _skip(SKIP_DISABLED)

        min_entries = self.TANGENT_HALF_SPAN * 2 + 2
        if len(self._tx_log) < min_entries:
            return _skip(SKIP_TX_EMPTY)

        # Einmaliger Snapshot — konsistente Indizes für alle Zugriffe
        log_snapshot = list(self._tx_log)

        # ── 1. Suchfenster ────────────────────────────────────
        expected_tx_time = (rx_timestamp
                            - self._T_delay
                            - self.offset_back_s)
        t_min = expected_tx_time - self.SEARCH_WINDOW_S
        t_max = expected_tx_time + self.SEARCH_WINDOW_S

        # ── 2. XY-Matching ────────────────────────────────────
        match_result = self._find_xy_match(rx_x, rx_y, t_min, t_max,
                                           log_snapshot)
        if match_result is None:
            return _skip(SKIP_NO_MATCH)

        match_idx, match_entry, match_dist = match_result
        debug.match_found = True
        debug.match_dist_mm = match_dist
        debug.match_age_ms = (rx_timestamp - match_entry.bridge_time) * 1000.0

        # ── 3. Tangentialvektor ────────────────────────────────
        tangent = self._compute_tangent(match_idx, log_snapshot)
        if tangent is None:
            return _skip(SKIP_TANGENT_SHORT)

        tang_x, tang_y = tangent

        # ── 4. Fehlerzerlegung ────────────────────────────────
        err_x = rx_x - match_entry.x
        err_y = rx_y - match_entry.y

        # Tangentialfehler (vorzeichenbehaftet):
        # positiv = Roboter ist dem Soll voraus (entlang Fahrtrichtung)
        # negativ = Roboter hinkt hinterher
        tang_err = err_x * tang_x + err_y * tang_y

        # Normalfehler (immer positiv): Pythagoras-Zerlegung
        norm_sq = max(0.0, err_x * err_x + err_y * err_y - tang_err * tang_err)
        norm_err = math.sqrt(norm_sq)

        tang_mag = abs(tang_err)

        debug.tang_err_mm = tang_err
        debug.norm_err_mm = norm_err

        # ── 5. Mindest-Tangentialfehler ────────────────────────
        # Zu kleine Tangentialfehler sind Rauschen, kein Timing-Signal
        if tang_mag < self.MIN_TANG_MM:
            debug.weight = 0.0
            return _skip(SKIP_TANG_SMALL)

        # ── 6. Gewichtung ─────────────────────────────────────
        denom = tang_mag + self.alpha_weight * norm_err
        weight = tang_mag / denom if denom > 1e-9 else 0.0
        debug.weight = weight

        # ── 7. Geschwindigkeit ────────────────────────────────
        if match_entry.velocity < self.MIN_VELOCITY_MM_S:
            return _skip(SKIP_VELOCITY_LOW)

        # ── 8. Absolute Delay-Messung (KERNFIX v5) ────────────
        # t_actual: wann der Roboter physisch an der RX-Position war
        t_actual = rx_timestamp - self.offset_back_s

        # T_d_raw: grobe Delay-Messung = (Roboter-Zeitpunkt - TX-Sendezeit)
        T_d_raw = t_actual - match_entry.bridge_time

        # Tangential-Zeitoffset: Roboter ist tang_err mm vor/hinter Soll
        # bei speed mm/s → das sind tang_err/speed Sekunden Versatz
        tang_time_s = tang_err / match_entry.velocity

        # T_d_refined: verfeinerte Delay-Schätzung
        # Wenn Roboter voraus ist (tang_err > 0), ist der wahre Delay kleiner
        T_d_refined = T_d_raw - tang_time_s
        T_d_refined = max(self.T_DELAY_MIN_S,
                          min(self.T_DELAY_MAX_S, T_d_refined))

        debug.t_d_raw_ms = T_d_raw * 1000.0
        debug.t_d_refined_ms = T_d_refined * 1000.0

        # ── 9. Plausibilitätscheck ─────────────────────────────
        # Prüfe ob die Messung im plausiblen Bereich liegt
        if T_d_refined <= self.T_DELAY_MIN_S or T_d_refined >= self.T_DELAY_MAX_S:
            return _skip(SKIP_PLAUSIBILITY)

        # ── 10. EMA-Rate ──────────────────────────────────────
        fast_trigger = (tang_mag * weight) > self.FAST_TRIGGER_MM
        ema_rate = self.ema_fast if fast_trigger else self.ema_slow
        debug.ema_rate_used = ema_rate

        # ── 11. T_delay EMA-Update — konvergiert auf Messwert ─
        # correction = Differenz zum ZIEL (T_d_refined), nicht blinde Integration
        old_delay = self._T_delay
        correction_s = T_d_refined - self._T_delay
        self._T_delay += weight * ema_rate * correction_s
        self._T_delay = max(self.T_DELAY_MIN_S,
                            min(self.T_DELAY_MAX_S, self._T_delay))

        debug.correction_ms = correction_s * 1000.0

        # ── 12. Output-Fading ─────────────────────────────────
        self._T_delay_output += self.ema_output * (
            self._T_delay - self._T_delay_output)

        debug.t_delay_raw_ms = self._T_delay * 1000.0
        debug.t_delay_output_ms = self._T_delay_output * 1000.0
        debug.skip_reason = SKIP_NONE
        self._match_count += 1

        if (self._update_count <= 5
                or self._update_count % 500 == 0
                or abs(correction_s * 1000) > 20.0):
            logger.info(
                "LATENCY-EST: #%d T_delay %.1f→%.1fms "
                "(out=%.1fms w=%.3f tang=%+.1fmm norm=%.1fmm "
                "T_d_raw=%.1fms T_d_ref=%.1fms corr=%+.1fms "
                "age=%.0fms dist=%.1fmm %s)",
                self._update_count,
                old_delay * 1000.0, self._T_delay * 1000.0,
                self._T_delay_output * 1000.0,
                weight, tang_err, norm_err,
                T_d_raw * 1000.0, T_d_refined * 1000.0,
                correction_s * 1000.0, debug.match_age_ms,
                debug.match_dist_mm,
                "FAST" if fast_trigger else "slow")

        self._last_debug = debug
        return debug

    # ── Lookahead ────────────────────────────────────────────

    def get_lookahead(self) -> float:
        if not self.enabled:
            return 0.0
        return self._T_delay_output

    # ── Hilfsmethoden ────────────────────────────────────────

    def _find_xy_match(self, rx_x: float, rx_y: float,
                       t_min: float, t_max: float,
                       log_snapshot: list
                       ) -> Optional[tuple[int, TxEntry, float]]:
        """
        Nächster TX-Punkt (XY) im Zeitfenster [t_min, t_max].
        Gibt (idx, entry, dist_mm) oder None zurück.
        """
        best_idx = None
        best_entry = None
        best_dist_sq = float('inf')

        for i, entry in enumerate(log_snapshot):
            if entry.bridge_time < t_min:
                continue
            if entry.bridge_time > t_max:
                break
            dx = entry.x - rx_x
            dy = entry.y - rx_y
            dist_sq = dx * dx + dy * dy
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i
                best_entry = entry

        if best_idx is None:
            return None
        return best_idx, best_entry, math.sqrt(best_dist_sq)

    def _compute_tangent(self, idx: int,
                         log_snapshot: list
                         ) -> Optional[tuple[float, float]]:
        """
        XY-Einheitstangentialvektor am Index idx via Zentraldifferenz.

        Asymmetrisches Clamping an Puffergrenzen (v4 Bugfix beibehalten).
        Gibt None nur zurück wenn der resultierende Vektor < 0.1mm ist
        (echter Stillstand, keine Richtung bestimmbar).
        """
        n = len(log_snapshot)
        span = self.TANGENT_HALF_SPAN

        i_back = max(0, idx - span)
        i_fwd = min(n - 1, idx + span)

        if i_back == i_fwd:
            return None

        p_back = log_snapshot[i_back]
        p_fwd = log_snapshot[i_fwd]

        dx = p_fwd.x - p_back.x
        dy = p_fwd.y - p_back.y
        mag = math.sqrt(dx * dx + dy * dy)

        if mag < 0.1:
            return None

        return (dx / mag, dy / mag)

    # ── Properties & Snapshot ────────────────────────────────

    @property
    def T_delay_ms(self) -> float:
        return self._T_delay * 1000.0

    @property
    def T_delay_output_ms(self) -> float:
        return self._T_delay_output * 1000.0

    @property
    def last_debug(self) -> EstimatorDebug:
        return self._last_debug

    def snapshot(self) -> dict:
        d = self._last_debug
        return {
            "enabled": self.enabled,
            "t_delay_ms": round(self._T_delay * 1000.0, 2),
            "t_delay_output_ms": round(self._T_delay_output * 1000.0, 2),
            "lookahead_ms": round(self.get_lookahead() * 1000.0, 2),
            "update_count": self._update_count,
            "match_count": self._match_count,
            "skip_count": self._skip_count,
            "skip_reasons": dict(self._skip_reasons),
            "match_rate_pct": round(
                100.0 * self._match_count / max(1, self._update_count), 1),
            "last_skip_reason": d.skip_reason,
            "last_weight": round(d.weight, 3),
            "last_tang_err_mm": round(d.tang_err_mm, 3),
            "last_norm_err_mm": round(d.norm_err_mm, 3),
            "last_correction_ms": round(d.correction_ms, 3),
            "last_t_d_raw_ms": round(d.t_d_raw_ms, 3),
            "last_t_d_refined_ms": round(d.t_d_refined_ms, 3),
            "tx_log_size": len(self._tx_log),
        }
