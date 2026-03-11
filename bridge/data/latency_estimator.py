from __future__ import annotations
# latency_estimator.py — Adaptiver Latenz-Estimator für Closed-Loop-Korrektur
#
# Schätzt laufend die Gesamtlatenz T_delay (TX-Senden → physische Ankunft)
# und passt den Lookahead der TrajectoryPlanner automatisch an.
#
# Methodik (aus EGM_Error_Test.py):
#   1. XY-Matching: Für jedes RX-Paket wird im TX-Log der räumlich
#      nächstgelegene Punkt innerhalb eines ±SEARCH_WINDOW_MS Fensters
#      gesucht. Z wird ignoriert (3D-Druck: hauptsächlich XY-Bewegung).
#   2. Tangentialvektor via Zentraldifferenz ±TANGENT_HALF_SPAN Punkte.
#   3. Fehlerzerlegung am Matchpunkt:
#      - Tangentialfehler → Zeitversatz (starkes Signal für Delay-Update)
#      - Normalfehler     → Bahnabweichung / Controller-Blending (schwach)
#   4. Gewichtung: weight = tang_mag / (tang_mag + alpha * norm_err)
#      Verhindert dass Ecken/Kurven den Offset springen lassen.
#   5. EMA-Update: EMA_SLOW (~0.04) im Normalbetrieb,
#                  EMA_FAST (~0.25) bei erkannter "Beschleunigung"
#   6. Output-Fading: separater geglätteter T_delay_output, damit
#      Sprünge im Lookahead gedämpft werden.
#
# Aufruf-Reihenfolge:
#   # Im TX-Pfad (egm-loop Thread):
#   estimator.record_tx(sample.timestamp, sample.x, sample.y, sample.z,
#                       sample.velocity)
#
#   # Im RX-Callback (egm-rx Thread):
#   debug = estimator.update(feedback.x, feedback.y, feedback.z,
#                            feedback.timestamp)
#   telemetry.log_estimator(debug)

import math
import logging
from collections import deque
from dataclasses import dataclass
from typing import Optional, NamedTuple

from .clock import bridge_now

logger = logging.getLogger("egm.latency")


# ── TX-Log Entry ─────────────────────────────────────────────

class TxEntry(NamedTuple):
    """Eintrag im TX-Log des Estimators."""
    bridge_time: float   # bridge_now() beim Senden
    x: float
    y: float
    z: float
    velocity: float      # mm/s (aus EgmSample, für tang→zeit-Umrechnung)


# ── Debug-Snapshot ───────────────────────────────────────────

@dataclass
class EstimatorDebug:
    """
    Vollständige Debug-Informationen nach einem update()-Aufruf.
    Wird in estimator.csv geloggt, ermöglicht Post-hoc-Analyse.
    """
    timestamp: float = 0.0           # bridge_now() des RX-Packets
    t_delay_raw_ms: float = 0.0      # T_delay nach EMA-Update (vor Fading)
    t_delay_output_ms: float = 0.0   # Geglätteter Output → aktiver Lookahead
    tang_err_mm: float = 0.0         # Tangentialfehler am Matchpunkt (mm)
    norm_err_mm: float = 0.0         # Normalfehler am Matchpunkt (mm)
    weight: float = 0.0              # Update-Gewicht [0..1]
    correction_ms: float = 0.0       # Angewandte Zeitkorrektur (ms)
    ema_rate_used: float = 0.0       # Verwendete EMA-Rate (slow/fast)
    match_found: bool = False        # TX-Matchpunkt gefunden
    match_dist_mm: float = 0.0       # XY-Distanz zum Matchpunkt (mm)
    match_age_ms: float = 0.0        # Alter des Matchpunkts = aktuell gemessenes Delay


class LatencyEstimator:
    """
    Adaptiver Latenz-Estimator für Closed-Loop-Lookahead-Korrektur.

    Der Estimator lernt die tatsächliche Controller-Latenz T_delay und stellt
    get_lookahead() bereit, das in TrajectoryPlanner.klipper_time_now()
    den fixen offset_s ersetzt.

    Thread-Sicherheit:
        record_tx()  → wird aus dem egm-loop Thread aufgerufen
        update()     → wird aus dem egm-rx Thread aufgerufen
        get_lookahead() → wird aus egm-loop aufgerufen

        _T_delay_output ist ein einzelner Python-float. Unter CPython
        sind atomare Float-Reads durch den GIL geschützt (ausreichend
        für diesen nicht-sicherheitskritischen Pfad).
    """

    # Unveränderliche Konstanten
    SEARCH_WINDOW_S: float = 0.040   # ±40ms Suchfenster um erwarteten Match
    TANGENT_HALF_SPAN: int = 3       # Zentraldifferenz über ±3 Punkte
    MIN_VELOCITY_MM_S: float = 1.0   # Darunter kein Update (Stillstand)
    T_DELAY_MIN_S: float = 0.005     # Clamp-Untergrenze (5ms)
    T_DELAY_MAX_S: float = 0.500     # Clamp-Obergrenze (500ms)
    FAST_TRIGGER_MM: float = 5.0     # Schwellwert für EMA_FAST-Aktivierung

    def __init__(self,
                 t_delay_init_ms: float = 50.0,
                 ema_slow: float = 0.04,
                 ema_fast: float = 0.25,
                 ema_output: float = 0.08,
                 offset_back_ms: float = 5.0,
                 alpha_weight: float = 10.0,
                 tx_buffer_size: int = 500,
                 enabled: bool = True):
        """
        Parameters
        ----------
        t_delay_init_ms : float
            Initialwert für T_delay in ms (z.B. aus time_offset_ms Config).
        ema_slow : float
            EMA-Lernrate im Normalbetrieb (~0.04 ≈ 25-Sample-Halbwertszeit).
        ema_fast : float
            EMA-Lernrate bei erkannter Beschleunigung (~0.25 ≈ 3-Sample).
        ema_output : float
            EMA-Rate für Output-Fading (~0.08). Dämpft Sprünge im Lookahead.
        offset_back_ms : float
            Geschätzte Netzwerklatenz RX-Paket → Python (fix, ~5ms).
            Wird beim Suchfenster-Zentrum abgezogen.
        alpha_weight : float
            Gewichtungsfaktor α. Höher = stärkere Unterdrückung bei Normal-
            fehler. Standard: 10.
        tx_buffer_size : int
            Anzahl TX-Einträge im Ringbuffer (~500 bei 50Hz = 10s).
        enabled : bool
            Estimator aktiv. Bei False gibt get_lookahead() immer 0.0 zurück
            und der TrajectoryPlanner fällt auf seinen fixen offset_s zurück.
        """
        self.enabled = enabled
        self.ema_slow = ema_slow
        self.ema_fast = ema_fast
        self.ema_output = ema_output
        self.offset_back_s = offset_back_ms / 1000.0
        self.alpha_weight = alpha_weight

        # Interner Zustand
        self._T_delay: float = t_delay_init_ms / 1000.0
        self._T_delay_output: float = t_delay_init_ms / 1000.0

        # TX-Ringbuffer (zeitlich sortiert — deque.append = rechts)
        self._tx_log: deque[TxEntry] = deque(maxlen=tx_buffer_size)

        # Letzter Debug-Snapshot (threadsicher genug für Logging)
        self._last_debug = EstimatorDebug(
            t_delay_raw_ms=t_delay_init_ms,
            t_delay_output_ms=t_delay_init_ms,
        )

        # Statistik
        self._update_count: int = 0
        self._match_count: int = 0
        self._skip_count: int = 0

        logger.info(
            "LATENCY-EST: Initialisiert — T_delay_init=%.0fms, "
            "EMA slow=%.3f fast=%.3f output=%.3f, "
            "offset_back=%.0fms, alpha=%.1f, buf=%d, enabled=%s",
            t_delay_init_ms, ema_slow, ema_fast, ema_output,
            offset_back_ms, alpha_weight, tx_buffer_size, enabled)

    # ── TX-Aufzeichnung (egm-loop Thread) ────────────────────

    def record_tx(self, timestamp: float, x: float, y: float,
                  z: float, velocity: float):
        """
        Speichert ein gesendetes Sample im TX-Ringbuffer.

        Muss für jeden gesendeten EGM-Sample aufgerufen werden —
        direkt nach egm.send_target(), bevor der RX-Callback
        verarbeitet werden kann.
        """
        self._tx_log.append(TxEntry(
            bridge_time=timestamp,
            x=x, y=y, z=z,
            velocity=velocity,
        ))

    # ── Update (egm-rx Thread) ───────────────────────────────

    def update(self, rx_x: float, rx_y: float, rx_z: float,
               rx_timestamp: float) -> EstimatorDebug:
        """
        Verarbeitet einen neuen RX-Feedback-Wert, aktualisiert T_delay.

        Wird im egm-rx-Thread aus bridge._on_feedback() aufgerufen.

        Parameters
        ----------
        rx_x, rx_y, rx_z  : Ist-Position vom Roboter (mm)
        rx_timestamp       : Empfangszeit (bridge_now(), gleiche Clock wie TX)

        Returns
        -------
        EstimatorDebug : Debug-Daten für Telemetrie-Logging
        """
        self._update_count += 1
        debug = EstimatorDebug(timestamp=rx_timestamp)

        # Immer aktuellen Wert setzen, auch wenn kein Update
        debug.t_delay_raw_ms = self._T_delay * 1000.0
        debug.t_delay_output_ms = self._T_delay_output * 1000.0

        if not self.enabled:
            self._last_debug = debug
            return debug

        min_entries = self.TANGENT_HALF_SPAN * 2 + 2
        if len(self._tx_log) < min_entries:
            self._skip_count += 1
            self._last_debug = debug
            return debug

        # ── 1. Suchfenster ────────────────────────────────────
        # Erwarteter Match-Zeitpunkt: RX-Zeit minus geschätzte Latenz
        expected_tx_time = (rx_timestamp
                            - self._T_delay
                            - self.offset_back_s)
        t_min = expected_tx_time - self.SEARCH_WINDOW_S
        t_max = expected_tx_time + self.SEARCH_WINDOW_S

        # ── 2. XY-Matching ────────────────────────────────────
        match_idx, match_entry, match_dist_sq = self._find_xy_match(
            rx_x, rx_y, t_min, t_max)

        if match_idx is None:
            # Kein passender TX-Punkt in Fenster
            self._skip_count += 1
            self._last_debug = debug
            return debug

        self._match_count += 1
        debug.match_found = True
        debug.match_dist_mm = math.sqrt(match_dist_sq)
        debug.match_age_ms = (rx_timestamp - match_entry.bridge_time) * 1000.0

        # ── 3. Tangentialvektor (Zentraldifferenz ±3 Punkte) ──
        tangent = self._compute_tangent(match_idx)
        if tangent is None:
            # Nicht genug Nachbarn oder Punkt auf Stillstand
            self._skip_count += 1
            self._last_debug = debug
            return debug
        tang_x, tang_y = tangent   # XY-Einheitsvektor

        # ── 4. Fehlerzerlegung ────────────────────────────────
        # Vektor vom TX-Matchpunkt zur aktuellen Ist-Position
        err_x = rx_x - match_entry.x
        err_y = rx_y - match_entry.y

        # Tangentialprojektion: positiv = Roboter hinkt hinter TX her
        tang_err = err_x * tang_x + err_y * tang_y

        # Normalprojektion: Bahnabweichung (Normalvektor = tangent rotiert 90°)
        norm_err = err_x * (-tang_y) + err_y * tang_x

        tang_mag = abs(tang_err)
        norm_mag = abs(norm_err)

        debug.tang_err_mm = tang_err
        debug.norm_err_mm = norm_err

        # ── 5. Gewichtung ─────────────────────────────────────
        # Große Normalfehler (Ecken/Blending) → weight → 0 → Estimator einfrieren
        denom = tang_mag + self.alpha_weight * norm_mag
        weight = tang_mag / denom if denom > 1e-9 else 0.0
        debug.weight = weight

        # ── 6. Geschwindigkeit prüfen ─────────────────────────
        velocity = match_entry.velocity
        if velocity < self.MIN_VELOCITY_MM_S:
            # Stillstand: kein sinnvolles Delay-Signal
            self._last_debug = debug
            return debug

        # ── 7. Korrektur ──────────────────────────────────────
        # tang_err (mm) / velocity (mm/s) = Zeitversatz (s)
        # Positiv: Roboter ist hinter TX → T_delay wird erhöht
        correction_s = tang_err / velocity
        debug.correction_ms = correction_s * 1000.0

        # ── 8. EMA-Rate bestimmen ─────────────────────────────
        # "Beschleunigung" = großer gewichteter Tang-Fehler → schnelle Anpassung
        fast_trigger = (tang_mag * weight) > self.FAST_TRIGGER_MM
        ema_rate = self.ema_fast if fast_trigger else self.ema_slow
        debug.ema_rate_used = ema_rate

        # ── 9. T_delay EMA-Update ─────────────────────────────
        old_delay = self._T_delay
        self._T_delay += weight * ema_rate * correction_s

        # Plausibilitätsgrenzen
        self._T_delay = max(self.T_DELAY_MIN_S,
                            min(self.T_DELAY_MAX_S, self._T_delay))

        # ── 10. Output-Fading ─────────────────────────────────
        # Separater geglätteter Ausgabewert — verhindert Sprünge > 10ms/Zyklus
        self._T_delay_output = (
            self.ema_output * self._T_delay_output
            + (1.0 - self.ema_output) * self._T_delay
        )

        # Debug aktualisieren (nach Update)
        debug.t_delay_raw_ms = self._T_delay * 1000.0
        debug.t_delay_output_ms = self._T_delay_output * 1000.0

        if (self._update_count <= 5
                or self._update_count % 500 == 0
                or abs(correction_s * 1000) > 10.0):
            logger.debug(
                "LATENCY-EST: #%d T_delay %.1f→%.1fms "
                "(output=%.1fms, w=%.2f, tang=%.1fmm, norm=%.1fmm, "
                "corr=%+.1fms, %s)",
                self._update_count,
                old_delay * 1000.0, self._T_delay * 1000.0,
                self._T_delay_output * 1000.0,
                weight, tang_err, norm_err,
                correction_s * 1000.0,
                "FAST" if fast_trigger else "slow")

        self._last_debug = debug
        return debug

    # ── Lookahead-Ausgabe (egm-loop Thread) ──────────────────

    def get_lookahead(self) -> float:
        """
        Aktueller Lookahead-Wert in Sekunden (positiv).

        Ersetzt den fixen offset_s im TrajectoryPlanner:
            klipper_time_now = bridge_time + lookahead + t0_offset

        Je größer T_delay, desto weiter voraus wird im geplanten
        Pfad nachgeschaut → kompensiert die Controller-Latenz.

        Bei enabled=False: gibt 0.0 zurück → Planner fällt auf
        seinen eigenen offset_s zurück.
        """
        if not self.enabled:
            return 0.0
        return self._T_delay_output

    # ── Hilfsmethoden ────────────────────────────────────────

    def _find_xy_match(self, rx_x: float, rx_y: float,
                       t_min: float, t_max: float
                       ) -> tuple[Optional[int], Optional[TxEntry], float]:
        """
        Sucht im TX-Log den Eintrag mit geringstem XY-Abstand
        zur RX-Position, mit bridge_time im Fenster [t_min, t_max].

        Der TX-Log ist zeitlich aufsteigend sortiert (deque, append rechts).
        Wir scannen vorwärts und brechen ab sobald t_max überschritten.

        Returns
        -------
        (list_index, TxEntry, dist_sq)  oder  (None, None, inf)
        """
        log_list = list(self._tx_log)  # Snapshot für stabilen Indexzugriff

        best_idx: Optional[int] = None
        best_entry: Optional[TxEntry] = None
        best_dist_sq = float('inf')

        for i, entry in enumerate(log_list):
            if entry.bridge_time < t_min:
                continue
            if entry.bridge_time > t_max:
                break   # Zeitlich sortiert → kein Match mehr möglich

            dx = entry.x - rx_x
            dy = entry.y - rx_y
            dist_sq = dx * dx + dy * dy

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i
                best_entry = entry

        return best_idx, best_entry, best_dist_sq

    def _compute_tangent(self, idx: int) -> Optional[tuple[float, float]]:
        """
        Berechnet den XY-Einheitstangentialvektor am Index idx
        via Zentraldifferenz: tangent = p[idx+span] - p[idx-span].

        Returns
        -------
        (tx, ty) Einheitsvektor, oder None wenn:
          - nicht genug Nachbarn vorhanden
          - Segmentlänge < 0.1mm (Stillstand, keine Richtung bestimmbar)
        """
        log_list = list(self._tx_log)
        span = self.TANGENT_HALF_SPAN

        i_back = idx - span
        i_fwd = idx + span

        if i_back < 0 or i_fwd >= len(log_list):
            return None

        p_back = log_list[i_back]
        p_fwd = log_list[i_fwd]

        dx = p_fwd.x - p_back.x
        dy = p_fwd.y - p_back.y
        mag = math.sqrt(dx * dx + dy * dy)

        if mag < 0.1:   # < 0.1mm → Stillstand, keine Richtung
            return None

        return (dx / mag, dy / mag)

    # ── Properties & Snapshot ────────────────────────────────

    @property
    def T_delay_ms(self) -> float:
        """Aktuelles T_delay in ms (vor Output-Fading)."""
        return self._T_delay * 1000.0

    @property
    def T_delay_output_ms(self) -> float:
        """Geglätteter Output-Wert in ms (=aktiver Lookahead)."""
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
            "match_rate_pct": round(
                100.0 * self._match_count / max(1, self._update_count), 1),
            "last_weight": round(d.weight, 3),
            "last_tang_err_mm": round(d.tang_err_mm, 3),
            "last_norm_err_mm": round(d.norm_err_mm, 3),
            "last_correction_ms": round(d.correction_ms, 3),
            "tx_log_size": len(self._tx_log),
        }
