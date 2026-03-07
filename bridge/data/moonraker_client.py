from __future__ import annotations
# moonraker_client.py — Moonraker-Websocket-Client für Extruder-E-Wert
#
# Verbindet sich per Websocket mit der Moonraker-API und pollt aktiv
# `motion_report.live_position` in konfigurierbarem Intervall.
#
# Abhängigkeit: websocket-client (pip install websocket-client)
#
# Zwei Modi:
#   1. Subscription (Baseline): ~4 Hz (Moonraker-intern, nicht steuerbar)
#   2. Active Polling (Default): Sendet printer.objects.query im eigenen
#      Takt (z.B. 50ms = 20 Hz). Deutlich höhere Auflösung.
#
# Der Polling-Thread sendet JSON-RPC Queries über den offenen Websocket.
# Antworten werden im on_message-Handler verarbeitet — gleicher Codepath
# für Subscribe-Pushes und Poll-Antworten.

import json
import time
import logging
import threading
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger("egm.moonraker")


@dataclass
class ExtruderState:
    """Thread-sicherer Snapshot des letzten E-Werts."""
    e_value: float           # Extruder-Position (mm)
    timestamp: float         # time.monotonic() beim Empfang
    age_ms: float = 0.0      # Alter seit letztem Update (berechnet bei Abfrage)


class MoonrakerClient:
    """
    Websocket-Client für Moonraker mit aktivem Polling.

    Subscribt auf `motion_report.live_position` als Baseline (~4 Hz)
    und pollt zusätzlich aktiv per `printer.objects.query` im
    konfigurierbaren Intervall (Default 50ms = 20 Hz).

    Thread-sicher: Läuft in eigenem Thread, der Bridge-Loop
    fragt get_extruder_state() im EGM-Takt ab.
    """

    def __init__(self,
                 host: str = "127.0.0.1",
                 port: int = 7125,
                 reconnect_interval_s: float = 3.0,
                 poll_interval_ms: float = 50.0):
        self.host = host
        self.port = port
        self.reconnect_interval_s = reconnect_interval_s
        self.poll_interval_ms = poll_interval_ms
        self._poll_interval_s = poll_interval_ms / 1000.0

        self._running = False
        self._connected = False
        self._thread: Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._ws = None  # websocket.WebSocketApp

        # Shared State (Lock-geschützt)
        self._lock = threading.Lock()
        self._e_value: float = 0.0
        self._e_timestamp: float = 0.0
        self._update_count: int = 0
        self._poll_count: int = 0

        self._last_error: Optional[str] = None
        self._rpc_id: int = 0
        self._rpc_id_lock = threading.Lock()

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Startet den Websocket-Thread mit Auto-Reconnect."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._connection_loop, daemon=True,
            name="moonraker-ws"
        )
        self._thread.start()
        logger.info("MOONRAKER: Client gestartet → ws://%s:%d/websocket "
                     "(poll: %.0fms)",
                     self.host, self.port, self.poll_interval_ms)

    def stop(self):
        """Beendet Websocket-Verbindung und Threads."""
        self._running = False
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
        if self._poll_thread:
            self._poll_thread.join(timeout=2.0)
        if self._thread:
            self._thread.join(timeout=5.0)
        logger.info("MOONRAKER: Client gestoppt "
                     "(%d Updates, %d Polls gesendet)",
                     self._update_count, self._poll_count)

    def get_extruder_state(self) -> ExtruderState:
        """
        Gibt den aktuellen E-Wert zurück.

        Thread-sicher — wird vom EGM-Loop aufgerufen.
        age_ms zeigt wie alt der Wert ist (für Telemetrie).
        """
        now = time.monotonic()
        with self._lock:
            age_ms = ((now - self._e_timestamp) * 1000.0
                      if self._e_timestamp > 0 else -1.0)
            return ExtruderState(
                e_value=self._e_value,
                timestamp=self._e_timestamp,
                age_ms=age_ms,
            )

    @property
    def connected(self) -> bool:
        return self._connected

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "connected": self._connected,
                "host": self.host,
                "port": self.port,
                "poll_interval_ms": self.poll_interval_ms,
                "e_value": round(self._e_value, 6),
                "update_count": self._update_count,
                "poll_count": self._poll_count,
                "last_error": self._last_error,
            }

    # ── Connection Loop (mit Reconnect) ──────────────────────

    def _connection_loop(self):
        """
        Äußere Schleife: Verbindet sich, bei Abbruch → Reconnect.
        Verwendet websocket-client (WebSocketApp) für stabiles Threading.
        """
        while self._running:
            try:
                import websocket  # websocket-client Paket

                url = f"ws://{self.host}:{self.port}/websocket"
                logger.info("MOONRAKER: Verbinde mit %s ...", url)

                self._ws = websocket.WebSocketApp(
                    url,
                    on_open=self._on_ws_open,
                    on_message=self._on_ws_message,
                    on_error=self._on_ws_error,
                    on_close=self._on_ws_close,
                )

                self._ws.run_forever(
                    ping_interval=10,
                    ping_timeout=5,
                    reconnect=0,
                )

            except ImportError:
                logger.error(
                    "MOONRAKER: 'websocket-client' Paket nicht installiert! "
                    "→ pip install websocket-client"
                )
                self._last_error = "websocket-client nicht installiert"
                self._running = False
                return

            except Exception as e:
                logger.warning("MOONRAKER: Verbindungsfehler: %s", e)
                self._last_error = str(e)

            self._connected = False

            if self._running:
                logger.info("MOONRAKER: Reconnect in %.1fs...",
                             self.reconnect_interval_s)
                deadline = time.monotonic() + self.reconnect_interval_s
                while self._running and time.monotonic() < deadline:
                    time.sleep(0.25)

    # ── Active Polling ───────────────────────────────────────

    def _poll_loop(self):
        """
        Polling-Thread: Sendet printer.objects.query Requests
        über den offenen Websocket im konfigurierten Intervall.

        Antworten werden im on_message-Handler verarbeitet —
        gleicher Codepath wie Subscribe-Pushes.
        """
        logger.info("MOONRAKER: Polling gestartet (%.0fms = %.0f Hz)",
                     self.poll_interval_ms,
                     1000.0 / self.poll_interval_ms)

        query_objects = {"motion_report": ["live_position"]}

        while self._running and self._connected:
            loop_start = time.perf_counter()

            try:
                with self._rpc_id_lock:
                    self._rpc_id += 1
                    rid = self._rpc_id

                query_msg = {
                    "jsonrpc": "2.0",
                    "method": "printer.objects.query",
                    "params": {"objects": query_objects},
                    "id": rid,
                }
                self._ws.send(json.dumps(query_msg))
                self._poll_count += 1

            except Exception as e:
                if self._running and self._connected:
                    logger.warning("MOONRAKER: Poll-Fehler: %s", e)
                break

            # Intervall einhalten
            elapsed = time.perf_counter() - loop_start
            sleep_time = self._poll_interval_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.debug("MOONRAKER: Polling beendet")

    # ── Websocket Callbacks ──────────────────────────────────

    def _on_ws_open(self, ws):
        """Verbindung steht → Subscribe + Polling starten."""
        self._connected = True
        self._last_error = None
        logger.info("MOONRAKER: Websocket verbunden")

        # Subscribe als Baseline (~4 Hz Pushes)
        with self._rpc_id_lock:
            self._rpc_id += 1
            rid = self._rpc_id

        subscribe_msg = {
            "jsonrpc": "2.0",
            "method": "printer.objects.subscribe",
            "params": {"objects": {"motion_report": ["live_position"]}},
            "id": rid,
        }
        ws.send(json.dumps(subscribe_msg))
        logger.info("MOONRAKER: Subscribe gesendet (Baseline ~4 Hz)")

        # Polling-Thread starten (höhere Rate)
        if self.poll_interval_ms > 0:
            self._poll_thread = threading.Thread(
                target=self._poll_loop, daemon=True,
                name="moonraker-poll"
            )
            self._poll_thread.start()

    def _on_ws_message(self, ws, message: str):
        """
        Eingehende Nachricht verarbeiten.

        Verarbeitet drei Arten von Nachrichten:
          1. Subscribe-Antwort (initialer Status)
          2. Poll-Antwort (printer.objects.query Result)
          3. Push-Notification (notify_status_update)
        """
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            return

        # Fall 1 + 2: RPC-Antwort (Subscribe oder Query)
        result = msg.get("result")
        if result and isinstance(result, dict):
            status = result.get("status")
            if status:
                self._process_status(status)
            return

        # Fall 3: Push-Notification vom Subscribe
        method = msg.get("method", "")
        if method == "notify_status_update":
            params = msg.get("params", [])
            if params and isinstance(params, list) and len(params) > 0:
                self._process_status(params[0])

    def _on_ws_error(self, ws, error):
        """Websocket-Fehler."""
        self._connected = False
        self._last_error = str(error)
        logger.warning("MOONRAKER: WS-Fehler: %s", error)

    def _on_ws_close(self, ws, close_status_code, close_msg):
        """Verbindung geschlossen."""
        self._connected = False
        logger.info("MOONRAKER: Verbindung geschlossen (code=%s)",
                     close_status_code)

    # ── Status-Verarbeitung ──────────────────────────────────

    def _process_status(self, status: dict):
        """
        Extrahiert E-Wert aus motion_report.live_position.

        Moonraker liefert: motion_report.live_position = [x, y, z, e]
        mit voller Float-Präzision. Wir brauchen nur e (Index 3).
        """
        motion = status.get("motion_report", {})
        position = motion.get("live_position")

        # Fallback: toolhead.position
        if position is None:
            toolhead = status.get("toolhead", {})
            position = toolhead.get("position")

        if position and isinstance(position, (list, tuple)) and len(position) >= 4:
            e_value = float(position[3])
            now = time.monotonic()

            with self._lock:
                self._e_value = e_value
                self._e_timestamp = now
                self._update_count += 1

            if self._update_count <= 3 or self._update_count % 1000 == 0:
                logger.debug("MOONRAKER: E=%.6f (Update #%d)",
                             e_value, self._update_count)
