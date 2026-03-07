from __future__ import annotations
# moonraker_client.py — Moonraker-Websocket-Subscriber für Extruder-E-Wert
#
# Verbindet sich per Websocket mit der Moonraker-API und subscribt
# auf `motion_report.live_position`. Hält den letzten E-Wert + Timestamp
# in einem thread-sicheren Shared State, den der EGM-Loop in jedem
# Zyklus abfragen kann.
#
# Abhängigkeit: websocket-client (pip install websocket-client)
#
# Warum motion_report statt toolhead?
#   - toolhead.position updated nur sporadisch (bei State-Change)
#     und liefert gerundete Werte (ganze mm)
#   - motion_report.live_position streamt kontinuierlich (~250ms)
#     mit voller Float-Präzision direkt aus dem Stepper-Tracking
#
# Moonraker JSON-RPC:
#   1. Connect:  ws://<host>:<port>/websocket
#   2. Subscribe: printer.objects.subscribe
#        → {"objects": {"motion_report": ["live_position"]}}
#   3. Updates:   notify_status_update
#        → motion_report.live_position = [x, y, z, e]

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
    Websocket-Client für Moonraker.

    Subscribt auf `motion_report.live_position` und extrahiert den
    E-Wert (4. Komponente: [x, y, z, e]).

    motion_report.live_position liefert — im Gegensatz zu
    toolhead.position — kontinuierliche Updates (~250ms) mit
    voller Float-Präzision direkt aus Klippers Stepper-Tracking.

    Thread-sicher: Läuft in eigenem Thread, der Bridge-Loop
    fragt get_extruder_state() im EGM-Takt ab.

    Reconnect bei Verbindungsverlust mit konfigurierbarem Intervall.
    """

    def __init__(self,
                 host: str = "127.0.0.1",
                 port: int = 7125,
                 reconnect_interval_s: float = 3.0,
                 subscribe_objects: Optional[dict] = None):
        self.host = host
        self.port = port
        self.reconnect_interval_s = reconnect_interval_s
        self.subscribe_objects = subscribe_objects or {
            "motion_report": ["live_position"],
        }

        self._running = False
        self._connected = False
        self._thread: Optional[threading.Thread] = None
        self._ws = None  # websocket.WebSocketApp

        # Shared State (Lock-geschützt)
        self._lock = threading.Lock()
        self._e_value: float = 0.0
        self._e_timestamp: float = 0.0
        self._update_count: int = 0

        self._last_error: Optional[str] = None
        self._rpc_id: int = 0

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
        logger.info("MOONRAKER: Client gestartet → ws://%s:%d/websocket",
                     self.host, self.port)

    def stop(self):
        """Beendet Websocket-Verbindung und Thread."""
        self._running = False
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=5.0)
        logger.info("MOONRAKER: Client gestoppt (%d Updates empfangen)",
                     self._update_count)

    def get_extruder_state(self) -> ExtruderState:
        """
        Gibt den aktuellen E-Wert zurück.

        Thread-sicher — wird vom EGM-Loop aufgerufen.
        age_ms zeigt wie alt der Wert ist (für Telemetrie).
        """
        now = time.monotonic()
        with self._lock:
            age_ms = (now - self._e_timestamp) * 1000.0 if self._e_timestamp > 0 else -1.0
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
                "e_value": round(self._e_value, 6),
                "update_count": self._update_count,
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

                # run_forever() blockiert bis Verbindung geschlossen wird
                self._ws.run_forever(
                    ping_interval=10,
                    ping_timeout=5,
                    reconnect=0,  # Wir machen Reconnect selbst
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
                # Interruptible sleep
                deadline = time.monotonic() + self.reconnect_interval_s
                while self._running and time.monotonic() < deadline:
                    time.sleep(0.25)

    # ── Websocket Callbacks ──────────────────────────────────

    def _on_ws_open(self, ws):
        """Verbindung steht → Subscribe senden."""
        self._connected = True
        self._last_error = None
        logger.info("MOONRAKER: Websocket verbunden")

        # JSON-RPC: printer.objects.subscribe
        self._rpc_id += 1
        subscribe_msg = {
            "jsonrpc": "2.0",
            "method": "printer.objects.subscribe",
            "params": {"objects": self.subscribe_objects},
            "id": self._rpc_id,
        }
        ws.send(json.dumps(subscribe_msg))
        logger.info("MOONRAKER: Subscribe gesendet: %s",
                     list(self.subscribe_objects.keys()))

    def _on_ws_message(self, ws, message: str):
        """Eingehende Nachricht verarbeiten."""
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            return

        # Fall 1: Subscribe-Antwort (enthält initialen Status)
        if "result" in msg and "status" in msg.get("result", {}):
            status = msg["result"]["status"]
            self._process_status(status)
            return

        # Fall 2: Status-Update-Notification
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
        # Primär: motion_report.live_position (präzise, kontinuierlich)
        motion = status.get("motion_report", {})
        position = motion.get("live_position")

        # Fallback: toolhead.position (falls jemand das subscribt)
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

            if self._update_count <= 3 or self._update_count % 500 == 0:
                logger.debug("MOONRAKER: E=%.6f (Update #%d)",
                             e_value, self._update_count)
