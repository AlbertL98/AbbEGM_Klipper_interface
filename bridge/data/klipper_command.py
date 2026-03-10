from __future__ import annotations
# klipper_command.py — Kommando-Kanal Bridge → Klipper
#
# Sendet Heartbeats und Stop/Pause-Befehle an das
# bridge_watchdog.py Klipper-Extra.
#
# Zwei Kanäle für maximale Zuverlässigkeit:
#   1. Dedizierter TCP (Heartbeat + Befehle) → bridge_watchdog.py
#   2. Moonraker JSON-RPC (Emergency Stop als Fallback)
#
# Heartbeat-Frequenz ist unabhängig vom EGM-Zyklus und
# läuft in eigenem Thread (Default: alle 1s).

import json
import socket
import time
import logging
import threading
from typing import Optional

logger = logging.getLogger("egm.command")


class KlipperCommandClient:
    """
    Sendet Heartbeats und Befehle an Klippers bridge_watchdog.py.

    Thread-sicher: Heartbeat läuft in eigenem Thread,
    stop()/pause() können von jedem Thread aufgerufen werden.

    Verwendung in bridge.py:
        self.klipper_cmd = KlipperCommandClient(
            host="127.0.0.1", port=7201,
            heartbeat_interval_s=1.0,
        )
        self.klipper_cmd.start()
        ...
        self.klipper_cmd.send_stop("Sync-Fehler")
        ...
        self.klipper_cmd.stop()
    """

    def __init__(self,
                 host: str = "127.0.0.1",
                 port: int = 7201,
                 heartbeat_interval_s: float = 1.0,
                 reconnect_interval_s: float = 2.0):
        self.host = host
        self.port = port
        self.heartbeat_interval_s = heartbeat_interval_s
        self.reconnect_interval_s = reconnect_interval_s

        self._socket: Optional[socket.socket] = None
        self._socket_lock = threading.Lock()
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # State für Heartbeat
        self._bridge_state: str = "INIT"

        # Statistik
        self._heartbeats_sent: int = 0
        self._commands_sent: int = 0
        self._reconnect_count: int = 0
        self._last_error: Optional[str] = None

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Startet Heartbeat-Thread mit Auto-Reconnect."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True,
            name="klipper-cmd"
        )
        self._thread.start()
        logger.info("KLIPPER-CMD: Gestartet → %s:%d "
                     "(Heartbeat: %.1fs)",
                     self.host, self.port, self.heartbeat_interval_s)

    def stop(self):
        """Beendet den Heartbeat-Thread."""
        self._running = False
        self._disconnect()
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("KLIPPER-CMD: Gestoppt "
                     "(%d Heartbeats, %d Commands, %d Reconnects)",
                     self._heartbeats_sent, self._commands_sent,
                     self._reconnect_count)

    def set_bridge_state(self, state: str):
        """Setzt den Bridge-State der im Heartbeat mitgesendet wird."""
        self._bridge_state = state

    def send_stop(self, reason: str) -> bool:
        """
        Sendet Stop-Befehl an Klipper.

        Klipper wird den Print pausieren oder Emergency-Stoppen
        (je nach Konfiguration in [bridge_watchdog]).

        Returns True wenn erfolgreich gesendet.
        """
        return self._send_command({
            "type": "stop",
            "reason": reason,
            "ts": time.monotonic(),
            "bridge_state": self._bridge_state,
        })

    def send_pause(self, reason: str) -> bool:
        """
        Sendet Pause-Befehl an Klipper.

        Sanfter als Stop — Klipper pausiert den Print,
        kann danach fortgesetzt werden.

        Returns True wenn erfolgreich gesendet.
        """
        return self._send_command({
            "type": "pause",
            "reason": reason,
            "ts": time.monotonic(),
            "bridge_state": self._bridge_state,
        })

    @property
    def connected(self) -> bool:
        return self._connected

    def snapshot(self) -> dict:
        return {
            "connected": self._connected,
            "target": f"{self.host}:{self.port}",
            "heartbeats_sent": self._heartbeats_sent,
            "commands_sent": self._commands_sent,
            "reconnect_count": self._reconnect_count,
            "bridge_state": self._bridge_state,
            "last_error": self._last_error,
        }

    # ── Heartbeat Loop ───────────────────────────────────────

    def _heartbeat_loop(self):
        """
        Hauptloop: Verbinden, Heartbeat senden, bei Fehler reconnecten.

        Heartbeat ist bewusst langsam (Default 1s) — es geht nicht
        um Echtzeit-Synchronisation sondern um Lebenszeichen.
        """
        while self._running:
            # Verbinden falls nötig
            if not self._connected:
                if not self._connect():
                    # Warten und retry
                    self._sleep_interruptible(self.reconnect_interval_s)
                    continue

            # Heartbeat senden
            success = self._send_command({
                "type": "heartbeat",
                "state": self._bridge_state,
                "ts": time.monotonic(),
            })

            if success:
                self._heartbeats_sent += 1
            else:
                # Verbindung verloren → wird im nächsten Loop reconnectet
                self._disconnect()
                continue

            # Intervall einhalten
            self._sleep_interruptible(self.heartbeat_interval_s)

    def _sleep_interruptible(self, duration_s: float):
        """Schläft in kleinen Schritten damit stop() schnell wirkt."""
        deadline = time.monotonic() + duration_s
        while self._running and time.monotonic() < deadline:
            time.sleep(min(0.25, deadline - time.monotonic()))

    # ── TCP Verbindung ───────────────────────────────────────

    def _connect(self) -> bool:
        """Verbindet sich mit dem bridge_watchdog TCP-Server."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self.host, self.port))
            # Keepalive aktivieren
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.settimeout(5.0)

            with self._socket_lock:
                self._socket = sock
                self._connected = True
                self._last_error = None

            self._reconnect_count += 1 if self._reconnect_count > 0 else 0
            logger.info("KLIPPER-CMD: Verbunden mit %s:%d",
                         self.host, self.port)
            return True

        except (ConnectionRefusedError, OSError) as e:
            self._last_error = str(e)
            # Nur alle 10s loggen um Spam zu vermeiden
            if self._reconnect_count % 5 == 0:
                logger.debug("KLIPPER-CMD: Verbindung zu %s:%d "
                             "fehlgeschlagen: %s",
                             self.host, self.port, e)
            self._reconnect_count += 1
            return False

    def _disconnect(self):
        """Schließt die TCP-Verbindung."""
        with self._socket_lock:
            self._connected = False
            if self._socket:
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None

    def _send_command(self, cmd: dict) -> bool:
        """
        Sendet ein JSON-Line-Kommando an Klipper.

        Thread-sicher dank _socket_lock.
        Returns True wenn erfolgreich.
        """
        with self._socket_lock:
            if not self._connected or not self._socket:
                return False
            try:
                line = json.dumps(cmd) + "\n"
                self._socket.sendall(line.encode("utf-8"))
                if cmd["type"] != "heartbeat":
                    self._commands_sent += 1
                    logger.info("KLIPPER-CMD: Gesendet: %s", cmd["type"])
                return True
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                self._last_error = str(e)
                self._connected = False
                logger.warning("KLIPPER-CMD: Sendefehler: %s", e)
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None
                return False


class MoonrakerEmergencyStop:
    """
    Fallback: Emergency Stop über Moonraker JSON-RPC.

    Wird verwendet wenn der direkte TCP-Kanal nicht verfügbar ist.
    Nutzt den bereits bestehenden Moonraker-Websocket (falls verbunden).
    """

    @staticmethod
    def send_via_http(host: str = "127.0.0.1",
                      port: int = 7125,
                      timeout_s: float = 2.0) -> bool:
        """
        Sendet Emergency Stop über Moonraker HTTP-API.

        Einfacher als Websocket — für Notfälle ausreichend.
        POST /printer/emergency_stop
        """
        try:
            import http.client
            conn = http.client.HTTPConnection(host, port,
                                              timeout=timeout_s)
            conn.request("POST", "/printer/emergency_stop")
            resp = conn.getresponse()
            conn.close()
            logger.info("KLIPPER-CMD: Moonraker Emergency Stop "
                         "gesendet (HTTP %d)", resp.status)
            return resp.status == 200
        except Exception as e:
            logger.error("KLIPPER-CMD: Moonraker Emergency Stop "
                          "fehlgeschlagen: %s", e)
            return False

    @staticmethod
    def send_pause_via_http(host: str = "127.0.0.1",
                            port: int = 7125,
                            timeout_s: float = 2.0) -> bool:
        """
        Sendet PAUSE über Moonraker HTTP-API.

        POST /printer/gcode/script?script=PAUSE
        """
        try:
            import http.client
            import urllib.parse
            conn = http.client.HTTPConnection(host, port,
                                              timeout=timeout_s)
            params = urllib.parse.urlencode({"script": "PAUSE"})
            conn.request("POST", "/printer/gcode/script?" + params)
            resp = conn.getresponse()
            conn.close()
            logger.info("KLIPPER-CMD: Moonraker PAUSE "
                         "gesendet (HTTP %d)", resp.status)
            return resp.status == 200
        except Exception as e:
            logger.error("KLIPPER-CMD: Moonraker PAUSE "
                          "fehlgeschlagen: %s", e)
            return False
