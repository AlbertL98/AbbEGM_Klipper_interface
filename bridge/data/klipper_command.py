from __future__ import annotations
# klipper_command.py — Kommando-Kanal Bridge → Klipper
#
# Sendet Heartbeats und Stop/Pause-Befehle an das
# bridge_watchdog.py Klipper-Extra.
#
# FIXES:
#   - Reconnect-Zähler: Erste Verbindung zählt nicht als Reconnect,
#     danach wird korrekt hochgezählt.
#   - Einheitliche Clock via bridge_now()

import json
import socket
import time
import logging
import threading
from typing import Optional

from .clock import bridge_now

logger = logging.getLogger("egm.command")


class KlipperCommandClient:
    """
    Sendet Heartbeats und Befehle an Klippers bridge_watchdog.py.

    Thread-sicher: Heartbeat läuft in eigenem Thread,
    stop()/pause() können von jedem Thread aufgerufen werden.
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
        self._connect_count: int = 0      # Gesamte Verbindungen
        self._reconnect_count: int = 0    # Nur Reconnects (nicht erste)
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
        """Sendet Stop-Befehl an Klipper."""
        return self._send_command({
            "type": "stop",
            "reason": reason,
            "ts": bridge_now(),
            "bridge_state": self._bridge_state,
        })

    def send_pause(self, reason: str) -> bool:
        """Sendet Pause-Befehl an Klipper."""
        return self._send_command({
            "type": "pause",
            "reason": reason,
            "ts": bridge_now(),
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
            "connect_count": self._connect_count,
            "reconnect_count": self._reconnect_count,
            "bridge_state": self._bridge_state,
            "last_error": self._last_error,
        }

    # ── Heartbeat Loop ───────────────────────────────────────

    def _heartbeat_loop(self):
        """Hauptloop: Verbinden, Heartbeat senden, bei Fehler reconnecten."""
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
                "ts": bridge_now(),
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

            # FIX: Korrekte Reconnect-Zählung
            self._connect_count += 1
            if self._connect_count > 1:
                self._reconnect_count += 1
                logger.info("KLIPPER-CMD: Reconnect #%d mit %s:%d",
                             self._reconnect_count, self.host, self.port)
            else:
                logger.info("KLIPPER-CMD: Verbunden mit %s:%d",
                             self.host, self.port)
            return True

        except (ConnectionRefusedError, OSError) as e:
            self._last_error = str(e)
            # Nur alle 5 Versuche loggen um Spam zu vermeiden
            if self._connect_count % 5 == 0:
                logger.debug("KLIPPER-CMD: Verbindung zu %s:%d "
                             "fehlgeschlagen: %s",
                             self.host, self.port, e)
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
        """Sendet ein JSON-Line-Kommando an Klipper. Thread-sicher."""
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
    """Fallback: Emergency Stop über Moonraker JSON-RPC."""

    @staticmethod
    def send_via_http(host: str = "127.0.0.1",
                      port: int = 7125,
                      timeout_s: float = 2.0) -> bool:
        """POST /printer/emergency_stop"""
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
        """POST /printer/gcode/script?script=PAUSE"""
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
