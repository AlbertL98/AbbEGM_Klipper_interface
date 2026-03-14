from __future__ import annotations
# klipper_client.py — Konsolidierter Klipper-Kommunikationskanal
#
# Vereint Segment-Empfang (TCP ← move_export.py, Port 7200) und
# Watchdog-Kanal (TCP → bridge_watchdog.py, Port 7201) in einer
# Klasse mit einem einzigen Thread.
#
# Design:
#   - select()-basierter Loop: Segment-Receive blockiert nicht,
#     Heartbeats feuern trotzdem pünktlich.
#   - Watchdog-Socket hat ein Lock → send_stop()/send_pause()
#     sind von jedem Thread aufrufbar (State-Machine-Callback).
#   - Moonraker-HTTP als statischer Fallback bleibt erhalten.
#
# Ersetzt: segment_source.py (TcpSegmentReceiver),
#          klipper_command.py (KlipperCommandClient),
#          moonraker_client.py (komplett)

import enum
import json
import select
import socket
import time
import logging
import threading
from typing import Optional, Callable

from .clock import bridge_now
from .config import KlipperClientConfig
from .path_segment import TrapezSegment, SegmentValidationError

logger = logging.getLogger("egm.klipper")


class KlipperState(enum.Enum):
    """Job-Status von Klipper (via Moonraker print_stats)."""
    STANDBY = "standby"
    PRINTING = "printing"
    PAUSED = "paused"
    COMPLETE = "complete"
    CANCELLED = "cancelled"
    ERROR = "error"
    UNKNOWN = "unknown"


class KlipperClient:
    """
    Einheitlicher Klipper-Kommunikationskanal.

    Empfängt Trapez-Segmente von move_export.py (Port 7200) und
    sendet Heartbeats + Stop/Pause an bridge_watchdog.py (Port 7201).

    Ein Thread, zwei TCP-Verbindungen, select()-basiert.

    Thread-sicher: send_stop()/send_pause()/set_bridge_state()
    können von jedem Thread aufgerufen werden.
    """

    def __init__(self, config: KlipperClientConfig,
                 on_segment: Optional[Callable[[TrapezSegment], None]] = None):
        self._cfg = config
        self.on_segment = on_segment

        self._running = False
        self._thread: Optional[threading.Thread] = None

        # ── Segment-Verbindung (Port 7200, empfangen) ────────
        self._seg_socket: Optional[socket.socket] = None
        self._seg_connected = False
        self._seg_buffer: str = ""
        self._segments_received: int = 0
        self._segments_rejected: int = 0
        self._last_print_time: float = -1.0
        self._monotone_violations: int = 0

        # ── Watchdog-Verbindung (Port 7201, senden) ──────────
        self._wd_socket: Optional[socket.socket] = None
        self._wd_lock = threading.Lock()
        self._wd_connected = False
        self._wd_connect_count: int = 0
        self._wd_reconnect_count: int = 0
        self._heartbeats_sent: int = 0
        self._commands_sent: int = 0
        self._next_heartbeat: float = 0.0

        # ── Klipper Job-Status (via Moonraker HTTP) ─────────
        self._klipper_job_state: KlipperState = KlipperState.UNKNOWN
        self._klipper_job_state_ts: float = 0.0

        # ── Gemeinsam ────────────────────────────────────────
        self._bridge_state: str = "INIT"
        self._last_error_seg: Optional[str] = None
        self._last_error_wd: Optional[str] = None

    # ══════════════════════════════════════════════════════════
    #  Public API
    # ══════════════════════════════════════════════════════════

    def start(self):
        """Startet den kombinierten Empfangs-/Heartbeat-Thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._main_loop, daemon=True,
            name="klipper-client"
        )
        self._thread.start()
        logger.info(
            "KLIPPER: Gestartet → Segmente %s:%d, "
            "Watchdog %s:%d (Heartbeat: %.1fs)",
            self._cfg.tcp_host, self._cfg.tcp_port_segment,
            self._cfg.tcp_host, self._cfg.tcp_port_watchdog,
            self._cfg.watchdog_heartbeat_s)

    def stop(self):
        """Beendet den Thread und schließt beide Verbindungen."""
        self._running = False
        self._seg_disconnect()
        self._wd_disconnect()
        if self._thread:
            self._thread.join(timeout=5.0)
        logger.info(
            "KLIPPER: Gestoppt — %d Segmente empfangen, "
            "%d abgelehnt, %d Heartbeats, %d Commands, "
            "%d Reconnects (WD)",
            self._segments_received, self._segments_rejected,
            self._heartbeats_sent, self._commands_sent,
            self._wd_reconnect_count)

    def set_bridge_state(self, state: str):
        """Setzt den Bridge-State der im Heartbeat mitgesendet wird."""
        self._bridge_state = state

    def send_stop(self, reason: str) -> bool:
        """Sendet Stop-Befehl an Klipper. Thread-sicher."""
        return self._wd_send({
            "type": "stop",
            "reason": reason,
            "ts": bridge_now(),
            "bridge_state": self._bridge_state,
        })

    def send_pause(self, reason: str) -> bool:
        """Sendet Pause-Befehl an Klipper. Thread-sicher."""
        return self._wd_send({
            "type": "pause",
            "reason": reason,
            "ts": bridge_now(),
            "bridge_state": self._bridge_state,
        })

    @property
    def seg_connected(self) -> bool:
        return self._seg_connected

    @property
    def wd_connected(self) -> bool:
        return self._wd_connected

    @property
    def segments_received(self) -> int:
        return self._segments_received

    @property
    def klipper_job_state(self) -> KlipperState:
        """Letzter bekannter Klipper-Job-Status."""
        return self._klipper_job_state

    @property
    def klipper_is_printing(self) -> bool:
        """True wenn Klipper aktiv druckt oder pausiert ist."""
        return self._klipper_job_state in (
            KlipperState.PRINTING, KlipperState.PAUSED)

    def snapshot(self) -> dict:
        """Status-Snapshot für Telemetrie/Debug."""
        return {
            "segment": {
                "connected": self._seg_connected,
                "target": f"{self._cfg.tcp_host}:"
                          f"{self._cfg.tcp_port_segment}",
                "received": self._segments_received,
                "rejected": self._segments_rejected,
                "monotone_violations": self._monotone_violations,
                "last_error": self._last_error_seg,
            },
            "watchdog": {
                "connected": self._wd_connected,
                "target": f"{self._cfg.tcp_host}:"
                          f"{self._cfg.tcp_port_watchdog}",
                "heartbeats_sent": self._heartbeats_sent,
                "commands_sent": self._commands_sent,
                "connect_count": self._wd_connect_count,
                "reconnect_count": self._wd_reconnect_count,
                "last_error": self._last_error_wd,
            },
            "bridge_state": self._bridge_state,
            "klipper_job_state": self._klipper_job_state.value,
        }

    # ══════════════════════════════════════════════════════════
    #  Hauptloop — ein Thread, select()-basiert
    # ══════════════════════════════════════════════════════════

    def _main_loop(self):
        """
        Kombinierter Loop für Segment-Empfang + Heartbeat.

        select() blockiert maximal so lange bis der nächste
        Heartbeat fällig ist. Dadurch kommen Segmente sofort
        rein und Heartbeats gehen pünktlich raus.
        """
        self._next_heartbeat = time.monotonic()

        while self._running:
            # ── Verbindungen aufbauen falls nötig ────────────
            if not self._seg_connected:
                self._seg_connect()
            if not self._wd_connected:
                self._wd_connect()

            # ── Timeout bis zum nächsten Heartbeat berechnen ─
            now = time.monotonic()
            timeout = max(0.0, self._next_heartbeat - now)

            # Wenn keine Segment-Verbindung: kurz schlafen
            # statt busy-loop, dann retry
            if not self._seg_connected:
                self._sleep_interruptible(
                    min(timeout, self._cfg.reconnect_interval_s))
                self._send_heartbeat_if_due()
                continue

            # ── select() auf Segment-Socket ──────────────────
            try:
                readable, _, _ = select.select(
                    [self._seg_socket], [], [], timeout)
            except (ValueError, OSError):
                # Socket wurde geschlossen
                self._seg_disconnect()
                self._send_heartbeat_if_due()
                continue

            # ── Segmente lesen wenn Daten da ─────────────────
            if readable:
                self._seg_receive()

            # ── Heartbeat senden wenn fällig ─────────────────
            self._send_heartbeat_if_due()

    def _send_heartbeat_if_due(self):
        """Prüft ob Heartbeat fällig ist und sendet ihn."""
        now = time.monotonic()
        if now < self._next_heartbeat:
            return

        success = self._wd_send({
            "type": "heartbeat",
            "state": self._bridge_state,
            "ts": bridge_now(),
        })

        if success:
            self._heartbeats_sent += 1
        elif self._wd_connected:
            # Senden fehlgeschlagen → disconnect, nächster
            # Loop-Durchlauf reconnectet
            self._wd_disconnect()

        # Job-Status bei Moonraker abfragen (selber Takt
        # wie Heartbeat, blockiert max. 1s)
        self._query_klipper_job_state()

        # Nächsten Heartbeat planen (auch bei Fehler, damit
        # wir nicht in einer Tight-Loop enden)
        self._next_heartbeat = now + self._cfg.watchdog_heartbeat_s

    def _sleep_interruptible(self, duration_s: float):
        """Schläft in kleinen Schritten damit stop() schnell wirkt."""
        deadline = time.monotonic() + duration_s
        while self._running and time.monotonic() < deadline:
            time.sleep(min(0.25, deadline - time.monotonic()))

    def _query_klipper_job_state(self):
        """
        Fragt den Job-Status über Moonraker HTTP ab.

        Wird im Heartbeat-Takt aufgerufen (~1s). Timeout ist
        kurz (1s) damit der Loop nicht hängt. Bei Fehler bleibt
        der letzte bekannte State erhalten.
        """
        try:
            import http.client
            conn = http.client.HTTPConnection(
                self._cfg.tcp_host, 7125, timeout=1.0)
            conn.request("GET",
                         "/printer/objects/query?print_stats")
            resp = conn.getresponse()
            if resp.status == 200:
                data = json.loads(resp.read())
                raw = (data.get("result", {})
                       .get("status", {})
                       .get("print_stats", {})
                       .get("state", "unknown"))
                try:
                    state = KlipperState(raw)
                except ValueError:
                    state = KlipperState.UNKNOWN
                prev = self._klipper_job_state
                self._klipper_job_state = state
                self._klipper_job_state_ts = bridge_now()
                if state != prev:
                    logger.info("KLIPPER: Job-Status: %s → %s",
                                prev.value, state.value)
            conn.close()
        except Exception as e:
            # Moonraker nicht erreichbar — kein Drama,
            # State bleibt auf letztem bekannten Wert
            logger.debug("KLIPPER: Job-Status-Abfrage "
                         "fehlgeschlagen: %s", e)

    # ══════════════════════════════════════════════════════════
    #  Segment-Verbindung (TCP ← move_export.py)
    # ══════════════════════════════════════════════════════════

    def _seg_connect(self) -> bool:
        """Verbindet sich mit move_export.py TCP-Server."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self._cfg.tcp_host,
                          self._cfg.tcp_port_segment))
            sock.setblocking(False)

            self._seg_socket = sock
            self._seg_connected = True
            self._seg_buffer = ""
            self._last_error_seg = None
            self._last_print_time = -1.0

            logger.info("KLIPPER: Segment-Verbindung hergestellt → "
                        "%s:%d", self._cfg.tcp_host,
                        self._cfg.tcp_port_segment)
            return True

        except (ConnectionRefusedError, OSError) as e:
            self._last_error_seg = str(e)
            self._seg_connected = False
            return False

    def _seg_disconnect(self):
        """Schließt die Segment-Verbindung."""
        self._seg_connected = False
        if self._seg_socket:
            try:
                self._seg_socket.close()
            except Exception:
                pass
            self._seg_socket = None

    def _seg_receive(self):
        """Liest verfügbare Daten vom Segment-Socket und parst JSON-Lines."""
        try:
            data = self._seg_socket.recv(65536)
            if not data:
                logger.warning("KLIPPER: Segment-Verbindung geschlossen")
                self._seg_disconnect()
                return

            self._seg_buffer += data.decode("utf-8")

            while "\n" in self._seg_buffer:
                line, self._seg_buffer = self._seg_buffer.split("\n", 1)
                line = line.strip()
                if line:
                    self._process_segment_line(line)

        except BlockingIOError:
            # Non-blocking Socket, keine Daten mehr → OK
            pass
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            logger.warning("KLIPPER: Segment-Empfangsfehler: %s", e)
            self._seg_disconnect()

    def _process_segment_line(self, line: str):
        """Parst eine JSON-Line vom Segment-Stream."""
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as e:
            logger.warning("KLIPPER: JSON-Fehler: %s | %s",
                           e, line[:80])
            return

        msg_type = msg.get("type", "")

        if msg_type == "hello":
            logger.info("KLIPPER: Server-Hello: %s",
                        msg.get("msg", ""))
            return

        if msg_type == "segment":
            try:
                seg = TrapezSegment.from_dict(msg)

                # print_time-Monotonie prüfen
                if seg.print_time < self._last_print_time:
                    self._monotone_violations += 1
                    logger.warning(
                        "KLIPPER: print_time nicht monoton! "
                        "Segment #%d: %.6f < vorheriges %.6f "
                        "(Verstoß #%d)",
                        seg.nr, seg.print_time,
                        self._last_print_time,
                        self._monotone_violations)
                self._last_print_time = seg.print_time

                self._segments_received += 1
                if self.on_segment:
                    self.on_segment(seg)

            except SegmentValidationError as e:
                self._segments_rejected += 1
                logger.error("KLIPPER: Segment abgelehnt: %s", e)
            except (KeyError, ValueError) as e:
                self._segments_rejected += 1
                logger.error("KLIPPER: Segment-Parse-Fehler: %s", e)
            return

        logger.debug("KLIPPER: Unbekannter Message-Typ: %s",
                      msg_type)

    # ══════════════════════════════════════════════════════════
    #  Watchdog-Verbindung (TCP → bridge_watchdog.py)
    # ══════════════════════════════════════════════════════════

    def _wd_connect(self) -> bool:
        """Verbindet sich mit bridge_watchdog.py TCP-Server."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self._cfg.tcp_host,
                          self._cfg.tcp_port_watchdog))
            sock.setsockopt(socket.SOL_SOCKET,
                            socket.SO_KEEPALIVE, 1)
            sock.settimeout(5.0)

            with self._wd_lock:
                self._wd_socket = sock
                self._wd_connected = True
                self._last_error_wd = None

            self._wd_connect_count += 1
            if self._wd_connect_count > 1:
                self._wd_reconnect_count += 1
                logger.info(
                    "KLIPPER: Watchdog-Reconnect #%d → %s:%d",
                    self._wd_reconnect_count,
                    self._cfg.tcp_host,
                    self._cfg.tcp_port_watchdog)
            else:
                logger.info(
                    "KLIPPER: Watchdog verbunden → %s:%d",
                    self._cfg.tcp_host,
                    self._cfg.tcp_port_watchdog)
            return True

        except (ConnectionRefusedError, OSError) as e:
            self._last_error_wd = str(e)
            if self._wd_connect_count % 5 == 0:
                logger.debug(
                    "KLIPPER: Watchdog-Verbindung zu %s:%d "
                    "fehlgeschlagen: %s",
                    self._cfg.tcp_host,
                    self._cfg.tcp_port_watchdog, e)
            return False

    def _wd_disconnect(self):
        """Schließt die Watchdog-Verbindung."""
        with self._wd_lock:
            self._wd_connected = False
            if self._wd_socket:
                try:
                    self._wd_socket.close()
                except Exception:
                    pass
                self._wd_socket = None

    def _wd_send(self, cmd: dict) -> bool:
        """
        Sendet ein JSON-Line-Kommando an bridge_watchdog.py.

        Thread-sicher — wird vom Main-Loop (Heartbeat) und von
        externen Threads (send_stop/send_pause) aufgerufen.
        """
        with self._wd_lock:
            if not self._wd_connected or not self._wd_socket:
                return False
            try:
                line = json.dumps(cmd) + "\n"
                self._wd_socket.sendall(line.encode("utf-8"))
                if cmd["type"] != "heartbeat":
                    self._commands_sent += 1
                    logger.info("KLIPPER: Gesendet: %s", cmd["type"])
                return True
            except (BrokenPipeError, ConnectionResetError,
                    OSError) as e:
                self._last_error_wd = str(e)
                self._wd_connected = False
                logger.warning("KLIPPER: Watchdog-Sendefehler: %s",
                               e)
                try:
                    self._wd_socket.close()
                except Exception:
                    pass
                self._wd_socket = None
                return False


# ══════════════════════════════════════════════════════════════
#  Moonraker-Fallback (statisch, kein Thread)
# ══════════════════════════════════════════════════════════════

class MoonrakerEmergencyStop:
    """
    Fallback: Emergency Stop / Pause über Moonraker HTTP-API.

    Wird von bridge.py aufgerufen wenn der TCP-Watchdog-Kanal
    nicht verfügbar ist. Kein Thread, kein State — reiner
    Fire-and-Forget HTTP-Call.
    """

    @staticmethod
    def send_stop(host: str = "127.0.0.1",
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
            logger.info("KLIPPER: Moonraker Emergency Stop "
                        "gesendet (HTTP %d)", resp.status)
            return resp.status == 200
        except Exception as e:
            logger.error("KLIPPER: Moonraker Emergency Stop "
                         "fehlgeschlagen: %s", e)
            return False

    @staticmethod
    def send_pause(host: str = "127.0.0.1",
                   port: int = 7125,
                   timeout_s: float = 2.0) -> bool:
        """POST /printer/gcode/script?script=PAUSE"""
        try:
            import http.client
            import urllib.parse
            conn = http.client.HTTPConnection(host, port,
                                              timeout=timeout_s)
            params = urllib.parse.urlencode({"script": "PAUSE"})
            conn.request("POST",
                         "/printer/gcode/script?" + params)
            resp = conn.getresponse()
            conn.close()
            logger.info("KLIPPER: Moonraker PAUSE "
                        "gesendet (HTTP %d)", resp.status)
            return resp.status == 200
        except Exception as e:
            logger.error("KLIPPER: Moonraker PAUSE "
                         "fehlgeschlagen: %s", e)
            return False