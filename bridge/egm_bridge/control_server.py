# control_server.py — TCP Control-Server für Remote-Steuerung
#
# Ermöglicht die Steuerung der Bridge über JSON-Commands:
#   {"cmd": "start", "job": "optional_name"}
#   {"cmd": "stop"}
#   {"cmd": "status"}
#   {"cmd": "set_param", "section": "sync", "key": "...", "value": ...}
#   {"cmd": "ping"}
#
# Wird von egm_commands.py (Klipper-Extra) angesprochen.
# Port: 7201 (default, konfigurierbar)

import json
import socket
import logging
import threading
import time
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .bridge import EgmBridge

logger = logging.getLogger("egm.control")


class ControlServer:
    """
    Einfacher TCP-Server für Bridge-Steuerung.

    Protokoll: JSON-Lines (eine Zeile pro Request/Response).
    Jeder Client kann Commands senden und bekommt eine JSON-Antwort.
    """

    def __init__(self, bridge: "EgmBridge", host: str = "0.0.0.0",
                 port: int = 7201):
        self.bridge = bridge
        self.host = host
        self.port = port

        self._server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Startet den Control-Server."""
        try:
            self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((self.host, self.port))
            self._server.listen(3)
            self._server.settimeout(1.0)

            self._running = True
            self._thread = threading.Thread(
                target=self._accept_loop, daemon=True, name="ctrl-server"
            )
            self._thread.start()

            logger.info("CONTROL: Server gestartet auf %s:%d",
                        self.host, self.port)
        except OSError as e:
            logger.error("CONTROL: Server-Start fehlgeschlagen: %s", e)
            raise

    def stop(self):
        """Stoppt den Control-Server."""
        if not self._running:
            return
        self._running = False
        if self._server:
            try:
                self._server.close()
            except Exception:
                pass
            self._server = None
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("CONTROL: Server gestoppt")

    def _accept_loop(self):
        while self._running:
            try:
                client, addr = self._server.accept()
                logger.info("CONTROL: Client verbunden: %s", addr)
                handler = threading.Thread(
                    target=self._handle_client,
                    args=(client, addr),
                    daemon=True,
                    name=f"ctrl-{addr[1]}",
                )
                handler.start()
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_client(self, client: socket.socket, addr):
        """Verarbeitet Commands von einem Client."""
        client.settimeout(30.0)
        buffer = ""

        try:
            while self._running:
                try:
                    data = client.recv(4096)
                    if not data:
                        break

                    buffer += data.decode("utf-8")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        response = self._dispatch(line)
                        client.sendall(
                            (json.dumps(response) + "\n").encode("utf-8")
                        )

                except socket.timeout:
                    continue
                except (ConnectionResetError, ConnectionAbortedError,
                        BrokenPipeError, OSError):
                    break
        except Exception:
            pass
        finally:
            try:
                client.close()
            except Exception:
                pass
            logger.debug("CONTROL: Client getrennt: %s", addr)

    def _dispatch(self, line: str) -> dict:
        """Verarbeitet einen einzelnen Command."""
        try:
            req = json.loads(line)
        except json.JSONDecodeError as e:
            return {"ok": False, "error": f"JSON-Fehler: {e}"}

        cmd = req.get("cmd", "")

        if cmd == "ping":
            return {"ok": True, "state": self.bridge.sm.state.value}

        elif cmd == "start":
            return self._cmd_start(req)

        elif cmd == "stop":
            return self._cmd_stop(req)

        elif cmd == "status":
            return self._cmd_status()

        elif cmd == "set_param":
            return self._cmd_set_param(req)

        else:
            return {"ok": False, "error": f"Unbekannter Command: {cmd}"}

    def _cmd_start(self, req: dict) -> dict:
        """Startet einen Job."""
        state = self.bridge.sm.state.value

        if state == "RUN":
            return {"ok": True, "state": "RUN", "msg": "Läuft bereits"}

        if state != "READY":
            return {"ok": False, "state": state,
                    "error": f"Kann nur aus READY starten (aktuell: {state})"}

        job_id = req.get("job", "default")
        result = self.bridge.run_job(job_id=job_id)

        if result:
            return {"ok": True, "state": self.bridge.sm.state.value,
                    "msg": f"Job '{job_id}' gestartet"}
        else:
            return {"ok": False, "state": self.bridge.sm.state.value,
                    "error": "Job-Start fehlgeschlagen"}

    def _cmd_stop(self, req: dict) -> dict:
        """Stoppt den laufenden Job. Bridge geht in STOP → READY automatisch."""
        state = self.bridge.sm.state.value

        if state in ("STOP", "INIT"):
            return {"ok": True, "state": state, "msg": "Bereits gestoppt"}

        if state == "READY":
            return {"ok": True, "state": state, "msg": "Kein Job aktiv"}

        reason = req.get("reason", "EGM_STOP Command")
        self.bridge.stop(reason=reason)

        return {"ok": True, "state": self.bridge.sm.state.value,
                "msg": "Job gestoppt — Bridge wechselt zurück in READY"}

    def _cmd_status(self) -> dict:
        """Gibt den aktuellen Status zurück."""
        try:
            snap = self.bridge.snapshot()

            # Kompakte Zusammenfassung für Terminal-Ausgabe
            sync_m = self.bridge.sync.metrics
            summary = {
                "ok": True,
                "state": self.bridge.sm.state.value,
                "loop_count": self.bridge._loop_count,
                "loop_overruns": self.bridge._loop_overruns,
                "queue_depth": self.bridge.planner.queue_depth,
                "segments_consumed": self.bridge.planner._segments_consumed,
                "egm_tx": self.bridge.egm.stats.tx_count,
                "egm_rx": self.bridge.egm.stats.rx_count,
                "tracking_mm": round(sync_m.tracking_error_mm, 2),
                "lag_ms": round(sync_m.lag_ms, 1),
                "jitter_p99_ms": round(sync_m.jitter_p99_ms, 1),
                "sync_level": sync_m.sync_level.value,
                "config_profile": self.bridge.cfg.profile_name,
            }

            # Klipper-Source Status
            if self.bridge.receiver:
                r = self.bridge.receiver
                summary["klipper_connected"] = r.connected
                summary["segments_received"] = r.segments_received

            return summary

        except Exception as e:
            return {"ok": False, "error": f"Status-Fehler: {e}"}

    def _cmd_set_param(self, req: dict) -> dict:
        """Setzt einen Parameter zur Laufzeit."""
        section = req.get("section", "")
        key = req.get("key", "")
        value = req.get("value")

        if not section or not key or value is None:
            return {"ok": False,
                    "error": "Benötigt: section, key, value"}

        # Typ-Konversion basierend auf bestehendem Wert
        section_obj = getattr(self.bridge.cfg, section, None)
        if section_obj is None:
            return {"ok": False, "error": f"Unbekannte Section: {section}"}

        if not hasattr(section_obj, key):
            return {"ok": False,
                    "error": f"Unbekannter Parameter: {section}.{key}"}

        old_value = getattr(section_obj, key)
        try:
            # Typ des alten Werts beibehalten
            if isinstance(old_value, float):
                value = float(value)
            elif isinstance(old_value, int):
                value = int(value)
            elif isinstance(old_value, bool):
                value = str(value).lower() in ("true", "1", "yes")
        except (ValueError, TypeError) as e:
            return {"ok": False, "error": f"Typ-Fehler: {e}"}

        success = self.bridge.set_param(section, key, value)
        if success:
            return {"ok": True, "section": section, "key": key,
                    "old": old_value, "new": value}
        else:
            return {"ok": False, "error": "Parameter-Änderung fehlgeschlagen"}
