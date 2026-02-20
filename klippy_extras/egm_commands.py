# egm_commands.py — Klipper Extra: EGM Bridge G-Code Commands
#
# Registriert G-Code-Kommandos zur Steuerung der EGM-Bridge:
#   EGM_START                     — Bridge-Prozess starten (wartet auf Job)
#   EGM_STOP                      — Laufenden Job stoppen (Bridge bleibt aktiv)
#   EGM_BRIDGE_STOP               — Bridge-Prozess komplett beenden
#   EGM_STATUS                    — Status im Terminal anzeigen
#   EGM_SET_PARAM SECTION= KEY= VALUE= — Parameter zur Laufzeit ändern
#
# Workflow:
#   1. EGM_START im Mainsail-Terminal → Bridge startet und wartet
#   2. Druckjob über Mainsail starten → Bridge erkennt Segmente
#      automatisch und beginnt mit der Übertragung
#   3. EGM_STOP zum Job-Stoppen / EGM_BRIDGE_STOP zum Beenden
#   4. EGM_STATUS für Statusanzeige
#
# Config in printer.cfg:
#   [egm_commands]
#   bridge_path: /home/klippy/egm_bridge/run_bridge.py
#   bridge_config: /home/klippy/egm_bridge/bridge_config.json
#   control_port: 7201
#   python_path: python3
#   auto_start_bridge: true
#   log_file: /home/klippy/printer_data/logs/egm_bridge.log

import os
import json
import socket
import logging
import subprocess
import time
import threading


class EgmCommands:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.logger = logging.getLogger('egm_commands')

        # Config aus printer.cfg
        self.bridge_path = config.get(
            'bridge_path',
            '/home/klippy/bridge/run_bridge.py'
        )
        self.bridge_config = config.get(
            'bridge_config',
            '/home/klippy/bridge/bridge_config.json'
        )
        self.control_port = config.getint('control_port', 7201)
        self.python_path = config.get('python_path', 'python3')
        self.auto_start_bridge = config.getboolean(
            'auto_start_bridge', True
        )
        self.log_file = config.get(
            'log_file',
            '/home/klippy/printer_data/logs/egm_bridge.log'
        )

        # Bridge-Prozess State
        self._bridge_proc = None
        self._bridge_log_fd = None

        # UI Connection
        self._ui_lock = threading.Lock()
        self._ui_string = "init"
        self._ui_int = 0
        self._ui_last_update = 0.0

        self._poll_stop = False
        self._poll_thread = threading.Thread(target=self._poll_bridge_status, daemon=True)
        self._poll_thread.start()

        # G-Code Commands registrieren
        self.gcode.register_command(
            'EGM_START', self.cmd_EGM_START,
            desc="EGM-Bridge Prozess starten (wartet auf Druckjob)"
        )
        self.gcode.register_command(
            'EGM_STOP', self.cmd_EGM_STOP,
            desc="EGM-Bridge Job stoppen (Bridge bleibt aktiv)"
        )
        self.gcode.register_command(
            'EGM_BRIDGE_STOP', self.cmd_EGM_BRIDGE_STOP,
            desc="EGM-Bridge Prozess komplett beenden"
        )
        self.gcode.register_command(
            'EGM_STATUS', self.cmd_EGM_STATUS,
            desc="EGM-Bridge Status anzeigen"
        )
        self.gcode.register_command(
            'EGM_SET_PARAM', self.cmd_EGM_SET_PARAM,
            desc="EGM-Bridge Parameter zur Laufzeit ändern"
        )

        # Bei Klipper-Disconnect aufräumen
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )

    # ── G-Code Command Handlers ──────────────────────────────

    def cmd_EGM_START(self, gcmd):
        """EGM_START — Bridge-Prozess starten (wartet auf Segmente)."""

        # Bridge-Prozess starten (falls nicht läuft)
        if self._is_bridge_running():
            gcmd.respond_info("EGM: Bridge läuft bereits (PID: %d)"
                              % self._bridge_proc.pid)
            return

        gcmd.respond_info("EGM: Starte Bridge-Prozess...")
        if not self._start_bridge_process():
            gcmd.respond_info("EGM: FEHLER — Bridge konnte "
                              "nicht gestartet werden!")
            return

        # Warten bis Bridge READY ist
        gcmd.respond_info("EGM: Warte auf Bridge-READY...")
        if not self._wait_for_ready(timeout=10.0):
            gcmd.respond_info("EGM: FEHLER — Bridge nicht "
                              "READY nach 10s!")
            return

        gcmd.respond_info(
            "EGM: Bridge bereit (PID: %d) — wartet auf "
            "Druckjob aus Mainsail" % self._bridge_proc.pid
        )

    def cmd_EGM_STOP(self, gcmd):
        """EGM_STOP — Laufenden Job stoppen (Bridge bleibt aktiv)."""
        if not self._is_bridge_running():
            gcmd.respond_info("EGM: Bridge läuft nicht")
            return

        resp = self._send_command({"cmd": "stop"})

        if resp is None:
            gcmd.respond_info("EGM: Keine Verbindung zur Bridge")
            return

        if resp.get("ok"):
            gcmd.respond_info(
                "EGM: %s (State: %s)"
                % (resp.get("msg", "Gestoppt"), resp.get("state", "?"))
            )
        else:
            gcmd.respond_info(
                "EGM: FEHLER — %s" % resp.get("error", "Unbekannt")
            )

    def cmd_EGM_BRIDGE_STOP(self, gcmd):
        """EGM_BRIDGE_STOP — Bridge-Prozess komplett beenden."""
        if not self._is_bridge_running():
            gcmd.respond_info("EGM: Bridge läuft nicht")
            return

        gcmd.respond_info("EGM: Beende Bridge-Prozess...")

        # Erst sauber stoppen
        self._send_command({"cmd": "stop"}, timeout=2.0)
        time.sleep(1.0)
        self._kill_bridge_process()

        gcmd.respond_info("EGM: Bridge-Prozess beendet")

    def cmd_EGM_STATUS(self, gcmd):
        """EGM_STATUS — Status der Bridge anzeigen."""
        # Prozess-Status
        if not self._is_bridge_running():
            gcmd.respond_info("EGM: Bridge-Prozess läuft NICHT")
            return

        gcmd.respond_info("EGM: Bridge-Prozess aktiv (PID: %d)"
                          % self._bridge_proc.pid)

        # Status über Control-Port holen
        resp = self._send_command({"cmd": "status"})

        if resp is None:
            gcmd.respond_info("EGM: Keine Verbindung zum Control-Port")
            return

        if not resp.get("ok"):
            gcmd.respond_info("EGM: Fehler — %s"
                              % resp.get("error", "Unbekannt"))
            return

        # Formatierte Ausgabe
        lines = [
            "── EGM Bridge Status ──",
            "State:     %s" % resp.get("state", "?"),
            "Sync:      %s" % resp.get("sync_level", "?"),
            "Tracking:  %.2f mm" % resp.get("tracking_mm", 0),
            "Lag:       %.1f ms" % resp.get("lag_ms", 0),
            "Jitter:    %.1f ms (p99)" % resp.get("jitter_p99_ms", 0),
            "Queue:     %d Segmente" % resp.get("queue_depth", 0),
            "EGM:       TX=%d  RX=%d" % (
                resp.get("egm_tx", 0), resp.get("egm_rx", 0)),
            "Zyklen:    %d (Overruns: %d)" % (
                resp.get("loop_count", 0),
                resp.get("loop_overruns", 0)),
            "Klipper:   %s (%d Segmente)" % (
                "verbunden" if resp.get("klipper_connected") else "getrennt",
                resp.get("segments_received", 0)),
            "Profil:    %s" % resp.get("config_profile", "?"),
        ]

        for line in lines:
            gcmd.respond_info(line)

    def cmd_EGM_SET_PARAM(self, gcmd):
        """EGM_SET_PARAM SECTION= KEY= VALUE= — Parameter ändern."""
        section = gcmd.get('SECTION', '')
        key = gcmd.get('KEY', '')
        value_str = gcmd.get('VALUE', '')

        if not section or not key or not value_str:
            gcmd.respond_info(
                "Nutzung: EGM_SET_PARAM SECTION=sync "
                "KEY=tracking_warn_mm VALUE=10.0"
            )
            return

        if not self._is_bridge_running():
            gcmd.respond_info("EGM: Bridge läuft nicht")
            return

        # Typ-Erkennung
        try:
            if '.' in value_str:
                value = float(value_str)
            elif value_str.lower() in ('true', 'false'):
                value = value_str.lower() == 'true'
            else:
                value = int(value_str)
        except ValueError:
            value = value_str

        resp = self._send_command({
            "cmd": "set_param",
            "section": section,
            "key": key,
            "value": value,
        })

        if resp is None:
            gcmd.respond_info("EGM: Keine Verbindung zur Bridge")
            return

        if resp.get("ok"):
            gcmd.respond_info(
                "EGM: %s.%s: %s → %s"
                % (section, key, resp.get("old"), resp.get("new"))
            )
        else:
            gcmd.respond_info(
                "EGM: FEHLER — %s" % resp.get("error", "Unbekannt")
            )

    # ── UI - Process ────────────────────────────
    def _poll_bridge_status(self):
        # Pollt alle 1s den Bridge-Status und cached zwei Felder
        while not self._poll_stop:
            if self._is_bridge_running():
                resp = self._send_command({"cmd": "status"}, timeout=0.5)
                if resp and resp.get("ok"):
                    # Hier musst du definieren, woher string/int kommen.
                    # Beispiel: resp["ui_msg"] und resp["ui_count"]
                    ui_msg = resp.get("ui_msg", resp.get("state", "unknown"))
                    ui_count = int(resp.get("ui_count", resp.get("queue_depth", 0)))

                    with self._ui_lock:
                        self._ui_string = str(ui_msg)
                        self._ui_int = ui_count
                        self._ui_last_update = time.time()
            time.sleep(1.0)

    def get_status(self, eventtime):
        # Das ist die Klipper-Status-Schnittstelle, die Moonraker abfragen kann
        with self._ui_lock:
            return {
                "ui_string": self._ui_string,
                "ui_int": self._ui_int,
                "ui_age_s": max(0.0, time.time() - self._ui_last_update),
                "bridge_running": self._is_bridge_running(),
            }

    # ── Bridge-Prozess Management ────────────────────────────

    def _start_bridge_process(self) -> bool:
        """Startet run_bridge.py als Subprocess."""
        try:
            # Log-Datei öffnen
            log_dir = os.path.dirname(self.log_file)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            self._bridge_log_fd = open(self.log_file, 'a')

            cmd = [
                self.python_path,
                self.bridge_path,
                '--config', self.bridge_config,
                '--control-port', str(self.control_port),
                '--auto-start',
            ]

            self._bridge_proc = subprocess.Popen(
                cmd,
                stdout=self._bridge_log_fd,
                stderr=subprocess.STDOUT,
                cwd=os.path.dirname(self.bridge_path) or '.',
            )

            self.logger.info(
                "EGM: Bridge gestartet (PID: %d, Log: %s)",
                self._bridge_proc.pid, self.log_file
            )
            return True

        except Exception as e:
            self.logger.error("EGM: Bridge-Start fehlgeschlagen: %s", e)
            return False

    def _is_bridge_running(self) -> bool:
        """Prüft ob der Bridge-Prozess noch läuft."""
        if self._bridge_proc is None:
            return False
        return self._bridge_proc.poll() is None

    def _wait_for_ready(self, timeout: float = 10.0) -> bool:
        """Wartet bis die Bridge auf dem Control-Port antwortet."""
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if not self._is_bridge_running():
                return False

            resp = self._send_command({"cmd": "ping"}, timeout=1.0)
            if resp and resp.get("ok"):
                return True

            time.sleep(0.5)

        return False

    def _kill_bridge_process(self):
        """Beendet den Bridge-Prozess."""
        if self._bridge_proc:
            try:
                self._bridge_proc.terminate()
                self._bridge_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bridge_proc.kill()
            except Exception:
                pass
            self._bridge_proc = None

        if self._bridge_log_fd:
            try:
                self._bridge_log_fd.close()
            except Exception:
                pass
            self._bridge_log_fd = None

    # ── TCP Control Communication ────────────────────────────

    def _send_command(self, cmd_dict: dict,
                      timeout: float = 5.0) -> dict:
        """Sendet einen Command an die Bridge und wartet auf Antwort."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect(('127.0.0.1', self.control_port))

            request = json.dumps(cmd_dict) + '\n'
            sock.sendall(request.encode('utf-8'))

            # Antwort lesen
            response_data = b''
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                response_data += chunk
                if b'\n' in response_data:
                    break

            sock.close()

            if response_data:
                line = response_data.decode('utf-8').strip()
                return json.loads(line)
            return None

        except (ConnectionRefusedError, socket.timeout, OSError) as e:
            self.logger.debug("EGM: Control-Verbindung fehlgeschlagen: %s", e)
            return None
        except json.JSONDecodeError:
            return None

    # ── Cleanup ──────────────────────────────────────────────

    def _handle_disconnect(self):
        """Bei Klipper-Disconnect Bridge sauber beenden."""
        if self._is_bridge_running():
            self.logger.info("EGM: Klipper trennt — beende Bridge...")
            # Erst sauber stoppen versuchen
            self._poll_stop = True
            self._send_command({"cmd": "stop"}, timeout=2.0)
            time.sleep(1.0)
            self._kill_bridge_process()


def load_config(config):
    return EgmCommands(config)
