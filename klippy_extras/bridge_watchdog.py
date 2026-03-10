# bridge_watchdog.py — Klipper Extra: Bridge-Heartbeat-Überwachung
#
# Überwacht ob die EGM-Bridge noch lebt und reagiert auf
# Stop-/Pause-Befehle von der Bridge.
#
# Zwei Schutzmechanismen:
#   1. Heartbeat-Timeout: Bridge sendet periodisch, Klipper stoppt bei Ausfall
#   2. Explizite Befehle: Bridge sendet "stop"/"pause" bei erkannten Problemen
#
# Sicherheitslogik:
#   - Heartbeat-Timeout löst NUR aus wenn ein Print aktiv ist UND
#     die Bridge WÄHREND dieses Prints verbunden war
#   - Ohne Bridge-Verbindung kann normal gedruckt werden (z.B. Extruder-Test)
#   - Explizite Stop-Befehle wirken immer sofort
#
# Protokoll (JSON-Lines über TCP, Bridge → Klipper):
#   {"type": "heartbeat", "state": "RUN", "ts": 1234.56}
#   {"type": "stop",  "reason": "Sync-Stop: Tracking 15mm > Limit"}
#   {"type": "pause", "reason": "Sync-Degrade: hoher Lag"}
#
# Config in printer.cfg:
#   [bridge_watchdog]
#   tcp_port: 7201
#   heartbeat_timeout: 5.0
#   action_on_timeout: pause        # "pause" oder "emergency_stop"
#   action_on_bridge_stop: pause    # "pause" oder "emergency_stop"
#   enabled: true

import json
import socket
import logging
import threading
import time


class BridgeWatchdog:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        # Konfiguration
        self.tcp_port = config.getint('tcp_port', 7201)
        self.heartbeat_timeout = config.getfloat('heartbeat_timeout', 5.0)
        self.action_on_timeout = config.get('action_on_timeout', 'pause')
        self.action_on_bridge_stop = config.get(
            'action_on_bridge_stop', 'pause')
        self.enabled = config.getboolean('enabled', True)

        self.logger = logging.getLogger('bridge_watchdog')

        # TCP-Server State
        self.server_socket = None
        self._accept_thread = None
        self._client_thread = None

        # Bridge-Verbindung (nur ein Client gleichzeitig)
        self._client_socket = None
        self._client_lock = threading.Lock()
        self._bridge_connected = False

        # Heartbeat-Tracking
        self._last_heartbeat = 0.0       # time.monotonic()
        self._bridge_state = ""          # Letzter State der Bridge

        # Print-Tracking
        self._print_active = False
        self._bridge_seen_this_print = False  # War Bridge verbunden?

        # Schutz gegen Mehrfach-Auslösung
        self._timeout_triggered = False
        self._stop_triggered = False

        # GCode-Kommandos registrieren
        self.gcode.register_command(
            'BRIDGE_STATUS', self.cmd_BRIDGE_STATUS,
            desc="Zeigt Bridge-Watchdog-Status an"
        )
        self.gcode.register_command(
            'BRIDGE_WATCHDOG_RESET', self.cmd_BRIDGE_WATCHDOG_RESET,
            desc="Setzt Bridge-Watchdog zurück (nach Fehler)"
        )

        # Events registrieren
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

        # Virtuelle SD-Card Events für Print-Tracking
        self.printer.register_event_handler(
            "virtual_sdcard:printing", self._handle_print_start)
        self.printer.register_event_handler(
            "virtual_sdcard:paused", self._handle_print_pause)
        self.printer.register_event_handler(
            "virtual_sdcard:complete", self._handle_print_end)
        self.printer.register_event_handler(
            "virtual_sdcard:cancelled", self._handle_print_end)

    # ── Lifecycle ────────────────────────────────────────────

    def _handle_connect(self):
        if not self.enabled:
            self.logger.info("BridgeWatchdog: Deaktiviert in Config")
            return
        self._start_tcp_server()
        # Heartbeat-Check-Timer starten (läuft permanent)
        self._check_timer = self.reactor.register_timer(
            self._heartbeat_check_callback,
            self.reactor.monotonic() + self.heartbeat_timeout
        )
        self.logger.info(
            "BridgeWatchdog: Aktiv (Port=%d, Timeout=%.1fs, "
            "Timeout-Aktion=%s, Stop-Aktion=%s)",
            self.tcp_port, self.heartbeat_timeout,
            self.action_on_timeout, self.action_on_bridge_stop)

    def _handle_disconnect(self):
        self._stop_tcp_server()

    # ── Print-Tracking ───────────────────────────────────────

    def _handle_print_start(self, *args):
        self._print_active = True
        self._timeout_triggered = False
        self._stop_triggered = False
        # Merken ob Bridge JETZT verbunden ist
        if self._bridge_connected:
            self._bridge_seen_this_print = True
        else:
            self._bridge_seen_this_print = False
        self.logger.info(
            "BridgeWatchdog: Print gestartet "
            "(Bridge verbunden: %s)", self._bridge_connected)

    def _handle_print_pause(self, *args):
        # Print ist pausiert — weiter überwachen
        pass

    def _handle_print_end(self, *args):
        self._print_active = False
        self._bridge_seen_this_print = False
        self._timeout_triggered = False
        self._stop_triggered = False

    # ── TCP Server ───────────────────────────────────────────

    def _start_tcp_server(self):
        try:
            self.server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)

            self._accept_thread = threading.Thread(
                target=self._accept_loop, daemon=True,
                name="bridge-wd-accept"
            )
            self._accept_thread.start()
            self.logger.info(
                "BridgeWatchdog: TCP-Server auf Port %d", self.tcp_port)
        except Exception as e:
            self.logger.error(
                "BridgeWatchdog: TCP-Server-Fehler: %s", e)
            self.server_socket = None

    def _accept_loop(self):
        """Akzeptiert Bridge-Verbindungen (nur eine gleichzeitig)."""
        while self.server_socket:
            try:
                client, addr = self.server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            # Alte Verbindung schließen
            with self._client_lock:
                if self._client_socket:
                    try:
                        self._client_socket.close()
                    except Exception:
                        pass
                self._client_socket = client
                self._bridge_connected = True
                self._last_heartbeat = time.monotonic()
                self._timeout_triggered = False

            # Wenn gerade gedruckt wird → ab jetzt überwachen
            if self._print_active:
                self._bridge_seen_this_print = True

            self.logger.info(
                "BridgeWatchdog: Bridge verbunden von %s", addr)

            # Meldung in Klipper-Konsole
            self.reactor.register_async_callback(
                lambda et: self.gcode.respond_info(
                    "Bridge-Watchdog: Bridge verbunden"))

            # Client-Thread für Empfang
            self._client_thread = threading.Thread(
                target=self._receive_from_client,
                args=(client,), daemon=True,
                name="bridge-wd-rx"
            )
            self._client_thread.start()
            # Warten bis Client-Thread endet bevor neuer akzeptiert wird
            self._client_thread.join()

    def _receive_from_client(self, client):
        """Empfängt JSON-Lines von der Bridge."""
        buffer = ""
        client.settimeout(2.0)

        while True:
            try:
                data = client.recv(4096)
                if not data:
                    break
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._process_message(line)
            except socket.timeout:
                continue
            except (ConnectionResetError, BrokenPipeError, OSError):
                break

        # Verbindung verloren
        with self._client_lock:
            self._bridge_connected = False
            self._client_socket = None
        self.logger.warning("BridgeWatchdog: Bridge-Verbindung verloren")

        self.reactor.register_async_callback(
            lambda et: self.gcode.respond_info(
                "Bridge-Watchdog: Bridge-Verbindung verloren!"))

    def _process_message(self, line):
        """Verarbeitet eine JSON-Line von der Bridge."""
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self.logger.warning(
                "BridgeWatchdog: Ungültiges JSON: %s", line[:80])
            return

        msg_type = msg.get('type', '')

        if msg_type == 'heartbeat':
            self._last_heartbeat = time.monotonic()
            self._bridge_state = msg.get('state', '')
            # Wenn Bridge sich während Print verbindet
            if self._print_active and not self._bridge_seen_this_print:
                self._bridge_seen_this_print = True

        elif msg_type == 'stop':
            reason = msg.get('reason', 'Unbekannter Grund')
            self.logger.error(
                "BridgeWatchdog: STOP von Bridge empfangen: %s", reason)
            self._execute_action(
                self.action_on_bridge_stop,
                f"Bridge-Stop: {reason}",
                is_explicit_stop=True)

        elif msg_type == 'pause':
            reason = msg.get('reason', 'Unbekannter Grund')
            self.logger.warning(
                "BridgeWatchdog: PAUSE von Bridge empfangen: %s", reason)
            self._execute_action(
                'pause',
                f"Bridge-Pause: {reason}",
                is_explicit_stop=False)

    # ── Heartbeat-Prüfung (Reactor-Timer) ────────────────────

    def _heartbeat_check_callback(self, eventtime):
        """
        Wird periodisch vom Reactor aufgerufen.
        Prüft ob der Heartbeat noch aktuell ist.
        """
        if not self.enabled:
            return self.reactor.NEVER

        # Nur prüfen wenn:
        # 1. Print aktiv
        # 2. Bridge war in diesem Print verbunden
        # 3. Noch nicht getriggert
        if (not self._print_active
                or not self._bridge_seen_this_print
                or self._timeout_triggered):
            return eventtime + 1.0

        now = time.monotonic()
        since_heartbeat = now - self._last_heartbeat

        if since_heartbeat > self.heartbeat_timeout:
            self.logger.error(
                "BridgeWatchdog: HEARTBEAT-TIMEOUT! "
                "Kein Heartbeat seit %.1fs (Limit: %.1fs)",
                since_heartbeat, self.heartbeat_timeout)
            self._timeout_triggered = True
            self._execute_action(
                self.action_on_timeout,
                "Heartbeat-Timeout (%.1fs)" % since_heartbeat,
                is_explicit_stop=False)

        return eventtime + 1.0

    # ── Aktionen ausführen ───────────────────────────────────

    def _execute_action(self, action, reason, is_explicit_stop=False):
        """
        Führt die konfigurierte Aktion aus (PAUSE oder EMERGENCY_STOP).

        Wird sowohl bei Heartbeat-Timeout als auch bei
        expliziten Bridge-Befehlen aufgerufen.
        """
        if is_explicit_stop and self._stop_triggered:
            return  # Bereits gestoppt
        if is_explicit_stop:
            self._stop_triggered = True

        if action == 'emergency_stop':
            self.logger.error(
                "BridgeWatchdog: EMERGENCY STOP — %s", reason)
            self.reactor.register_async_callback(
                lambda et: self.printer.invoke_shutdown(
                    "Bridge-Watchdog: %s" % reason))
        else:
            # Default: PAUSE — sicherer, Print kann fortgesetzt werden
            self.logger.warning(
                "BridgeWatchdog: PAUSE — %s", reason)
            self.reactor.register_async_callback(
                lambda et: self._safe_pause(reason))

    def _safe_pause(self, reason):
        """Pausiert den Print sicher aus dem Reactor-Kontext."""
        try:
            self.gcode.respond_info(
                "!! Bridge-Watchdog: %s — Print pausiert" % reason)
            self.gcode.run_script_from_command("PAUSE")
        except Exception as e:
            self.logger.error(
                "BridgeWatchdog: PAUSE fehlgeschlagen: %s — "
                "versuche Emergency Stop", e)
            try:
                self.printer.invoke_shutdown(
                    "Bridge-Watchdog: PAUSE fehlgeschlagen nach: %s"
                    % reason)
            except Exception:
                pass

    # ── GCode-Kommandos ──────────────────────────────────────

    def cmd_BRIDGE_STATUS(self, gcmd):
        """Zeigt den aktuellen Bridge-Watchdog-Status."""
        now = time.monotonic()
        parts = []

        if not self.enabled:
            gcmd.respond_info("Bridge-Watchdog: DEAKTIVIERT")
            return

        if self._bridge_connected:
            age = now - self._last_heartbeat
            parts.append("Bridge: VERBUNDEN")
            parts.append("Letzter Heartbeat: vor %.1fs" % age)
            parts.append("Bridge-State: %s" % self._bridge_state)
        else:
            parts.append("Bridge: NICHT VERBUNDEN")

        parts.append("Print aktiv: %s" % self._print_active)
        parts.append("Bridge in Print: %s" % self._bridge_seen_this_print)
        parts.append("Timeout: %.1fs" % self.heartbeat_timeout)
        parts.append("Timeout-Aktion: %s" % self.action_on_timeout)
        parts.append("Stop-Aktion: %s" % self.action_on_bridge_stop)
        parts.append("Timeout getriggert: %s" % self._timeout_triggered)
        parts.append("Stop getriggert: %s" % self._stop_triggered)

        gcmd.respond_info(" | ".join(parts))

    def cmd_BRIDGE_WATCHDOG_RESET(self, gcmd):
        """Setzt den Watchdog zurück (nach einem Trigger)."""
        self._timeout_triggered = False
        self._stop_triggered = False
        self._last_heartbeat = time.monotonic()
        gcmd.respond_info(
            "Bridge-Watchdog: Zurückgesetzt. "
            "Heartbeat-Timer neu gestartet.")

    # ── Cleanup ──────────────────────────────────────────────

    def _stop_tcp_server(self):
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None
        with self._client_lock:
            if self._client_socket:
                try:
                    self._client_socket.close()
                except Exception:
                    pass
                self._client_socket = None
        self.logger.info("BridgeWatchdog: TCP-Server gestoppt")


def load_config(config):
    return BridgeWatchdog(config)
