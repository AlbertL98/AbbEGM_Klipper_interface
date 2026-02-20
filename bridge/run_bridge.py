#!/usr/bin/env python3
# run_bridge.py — Startet den EGM-Bridge-Core als eigenständigen Service
#
# Nutzung:
#   python run_bridge.py                          # Mit Default-Config
#   python run_bridge.py --config my_config.json  # Mit eigener Config
#   python run_bridge.py --control-port 7201      # Control-Server Port
#   python run_bridge.py --auto-start             # Startet Job automatisch
#                                                 # bei Segment-Empfang
#   python run_bridge.py --dry-run                # Nur Config validieren
#
# Workflow (mit --auto-start, Default wenn über Klipper gestartet):
#   1. Bridge startet und wartet im READY-Zustand
#   2. Druckjob über Mainsail starten → Segmente kommen rein
#   3. Bridge erkennt Segmente und startet Job automatisch
#   4. Nach Job-Ende → zurück in READY → wartet auf nächsten Job
#
# Steuerung über TCP Control-Port (Default 7201):
#   {"cmd": "start", "job": "my_print"}  — Manueller Job-Start
#   {"cmd": "stop"}
#   {"cmd": "status"}
#   {"cmd": "set_param", "section": "sync", "key": "...", "value": ...}
#
# Wird von egm_commands.py (Klipper-Extra) per G-Code angesprochen.

import argparse
import json
import logging
import signal
import sys
import time

from egm_bridge import (
    EgmBridge, BridgeConfig, load_config, save_config, validate_config
)
from egm_bridge.control_server import ControlServer


def setup_logging(level: str = "INFO"):
    fmt = ("%(asctime)s.%(msecs)03d │ %(levelname)-7s │ "
           "%(name)-12s │ %(message)s")
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format=fmt,
        datefmt="%H:%M:%S",
    )


def main():
    parser = argparse.ArgumentParser(
        description="EGM-Bridge-Core: Klipper → ABB Roboter"
    )
    parser.add_argument(
        "--config", "-c", default="bridge_config.json",
        help="Pfad zur Konfigurationsdatei (JSON)"
    )
    parser.add_argument(
        "--control-port", type=int, default=7201,
        help="TCP-Port für Control-Server (Default: 7201)"
    )
    parser.add_argument(
        "--auto-start", action="store_true",
        help="Job automatisch starten sobald Segmente da sind"
    )
    parser.add_argument(
        "--generate-config", action="store_true",
        help="Erzeugt eine Default-Konfigurationsdatei"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Nur Config laden und validieren, nicht starten"
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    args = parser.parse_args()

    setup_logging(args.log_level)
    logger = logging.getLogger("egm.main")

    # ── Config generieren ────────────────────────────────
    if args.generate_config:
        cfg = BridgeConfig()
        save_config(cfg, args.config)
        print(f"Default-Config geschrieben: {args.config}")
        return

    # ── Config laden ─────────────────────────────────────
    logger.info("Lade Config: %s", args.config)
    cfg = load_config(args.config)

    # ── Validieren ───────────────────────────────────────
    errors = validate_config(cfg)
    if errors:
        logger.error("Config-Validierung fehlgeschlagen:")
        for e in errors:
            logger.error("  • %s", e)
        sys.exit(1)
    else:
        logger.info("Config OK (Profil: %s v%s)",
                     cfg.profile_name, cfg.profile_version)

    if args.dry_run:
        print("Config gültig. Dry-Run beendet.")
        print(json.dumps(cfg.to_dict(), indent=2))
        return

    # ── Bridge starten ───────────────────────────────────
    bridge = EgmBridge(cfg)

    if not bridge.start():
        logger.error("Bridge-Start fehlgeschlagen!")
        sys.exit(1)

    # ── Control-Server starten ───────────────────────────
    control = ControlServer(bridge, port=args.control_port)
    try:
        control.start()
    except OSError:
        logger.error("Control-Server konnte nicht starten (Port %d belegt?)",
                     args.control_port)
        bridge.shutdown()
        sys.exit(1)

    # ── Graceful Shutdown ────────────────────────────────
    _shutdown_done = False

    def do_shutdown():
        nonlocal _shutdown_done
        if _shutdown_done:
            return
        _shutdown_done = True
        control.stop()
        bridge.shutdown()

    def signal_handler(sig, frame):
        logger.info("Signal %d empfangen — fahre herunter...", sig)
        do_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ── Hauptschleife ────────────────────────────────────
    logger.info("Bridge READY — warte auf Commands (Control-Port: %d)",
                args.control_port)

    if args.auto_start:
        logger.info("Auto-Start aktiviert — Job startet automatisch "
                    "sobald Segmente empfangen werden")
    else:
        logger.info("Steuerung: EGM_START / EGM_STOP / EGM_STATUS "
                    "im Mainsail-Terminal")

    try:
        while True:
            state = bridge.sm.state.value

            # Auto-Start Modus: Job starten sobald Segmente da sind
            if (args.auto_start
                    and state == "READY"
                    and bridge.planner.queue_depth > 0):
                logger.info("Auto-Start: Erstes Segment empfangen "
                            "(Queue: %d) — starte Job...",
                            bridge.planner.queue_depth)
                bridge.run_job()

            elif state == "FAULT":
                logger.error("Bridge im FAULT-Zustand!")
                break

            elif state == "STOP":
                # Job abgeschlossen — Statistik loggen
                snap = bridge.planner.snapshot()
                logger.info("Job abgeschlossen!")
                logger.info("  Samples: %d | Segmente: %d | Overruns: %d",
                            snap["samples_generated"],
                            snap["segments_consumed"],
                            bridge._loop_overruns)

                # Zurück in READY-Zustand → bereit für nächsten Job
                logger.info("Wechsle zurück in READY — warte auf "
                            "nächsten Druckjob...")
                bridge.reset_to_ready()

            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        do_shutdown()


if __name__ == "__main__":
    main()
