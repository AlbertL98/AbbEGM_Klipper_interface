#!/usr/bin/env python3
# run_bridge.py — Startet den EGM-Bridge-Core als eigenständigen Service
#
# Nutzung:
#   python run_bridge.py                          # Mit Default-Config
#   python run_bridge.py --config my_config.json  # Mit eigener Config
#   python run_bridge.py --dry-run                # Nur Config validieren
#
# CodingPlan §G2: Core muss auch ohne GUI vollständig bedienbar bleiben

import argparse
import json
import logging
import signal
import sys
import time

from data import (
    EgmBridge, BridgeConfig, load_config, save_config, validate_config
)


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

    # Graceful Shutdown bei SIGINT/SIGTERM
    def signal_handler(sig, frame):
        logger.info("Signal %d empfangen — fahre herunter...", sig)
        bridge.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not bridge.start():
        logger.error("Bridge-Start fehlgeschlagen!")
        sys.exit(1)

    # ── Warten auf Segmente, dann Job starten ────────────
    logger.info("Bridge READY — warte auf Segmente von Klipper...")
    logger.info("Tipp: Starte einen Print in Klipper, "
                "oder sende Segmente per TCP an Port %d",
                cfg.klipper.tcp_port)

    # Auto-Start: Sobald erstes Segment da ist
    try:
        while True:
            if (bridge.sm.state.value == "READY"
                    and bridge.planner.queue_depth > 0):
                logger.info("Erstes Segment empfangen (Queue: %d) "
                            "— starte Job...",
                            bridge.planner.queue_depth)
                bridge.run_job()

            elif bridge.sm.state.value == "FAULT":
                logger.error("Bridge im FAULT-Zustand!")
                break

            elif bridge.sm.state.value == "STOP":
                logger.info("Job abgeschlossen!")
                snap = bridge.planner.snapshot()
                logger.info("  Samples: %d | Segmente: %d | Overruns: %d",
                            snap["samples_generated"],
                            snap["segments_consumed"],
                            bridge._loop_overruns)
                break

            # Periodischer Status im Betrieb
            elif bridge.sm.state.value in ("RUN", "DEGRADED"):
                snap = bridge.planner.snapshot()
                logger.debug(
                    "Loop: Queue=%d Samples=%d Seg=%s",
                    snap["queue_depth"],
                    snap["samples_generated"],
                    snap["current_segment"],
                )

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
