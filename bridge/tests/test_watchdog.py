#!/usr/bin/env python3
# test_watchdog.py — Tests für KlipperCommandClient
#
# Testet:
#   1. KlipperCommandClient — Verbindung, Heartbeat, Stop/Pause
#   2. WatchdogConfig — Validierung
#   3. Integration mit Bridge-State-Machine
#
# Braucht KEINEN laufenden Klipper — simuliert den TCP-Server.

import sys
import os
import json
import socket
import time
import threading

# Modul-Pfad setzen
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, base_dir)

from data.config import BridgeConfig, WatchdogConfig, validate_config
from data.klipper_command import KlipperCommandClient
from data.state_machine import BridgeStateMachine, State


def test_watchdog_config():
    print("\n═══ Test: WatchdogConfig ═══")

    cfg = BridgeConfig()

    # Default Watchdog-Config vorhanden
    assert hasattr(cfg, 'watchdog')
    assert cfg.watchdog.enabled is True
    assert cfg.watchdog.tcp_port == 7201
    assert cfg.watchdog.heartbeat_interval_s == 1.0
    print("  ✓ Default-Werte korrekt")

    # Validierung: Defaults OK
    errors = validate_config(cfg)
    assert len(errors) == 0, f"Default-Config hat Fehler: {errors}"
    print("  ✓ Default-Validierung OK")

    # Ungültige Werte erkennen
    cfg.watchdog.tcp_port = -1
    errors = validate_config(cfg)
    assert any("watchdog.tcp_port" in e for e in errors)
    cfg.watchdog.tcp_port = 7201  # Zurücksetzen

    cfg.watchdog.heartbeat_interval_s = 0.01  # Zu klein
    errors = validate_config(cfg)
    assert any("heartbeat_interval_s" in e for e in errors)
    cfg.watchdog.heartbeat_interval_s = 1.0

    # Port-Kollision erkennen
    cfg.watchdog.tcp_port = cfg.klipper.tcp_port  # Gleicher Port!
    errors = validate_config(cfg)
    assert any("kollidiert" in e for e in errors)
    print("  ✓ Ungültige Werte erkannt")

    print("  ✓ WatchdogConfig OK")


def test_klipper_command_client():
    print("\n═══ Test: KlipperCommandClient ═══")

    # Simulierten Server starten
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 17201))  # Test-Port
    server.listen(1)
    server.settimeout(5.0)

    received_messages = []

    def server_thread():
        try:
            client, addr = server.accept()
            client.settimeout(5.0)
            buffer = ""
            while True:
                try:
                    data = client.recv(4096)
                    if not data:
                        break
                    buffer += data.decode('utf-8')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            msg = json.loads(line.strip())
                            received_messages.append(msg)
                except socket.timeout:
                    continue
                except Exception:
                    break
            client.close()
        except Exception:
            pass

    srv_thread = threading.Thread(target=server_thread, daemon=True)
    srv_thread.start()

    # Client starten
    client = KlipperCommandClient(
        host='127.0.0.1',
        port=17201,
        heartbeat_interval_s=0.2,  # Schnell für Tests
        reconnect_interval_s=0.5,
    )
    client.start()

    # Warten auf erste Heartbeats
    time.sleep(1.0)
    assert client.connected, "Client sollte verbunden sein"
    print("  ✓ Client verbunden")

    # Heartbeats empfangen?
    heartbeats = [m for m in received_messages if m['type'] == 'heartbeat']
    assert len(heartbeats) >= 2, \
        f"Mindestens 2 Heartbeats erwartet, bekam {len(heartbeats)}"
    print(f"  ✓ {len(heartbeats)} Heartbeats empfangen")

    # Bridge-State wird mitgesendet
    client.set_bridge_state("RUN")
    time.sleep(0.5)
    run_hbs = [m for m in received_messages
               if m['type'] == 'heartbeat' and m.get('state') == 'RUN']
    assert len(run_hbs) >= 1
    print("  ✓ Bridge-State 'RUN' im Heartbeat")

    # Stop-Befehl senden
    before_count = len(received_messages)
    result = client.send_stop("Test-Stop-Grund")
    assert result is True
    time.sleep(0.3)

    stops = [m for m in received_messages[before_count:]
             if m['type'] == 'stop']
    assert len(stops) == 1
    assert "Test-Stop-Grund" in stops[0]['reason']
    print("  ✓ Stop-Befehl gesendet und empfangen")

    # Pause-Befehl senden
    before_count = len(received_messages)
    result = client.send_pause("Test-Pause-Grund")
    assert result is True
    time.sleep(0.3)

    pauses = [m for m in received_messages[before_count:]
              if m['type'] == 'pause']
    assert len(pauses) == 1
    print("  ✓ Pause-Befehl gesendet und empfangen")

    # Snapshot prüfen
    snap = client.snapshot()
    assert snap['connected'] is True
    assert snap['heartbeats_sent'] >= 2
    assert snap['commands_sent'] >= 2
    print(f"  ✓ Snapshot: {snap['heartbeats_sent']} HBs, "
          f"{snap['commands_sent']} Cmds")

    # Aufräumen
    client.stop()
    server.close()
    print("  ✓ KlipperCommandClient OK")


def test_state_machine_watchdog_integration():
    print("\n═══ Test: State Machine → Watchdog Integration ═══")

    # Simuliert die Integration: State-Change → Callback → Command

    sm = BridgeStateMachine()
    commands_received = []

    # Simulierter Callback (wie in bridge.py _on_state_change)
    def on_state_change(event):
        commands_received.append({
            "from": event.from_state.value,
            "to": event.to_state.value,
            "reason": event.reason,
        })

    sm.add_listener(on_state_change)

    # Normaler Ablauf
    sm.to_ready()
    sm.to_run()
    assert len(commands_received) == 2

    # FAULT → sollte Stop-Befehl auslösen
    sm.to_fault("Sync-Stop: Tracking 15mm > Limit")
    fault_event = commands_received[-1]
    assert fault_event['to'] == 'FAULT'
    assert 'Sync-Stop' in fault_event['reason']
    print("  ✓ FAULT-Event korrekt erfasst")

    # Reset und nochmal
    sm.reset()
    sm.to_ready()
    sm.to_run()

    # DEGRADED → optional Pause
    sm.to_degraded("Hoher Lag")
    degrade_event = commands_received[-1]
    assert degrade_event['to'] == 'DEGRADED'
    print("  ✓ DEGRADED-Event korrekt erfasst")

    # Recovery
    sm.to_run("Recovery")
    recovery_event = commands_received[-1]
    assert recovery_event['to'] == 'RUN'
    print("  ✓ Recovery-Event korrekt erfasst")

    print(f"  ✓ {len(commands_received)} State-Events insgesamt")
    print("  ✓ State Machine → Watchdog Integration OK")


def main():
    print("╔══════════════════════════════════════════╗")
    print("║  Bridge-Watchdog Test                    ║")
    print("╚══════════════════════════════════════════╝")

    tests = [
        test_watchdog_config,
        test_klipper_command_client,
        test_state_machine_watchdog_integration,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"\n  ✗ FEHLER: {e}")
            failed += 1
        except Exception as e:
            print(f"\n  ✗ EXCEPTION: {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    print(f"\n{'═' * 44}")
    print(f"  Ergebnis: {passed} bestanden, {failed} fehlgeschlagen")
    if failed == 0:
        print("  ✓ Alle Tests bestanden!")
    else:
        print("  ✗ Es gibt Fehler!")
        sys.exit(1)


if __name__ == "__main__":
    main()
