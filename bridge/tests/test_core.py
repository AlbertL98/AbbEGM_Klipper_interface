#!/usr/bin/env python3
# test_core.py — Smoke-Test für den EGM-Bridge-Core
#
# Testet die Kernkomponenten in Isolation:
#   1. State Machine — Transitions
#   2. Config — Laden/Validieren
#   3. TrapezSegment — Interpolation
#   4. TrajectoryPlanner — Queue + Sample-Erzeugung
#   5. SyncMonitor — Metriken
#
# Braucht keine Netzwerkverbindungen.

import sys
import os
import math
import time

# Modul-Pfad setzen
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from egm_bridge.state_machine import BridgeStateMachine, State
from egm_bridge.config import BridgeConfig, validate_config
from egm_bridge.segment_source import TrapezSegment
from egm_bridge.trajectory_planner import TrajectoryPlanner, EgmSample
from egm_bridge.sync_monitor import SyncMonitor, SyncLevel, SyncMetrics
from egm_bridge.egm_client import EgmFeedback


def test_state_machine():
    print("\n═══ Test: State Machine ═══")
    sm = BridgeStateMachine()

    assert sm.state == State.INIT
    assert sm.to_ready()
    assert sm.state == State.READY
    assert sm.to_run()
    assert sm.state == State.RUN
    assert sm.to_degraded("Test-Degrade")
    assert sm.state == State.DEGRADED
    assert sm.to_run("Recovery")
    assert sm.state == State.RUN
    assert sm.to_stop("Manuell")
    assert sm.state == State.STOP

    # Ungültiger Übergang: STOP → RUN
    assert not sm.transition(State.RUN, "Ungültig")
    assert sm.state == State.STOP

    # STOP → READY → RUN → FAULT → INIT
    assert sm.to_ready()
    assert sm.to_run()
    assert sm.to_fault("Test-Fault")
    assert sm.state == State.FAULT
    assert sm.reset()
    assert sm.state == State.INIT

    print(f"  ✓ {len(sm.history)} Transitions korrekt")
    print("  ✓ Ungültige Transitions abgelehnt")
    print("  ✓ State Machine OK")


def test_config():
    print("\n═══ Test: Config ═══")
    cfg = BridgeConfig()

    # Default-Werte (passend zu workingEGM.py)
    assert cfg.connection.cycle_ms == 20
    assert cfg.connection.robot_ip == "127.0.0.1"
    assert cfg.connection.send_port == 6599
    assert cfg.connection.recv_port == 6510

    # Validierung — Defaults sollten OK sein
    errors = validate_config(cfg)
    assert len(errors) == 0, f"Default-Config hat Fehler: {errors}"

    # Absichtlich kaputte Config
    cfg.queues.low_watermark = 9999
    cfg.queues.high_watermark = 10
    errors = validate_config(cfg)
    assert len(errors) > 0
    print(f"  ✓ Kaputte Config erkannt: {errors[0]}")

    # Snapshot
    snap = cfg.snapshot()
    assert "config" in snap
    assert "timestamp" in snap

    print("  ✓ Default-Validierung OK")
    print("  ✓ Config OK")


def test_trapez_segment():
    print("\n═══ Test: TrapezSegment Interpolation ═══")

    # Einfaches Segment: Gerade Linie in X-Richtung
    # 10mm bei konstant 10mm/s = 1s Dauer
    seg = TrapezSegment(
        nr=1, print_time=0.0, duration=1.0,
        start_x=0.0, start_y=0.0, start_z=100.0,
        end_x=10.0, end_y=0.0, end_z=100.0,
        distance=10.0,
        start_v=10.0, cruise_v=10.0, end_v=10.0,
        accel=0.0,
        accel_t=0.0, cruise_t=1.0, decel_t=0.0,
        axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
    )

    # t=0 → Start
    x, y, z = seg.position_at(0.0)
    assert abs(x - 0.0) < 0.001
    assert abs(z - 100.0) < 0.001

    # t=0.5 → Mitte
    x, y, z = seg.position_at(0.5)
    assert abs(x - 5.0) < 0.001

    # t=1.0 → Ende
    x, y, z = seg.position_at(1.0)
    assert abs(x - 10.0) < 0.001

    # Geschwindigkeit
    assert abs(seg.velocity_at(0.5) - 10.0) < 0.001

    print("  ✓ Position bei t=0: (0, 0, 100)")
    print("  ✓ Position bei t=0.5: (5, 0, 100)")
    print("  ✓ Position bei t=1.0: (10, 0, 100)")
    print("  ✓ Geschwindigkeit konstant 10mm/s")

    # Segment mit Trapez-Profil (beschleunigen + bremsen)
    seg2 = TrapezSegment(
        nr=2, print_time=1.0, duration=0.6,
        start_x=10.0, start_y=0.0, start_z=100.0,
        end_x=16.6, end_y=0.0, end_z=100.0,
        distance=6.6,
        start_v=0.0, cruise_v=20.0, end_v=0.0,
        accel=100.0,
        accel_t=0.2, cruise_t=0.2, decel_t=0.2,
        axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
    )

    # Geschwindigkeit soll von 0 → 20 → 0 gehen
    v_start = seg2.velocity_at(0.0)
    v_mid = seg2.velocity_at(0.3)
    v_end = seg2.velocity_at(0.6)
    assert v_start < 1.0  # ≈ 0
    assert abs(v_mid - 20.0) < 0.1  # cruise
    assert v_end < 1.0  # ≈ 0

    print("  ✓ Trapez-Profil: 0 → 20 → 0 mm/s")
    print("  ✓ TrapezSegment OK")


def test_trajectory_planner():
    print("\n═══ Test: Trajectory Planner ═══")

    cycle_s = 0.004  # 4ms = 250Hz
    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100)

    assert planner.queue_depth == 0
    assert planner.is_starved

    # 10 Segmente einfügen (jeweils 100ms = 25 Samples bei 4ms)
    for i in range(10):
        seg = TrapezSegment(
            nr=i + 1, print_time=i * 0.1, duration=0.1,
            start_x=float(i), start_y=0.0, start_z=100.0,
            end_x=float(i + 1), end_y=0.0, end_z=100.0,
            distance=1.0,
            start_v=10.0, cruise_v=10.0, end_v=10.0,
            accel=0.0,
            accel_t=0.0, cruise_t=0.1, decel_t=0.0,
            axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
        )
        planner.add_segment(seg)

    assert planner.queue_depth == 10
    print(f"  ✓ 10 Segmente eingefügt (Queue-Zeit: {planner.queue_time_s:.1f}s)")

    # Samples erzeugen
    samples = []
    t = 0.0
    while not planner.is_starved:
        sample = planner.next_sample(t)
        if sample:
            samples.append(sample)
        t += cycle_s

    print(f"  ✓ {len(samples)} Samples erzeugt")
    assert len(samples) > 200  # 10 * 25 = 250 theoretisch

    # Erster und letzter Sample prüfen
    first = samples[0]
    last = samples[-1]
    assert abs(first.x - 0.0) < 0.1
    assert abs(last.x - 10.0) < 0.5  # Ungefähr Endposition
    print(f"  ✓ Start: ({first.x:.1f}, {first.y:.1f}, {first.z:.1f})")
    print(f"  ✓ Ende:  ({last.x:.1f}, {last.y:.1f}, {last.z:.1f})")

    # Monoton steigend in X?
    for i in range(1, len(samples)):
        assert samples[i].x >= samples[i-1].x - 0.001

    print("  ✓ X-Position monoton steigend")
    print("  ✓ Trajectory Planner OK")


def test_sync_monitor():
    print("\n═══ Test: Sync Monitor ═══")

    from egm_bridge.config import SyncConfig
    cfg = SyncConfig()
    mon = SyncMonitor(cfg)

    # Warmup durchlaufen lassen
    sample_ok = EgmSample(
        timestamp=1.0, x=100.0, y=50.0, z=200.0,
        velocity=10.0, segment_nr=1, segment_progress=0.5,
        sequence_id=1,
    )
    feedback_ok = EgmFeedback(
        sequence_id=1, timestamp=1.001, robot_time=1000.0,
        x=100.0, y=50.0, z=200.0,
    )
    for i in range(105):  # > warmup_cycles (100)
        mon.update(sample_ok, feedback_ok, buffer_depth=50, buffer_time_s=0.5)

    assert mon.sync_level == SyncLevel.OK
    print(f"  ✓ Nach Warmup: Level={mon.sync_level.value}")

    # Großer Tracking-Error → WARN/DEGRADE
    feedback_off = EgmFeedback(
        sequence_id=2, timestamp=1.005, robot_time=1004.0,
        x=103.0, y=50.0, z=200.0,  # 3mm Abweichung
    )
    sample2 = EgmSample(
        timestamp=1.004, x=100.0, y=50.0, z=200.0,
        velocity=10.0, segment_nr=1, segment_progress=0.6,
        sequence_id=2,
    )
    mon.update(sample2, feedback_off, buffer_depth=50, buffer_time_s=0.5)
    assert mon.sync_level in (SyncLevel.WARN, SyncLevel.DEGRADE)
    print(f"  ✓ 3mm Abweichung: Level={mon.sync_level.value} "
          f"(Error={mon.metrics.tracking_error_mm:.1f}mm)")

    # Riesiger Error → STOP
    feedback_bad = EgmFeedback(
        sequence_id=3, timestamp=1.009, robot_time=1008.0,
        x=115.0, y=50.0, z=200.0,  # 15mm Abweichung
    )
    sample3 = EgmSample(
        timestamp=1.008, x=100.0, y=50.0, z=200.0,
        velocity=10.0, segment_nr=1, segment_progress=0.7,
        sequence_id=3,
    )
    mon.update(sample3, feedback_bad, buffer_depth=50, buffer_time_s=0.5)
    assert mon.sync_level == SyncLevel.STOP
    print(f"  ✓ 15mm Abweichung: Level={mon.sync_level.value}")

    print("  ✓ Sync Monitor OK")


def main():
    print("╔══════════════════════════════════════════╗")
    print("║  EGM-Bridge-Core Smoke Test              ║")
    print("╚══════════════════════════════════════════╝")

    tests = [
        test_state_machine,
        test_config,
        test_trapez_segment,
        test_trajectory_planner,
        test_sync_monitor,
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
