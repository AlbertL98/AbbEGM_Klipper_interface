#!/usr/bin/env python3
# test_core.py — Smoke-Test für den EGM-Bridge-Core
#
# Testet die Kernkomponenten in Isolation:
#   1. State Machine — Transitions
#   2. Config — Laden/Validieren (inkl. Workspace Envelope)
#   3. TrapezSegment — Interpolation + Validierung
#   4. TrajectoryPlanner — Time-indexed Playback + Queue
#   5. SyncMonitor — Metriken
#   6. Workspace Envelope — Begrenzungsbox
#   7. Unified Clock — bridge_now()
#
# Braucht keine Netzwerkverbindungen.

import sys
import os
import math
import time

# Modul-Pfad setzen
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, base_dir)

from data.clock import bridge_now
from data.state_machine import BridgeStateMachine, State
from data.config import (BridgeConfig, WorkspaceEnvelopeConfig,
                         validate_config)
from data.segment_source import (TrapezSegment, SegmentValidationError)
from data.trajectory_planner import TrajectoryPlanner, EgmSample
from data.sync_monitor import SyncMonitor, SyncLevel, SyncMetrics
from data.egm_client import EgmFeedback


def test_bridge_now():
    print("\n═══ Test: Unified Clock (bridge_now) ═══")

    t1 = bridge_now()
    time.sleep(0.01)
    t2 = bridge_now()

    assert t2 > t1, "bridge_now() muss monoton steigen"
    diff_ms = (t2 - t1) * 1000
    assert diff_ms >= 9.0, f"Mindestens ~10ms erwartet, bekam {diff_ms:.1f}ms"
    assert diff_ms < 50.0, f"Höchstens ~50ms erwartet, bekam {diff_ms:.1f}ms"

    print(f"  ✓ bridge_now() monoton, Δ={diff_ms:.1f}ms")
    print("  ✓ Unified Clock OK")


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

    # Default-Werte
    assert cfg.connection.cycle_ms == 20
    assert cfg.connection.robot_ip == "127.0.0.1"
    assert cfg.sync.time_offset_ms == 0.0

    # Workspace Envelope existiert
    assert hasattr(cfg, 'workspace')
    assert cfg.workspace.enabled is True
    assert cfg.workspace.min_z == -10.0
    print("  ✓ WorkspaceEnvelopeConfig vorhanden")

    # starvation_timeout_s existiert
    assert hasattr(cfg.queues, 'starvation_timeout_s')
    assert cfg.queues.starvation_timeout_s == 3.0
    print("  ✓ starvation_timeout_s vorhanden")

    # Validierung — Defaults sollten OK sein
    errors = validate_config(cfg)
    assert len(errors) == 0, f"Default-Config hat Fehler: {errors}"

    # Absichtlich kaputte Config
    cfg.queues.low_watermark = 9999
    cfg.queues.high_watermark = 10
    errors = validate_config(cfg)
    assert len(errors) > 0
    print(f"  ✓ Kaputte Config erkannt: {errors[0]}")

    # Kaputte Workspace
    cfg2 = BridgeConfig()
    cfg2.workspace.min_x = 100.0
    cfg2.workspace.max_x = -100.0  # min > max!
    errors2 = validate_config(cfg2)
    assert any("workspace" in e for e in errors2)
    print("  ✓ Kaputte Workspace erkannt")

    # Kaputte starvation_timeout
    cfg3 = BridgeConfig()
    cfg3.queues.starvation_timeout_s = 0.1  # zu klein
    errors3 = validate_config(cfg3)
    assert any("starvation_timeout" in e for e in errors3)
    print("  ✓ Kaputte starvation_timeout erkannt")

    print("  ✓ Config OK")


def test_workspace_envelope():
    print("\n═══ Test: Workspace Envelope ═══")

    ws = WorkspaceEnvelopeConfig(
        enabled=True,
        min_x=-100.0, max_x=100.0,
        min_y=-100.0, max_y=100.0,
        min_z=0.0, max_z=200.0,
    )

    # Innerhalb
    assert ws.contains(0.0, 0.0, 100.0)
    assert ws.contains(-100.0, -100.0, 0.0)  # Grenze inklusiv
    assert ws.contains(100.0, 100.0, 200.0)
    print("  ✓ Positionen innerhalb erkannt")

    # Außerhalb
    assert not ws.contains(101.0, 0.0, 100.0)  # X zu groß
    assert not ws.contains(0.0, -101.0, 100.0)  # Y zu klein
    assert not ws.contains(0.0, 0.0, -1.0)      # Z unter 0
    assert not ws.contains(0.0, 0.0, 201.0)     # Z zu groß
    print("  ✓ Positionen außerhalb erkannt")

    # Violation-Reason
    reason = ws.violation_reason(150.0, 0.0, -5.0)
    assert "X=150.00 > max_x=100.00" in reason
    assert "Z=-5.00 < min_z=0.00" in reason
    print(f"  ✓ Violation-Reason: {reason}")

    # enabled-Flag wird in bridge.py geprüft, nicht in contains()
    # contains() ist rein geometrisch
    ws_off = WorkspaceEnvelopeConfig(enabled=False)
    assert not ws_off.contains(99999.0, 99999.0, 99999.0)
    print("  ✓ contains() ist rein geometrisch (enabled in bridge.py)")

    # Innerhalb der Default-Bounds geht's auch bei disabled
    assert ws_off.contains(0.0, 0.0, 100.0)
    print("  ✓ Default-Bounds: (0,0,100) innerhalb")

    print("  ✓ Workspace Envelope OK")


def test_segment_validation():
    print("\n═══ Test: Segment-Validierung ═══")

    # Gültiges Segment
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
    seg.validate()  # Sollte nicht werfen
    print("  ✓ Gültiges Segment akzeptiert")

    # duration <= 0
    try:
        bad = TrapezSegment(
            nr=2, print_time=0.0, duration=0.0,
            start_x=0.0, start_y=0.0, start_z=0.0,
            end_x=0.0, end_y=0.0, end_z=0.0,
            distance=0.0,
            start_v=0.0, cruise_v=0.0, end_v=0.0,
            accel=0.0,
            accel_t=0.0, cruise_t=0.0, decel_t=0.0,
            axes_r_x=0.0, axes_r_y=0.0, axes_r_z=0.0,
        )
        bad.validate()
        assert False, "Hätte SegmentValidationError werfen sollen"
    except SegmentValidationError as e:
        assert "duration" in str(e)
        print(f"  ✓ duration=0 abgelehnt: {e}")

    # Negative Geschwindigkeit
    try:
        bad2 = TrapezSegment(
            nr=3, print_time=0.0, duration=1.0,
            start_x=0.0, start_y=0.0, start_z=0.0,
            end_x=10.0, end_y=0.0, end_z=0.0,
            distance=10.0,
            start_v=-5.0, cruise_v=10.0, end_v=10.0,
            accel=0.0,
            accel_t=0.0, cruise_t=1.0, decel_t=0.0,
            axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
        )
        bad2.validate()
        assert False, "Hätte SegmentValidationError werfen sollen"
    except SegmentValidationError as e:
        assert "Geschwindigkeit" in str(e)
        print(f"  ✓ Negative Geschwindigkeit abgelehnt")

    # Phasensumme != duration
    try:
        bad3 = TrapezSegment(
            nr=4, print_time=0.0, duration=1.0,
            start_x=0.0, start_y=0.0, start_z=0.0,
            end_x=10.0, end_y=0.0, end_z=0.0,
            distance=10.0,
            start_v=10.0, cruise_v=10.0, end_v=10.0,
            accel=0.0,
            accel_t=0.0, cruise_t=0.5, decel_t=0.0,  # Summe = 0.5 != 1.0
            axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
        )
        bad3.validate()
        assert False, "Hätte SegmentValidationError werfen sollen"
    except SegmentValidationError as e:
        assert "Phasensumme" in str(e)
        print(f"  ✓ Phasensumme != duration abgelehnt")

    print("  ✓ Segment-Validierung OK")


def test_trapez_segment():
    print("\n═══ Test: TrapezSegment Interpolation ═══")

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

    x, y, z = seg.position_at(0.0)
    assert abs(x - 0.0) < 0.001
    assert abs(z - 100.0) < 0.001

    x, y, z = seg.position_at(0.5)
    assert abs(x - 5.0) < 0.001

    x, y, z = seg.position_at(1.0)
    assert abs(x - 10.0) < 0.001

    assert abs(seg.velocity_at(0.5) - 10.0) < 0.001
    assert abs(seg.end_time - 1.0) < 0.001

    print("  ✓ Interpolation korrekt")

    # Distanz-Clamp-Test: position_at darf nicht über distance hinaus
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

    # Am Ende darf Position nicht über end_x hinausgehen
    x_end, _, _ = seg2.position_at(seg2.duration)
    assert x_end <= seg2.end_x + 0.01, (
        f"Position {x_end:.3f} überschreitet end_x {seg2.end_x:.3f}")
    print(f"  ✓ Distanz-Clamp: Ende bei x={x_end:.3f} (max {seg2.end_x})")

    print("  ✓ TrapezSegment OK")


def test_trajectory_planner_time_indexed():
    print("\n═══ Test: Trajectory Planner (Time-Indexed) ═══")

    cycle_s = 0.004
    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100,
                                offset_s=0.0)

    assert planner.queue_depth == 0
    assert planner.is_starved
    assert not planner.time_synced

    sample = planner.next_sample(bridge_now())
    assert sample is None
    print("  ✓ Ohne Zeitkopplung: kein Sample")

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
    print(f"  ✓ 10 Segmente eingefügt")

    t0_bridge = 100.0
    planner.init_time_sync(bridge_time=t0_bridge, klipper_time=0.0)
    assert planner.time_synced

    samples = []
    t = t0_bridge
    while t < t0_bridge + 1.05:
        sample = planner.next_sample(t)
        if sample and sample.velocity > 0:
            samples.append(sample)
        t += cycle_s

    print(f"  ✓ {len(samples)} Samples erzeugt")
    assert len(samples) > 200

    first = samples[0]
    assert abs(first.x - 0.0) < 0.1
    last = samples[-1]
    assert abs(last.x - 10.0) < 0.5
    print(f"  ✓ Start: ({first.x:.1f}), Ende: ({last.x:.1f})")

    for i in range(1, len(samples)):
        assert samples[i].x >= samples[i - 1].x - 0.001
    print("  ✓ X monoton steigend")

    for i in range(1, len(samples)):
        assert samples[i].t_klipper >= samples[i - 1].t_klipper - 0.0001
    print("  ✓ t_klipper monoton steigend")

    print("  ✓ Trajectory Planner (Time-Indexed) OK")


def test_trajectory_planner_offset():
    print("\n═══ Test: Trajectory Planner (Offset) ═══")

    cycle_s = 0.004
    offset_s = 0.05

    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100,
                                offset_s=offset_s)

    seg = TrapezSegment(
        nr=1, print_time=1.0, duration=1.0,
        start_x=0.0, start_y=0.0, start_z=100.0,
        end_x=10.0, end_y=0.0, end_z=100.0,
        distance=10.0,
        start_v=10.0, cruise_v=10.0, end_v=10.0,
        accel=0.0,
        accel_t=0.0, cruise_t=1.0, decel_t=0.0,
        axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
    )
    planner.add_segment(seg)
    planner.init_time_sync(bridge_time=0.0, klipper_time=1.0)

    sample = planner.next_sample(0.0)
    assert sample is not None
    assert abs(sample.t_klipper - 1.05) < 0.001
    assert abs(sample.x - 0.5) < 0.1

    print(f"  ✓ Offset {offset_s * 1000:.0f}ms: t_klipper="
          f"{sample.t_klipper:.3f}, x={sample.x:.2f}")
    print("  ✓ Trajectory Planner (Offset) OK")


def test_trajectory_planner_gap():
    print("\n═══ Test: Trajectory Planner (Zeitlücke) ═══")

    cycle_s = 0.004
    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100)

    seg1 = TrapezSegment(
        nr=1, print_time=0.0, duration=0.1,
        start_x=0.0, start_y=0.0, start_z=100.0,
        end_x=1.0, end_y=0.0, end_z=100.0,
        distance=1.0,
        start_v=10.0, cruise_v=10.0, end_v=10.0,
        accel=0.0,
        accel_t=0.0, cruise_t=0.1, decel_t=0.0,
        axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
    )
    seg2 = TrapezSegment(
        nr=2, print_time=0.5, duration=0.1,
        start_x=1.0, start_y=0.0, start_z=100.0,
        end_x=2.0, end_y=0.0, end_z=100.0,
        distance=1.0,
        start_v=10.0, cruise_v=10.0, end_v=10.0,
        accel=0.0,
        accel_t=0.0, cruise_t=0.1, decel_t=0.0,
        axes_r_x=1.0, axes_r_y=0.0, axes_r_z=0.0,
    )

    planner.add_segment(seg1)
    planner.add_segment(seg2)

    t0 = 100.0
    planner.init_time_sync(bridge_time=t0, klipper_time=0.0)

    s1 = planner.next_sample(t0 + 0.05)
    assert s1 is not None and s1.velocity > 0
    assert s1.segment_nr == 1
    print(f"  ✓ t_klipper=0.05: Segment #{s1.segment_nr}")

    s_gap = planner.next_sample(t0 + 0.25)
    if s_gap is not None:
        assert s_gap.velocity == 0.0
        assert abs(s_gap.x - 1.0) < 0.01
        print(f"  ✓ t_klipper=0.25: Lücke, Hold bei x={s_gap.x:.2f}")

    s2 = planner.next_sample(t0 + 0.55)
    assert s2 is not None and s2.velocity > 0
    assert s2.segment_nr == 2
    print(f"  ✓ t_klipper=0.55: Segment #{s2.segment_nr}")

    print("  ✓ Trajectory Planner (Zeitlücke) OK")


def test_sync_monitor():
    print("\n═══ Test: Sync Monitor ═══")

    from data.config import SyncConfig
    cfg = SyncConfig()
    mon = SyncMonitor(cfg)

    sample_ok = EgmSample(
        timestamp=1.0, x=100.0, y=50.0, z=200.0,
        velocity=10.0, segment_nr=1, segment_progress=0.5,
        sequence_id=1,
    )
    feedback_ok = EgmFeedback(
        sequence_id=1, timestamp=1.001, robot_time=1000.0,
        x=100.0, y=50.0, z=200.0,
    )
    for i in range(105):
        mon.update(sample_ok, feedback_ok,
                   buffer_depth=50, buffer_time_s=0.5)

    assert mon.sync_level == SyncLevel.OK
    print(f"  ✓ Nach Warmup: Level={mon.sync_level.value}")

    feedback_off = EgmFeedback(
        sequence_id=2, timestamp=1.005, robot_time=1004.0,
        x=103.0, y=50.0, z=200.0,
    )
    sample2 = EgmSample(
        timestamp=1.004, x=100.0, y=50.0, z=200.0,
        velocity=10.0, segment_nr=1, segment_progress=0.6,
        sequence_id=2,
    )
    mon.update(sample2, feedback_off,
               buffer_depth=50, buffer_time_s=0.5)
    assert mon.sync_level in (SyncLevel.WARN, SyncLevel.DEGRADE)
    print(f"  ✓ 3mm Abweichung: Level={mon.sync_level.value}")

    feedback_bad = EgmFeedback(
        sequence_id=3, timestamp=1.009, robot_time=1008.0,
        x=115.0, y=50.0, z=200.0,
    )
    sample3 = EgmSample(
        timestamp=1.008, x=100.0, y=50.0, z=200.0,
        velocity=10.0, segment_nr=1, segment_progress=0.7,
        sequence_id=3,
    )
    mon.update(sample3, feedback_bad,
               buffer_depth=50, buffer_time_s=0.5)
    assert mon.sync_level == SyncLevel.STOP
    print(f"  ✓ 15mm Abweichung: Level={mon.sync_level.value}")

    print("  ✓ Sync Monitor OK")


def main():
    print("╔══════════════════════════════════════════╗")
    print("║  EGM-Bridge-Core Smoke Test v0.3         ║")
    print("║  (+ Workspace Envelope, Clock, Validate) ║")
    print("╚══════════════════════════════════════════╝")

    tests = [
        test_bridge_now,
        test_state_machine,
        test_config,
        test_workspace_envelope,
        test_segment_validation,
        test_trapez_segment,
        test_trajectory_planner_time_indexed,
        test_trajectory_planner_offset,
        test_trajectory_planner_gap,
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
