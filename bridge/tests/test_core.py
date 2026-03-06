#!/usr/bin/env python3
# test_core.py — Smoke-Test für den EGM-Bridge-Core
#
# Testet die Kernkomponenten in Isolation:
#   1. State Machine — Transitions
#   2. Config — Laden/Validieren
#   3. TrapezSegment — Interpolation
#   4. TrajectoryPlanner — Time-indexed Playback + Queue
#   5. SyncMonitor — Metriken
#
# Braucht keine Netzwerkverbindungen.

import sys
import os
import math
import time

# Modul-Pfad setzen
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, base_dir)

from data.state_machine import BridgeStateMachine, State
from data.config import BridgeConfig, validate_config
from data.segment_source import TrapezSegment
from data.trajectory_planner import TrajectoryPlanner, EgmSample
from data.sync_monitor import SyncMonitor, SyncLevel, SyncMetrics
from data.egm_client import EgmFeedback


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

    # time_offset_ms existiert und ist 0 per Default
    assert cfg.sync.time_offset_ms == 0.0

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
    print("  ✓ time_offset_ms vorhanden")
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

    # end_time Property
    assert abs(seg.end_time - 1.0) < 0.001

    print("  ✓ Position bei t=0: (0, 0, 100)")
    print("  ✓ Position bei t=0.5: (5, 0, 100)")
    print("  ✓ Position bei t=1.0: (10, 0, 100)")
    print("  ✓ Geschwindigkeit konstant 10mm/s")
    print("  ✓ end_time Property korrekt")

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

    v_start = seg2.velocity_at(0.0)
    v_mid = seg2.velocity_at(0.3)
    v_end = seg2.velocity_at(0.6)
    assert v_start < 1.0
    assert abs(v_mid - 20.0) < 0.1
    assert v_end < 1.0

    print("  ✓ Trapez-Profil: 0 → 20 → 0 mm/s")
    print("  ✓ TrapezSegment OK")


def test_trajectory_planner_time_indexed():
    print("\n═══ Test: Trajectory Planner (Time-Indexed) ═══")

    cycle_s = 0.004  # 4ms = 250Hz
    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100,
                                offset_s=0.0)

    assert planner.queue_depth == 0
    assert planner.is_starved
    assert not planner.time_synced

    # Ohne Zeitkopplung → kein Sample
    sample = planner.next_sample(time.monotonic())
    assert sample is None, "Ohne time_sync sollte kein Sample kommen"
    print("  ✓ Ohne Zeitkopplung: kein Sample")

    # 10 Segmente einfügen: jeweils 100ms, lückenlos
    # print_time: 0.0, 0.1, 0.2, ... 0.9
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

    # Zeitkopplung herstellen: bridge_time=100.0 ↔ klipper_time=0.0
    t0_bridge = 100.0
    planner.init_time_sync(bridge_time=t0_bridge, klipper_time=0.0)
    assert planner.time_synced
    print("  ✓ Zeitkopplung hergestellt")

    # Samples erzeugen über die gesamte Dauer (1.0s)
    samples = []
    t = t0_bridge
    while t < t0_bridge + 1.05:  # Etwas über 1s hinaus
        sample = planner.next_sample(t)
        if sample and sample.velocity > 0:
            samples.append(sample)
        t += cycle_s

    print(f"  ✓ {len(samples)} Samples erzeugt")
    assert len(samples) > 200  # 10 * 25 = 250 theoretisch

    # Erster Sample: sollte nahe Position (0, 0, 100) sein
    first = samples[0]
    assert abs(first.x - 0.0) < 0.1
    assert abs(first.z - 100.0) < 0.001
    print(f"  ✓ Start: ({first.x:.1f}, {first.y:.1f}, {first.z:.1f})")

    # Letzter Sample: sollte nahe Position (10, 0, 100) sein
    last = samples[-1]
    assert abs(last.x - 10.0) < 0.5
    print(f"  ✓ Ende:  ({last.x:.1f}, {last.y:.1f}, {last.z:.1f})")

    # X monoton steigend
    for i in range(1, len(samples)):
        assert samples[i].x >= samples[i - 1].x - 0.001

    print("  ✓ X-Position monoton steigend")

    # t_klipper ist im Sample vorhanden und steigend
    for i in range(1, len(samples)):
        assert samples[i].t_klipper >= samples[i - 1].t_klipper - 0.0001

    print("  ✓ t_klipper monoton steigend in Samples")

    # Segment-Zuordnung prüfen: bei t_klipper ≈ 0.55 sollte Segment 6
    # Seg 6 hat print_time=0.5, duration=0.1 → deckt [0.5, 0.6) ab
    mid_sample = None
    for s in samples:
        if 0.54 < s.t_klipper < 0.56:
            mid_sample = s
            break
    assert mid_sample is not None
    assert mid_sample.segment_nr == 6, f"Bei t_klipper≈0.55 erwartet Seg 6, bekam {mid_sample.segment_nr}"
    print(f"  ✓ Segment-Zuordnung korrekt (t_klipper≈0.55 → Seg #{mid_sample.segment_nr})")

    print("  ✓ Trajectory Planner (Time-Indexed) OK")


def test_trajectory_planner_offset():
    print("\n═══ Test: Trajectory Planner (Offset) ═══")

    cycle_s = 0.004
    offset_s = 0.05  # 50ms Vorlauf

    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100,
                                offset_s=offset_s)

    # Ein Segment: print_time=1.0, duration=1.0
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

    # Kopplung: bridge_time=0.0 ↔ klipper_time=1.0
    planner.init_time_sync(bridge_time=0.0, klipper_time=1.0)

    # Ohne Offset: bei bridge_time=0.0 → t_klipper=1.0 (Segment-Start)
    # Mit Offset +0.05: bei bridge_time=0.0 → t_klipper=1.05 (schon 50ms drin)
    sample = planner.next_sample(0.0)
    assert sample is not None
    assert abs(sample.t_klipper - 1.05) < 0.001, f"t_klipper={sample.t_klipper}, erwartet 1.05"
    # Position sollte ≈ 0.5mm sein (10mm/s * 0.05s)
    assert abs(sample.x - 0.5) < 0.1, f"x={sample.x}, erwartet ≈0.5"

    print(f"  ✓ Offset {offset_s * 1000:.0f}ms: t_klipper={sample.t_klipper:.3f}, x={sample.x:.2f}")
    print("  ✓ Trajectory Planner (Offset) OK")


def test_trajectory_planner_gap():
    print("\n═══ Test: Trajectory Planner (Zeitlücke) ═══")

    cycle_s = 0.004
    planner = TrajectoryPlanner(cycle_s=cycle_s, max_queue_size=100)

    # Segment 1: print_time=0.0, duration=0.1
    # Segment 2: print_time=0.5, duration=0.1 (400ms Lücke!)
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

    # Bei t_klipper=0.05 → im Segment 1
    s1 = planner.next_sample(t0 + 0.05)
    assert s1 is not None and s1.velocity > 0
    assert s1.segment_nr == 1
    print(f"  ✓ t_klipper=0.05: Segment #{s1.segment_nr}, x={s1.x:.2f}")

    # Bei t_klipper=0.25 → in der Lücke, Position halten
    s_gap = planner.next_sample(t0 + 0.25)
    # Sollte ein Hold-Sample sein (velocity=0) oder None + Hold
    # Der Planner gibt None zurück für Lücken (letzte Position halten
    # passiert im Bridge-Loop). Aber wir haben _last_position gesetzt,
    # also kommt ein Hold-Sample mit velocity=0.
    if s_gap is not None:
        assert s_gap.velocity == 0.0, "In Lücke sollte velocity=0 sein"
        assert abs(s_gap.x - 1.0) < 0.01, "Hold-Position sollte Seg1-Ende sein"
        print(f"  ✓ t_klipper=0.25: Lücke, Hold bei x={s_gap.x:.2f}")
    else:
        print("  ✓ t_klipper=0.25: Lücke, None (kein Hold-Sample)")

    # Bei t_klipper=0.55 → im Segment 2
    s2 = planner.next_sample(t0 + 0.55)
    assert s2 is not None and s2.velocity > 0
    assert s2.segment_nr == 2
    print(f"  ✓ t_klipper=0.55: Segment #{s2.segment_nr}, x={s2.x:.2f}")

    print("  ✓ Trajectory Planner (Zeitlücke) OK")


def test_sync_monitor():
    print("\n═══ Test: Sync Monitor ═══")

    from data.config import SyncConfig
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
        x=103.0, y=50.0, z=200.0,
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
        x=115.0, y=50.0, z=200.0,
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
    print("║  (Time-Indexed Playback)                 ║")
    print("╚══════════════════════════════════════════╝")

    tests = [
        test_state_machine,
        test_config,
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
