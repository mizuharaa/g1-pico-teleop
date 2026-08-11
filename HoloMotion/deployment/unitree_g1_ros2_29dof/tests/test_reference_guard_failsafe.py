"""Failsafe tests for the reference guard hold -> suppress -> recover chain.

The contract under test (2026-08-10 failsafe rework):
- transient garbage (< max_hold_s): guard HOLDS the last good reference
- sustained garbage (> max_hold_s): gate() returns None (SUPPRESS) so the
  caller publishes nothing and the policy's data-age failsafe returns the
  robot to the default standing pose
- clean input after suppression: guard recovers and passes frames again
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from holoretarget.reference_guard import ReferenceGuard


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def tick(self, dt: float) -> None:
        self.now += dt


def good_body_poses() -> np.ndarray:
    """A plausible standing skeleton (Unity Y-up, SMPL-24 order)."""
    bp = np.zeros((24, 7), dtype=np.float32)
    bp[:, 4] = 1.0            # identity-ish quats, irrelevant to checks
    bp[0, :3] = [0.0, 0.9, 0.0]     # pelvis
    bp[7, :3] = [-0.1, 0.1, 0.0]    # left ankle
    bp[8, :3] = [0.1, 0.1, 0.0]     # right ankle
    bp[15, :3] = [0.0, 1.60, 0.0]   # head
    bp[16, :3] = [-0.20, 1.40, 0.0]  # left shoulder
    bp[17, :3] = [0.20, 1.40, 0.0]   # right shoulder
    bp[20, :3] = [-0.30, 0.95, 0.0]  # left wrist
    bp[21, :3] = [0.30, 0.95, 0.0]   # right wrist
    return bp


def good_qpos() -> np.ndarray:
    q = np.zeros(36, dtype=np.float32)
    q[2] = 0.79               # root z
    q[3] = 1.0                # unit quat wxyz
    return q


def make_guard(clock: FakeClock) -> ReferenceGuard:
    return ReferenceGuard(clock=clock)


def step_clean(guard: ReferenceGuard, clock: FakeClock, n: int = 1):
    out = None
    for _ in range(n):
        out = guard.gate(good_body_poses(), good_qpos())
        clock.tick(0.02)
    return out


def test_clean_frames_pass():
    clock = FakeClock()
    guard = make_guard(clock)
    out = step_clean(guard, clock, 5)
    assert out is not None
    assert not guard.stale


def test_nan_holds_then_suppresses_then_recovers():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 5)

    bad = good_body_poses()
    bad[3, 0] = np.nan
    # inside the hold window: last good reference is returned
    out = guard.gate(bad, good_qpos())
    assert out is not None and np.isfinite(out).all()
    assert not guard.stale
    # sustained garbage past max_hold_s: suppression
    suppressed = False
    for _ in range(int(guard.max_hold_s / 0.02) + 10):
        clock.tick(0.02)
        out = guard.gate(bad, good_qpos())
        if out is None:
            suppressed = True
            break
    assert suppressed, "guard never escalated to suppression"
    assert guard.stale
    # clean input again: frames flow, stale clears
    out = step_clean(guard, clock, 2)
    assert out is not None
    assert not guard.stale


def test_startup_bad_frame_publishes_nothing():
    clock = FakeClock()
    guard = make_guard(clock)
    bad = good_body_poses()
    bad[0, 0] = np.nan
    assert guard.gate(bad, good_qpos()) is None


def test_floating_legs_trips():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)   # learn ankle floor at 0.1
    floating = good_body_poses()
    floating[7, 1] = 0.1 + guard.float_rise_m + 0.05
    floating[8, 1] = 0.1 + guard.float_rise_m + 0.05
    assert guard.check_input(floating) == "floating_legs"


def test_root_teleport_trips_and_reanchors():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    teleported = good_body_poses()
    teleported[0, :3] = [5.0, 0.9, 0.0]
    assert guard.check_input(teleported) == "root_teleport"
    # re-anchored: the SAME position is no longer a teleport
    assert guard.check_input(teleported) is None


def test_dof_jump_trips_within_streak():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    jumped = good_qpos()
    jumped[10] = guard.max_dof_step_rad + 0.2
    assert guard.check_output(jumped) == "dof_jump"


def test_dof_jump_skipped_across_stream_gap():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    clock.tick(1.0)               # stream gap > 0.5 s (policy already fell back)
    jumped = good_qpos()
    jumped[10] = guard.max_dof_step_rad + 0.2
    assert guard.check_output(jumped) is None


def test_root_band_and_tilt():
    clock = FakeClock()
    guard = make_guard(clock)
    low = good_qpos()
    low[2] = guard.root_z_min - 0.05
    assert guard.check_output(low) == "root_too_low"
    high = good_qpos()
    high[2] = guard.root_z_max + 0.05
    assert guard.check_output(high) == "root_too_high"
    tilted = good_qpos()
    # 90-degree pitch: quat wxyz = [cos45, 0, sin45, 0]
    tilted[3:7] = [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]
    assert guard.check_output(tilted) == "root_tilt"


def test_register_trip_escalation_timeline():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 2)
    assert guard.register_trip("nan_input") == "hold"
    clock.tick(guard.max_hold_s + 0.1)
    assert guard.register_trip("nan_input") == "suppress"
    assert guard.stale


def test_suppression_recovers_via_mark_good():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 2)
    guard.register_trip("nan_input")
    clock.tick(guard.max_hold_s + 0.1)
    assert guard.register_trip("nan_input") == "suppress"
    out = step_clean(guard, clock, 1)
    assert out is not None
    assert not guard.stale


def test_head_below_pelvis_trips():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 2)
    collapsed = good_body_poses()
    collapsed[15, 1] = 0.5    # head below pelvis (0.9)
    assert guard.check_input(collapsed) == "head_below_pelvis"


def test_ankle_above_head_trips():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 2)
    glitched = good_body_poses()
    glitched[7, 1] = 1.8      # left ankle above the head
    assert guard.check_input(glitched) == "ankle_above_head"


def test_hand_far_trips():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 2)
    thrown = good_body_poses()
    thrown[21, :3] = [2.0, 0.95, 0.0]   # controller flew away
    assert guard.check_input(thrown) == "hand_far"


def test_arm_stretch_trips():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 2)
    stretched = good_body_poses()
    stretched[20, :3] = [-0.20, 0.44, 0.0]   # 0.96 m below the shoulder
    # keep it within hand_far reach so we isolate the stretch check
    assert float(np.linalg.norm(stretched[20, :3] - stretched[15, :3])) < 1.2
    assert guard.check_input(stretched) == "arm_stretch"


def test_hand_jump_trips_and_reanchors():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    jumped = good_body_poses()
    jumped[21, :3] = [0.30 + 0.5, 0.95, 0.0]   # wrist teleports 0.5 m
    assert guard.check_input(jumped) == "hand_jump"
    # re-anchored: same position next frame is accepted again
    assert guard.check_input(jumped) is None


def test_hand_jump_skipped_across_gap():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    clock.tick(1.0)   # stream gap
    jumped = good_body_poses()
    jumped[21, :3] = [0.30 + 0.5, 0.95, 0.0]
    assert guard.check_input(jumped) is None


def test_floating_legs_floor_relearn_allows_recovery():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)   # ankle floor learned at 0.1
    floating = good_body_poses()
    floating[7, 1] = floating[8, 1] = 0.1 + guard.float_rise_m + 0.1
    # sustained floating: hold -> suppress
    for _ in range(int(guard.max_hold_s / 0.02) + 10):
        guard.gate(floating, good_qpos())
        clock.tick(0.02)
    assert guard.stale
    # keep floating past the re-learn window -> floor recalibrates -> recovery
    clock.tick(guard.floor_relearn_s)
    out = guard.gate(floating, good_qpos())
    assert out is not None
    assert not guard.stale


def test_dof_limit_trips():
    clock = FakeClock()
    guard = make_guard(clock)
    assert guard._dof_limits is not None, "MJCF joint limits failed to load"
    bad = good_qpos()
    # left knee (index 3 in the 29-dof order) far past its 2.88 rad limit
    bad[7 + 3] = 4.0
    assert guard.check_output(bad) == "dof_limit"
    ok = good_qpos()
    assert guard.check_output(ok) is None


def _wrist_quat_frame(angle_rad: float) -> np.ndarray:
    """good_body_poses with the right wrist rotated about Z by angle."""
    bp = good_body_poses()
    bp[21, 3:7] = [np.cos(angle_rad / 2), 0.0, 0.0, np.sin(angle_rad / 2)]
    return bp


def test_hand_spin_trips_on_sustained_twirl():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    # 20 deg/frame at 50 Hz = 1000 deg/s — a controller being twirled
    angle = 0.0
    tripped = None
    for _ in range(guard.spin_frames + 2):
        angle += np.radians(20.0)
        tripped = guard.check_input(_wrist_quat_frame(angle))
        clock.tick(0.02)
        if tripped:
            break
    assert tripped == "hand_spin"
    # after the trip the anchor resets: a now-stationary wrist recovers
    assert guard.check_input(_wrist_quat_frame(angle)) is None


def test_hand_spin_ignores_normal_rotation():
    clock = FakeClock()
    guard = make_guard(clock)
    step_clean(guard, clock, 3)
    # 5 deg/frame = 250 deg/s — brisk but normal hand motion
    angle = 0.0
    for _ in range(20):
        angle += np.radians(5.0)
        assert guard.check_input(_wrist_quat_frame(angle)) is None
        clock.tick(0.02)


def test_stream_resume_gate_blocks_zombie_frame():
    from holoretarget.stream_gate import StreamResumeGate

    clock = FakeClock()
    logs = []
    gate = StreamResumeGate(clock=clock, log=logs.append)
    # startup: first frame is a probe, second (11 ms later) flows
    assert gate.accept() is False
    clock.tick(0.011)
    assert gate.accept() is True
    # live stream keeps flowing
    for _ in range(10):
        clock.tick(0.011)
        assert gate.accept() is True
    # 5 s silence, then ONE cached zombie frame: blocked
    clock.tick(5.0)
    assert gate.accept() is False
    # no follow-up within the probe window, then another lone frame: blocked
    clock.tick(3.0)
    assert gate.accept() is False
    # a real resume: next frame arrives at stream rate and flows
    clock.tick(0.011)
    assert gate.accept() is True


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {name}: {exc}")
    sys.exit(1 if failures else 0)
