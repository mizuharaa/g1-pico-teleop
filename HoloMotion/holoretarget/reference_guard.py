"""Reference sanity guard — LOCAL ADDITION (2026-08-07).

Sits between the PICO stream and the retargeted reference. When the operator
does something the robot must not follow (floating both legs, teleporting,
sensor glitches), the guard HOLDS the last good reference instead of passing
the impossible one to the balance policy. A held reference is the safest
possible command: the robot simply keeps doing what it was already doing.

Edge cases covered (input side, body_poses[24,7], PICO/SMPL-24 order):
  floating_legs   both ankles risen far above their calibrated height floor
                  (operator jumped, sat down, lifted both feet, or leg
                  trackers glitched) — a biped reference with no support
  root_teleport   pelvis moved more than MAX_ROOT_STEP_M in one frame
                  (tracking loss/re-acquire snap, occlusion glitch)
  nan_input       any non-finite value in the stream
Edge cases covered (output side, qpos[36]):
  root_too_low / root_too_high   reference pelvis outside plausible band
                  (squatted through the floor, or flying)
  root_tilt      reference root tilted > MAX_ROOT_TILT_DEG from vertical
                  (waist tracker slipped, or operator bent double — either
                  way the balance policy should not chase it)
  nan_output     solver produced non-finite values
Escalation (UPDATED 2026-08-10 — auto return-to-default):
  HOLD            single-frame violations: freeze reference, log, continue
  SUPPRESS        held longer than MAX_HOLD_S: gate() returns None and the
                  caller must publish NOTHING. Downstream, the policy node's
                  own data-age failsafe then switches the robot back to
                  velocity mode (standing default pose) — the same recoverable
                  state as pressing Y. This is what makes sustained garbage
                  (thrown controller, tracker on the floor, operator asleep)
                  end in "robot stands in default pose" instead of "robot
                  frozen in the last pose forever".
  RECOVERY        when the input is clean again the guard resumes passing
                  frames and logs it; the operator re-enters tracking with B.
                  The .stale flag mirrors the suppressed state for callers.

Tunables via environment (defaults chosen for gantry sessions):
  GUARD_FLOAT_RISE_M      both-ankle rise vs calibration floor  [0.25]
  GUARD_MAX_ROOT_STEP_M   per-frame root translation limit      [0.40]
  GUARD_ROOT_Z_MIN_M / GUARD_ROOT_Z_MAX_M                       [0.35 / 1.10]
  GUARD_MAX_ROOT_TILT_DEG                                       [50]
  GUARD_MAX_HOLD_S                                              [2.0]
  GUARD_MAX_DOF_STEP_RAD  per-frame joint-space jump limit      [0.60]
                          (violent arm flail from a thrown/glitched
                          controller; ~30 rad/s at 50 Hz)
"""
from __future__ import annotations

import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# SMPL-24 indices in the PICO stream
_PELVIS, _L_ANKLE, _R_ANKLE = 0, 7, 8
_HEAD = 15
_L_SHOULDER, _R_SHOULDER = 16, 17
_L_WRIST, _R_WRIST = 20, 21

_MJCF_PATH = (
    Path(__file__).resolve().parent
    / "assets" / "unitree_g1" / "g1_mocap_29dof.xml"
)


def _load_dof_limits() -> np.ndarray | None:
    """Joint limits [29, 2] (radians) parsed from the packaged G1 MJCF.

    Document order of the hinge joints matches qpos[7:] by construction
    (the same MJCF drives the MuJoCo viewers). Returns None (check
    disabled) on any parse surprise rather than guessing limits.
    """
    try:
        limits = []
        for joint in ET.parse(_MJCF_PATH).getroot().iter("joint"):
            rng = joint.get("range")
            if rng is None or joint.get("name") is None:
                continue   # defaults block / unnamed
            lo, hi = (float(v) for v in rng.split())
            limits.append((lo, hi))
        if len(limits) != 29:
            print(
                f"[GUARD] dof-limit check disabled: expected 29 ranged "
                f"joints in MJCF, found {len(limits)}",
                flush=True,
            )
            return None
        return np.asarray(limits, dtype=np.float64)
    except Exception as exc:
        print(f"[GUARD] dof-limit check disabled: {exc}", flush=True)
        return None


def _env_f(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


class ReferenceGuard:
    def __init__(self, clock=None) -> None:
        self.float_rise_m = _env_f("GUARD_FLOAT_RISE_M", 0.25)
        self.max_root_step_m = _env_f("GUARD_MAX_ROOT_STEP_M", 0.40)
        self.root_z_min = _env_f("GUARD_ROOT_Z_MIN_M", 0.35)
        self.root_z_max = _env_f("GUARD_ROOT_Z_MAX_M", 1.10)
        self.max_root_tilt_deg = _env_f("GUARD_MAX_ROOT_TILT_DEG", 50.0)
        self.max_hold_s = _env_f("GUARD_MAX_HOLD_S", 2.0)
        self.max_dof_step_rad = _env_f("GUARD_MAX_DOF_STEP_RAD", 0.60)
        self.max_hand_head_m = _env_f("GUARD_MAX_HAND_HEAD_M", 1.20)
        self.max_hand_step_m = _env_f("GUARD_MAX_HAND_STEP_M", 0.30)
        self.max_hand_spin_dps = _env_f("GUARD_MAX_HAND_SPIN_DPS", 720.0)
        self.spin_frames = int(_env_f("GUARD_SPIN_FRAMES", 5))
        self.max_arm_m = _env_f("GUARD_MAX_ARM_M", 0.95)
        self.floor_relearn_s = _env_f("GUARD_FLOOR_RELEARN_S", 10.0)
        self.dof_limit_margin_rad = _env_f("GUARD_DOF_LIMIT_MARGIN_RAD", 0.10)
        self._dof_limits = _load_dof_limits()   # [29, 2] radians or None
        self._clock = clock or time.time

        self._ankle_floor: float | None = None   # min ankle height seen (calibration)
        self._last_root: np.ndarray | None = None
        self._last_wrists: np.ndarray | None = None
        self._last_wrists_time: float | None = None
        self._last_wrist_quats: np.ndarray | None = None
        self._spin_streak = 0
        self._last_good_qpos: np.ndarray | None = None
        self._last_good_time: float | None = None
        self._hold_since: float | None = None
        self.stale = False
        self.trip_counts: dict[str, int] = {}
        self._last_log = 0.0
        self._streak_reason: str | None = None

    # ------------------------------------------------------------------ util
    def _trip(self, reason: str) -> None:
        self.trip_counts[reason] = self.trip_counts.get(reason, 0) + 1
        self._streak_reason = reason
        now = self._clock()
        if now - self._last_log > 1.0:   # rate-limited console line
            self._last_log = now
            held = f", holding {now - self._hold_since:.1f}s" if self._hold_since else ""
            print(f"[GUARD] {reason} (x{self.trip_counts[reason]}{held})", flush=True)

    def register_trip(self, reason: str) -> str:
        """Record a violation and decide the caller's action.

        Returns "hold" while inside the hold window (caller re-uses the last
        good reference) or "suppress" once the window is exhausted (caller
        must publish NOTHING so the policy's data-age failsafe returns the
        robot to the default standing pose).
        """
        self._trip(reason)
        now = self._clock()
        if self._hold_since is None:
            self._hold_since = now
        if now - self._hold_since > self.max_hold_s:
            if not self.stale:
                print(
                    f"[GUARD] reference held > {self.max_hold_s:.0f}s — "
                    f"SUPPRESSING output so the policy falls back to the "
                    f"default standing pose (velocity mode). Re-enter "
                    f"tracking with B once the input is clean.",
                    flush=True,
                )
            self.stale = True
            return "suppress"
        return "hold"

    def _mark_good(self) -> None:
        now = self._clock()
        if self._streak_reason is not None:
            held = (
                f" after {now - self._hold_since:.1f}s"
                if self._hold_since is not None
                else ""
            )
            print(
                f"[GUARD] input clean again{held} "
                f"(last violation: {self._streak_reason})",
                flush=True,
            )
        self._streak_reason = None
        self._hold_since = None
        self.stale = False
        self._last_good_time = now

    def reset(self) -> None:
        self._ankle_floor = None
        self._last_root = None
        self._last_wrists = None
        self._last_wrists_time = None
        self._last_wrist_quats = None
        self._spin_streak = 0
        self._last_good_qpos = None
        self._last_good_time = None
        self._hold_since = None
        self.stale = False
        self._streak_reason = None

    # ----------------------------------------------------------------- input
    def check_input(self, body_poses: np.ndarray) -> str | None:
        """Return a violation reason, or None if the frame is safe."""
        bp = np.asarray(body_poses)
        if not np.all(np.isfinite(bp)):
            return "nan_input"

        # PICO frame: Unity Y-up -> height is column 1
        pelvis = bp[_PELVIS, :3]
        head = bp[_HEAD, :3]
        l_ankle_y, r_ankle_y = float(bp[_L_ANKLE, 1]), float(bp[_R_ANKLE, 1])
        low_ankle = min(l_ankle_y, r_ankle_y)
        now = self._clock()

        # -------- skeleton structure (tracker/controller glitches, throws)
        # operator collapsed / headset dropped or dangling
        if float(head[1]) < float(pelvis[1]) - 0.15:
            return "head_below_pelvis"
        # a foot above the head = tracker glitch (no handstand support)
        if max(l_ankle_y, r_ankle_y) > float(head[1]):
            return "ankle_above_head"
        wrists = bp[[_L_WRIST, _R_WRIST], :3].astype(np.float64)
        # controller thrown / flung far from the body
        hand_head = np.linalg.norm(wrists - head[None, :], axis=1)
        if float(hand_head.max()) > self.max_hand_head_m:
            return "hand_far"
        # skeleton stretched beyond an arm's length = tracking corruption
        shoulders = bp[[_L_SHOULDER, _R_SHOULDER], :3].astype(np.float64)
        arm_len = np.linalg.norm(wrists - shoulders, axis=1)
        if float(arm_len.max()) > self.max_arm_m:
            return "arm_stretch"
        # per-frame wrist teleport (throw release, tracking re-acquire) and
        # sustained wrist SPIN (operator twirling a controller — the IK
        # chases the rotating wrist target and churns the whole arm)
        wrist_quats = bp[[_L_WRIST, _R_WRIST], 3:7].astype(np.float64)
        if (
            self._last_wrists is not None
            and self._last_wrists_time is not None
            and now - self._last_wrists_time < 0.5
        ):
            dt = max(now - self._last_wrists_time, 1e-3)
            hand_step = np.linalg.norm(wrists - self._last_wrists, axis=1)
            if float(hand_step.max()) > self.max_hand_step_m:
                self._last_wrists = wrists.copy()   # re-anchor for recovery
                self._last_wrists_time = now
                self._last_wrist_quats = wrist_quats.copy()
                return "hand_jump"
            if self._last_wrist_quats is not None:
                # angle between orientations; |dot| makes this immune to the
                # quat component layout and double-cover sign flips
                dots = np.abs(
                    np.sum(wrist_quats * self._last_wrist_quats, axis=1)
                )
                norms = (
                    np.linalg.norm(wrist_quats, axis=1)
                    * np.linalg.norm(self._last_wrist_quats, axis=1)
                )
                dots = np.clip(dots / np.maximum(norms, 1e-9), 0.0, 1.0)
                ang_dps = np.degrees(2.0 * np.arccos(dots)) / dt
                if float(ang_dps.max()) > self.max_hand_spin_dps:
                    self._spin_streak += 1
                else:
                    self._spin_streak = 0
                if self._spin_streak >= self.spin_frames:
                    self._last_wrists = wrists.copy()
                    self._last_wrists_time = now
                    self._last_wrist_quats = wrist_quats.copy()
                    return "hand_spin"
        else:
            self._spin_streak = 0
        self._last_wrists = wrists.copy()
        self._last_wrists_time = now
        self._last_wrist_quats = wrist_quats.copy()

        # -------- support (both feet floating) + ankle-floor calibration
        if self._ankle_floor is None:
            self._ankle_floor = low_ankle
        else:
            self._ankle_floor = min(self._ankle_floor, low_ankle)
            if low_ankle - self._ankle_floor > self.float_rise_m:
                # after prolonged floating suppression the ground level has
                # genuinely changed (moved to a platform, recalibrated):
                # re-learn the floor so recovery is possible at all
                if (
                    self._streak_reason == "floating_legs"
                    and self._hold_since is not None
                    and now - self._hold_since > self.floor_relearn_s
                ):
                    print(
                        f"[GUARD] floating_legs held {self.floor_relearn_s:.0f}s"
                        f" — re-learning ankle floor at {low_ankle:.3f}",
                        flush=True,
                    )
                    self._ankle_floor = low_ankle
                else:
                    return "floating_legs"

        # -------- temporal (root teleport)
        if self._last_root is not None:
            step = float(np.linalg.norm(pelvis - self._last_root))
            if step > self.max_root_step_m:
                self._last_root = pelvis.copy()   # re-anchor so recovery is possible
                return "root_teleport"
        self._last_root = pelvis.copy()
        return None

    # ---------------------------------------------------------------- output
    def check_output(self, qpos: np.ndarray) -> str | None:
        q = np.asarray(qpos).ravel()
        if not np.all(np.isfinite(q)):
            return "nan_output"
        z = float(q[2])
        if z < self.root_z_min:
            return "root_too_low"
        if z > self.root_z_max:
            return "root_too_high"
        # root tilt from vertical: rotate unit-z by root quaternion (wxyz)
        w, x, y, zq = q[3:7]
        up_z = 1.0 - 2.0 * (x * x + y * y)     # z-component of R(q) @ [0,0,1]
        tilt_deg = float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))
        if tilt_deg > self.max_root_tilt_deg:
            return "root_tilt"
        # impossible DOFs: reference outside the robot's physical joint
        # limits (+ margin) — the balance policy must never chase these
        if self._dof_limits is not None:
            dof = q[7:36]
            if (
                np.any(dof < self._dof_limits[:, 0] - self.dof_limit_margin_rad)
                or np.any(dof > self._dof_limits[:, 1] + self.dof_limit_margin_rad)
            ):
                return "dof_limit"
        # joint-space jump vs the last good frame (thrown controller, IK
        # glitch). Skipped across stream gaps (> 0.5 s) — after silence the
        # policy is already back in velocity mode and the first fresh frame
        # legitimately differs from the last one.
        if (
            self._last_good_qpos is not None
            and self._last_good_time is not None
            and self._clock() - self._last_good_time < 0.5
        ):
            dof_step = float(
                np.max(np.abs(q[7:] - self._last_good_qpos[7:]))
            )
            if dof_step > self.max_dof_step_rad:
                return "dof_jump"
        return None

    # ------------------------------------------------------------------ gate
    def gate(self, body_poses: np.ndarray, qpos: np.ndarray) -> np.ndarray | None:
        """Main entry: returns qpos to publish, held last-good, or None.

        None means SUPPRESSED — the caller must not publish anything this
        tick, so the downstream data-age failsafe returns the robot to the
        default standing pose.
        """
        reason = self.check_input(body_poses) or self.check_output(qpos)
        if reason is None:
            self._last_good_qpos = np.asarray(qpos, dtype=np.float32).copy()
            self._mark_good()
            return qpos

        if self.register_trip(reason) == "suppress":
            return None
        if self._last_good_qpos is not None:
            return self._last_good_qpos
        return None   # nothing good yet (startup) — publish nothing
