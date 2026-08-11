"""Closed-loop smoke tests: real deployed policy stack + faithful FSM port
+ MuJoCo G1, replaying the real operating procedure and its edge cases.

Usage:
    python run_smoke.py               # run all scenarios, print PASS/FAIL table
    python run_smoke.py startup y_exit  # run matching scenarios only
    HOLOMOTION_MODE_BLEND_S=0 python run_smoke.py y_exit   # A/B the blend
    python run_smoke.py --view b_press   # WATCH live in the MuJoCo viewer
    python run_smoke.py --record         # write MP4s to ~/Videos/g1-demos/
    python run_smoke.py --record --speed 0.5 y_exit  # half-speed video

Wiring per 500 Hz tick (mirrors the two-node deployment):
    FSM.control()  reads lowstate+buttons -> writes robot.cmd
    robot.step()   firmware PD + physics
    every 10 ticks (50 Hz): feed VR reference, sync lowstate + buttons into
    the policy runtime, run_policy_step(), push targets/kps/kds to the FSM
    ("topics").
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from g1_mujoco import G1MujocoRobot  # noqa: E402
from fsm_sim import MainNodeSim, RobotState, KeyMap  # noqa: E402
from sim_port import SimPort, REPO, HOLOMOTION  # noqa: E402

SCENE = str(HOLOMOTION / "assets/robots/unitree/G1/29dof/scene_29dof.xml")

# visualization config, set by main(): mode in (None, "record", "view")
RENDER = {"mode": None, "name": "", "dir": None, "speed": 1.0, "open": []}


class _Visual:
    """Per-Rig renderer: MP4 recording (GLFW offscreen) or live viewer."""

    FPS = 50  # one frame per policy tick

    def __init__(self, rig):
        import mujoco

        self.rig = rig
        self.mode = RENDER["mode"]
        self.viewer = None
        self.writer = None
        self._wall_next = None
        if self.mode is None:
            return
        m = rig.robot.model
        self.cam = mujoco.MjvCamera()
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = body if body >= 0 else 1
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 2.4, -12, 130
        RENDER["open"].append(self)
        if self.mode == "record":
            import cv2

            os.environ.setdefault("MUJOCO_GL", "glfw")
            self.cv2 = cv2
            self.renderer = mujoco.Renderer(m, 480, 640)
            n = RENDER["rig_count"] = RENDER.get("rig_count", 0) + 1
            suffix = f"_rig{n}" if n > 1 else ""
            path = os.path.join(RENDER["dir"], f"{RENDER['name']}{suffix}.mp4")
            self.writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"),
                self.FPS * RENDER["speed"], (640, 480),
            )
            self.path = path
        elif self.mode == "view":
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(
                m, rig.robot.data, show_left_ui=False, show_right_ui=False
            )
            self.viewer.cam.type = self.cam.type
            self.viewer.cam.trackbodyid = self.cam.trackbodyid
            self.viewer.cam.distance = self.cam.distance
            self.viewer.cam.elevation = self.cam.elevation
            self.viewer.cam.azimuth = self.cam.azimuth

    def _caption(self):
        r = self.rig
        return (
            f"{RENDER['name']}  t={r.clock.now:5.2f}s  "
            f"FSM={r.fsm.current_state.name}  "
            f"policy={r.port.current_policy_mode if r.port.policy_enabled else 'off'}"
            f"{'  GANTRY' if r.gantry_on else ''}"
        )

    def frame(self):
        if self.mode == "record":
            self.renderer.update_scene(self.rig.robot.data, camera=self.cam)
            img = self.renderer.render()
            bgr = self.cv2.cvtColor(img, self.cv2.COLOR_RGB2BGR)
            self.cv2.putText(
                bgr, self._caption(), (8, 20),
                self.cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
            )
            self.writer.write(bgr)
        elif self.mode == "view" and self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt("viewer closed")
            self.viewer.sync()
            # pace to real time * speed
            period = 0.02 / RENDER["speed"]
            now = time.perf_counter()
            if self._wall_next is None:
                self._wall_next = now
            self._wall_next += period
            if self._wall_next > now:
                time.sleep(self._wall_next - now)
            else:
                self._wall_next = now

    def close(self):
        if self.writer is not None:
            self.writer.release()
            print(f"    video: {self.path}")
        if self.viewer is not None:
            self.viewer.close()
FSM_YAML = str(
    HOLOMOTION
    / "deployment/unitree_g1_ros2_29dof/src/config/g1_29dof_holomotion.yaml"
)
STAND_H = 0.793


class _SimClock:
    """Injected in place of the `time` module inside policy_runtime so that
    reference data-age, soft-start and mode-blend all run on SIM time — the
    deployed code is unchanged, only its clock source is redirected (the
    real node gets wall time because it runs in real time)."""

    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now

    def perf_counter(self):
        return time.perf_counter()  # timings/telemetry stay wall-clock


class Rig:
    """One full simulated deployment: robot + FSM + policy node + 'gantry'."""

    def __init__(self, *, teleop=True, l1_patch=True, verbose=False,
                 max_data_age=1.5):
        self.clock = _SimClock()
        import humanoid_policy.policy_runtime as _prt
        _prt.time = self.clock
        self.robot = G1MujocoRobot(SCENE)
        self.fsm = MainNodeSim(FSM_YAML, self.robot, l1_patch=l1_patch)
        self.port = SimPort(
            enable_teleop_reference=teleop, verbose=verbose,
            max_data_age=max_data_age,
        )
        cfg = yaml.safe_load(open(FSM_YAML))
        self.default_real = np.array(
            [cfg["default_joint_angles"][n] for n in cfg["complete_dof_order"]],
            dtype=np.float64,
        )
        self.gantry_on = True
        self.tick = 0
        self._policy_paused = False
        self._last_fsm_state = None
        self._ref_frame_idx = 0
        self._published_seen = 0
        # start from "operator lowered robot to near-standing under gantry"
        self.robot.set_pose(self.default_real, base_pos=(0, 0, STAND_H))
        self.metrics = {
            "min_height": np.inf,
            "max_tilt": 0.0,
            "max_target_step": 0.0,
            "fell": False,
        }
        self._prev_target = None
        # joystick floats injected into the wireless_remote payload
        # (lx, rx, ry, ly) — parsed by the real RemoteController.set
        self.joystick = {"lx": 0.0, "rx": 0.0, "ry": 0.0, "ly": 0.0}
        self.visual = _Visual(self)

    # ------------------------------------------------------------- helpers
    def press(self, *buttons, ticks=25):
        """Hold buttons for `ticks` control ticks (50 ms default)."""
        self._pressed = (list(buttons), self.tick + ticks)

    _pressed = ([], -1)

    gantry_gain = 1.0  # 0..1, lets callers FADE the hold out instead of
    # cutting it — an instant cut releases a wound-up balance policy and
    # causes the launch lunge seen live on 2026-08-11

    def _apply_gantry(self):
        """Soft harness: pulls the pelvis toward the standing pose. Mirrors
        the physical gantry strap (snug, not rigid)."""
        d = self.robot.data
        if not self.gantry_on or self.gantry_gain <= 0.0:
            d.xfrc_applied[1][:] = 0.0
            return
        g = min(self.gantry_gain, 1.0)
        kp, kd = 2000.0 * g, 200.0 * g
        err = np.array([0.0, 0.0, STAND_H]) - d.qpos[0:3]
        vel = d.qvel[0:3]
        d.xfrc_applied[1][0:3] = kp * err - kd * vel
        # keep the trunk upright too
        d.xfrc_applied[1][3:6] = -50.0 * g * d.qvel[3:6]

    def stand_reference_qpos(self):
        q = np.zeros(36, dtype=np.float32)
        q[0:3] = (0.0, 0.0, STAND_H)
        q[3] = 1.0
        q[7:36] = self.default_real
        return q

    def feed_reference(self, qpos36):
        self.port._vr_reference.store(
            np.asarray(qpos36, dtype=np.float32),
            current_time=self.clock.now,
            sample_time=self.clock.now,
            frame_index=self._ref_frame_idx,
        )
        self._ref_frame_idx += 1

    # ------------------------------------------------------------- main loop
    def run(self, seconds, *, reference_fn=None, feed_reference=True):
        """Advance the world. reference_fn(t)->qpos36 or None (default stand)."""
        n = int(seconds * 500)
        for _ in range(n):
            t = self.tick / 500.0
            buttons, until = self._pressed
            active = self.tick < until
            for i in range(16):
                self.fsm.buttons[i] = 0
            if active:
                for b in buttons:
                    self.fsm.buttons[b] = 1

            self.fsm.control()
            self._apply_gantry()
            self.robot.step()

            # FSM state transitions -> policy node (robot_state topic)
            st = self.fsm.current_state
            if st is not self._last_fsm_state:
                self.port.runtime.set_robot_state(st.name)
                self._last_fsm_state = st

            if self.tick % 10 == 0 and not self._policy_paused:
                # 50 Hz policy node cycle
                if feed_reference and self.port.enable_teleop_reference:
                    ref = (
                        reference_fn(t)
                        if reference_fn is not None
                        else self.stand_reference_qpos()
                    )
                    if ref is not None:
                        self.feed_reference(ref)
                ls = self.robot.lowstate
                # encode buttons as the real 40-byte wireless_remote payload
                # (uint16 bitmask at bytes 2:4, parsed by RemoteController.set)
                mask = 0
                for i in range(16):
                    if self.fsm.buttons[i]:
                        mask |= 1 << i
                import struct as _struct

                js = self.joystick
                ls.wireless_remote = (
                    b"\x00\x00"
                    + mask.to_bytes(2, "little")
                    + _struct.pack("ffff", js["lx"], js["rx"], js["ry"], 0.0)
                    + _struct.pack("f", js["ly"])
                    + b"\x00" * 16
                )
                self.port.runtime.handle_low_state(ls)
                before = self.port.control_param_publishes
                self.port.runtime.run_policy_step()
                # "topics" to the FSM
                if self.port.control_param_publishes != before or not self.fsm.kps_received:
                    self.fsm.kps_handler(self.port.last_kps_real)
                    self.fsm.kds_handler(self.port.last_kds_real)
                if len(self.port.published_targets) > self._published_seen:
                    tgt = self.port.published_targets[-1]
                    self._published_seen = len(self.port.published_targets)
                    self.fsm.policy_action_handler(tgt)
                    if self._prev_target is not None:
                        step = float(np.max(np.abs(tgt - self._prev_target)))
                        self.metrics["max_target_step"] = max(
                            self.metrics["max_target_step"], step
                        )
                    self._prev_target = tgt

            self.tick += 1
            self.clock.now = self.tick / 500.0
            if self.visual.mode and self.tick % 10 == 0:
                self.visual.frame()
            h = self.robot.base_height
            tilt = self.robot.base_rpy_tilt
            self.metrics["min_height"] = min(self.metrics["min_height"], h)
            self.metrics["max_tilt"] = max(self.metrics["max_tilt"], tilt)
            if h < 0.35 or tilt > 1.0:
                self.metrics["fell"] = True

    # ------------------------------------------------------------- procedures
    def startup_to_policy(self):
        """The real ladder: ZERO_TORQUE -> Start -> 3.5 s MOVE_TO_DEFAULT ->
        A -> POLICY(velocity), all under the gantry."""
        self.run(0.2)
        self.press(KeyMap.start)
        self.run(3.5)
        assert self.fsm.current_state == RobotState.MOVE_TO_DEFAULT, self.fsm.log
        self.press(KeyMap.A)
        self.run(1.0)
        assert self.fsm.current_state == RobotState.POLICY, self.fsm.log
        assert self.port.policy_enabled, "policy node did not enable"

    def release_gantry(self, settle=2.0):
        self.run(settle)
        self.gantry_on = False
        self.robot.data.xfrc_applied[1][:] = 0.0

    def enter_tracking(self, warmup_s=0.5):
        """Feed reference until ready, then B (mirrors app connect + B)."""
        self.run(max(warmup_s, 15 * 0.02))  # >=13 frames for is_ready
        self.press(KeyMap.B)
        self.run(0.3)
        assert self.port.current_policy_mode == "motion", (
            "B did not enter motion: " + str(self.port.logger.records[-4:])
        )


# ================================================================ scenarios

def scenario_startup_velocity_stand(report):
    """Boot ladder then free-stand 5 s on the velocity policy."""
    rig = Rig(teleop=False)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.metrics["min_height"] = np.inf
    rig.metrics["max_tilt"] = 0.0
    rig.run(5.0)
    report(
        ok=not rig.metrics["fell"],
        detail=f"free-stand h_min={rig.metrics['min_height']:.3f} "
        f"tilt_max={np.degrees(rig.metrics['max_tilt']):.1f}deg",
    )


def scenario_b_press_and_track(report):
    """Enter tracking (soft-start) at stand, then a slow 2x squat."""
    rig = Rig(teleop=True)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.enter_tracking()

    names = rig.port.real_dof_names
    idx = {n: i for i, n in enumerate(names)}

    def squat_ref(t):
        q = rig.stand_reference_qpos()
        a = 0.5 * (1 - np.cos(2 * np.pi * (t % 3.0) / 3.0))  # 0..1..0 per 3 s
        a *= 0.5  # shallow squat
        for side in ("left", "right"):
            q[7 + idx[f"{side}_hip_pitch_joint"]] += -0.5 * a
            q[7 + idx[f"{side}_knee_joint"]] += 0.8 * a
            q[7 + idx[f"{side}_ankle_pitch_joint"]] += -0.3 * a
        q[2] = STAND_H - 0.12 * a
        return q

    rig.metrics["fell"] = False
    rig.run(6.0, reference_fn=squat_ref)
    knee = rig.robot.joint_pos()[idx["left_knee_joint"]]
    report(
        ok=not rig.metrics["fell"],
        detail=f"tracked 2 squat cycles h_min={rig.metrics['min_height']:.3f} "
        f"final_knee={knee:.2f}",
    )


def scenario_y_exit_blend(report):
    """Y mid-squat: tracking -> velocity. Gate: no one-step target snap and
    no fall. Set HOLOMOTION_MODE_BLEND_S=0 to see the unpatched snap."""
    rig = Rig(teleop=True)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.enter_tracking()
    names = rig.port.real_dof_names
    idx = {n: i for i, n in enumerate(names)}

    def half_squat(t):
        q = rig.stand_reference_qpos()
        a = min(t / 2.0, 1.0) * 0.5
        for side in ("left", "right"):
            q[7 + idx[f"{side}_hip_pitch_joint"]] += -0.5 * a
            q[7 + idx[f"{side}_knee_joint"]] += 0.8 * a
            q[7 + idx[f"{side}_ankle_pitch_joint"]] += -0.3 * a
        q[2] = STAND_H - 0.12 * a
        return q

    rig.run(3.0, reference_fn=half_squat)  # settle into held half-squat
    rig.metrics["max_target_step"] = 0.0
    rig.metrics["fell"] = False
    rig.press(KeyMap.Y)
    rig.run(3.0, reference_fn=half_squat)
    snap = rig.metrics["max_target_step"]
    report(
        ok=(not rig.metrics["fell"]) and snap < 0.15,
        detail=f"mode={rig.port.current_policy_mode} max_target_step={snap:.3f} rad "
        f"(blend_s={os.environ.get('HOLOMOTION_MODE_BLEND_S', '1.5')})",
    )


def scenario_stale_reference_fallback(report):
    """Headset stream dies mid-tracking -> max_data_age fallback to velocity.
    The exact mechanism of the 08-10 Wi-Fi bounces."""
    rig = Rig(teleop=True)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.enter_tracking()
    rig.run(1.0)
    rig.metrics["fell"] = False
    # starve until the max_data_age fallback fires (age > 1.5 s)
    n0 = rig.tick
    while rig.port.current_policy_mode == "motion" and rig.tick - n0 < 2500:
        rig.run(0.02, feed_reference=False)
    switched = rig.port.current_policy_mode == "velocity"
    # gate: the TRANSITION itself must be blended (no snap) and survivable.
    # (pre-switch ankle thrash on a frozen reference is motion-policy
    # behavior, measured by the trace tooling, not this gate)
    rig.metrics["max_target_step"] = 0.0
    rig._prev_target = None
    rig.run(2.0, feed_reference=False)
    ok = (
        switched
        and not rig.metrics["fell"]
        and rig.metrics["max_target_step"] < 0.15
    )
    report(
        ok=ok,
        detail=f"fallback={'yes' if switched else 'NO'} "
        f"post-switch max_target_step={rig.metrics['max_target_step']:.3f} "
        f"h_min={rig.metrics['min_height']:.3f}",
    )


def scenario_impossible_pose(report):
    """INFORMATIONAL: reference teleports to garbage (root 2 m away, joints
    at limits) with no reference_guard in the zmq path. Documents the
    unguarded worst case."""
    rig = Rig(teleop=True)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.enter_tracking()
    rig.run(1.0)
    garbage = rig.stand_reference_qpos()
    garbage[0] += 2.0
    garbage[2] = 0.2
    garbage[7:36] = 2.5

    rig.metrics["fell"] = False
    rig.run(2.0, reference_fn=lambda t: garbage)
    report(
        ok=True,  # informational — record, never gate
        detail=f"UNGUARDED garbage ref: fell={rig.metrics['fell']} "
        f"h_min={rig.metrics['min_height']:.3f} "
        f"max_step={rig.metrics['max_target_step']:.3f}",
        info=True,
    )


def scenario_select_estop(report):
    """Select mid-tracking: damp 2 s -> disable -> node dead. Collapse is
    EXPECTED; gate is the state sequence + shutdown."""
    rig = Rig(teleop=True)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.enter_tracking()
    rig.run(1.0)
    rig.press(KeyMap.select)
    rig.run(3.0)
    ok = rig.fsm.shutdown_done and not rig.port.policy_enabled
    report(
        ok=ok,
        detail=f"shutdown={rig.fsm.shutdown_done} "
        f"policy_enabled={rig.port.policy_enabled} h={rig.robot.base_height:.2f}",
    )


def scenario_l1_patch_vs_upstream(report):
    """L1 mid-policy: patched -> damped e-stop; upstream -> instant free-fall.
    Gate: patched peak fall speed is lower (damping does its job)."""
    peaks = {}
    for patched in (True, False):
        rig = Rig(teleop=False, l1_patch=patched)
        rig.startup_to_policy()
        rig.release_gantry()
        rig.run(1.0)
        rig.press(KeyMap.L1)
        peak = 0.0
        n0 = rig.tick
        while rig.tick - n0 < int(1.5 * 500):
            rig.run(0.02)
            peak = max(peak, float(np.max(np.abs(rig.robot.joint_vel()))))
        peaks[patched] = peak
    report(
        ok=peaks[True] < peaks[False],
        detail=f"peak joint speed patched={peaks[True]:.1f} rad/s "
        f"vs upstream-L1={peaks[False]:.1f} rad/s",
    )


def scenario_policy_loop_stall(report):
    """INFORMATIONAL: replay of the 08-10 fall mechanism — policy loop
    freezes 2.65 s (Warp JIT) while the 500 Hz FSM holds the last target."""
    rig = Rig(teleop=True)
    rig.startup_to_policy()
    rig.release_gantry()
    rig.enter_tracking()
    rig.run(1.0)
    rig.metrics["fell"] = False
    rig._policy_paused = True
    rig.run(2.65)
    rig._policy_paused = False
    rig.run(2.0)
    report(
        ok=True,
        detail=f"2.65 s stall at stand: fell={rig.metrics['fell']} "
        f"h_min={rig.metrics['min_height']:.3f} mode={rig.port.current_policy_mode}",
        info=True,
    )


SCENARIOS = [
    ("startup_velocity_stand", scenario_startup_velocity_stand),
    ("b_press_and_track", scenario_b_press_and_track),
    ("y_exit_blend", scenario_y_exit_blend),
    ("stale_reference_fallback", scenario_stale_reference_fallback),
    ("impossible_pose", scenario_impossible_pose),
    ("select_estop", scenario_select_estop),
    ("l1_patch_vs_upstream", scenario_l1_patch_vs_upstream),
    ("policy_loop_stall", scenario_policy_loop_stall),
]


def main():
    args = sys.argv[1:]
    if "--view" in args:
        RENDER["mode"] = "view"
    if "--record" in args:
        RENDER["mode"] = "record"
        RENDER["dir"] = os.path.expanduser(
            "~/Videos/g1-demos/sim_smoke_" + time.strftime("%Y%m%d_%H%M")
        )
        os.makedirs(RENDER["dir"], exist_ok=True)
        print(f"recording to {RENDER['dir']}/")
    if "--speed" in args:
        RENDER["speed"] = float(args[args.index("--speed") + 1])
    skip_next = False
    filters = []
    for a in args:
        if skip_next:
            skip_next = False
            continue
        if a == "--speed":
            skip_next = True
        elif not a.startswith("-"):
            filters.append(a)
    results = []
    for name, fn in SCENARIOS:
        if filters and not any(f in name for f in filters):
            continue
        t0 = time.time()
        out = {}
        RENDER["name"] = name
        RENDER["rig_count"] = 0

        def report(ok, detail, info=False, _out=out):
            _out.update(ok=ok, detail=detail, info=info)

        try:
            fn(report)
        except KeyboardInterrupt:
            print("viewer closed / interrupted — stopping")
            for v in RENDER["open"]:
                v.close()
            return 130
        except AssertionError as e:
            out.update(ok=False, detail=f"ASSERT: {e}", info=False)
        except Exception as e:  # noqa: BLE001
            out.update(ok=False, detail=f"ERROR: {type(e).__name__}: {e}", info=False)
        finally:
            for v in RENDER["open"]:
                v.close()
            RENDER["open"].clear()
        dt = time.time() - t0
        tag = "INFO" if out.get("info") else ("PASS" if out["ok"] else "FAIL")
        print(f"[{tag}] {name:28s} ({dt:5.1f}s)  {out['detail']}")
        results.append((name, out))

    gating = [r for _, r in results if not r.get("info")]
    failed = [n for n, r in results if not r["ok"] and not r.get("info")]
    print(
        f"\n{len(gating) - len(failed)}/{len(gating)} gating scenarios passed"
        + (f" — FAILED: {', '.join(failed)}" if failed else "")
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
