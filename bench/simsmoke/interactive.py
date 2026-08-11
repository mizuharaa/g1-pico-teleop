"""Physics teleop sim — real deployed stack, minimal loop, no ceremony.

  - stream live -> tracking. No gates, no keys, no conditions.
  - fall -> gantry-bridged reset (teleport + 1.5 s soft hold so the policy's
    obs history re-fills with consistent data — bare teleports poisoned the
    history and caused instant re-falls on 2026-08-11), then tracking again.
  - sleep -> keeper gives a fresh window. ESC/close = quit for real.

The kinematic reference viewer (the trusted old mirror) runs alongside via
the chain supervisor — that one never falls; this one runs physics.
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_smoke import Rig, STAND_H  # noqa: E402
from fsm_sim import KeyMap  # noqa: E402


class PicoFeed:
    def __init__(self, rig, uri="tcp://127.0.0.1:6001"):
        from humanoid_policy.reference_transport import (
            ReferenceBuffer,
            ZmqReferenceSubscriber,
        )

        self.rig = rig
        self.buffer = ReferenceBuffer()
        self.sub = ZmqReferenceSubscriber(
            uri=uri, topic=b"reference_qpos", buffer=self.buffer,
            logger=rig.port.get_logger(), mode="connect",
        )
        self.sub.start()
        self._last_seq = None
        self.frames = 0
        # stride scale: amplify the operator's root translation so one human
        # step covers more robot distance. Relative to the first frame's xy
        # (absolute scaling would teleport the origin).
        # DEFAULT OFF (1.0) — 2026-08-11 review: scaling root xy alone makes
        # the reference kinematically inconsistent (pelvis moves farther than
        # the legs step) and was a confirmed fall source at the 1.2 default.
        try:
            self.stride = float(os.environ.get("HOLOSIM_STRIDE_SCALE", "1.0"))
        except ValueError:
            self.stride = 1.0
        self._origin_xy = None

    def poll(self):
        data, _ts, is_stale, _fi, _st, seq = (
            self.buffer.get_with_age_and_delay(max_age=0.5, delay_steps=0)
        )
        if data is None or is_stale or seq == self._last_seq:
            return
        self._last_seq = seq
        qpos = np.asarray(data, dtype=np.float32).reshape(-1).copy()
        if self.stride != 1.0:
            if self._origin_xy is None:
                self._origin_xy = qpos[0:2].copy()
            qpos[0:2] = self._origin_xy + self.stride * (
                qpos[0:2] - self._origin_xy
            )
        self.rig.feed_reference(qpos)
        self.frames += 1


def main():
    import mujoco.viewer

    print(__doc__, flush=True)
    rig = Rig(teleop=True, verbose=False)
    pico = PicoFeed(rig)
    quit_flag = {"q": False}

    def on_key(keycode):
        if keycode == 256:
            quit_flag["q"] = True

    rig.startup_to_policy()
    # faded release instead of rig.release_gantry()'s hard cut: ramp the
    # hold out over 1 s, then give the policy 1.5 s to settle before
    # tracking can engage — kills the launch lunge.
    rig.run(1.0)
    for _ in range(50):
        rig.gantry_gain = max(0.0, rig.gantry_gain - 0.02)
        rig.run(0.02)
    rig.gantry_on = False
    rig.gantry_gain = 1.0
    rig.robot.data.xfrc_applied[1][:] = 0.0
    rig.run(1.5)
    print("[boot] free-standing (faded release); tracking follows the stream.",
          flush=True)

    viewer = mujoco.viewer.launch_passive(
        rig.robot.model, rig.robot.data, key_callback=on_key,
    )
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 2.5, -15, 135

    gantry_release_at = None  # sim-time when a reset hold ends

    def reset_with_bridge(reason):
        nonlocal gantry_release_at
        print(f"[reset] {reason} -> gantry-bridged restart", flush=True)
        rig.port.runtime.switch_to_velocity_mode("sim reset")
        st = rig.port.runtime.state
        st.mode_blend_t0 = None
        st.mode_blend_from_real = None
        # cross-tick state must not survive a teleport: a stale slew anchor
        # drags the command from the pre-fall pose, and a stale stride origin
        # re-applies the accumulated (amplified) root offset -> instant re-fall.
        rig.port.runtime._slew_prev_target = None
        pico._origin_xy = None
        rig.robot.set_pose(rig.default_real, base_pos=(0.0, 0.0, STAND_H))
        rig.gantry_on = True
        rig.gantry_gain = 1.0
        gantry_release_at = rig.clock.now + 1.5

    HEARTBEAT = os.environ.get("HOLOSIM_HEARTBEAT", "/tmp/holosim_heartbeat")
    last_beat = 0.0
    last_wall = time.time()
    wall_next = time.perf_counter()
    CHUNK = 0.02
    beat = 0
    ready_since = None
    bad_since = None
    try:
        while viewer.is_running() and not quit_flag["q"]:
            now_wall = time.time()
            mono_gap = time.perf_counter() - wall_next
            if now_wall - last_wall > 30.0 and mono_gap < 10.0:
                print("[wake] suspend detected -> fresh window", flush=True)
                os._exit(3)
            last_wall = now_wall
            if now_wall - last_beat > 1.0:
                last_beat = now_wall
                try:
                    with open(HEARTBEAT, "w") as hb:
                        hb.write(str(now_wall))
                except OSError:
                    pass

            pico.poll()

            if gantry_release_at is not None and rig.clock.now >= gantry_release_at:
                # faded release here too (0.02 per beat over ~1 s)
                rig.gantry_gain -= 0.02
                if rig.gantry_gain <= 0.0:
                    rig.gantry_on = False
                    rig.gantry_gain = 1.0
                    rig.robot.data.xfrc_applied[1][:] = 0.0
                    gantry_release_at = None

            if (
                gantry_release_at is None
                and rig.port.policy_enabled
                and rig.port.current_policy_mode == "velocity"
                and rig.port._is_vr_ready_for_motion()
                and rig.port._vr_reference.data_age(rig.clock.now) < 0.3
            ):
                if ready_since is None:
                    ready_since = rig.clock.now
                elif rig.clock.now - ready_since > 0.5:
                    print("[track] ON", flush=True)
                    rig.press(KeyMap.B)
                    ready_since = None
            else:
                ready_since = None

            rig.run(CHUNK, reference_fn=lambda t: None)

            # fall detection with a dead-zone catcher: a half-collapsed
            # crouch-lean can sit ABOVE the hard thresholds forever while the
            # policy "reaches" for the standing reference (seen live
            # 2026-08-11). Hard fall = instant reset; lingering in the bad
            # band (too low OR too tilted) for 1.5 s = reset too. Real squats
            # are shallower/shorter and never trip it.
            h = rig.robot.base_height
            tilt = rig.robot.base_rpy_tilt
            if h < 0.35 or tilt > 1.2:
                bad_since = None
                reset_with_bridge(f"fell h={h:.2f} tilt={tilt:.2f}")
            elif h < 0.52 or tilt > 0.7:
                if bad_since is None:
                    bad_since = rig.clock.now
                elif rig.clock.now - bad_since > 1.5:
                    bad_since = None
                    reset_with_bridge(
                        f"stuck half-collapsed h={h:.2f} tilt={tilt:.2f}"
                    )
            else:
                bad_since = None

            beat += 1
            # 50 Hz render when the machine keeps up; degrade to 25 Hz
            # under load instead of stalling the control loop
            behind = time.perf_counter() > wall_next
            if beat % 2 == 0 or not behind:
                viewer.cam.lookat[:] = rig.robot.data.qpos[0:3]
                viewer.sync()
            wall_next += CHUNK
            now = time.perf_counter()
            if wall_next > now:
                time.sleep(wall_next - now)
            else:
                wall_next = now
    except KeyboardInterrupt:
        pass
    except BaseException:
        # os._exit in finally would swallow the traceback AND exit 0, making
        # every crash look like a deliberate quit (keeper then never relaunches)
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        try:
            pico.sub.stop()
        except Exception:
            pass
        os._exit(1)
    finally:
        print(f"session over. pico_frames={pico.frames}", flush=True)
        try:
            pico.sub.stop()
        except Exception:
            pass
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
