#!/usr/bin/env python
"""Two-direction low-torque probe of waist_pitch (joint 14). Audit 2026-08-17.

Discriminates:
  - HEALTHY:            follows both directions at < 2 Nm, torso visibly bows.
  - OFFSET-AT-STOP:     moves freely ONE direction, pins hard the other
                        (previous devs changed motor offsets -> zero may be
                        parked at a physical hard stop).
  - MECHANICAL JAM:     pins within ~2 deg in BOTH directions.

Safety: kp=10 kd=1 on waist_pitch ONLY (5 Nm at 0.5 rad error); every other
joint commanded at zero gains (no torque). Aborts if |tau_est| > ABORT_NM.
PRECONDITIONS (operator!): robot hanging/supported, debug mode, NO other
commander (no Teleopit session, no factory controller), covers clear of the
waist so you can SEE the mechanism.

Run (laptop): CYCLONEDDS_HOME=~/.local/cyclonedds \
  ~/miniconda3/envs/holomotion_teleop/bin/python waist_pitch_probe.py enx000ec6c3d44a
"""
import sys
import time
import csv

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

WAIST_PITCH = 14
KP, KD = 10.0, 1.0
ABORT_NM = 10.0
RATE_HZ = 50.0
CSV_PATH = "/tmp/session_monitor/waist_probe.csv"

# ramp plan: (duration_s, target_rad_at_end)
PLAN = [(3.0, 0.0), (15.0, +0.15), (3.0, +0.15), (30.0, -0.15), (3.0, -0.15), (10.0, 0.0)]

state = {"msg": None}

def on_lowstate(msg: LowState_):
    state["msg"] = msg

def main() -> None:
    iface = sys.argv[1] if len(sys.argv) > 1 else "enx000ec6c3d44a"
    print(f"probe on {iface}; ctrl-C aborts (gains zeroed on exit)")
    ChannelFactoryInitialize(0, iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_lowstate, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    crc = CRC()

    t_wait = time.time()
    while state["msg"] is None:
        if time.time() - t_wait > 5.0:
            sys.exit("no lowstate after 5 s — is the robot on and the cable in?")
        time.sleep(0.05)

    q0 = state["msg"].motor_state[WAIST_PITCH].q
    print(f"start q[waist_pitch] = {q0:+.4f} rad — probing +/-0.15 rad around ZERO")

    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = state["msg"].mode_machine
    for i in range(35):
        cmd.motor_cmd[i].mode = 1
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    rows = []
    aborted = None
    over_since = None
    try:
        target = 0.0
        for seg_dur, seg_end in PLAN:
            seg_start_target = target
            steps = max(1, int(seg_dur * RATE_HZ))
            for k in range(steps):
                target = seg_start_target + (seg_end - seg_start_target) * (k + 1) / steps
                m = state["msg"].motor_state[WAIST_PITCH]
                cmd.motor_cmd[WAIST_PITCH].q = target
                cmd.motor_cmd[WAIST_PITCH].kp = KP
                cmd.motor_cmd[WAIST_PITCH].kd = KD
                cmd.crc = crc.Crc(cmd)
                pub.Write(cmd)
                rows.append((time.time(), target, m.q, m.dq, m.tau_est,
                             m.temperature[0] if hasattr(m, "temperature") else -1))
                if abs(m.tau_est) > ABORT_NM:
                    if over_since is None:
                        over_since = time.time()
                    elif time.time() - over_since > 0.2:
                        aborted = f"ABORT: |tau|={m.tau_est:.1f} Nm at target {target:+.3f}, q={m.q:+.4f}"
                        raise KeyboardInterrupt
                else:
                    over_since = None
                if len(rows) % 25 == 0:
                    print(f"  target {target:+.3f}  q {m.q:+.4f}  tau {m.tau_est:+.1f} Nm")
                time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        # release: zero gains, several times for safety
        cmd.motor_cmd[WAIST_PITCH].kp = 0.0
        cmd.motor_cmd[WAIST_PITCH].kd = 0.0
        for _ in range(10):
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
            time.sleep(0.02)
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "target", "q", "dq", "tau_est", "temp"])
            w.writerows(rows)
        if aborted:
            print(aborted)
        qs = [r[2] for r in rows]
        print(f"released. q range visited: [{min(qs):+.4f}, {max(qs):+.4f}] rad "
              f"({(max(qs)-min(qs))*57.3:.1f} deg). log: {CSV_PATH}")

if __name__ == "__main__":
    main()
