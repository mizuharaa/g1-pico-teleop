#!/usr/bin/env python
"""G1 joint & actuator watchdog.

Monitors the robot's low-level state stream (DDS topic ``rt/lowstate``,
``unitree_hg.msg.dds_.LowState_``) and raises WARN/ALARM when any servo
leaves its safe envelope. Everything is logged to CSV so sessions are
recordable and post-mortems have data.

Monitored per servo (proper LowState field names):
  tau_est      joint torque estimate [Nm]      -> % of actuator rating
  temperature  motor winding/housing temp [degC]
  vol          bus voltage at the drive [V]
  motorstate   servo mode/fault word            -> any change is reported
  dq           joint velocity [rad/s]           (logged, no threshold)
Stream-level:
  staleness    gap since last LowState message  -> comms/e-stop indicator

Usage:
  python joint_watchdog.py --iface eth0            # on the robot / over Ethernet DDS
  python joint_watchdog.py --dry-run               # synthetic data, no robot: test thresholds + CSV
  python joint_watchdog.py --iface eth0 --csv /tmp/session1.csv

IMPORTANT: this is an OBSERVER. It prints and logs; it does not and cannot
stop the robot. The spotter with the Unitree remote remains the failsafe.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Actuator torque ratings [Nm], extracted from g1_mocap_29dof.xml
# actuatorfrcrange (matches Unitree's 29-DoF EDU spec sheet classes).
# Order = UNITREE_G1_29DOF_NAMES = LowState.motor_state[0:29] order.
# ---------------------------------------------------------------------------
JOINTS: list[tuple[str, float]] = [
    ("left_hip_pitch", 88.0), ("left_hip_roll", 139.0), ("left_hip_yaw", 88.0),
    ("left_knee", 139.0), ("left_ankle_pitch", 50.0), ("left_ankle_roll", 50.0),
    ("right_hip_pitch", 88.0), ("right_hip_roll", 139.0), ("right_hip_yaw", 88.0),
    ("right_knee", 139.0), ("right_ankle_pitch", 50.0), ("right_ankle_roll", 50.0),
    ("waist_yaw", 88.0), ("waist_roll", 50.0), ("waist_pitch", 50.0),
    ("left_shoulder_pitch", 25.0), ("left_shoulder_roll", 25.0),
    ("left_shoulder_yaw", 25.0), ("left_elbow", 25.0),
    ("left_wrist_roll", 25.0), ("left_wrist_pitch", 5.0), ("left_wrist_yaw", 5.0),
    ("right_shoulder_pitch", 25.0), ("right_shoulder_roll", 25.0),
    ("right_shoulder_yaw", 25.0), ("right_elbow", 25.0),
    ("right_wrist_roll", 25.0), ("right_wrist_pitch", 5.0), ("right_wrist_yaw", 5.0),
]
N_JOINTS = len(JOINTS)

# Thresholds ----------------------------------------------------------------
TORQUE_WARN_FRAC = 0.80    # sustained above 80% of rating -> WARN
TORQUE_ALARM_FRAC = 0.95   # above 95% of rating           -> ALARM
TEMP_WARN_C = 70.0         # winding temp WARN
TEMP_ALARM_C = 85.0        # winding temp ALARM (drives derate/cut ~90+)
VOLT_LOW_V = 44.0          # sagging battery under load    -> WARN
VOLT_HIGH_V = 60.0         # implausible/regen spike       -> WARN
STALE_WARN_S = 0.10        # no LowState for 100 ms        -> WARN
STALE_ALARM_S = 0.50       # no LowState for 500 ms        -> ALARM
TORQUE_SUSTAIN_S = 0.5     # torque must stay high this long to fire (filters spikes)


@dataclass
class JointAlarmState:
    torque_high_since: float | None = None
    last_motorstate: int | None = None
    worst_level: int = 0  # 0 ok, 1 warn, 2 alarm


EVENT_REPEAT_S = 3.0       # identical persisting condition re-reports at most this often
ALARM_ACTION_COOLDOWN_S = 10.0  # on-alarm damp re-fires this often while an alarm persists


class Watchdog:
    def __init__(self, csv_path: str, log_every_s: float = 0.2, on_alarm: str = "") -> None:
        self.states = [JointAlarmState() for _ in range(N_JOINTS)]
        self.last_msg_time: float | None = None
        self.log_every_s = log_every_s
        self._last_csv_write = 0.0
        self._last_fire: dict[str, float] = {}  # condition key -> last report time
        self.on_alarm = on_alarm
        self._last_alarm_action_t = 0.0
        self._csv_file = open(csv_path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        header = ["t_wall", "staleness_s"]
        for name, _ in JOINTS:
            header += [f"{name}.q", f"{name}.dq", f"{name}.tau_est",
                       f"{name}.temp_c", f"{name}.vol_v", f"{name}.motorstate"]
        self._csv.writerow(header)
        self.events: list[str] = []

    # -- reporting ----------------------------------------------------------
    def _event(
        self,
        level: str,
        msg: str,
        key: str | None = None,
        actionable: bool = True,
    ) -> None:
        """Report an event; a persisting condition (same key) repeats at most
        every EVENT_REPEAT_S so the console stays readable at 50 Hz.

        actionable=False (stream stalls): warn loudly but never fire the
        on-alarm damp — a stalled link means the damp command cannot reach
        the robot anyway, and it must not spend the action (2026-08-10: a
        stall false-fire at 12:11 left the REAL thermal emergency at 14:27,
        waist 130 C, with no auto-damp).
        """
        if key is not None:
            now = time.time()
            if now - self._last_fire.get(key, 0.0) < EVENT_REPEAT_S:
                return
            self._last_fire[key] = now
        line = f"[{time.strftime('%H:%M:%S')}] {level:5s} {msg}"
        print(line, flush=True)
        self.events.append(line)
        # Re-arming action: fires again every ALARM_ACTION_COOLDOWN_S while
        # an actionable ALARM persists (was single-shot; see docstring).
        if level == "ALARM" and self.on_alarm and actionable:
            now = time.time()
            if now - self._last_alarm_action_t >= ALARM_ACTION_COOLDOWN_S:
                self._last_alarm_action_t = now
                print(f"[watchdog] ALARM -> executing: {self.on_alarm}", flush=True)
                import subprocess
                subprocess.Popen(self.on_alarm, shell=True)

    # -- one LowState sample -------------------------------------------------
    def process(self, now: float, q, dq, tau_est, temp_c, vol_v, motorstate) -> None:
        # staleness
        if self.last_msg_time is not None:
            gap = now - self.last_msg_time
            if gap > STALE_ALARM_S:
                self._event("ALARM", f"LowState stream stalled {gap*1000:.0f} ms — comms loss or e-stop",
                            key="staleness_alarm", actionable=False)
            elif gap > STALE_WARN_S:
                self._event("WARN", f"LowState gap {gap*1000:.0f} ms", key="staleness_warn")
        self.last_msg_time = now

        for i, (name, rating) in enumerate(JOINTS):
            st = self.states[i]
            # torque envelope (sustained)
            frac = abs(tau_est[i]) / rating
            if frac >= TORQUE_WARN_FRAC:
                if st.torque_high_since is None:
                    st.torque_high_since = now
                elif now - st.torque_high_since >= TORQUE_SUSTAIN_S:
                    level = "ALARM" if frac >= TORQUE_ALARM_FRAC else "WARN"
                    self._event(level, f"{name}: tau_est {tau_est[i]:+.1f} Nm = "
                                       f"{frac*100:.0f}% of {rating:.0f} Nm rating (sustained)")
                    st.torque_high_since = now  # rate-limit repeats
            else:
                st.torque_high_since = None
            # winding temperature
            if temp_c[i] >= TEMP_ALARM_C:
                self._event("ALARM", f"{name}: winding temp {temp_c[i]:.0f} C >= {TEMP_ALARM_C:.0f} C — stop and cool",
                            key=f"temp_alarm.{name}")
            elif temp_c[i] >= TEMP_WARN_C:
                self._event("WARN", f"{name}: winding temp {temp_c[i]:.0f} C", key=f"temp_warn.{name}")
            # bus voltage
            if vol_v[i] and (vol_v[i] < VOLT_LOW_V or vol_v[i] > VOLT_HIGH_V):
                self._event("WARN", f"{name}: bus voltage {vol_v[i]:.1f} V outside [{VOLT_LOW_V},{VOLT_HIGH_V}]",
                            key=f"vol.{name}")
            # servo mode/fault word transitions
            ms = int(motorstate[i])
            if st.last_motorstate is not None and ms != st.last_motorstate:
                self._event("WARN", f"{name}: motorstate changed {st.last_motorstate} -> {ms}")
            st.last_motorstate = ms

        # periodic CSV row
        if now - self._last_csv_write >= self.log_every_s:
            self._last_csv_write = now
            row = [f"{now:.3f}", ""]
            for i in range(N_JOINTS):
                row += [f"{q[i]:.4f}", f"{dq[i]:.3f}", f"{tau_est[i]:.2f}",
                        f"{temp_c[i]:.0f}", f"{vol_v[i]:.1f}", int(motorstate[i])]
            self._csv.writerow(row)

    def close(self) -> None:
        self._csv_file.flush()
        self._csv_file.close()


# ---------------------------------------------------------------------------
def run_dds(iface: str, wd: Watchdog) -> None:
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError:
        sys.exit("unitree_sdk2py not installed. On the robot/laptop: pip install unitree_sdk2py\n"
                 "(or run with --dry-run to test without a robot)")

    ChannelFactoryInitialize(0, iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    print(f"watchdog: subscribed to rt/lowstate on {iface}; Ctrl+C to stop")

    while True:
        msg = sub.Read(int(STALE_ALARM_S * 1000))  # ms timeout
        now = time.time()
        if msg is None:
            wd.process(now, [0.0] * N_JOINTS, [0.0] * N_JOINTS, [0.0] * N_JOINTS,
                       [0.0] * N_JOINTS, [0.0] * N_JOINTS, [0] * N_JOINTS)
            continue
        ms = msg.motor_state
        q = [ms[i].q for i in range(N_JOINTS)]
        dq = [ms[i].dq for i in range(N_JOINTS)]
        tau = [ms[i].tau_est for i in range(N_JOINTS)]
        # temperature is a 2-element array on hg MotorState (two sensors); take max
        temp = [max(ms[i].temperature) if hasattr(ms[i].temperature, "__len__")
                else float(ms[i].temperature) for i in range(N_JOINTS)]
        vol = [float(getattr(ms[i], "vol", 0.0)) for i in range(N_JOINTS)]
        mstate = [int(getattr(ms[i], "motorstate", 0)) for i in range(N_JOINTS)]
        wd.process(now, q, dq, tau, temp, vol, mstate)


def run_dry(wd: Watchdog, seconds: float = 10.0) -> None:
    """Synthetic exercise of every alarm path (no robot)."""
    import math
    print("watchdog: DRY RUN — synthetic faults will fire on purpose")
    t0 = time.time()
    step = 0
    while time.time() - t0 < seconds:
        now = time.time()
        t = now - t0
        q = [0.3 * math.sin(t + i) for i in range(N_JOINTS)]
        dq = [0.3 * math.cos(t + i) for i in range(N_JOINTS)]
        tau = [0.10 * rating for _, rating in JOINTS]  # nominal load: 10% of rating
        temp = [45.0] * N_JOINTS
        vol = [52.0] * N_JOINTS
        mstate = [1] * N_JOINTS
        if 2.0 < t < 4.0:
            tau[3] = 135.0        # left_knee near its 139 Nm rating (sustained)
        if 5.0 < t < 6.0:
            temp[16] = 88.0       # left_shoulder_roll overtemp ALARM
        if 7.0 < t < 7.5:
            vol[0] = 41.0         # sagging bus voltage
        if abs(t - 8.0) < 0.05:
            mstate[10] = 9        # right_knee fault-word change
        wd.process(now, q, dq, tau, temp, vol, mstate)
        step += 1
        time.sleep(0.02)          # ~50 Hz, matches real LowState rate
    print(f"dry run done: {step} samples, {len(wd.events)} events")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="eth0", help="network interface for DDS (robot: eth0)")
    ap.add_argument("--csv", default="watchdog_session.csv", help="CSV log path")
    ap.add_argument("--dry-run", action="store_true", help="synthetic data, no robot needed")
    ap.add_argument("--on-alarm", default="",
                    help="shell command to run on the FIRST ALARM (e.g. "
                         "'python g1_estop.py --iface eth0 --now'). Fires once per session.")
    args = ap.parse_args()

    wd = Watchdog(args.csv, on_alarm=args.on_alarm)
    try:
        if args.dry_run:
            run_dry(wd)
        else:
            run_dds(args.iface, wd)
    except KeyboardInterrupt:
        pass
    finally:
        wd.close()
        print(f"CSV log: {args.csv}")


if __name__ == "__main__":
    main()
