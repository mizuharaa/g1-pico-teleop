"""MuJoCo stand-in for the physical G1 + its motor firmware.

Mimics exactly what the deployed stack sees/commands:
- consumes LowCmd-style per-motor (mode, q, dq, kp, kd, tau) at 500 Hz and
  applies the firmware PD law  tau = tau_ff + kp*(q_des-q) + kd*(dq_des-dq),
  clamped by the actuator torque range in the MJCF (same values as the real
  motor limits);
- produces a duck-typed LowState (imu_state.quaternion wxyz,
  imu_state.gyroscope body-frame, motor_state[i].q/.dq in motor index order
  == complete_dof_order) and a wireless_remote button array.

Nothing in here knows about policies or the FSM — it is "the robot".
"""
from __future__ import annotations

import numpy as np
import mujoco

HOLOMOTION_ROOT = None  # resolved by callers via repo layout

CONTROL_HZ = 500.0
CONTROL_DT = 1.0 / CONTROL_HZ


class _MotorState:
    __slots__ = ("q", "dq", "tau_est")

    def __init__(self):
        self.q = 0.0
        self.dq = 0.0
        self.tau_est = 0.0


class _MotorCmd:
    __slots__ = ("mode", "q", "dq", "kp", "kd", "tau")

    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.tau = 0.0


class _ImuState:
    __slots__ = ("quaternion", "gyroscope")

    def __init__(self):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.gyroscope = np.zeros(3, dtype=np.float32)


class FakeLowState:
    """Duck-type of unitree_hg LowState as read by the deployed code."""

    def __init__(self, num_motors: int):
        self.imu_state = _ImuState()
        self.motor_state = [_MotorState() for _ in range(num_motors)]
        self.wireless_remote = b"\x00" * 40
        self.mode_machine = 1


class G1MujocoRobot:
    def __init__(self, scene_xml: str, num_motors: int = 29):
        self.model = mujoco.MjModel.from_xml_path(str(scene_xml))
        self.data = mujoco.MjData(self.model)
        self.num_motors = num_motors
        assert self.model.nu == num_motors, (
            f"expected {num_motors} actuators, got {self.model.nu}"
        )
        # free joint occupies qpos[0:7] (xyz + wxyz quat), qvel[0:6]
        self._q_adr = np.array(
            [self.model.jnt_qposadr[j] for j in range(1, self.model.njnt)]
        )
        self._v_adr = np.array(
            [self.model.jnt_dofadr[j] for j in range(1, self.model.njnt)]
        )
        self.cmd = [_MotorCmd() for _ in range(num_motors)]
        self.lowstate = FakeLowState(num_motors)
        self._sim_time = 0.0
        assert abs(self.model.opt.timestep - CONTROL_DT) < 1e-9, (
            "MJCF timestep must equal the 500 Hz control_dt for 1:1 stepping"
        )

    # ------------------------------------------------------------- state io
    def set_pose(
        self,
        joint_pos: np.ndarray,
        base_pos=(0.0, 0.0, 0.793),
        base_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
    ) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[0:3] = base_pos
        self.data.qpos[3:7] = base_quat_wxyz
        self.data.qpos[self._q_adr] = joint_pos
        mujoco.mj_forward(self.model, self.data)
        self._refresh_lowstate()

    def settle(self, joint_pos: np.ndarray, kp=150.0, kd=5.0, seconds=1.5):
        """Hold a pose with a stiff external PD until transients die out.

        Used to establish the initial condition (e.g. 'operator lowered the
        robot to standing default before pressing Start/A') without claiming
        the deployed stack did it.
        """
        for _ in range(int(seconds * CONTROL_HZ)):
            q = self.data.qpos[self._q_adr]
            dq = self.data.qvel[self._v_adr]
            tau = kp * (joint_pos - q) - kd * dq
            self._apply_tau(tau)
            mujoco.mj_step(self.model, self.data)
        self._refresh_lowstate()

    @property
    def base_height(self) -> float:
        return float(self.data.qpos[2])

    @property
    def base_rpy_tilt(self) -> float:
        """Angle (rad) between the base +z axis and world up."""
        q = self.data.qpos[3:7]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, q)
        return float(np.arccos(np.clip(R.reshape(3, 3)[2, 2], -1.0, 1.0)))

    def joint_pos(self) -> np.ndarray:
        return self.data.qpos[self._q_adr].copy()

    def joint_vel(self) -> np.ndarray:
        return self.data.qvel[self._v_adr].copy()

    # ------------------------------------------------------------- stepping
    def _apply_tau(self, tau: np.ndarray) -> None:
        lo = self.model.actuator_ctrlrange[:, 0]
        hi = self.model.actuator_ctrlrange[:, 1]
        self.data.ctrl[:] = np.clip(tau, lo, hi)

    def step(self, n: int = 1) -> None:
        """Advance physics n control ticks applying the current LowCmd."""
        for _ in range(n):
            q = self.data.qpos[self._q_adr]
            dq = self.data.qvel[self._v_adr]
            tau = np.zeros(self.num_motors)
            for i, c in enumerate(self.cmd):
                if c.mode == 0:
                    tau[i] = 0.0  # disabled motor
                else:
                    tau[i] = (
                        c.tau
                        + c.kp * (c.q - q[i])
                        + c.kd * (c.dq - dq[i])
                    )
            self._apply_tau(tau)
            mujoco.mj_step(self.model, self.data)
            self._sim_time += CONTROL_DT
        self._refresh_lowstate()

    def _refresh_lowstate(self) -> None:
        ls = self.lowstate
        ls.imu_state.quaternion = self.data.qpos[3:7].astype(np.float32).copy()
        # free-joint qvel angular part is body-frame == IMU gyro convention
        ls.imu_state.gyroscope = self.data.qvel[3:6].astype(np.float32).copy()
        q = self.data.qpos[self._q_adr]
        dq = self.data.qvel[self._v_adr]
        frc = self.data.actuator_force
        for i in range(self.num_motors):
            m = ls.motor_state[i]
            m.q = float(q[i])
            m.dq = float(dq[i])
            m.tau_est = float(frc[i])

    @property
    def time(self) -> float:
        return self._sim_time
