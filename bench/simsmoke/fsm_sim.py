"""Faithful Python port of main_node.cpp (the on-robot FSM/control node),
including the 2026-08-11 local L1 patch. Line references are to
deployment/unitree_g1_ros2_29dof/src/src/main_node.cpp.

Behavioral fidelity notes (verified against source):
- states ZERO_TORQUE / MOVE_TO_DEFAULT / POLICY / EMERGENCY_STOP (l.31)
- Select -> EMERGENCY_STOP: 2.0 s damping kd=10 then motors DISABLED and the
  node shuts down (l.329-357, 622-651)
- bare L1 -> damped EMERGENCY_STOP; L1+L2 -> instant ZERO_TORQUE (local patch)
- Start only in ZERO_TORQUE (l.368); A only in MOVE_TO_DEFAULT with a
  0.4 rad lower-body deviation gate (l.382-434) and kps/kds must have been
  received from the policy node
- MOVE_TO_DEFAULT: per-tick target = (1-ratio)*CURRENT q + ratio*default
  over duration_=3 s, MoveToDefault kp/kd, torque-limited (l.486-568)
- POLICY: targets from the policy node, clamped to scaled position limits in
  the action handler (l.712-740); NO torque limiting (l.612)
- upstream joint-limit e-stop in LowStateHandler is STUBBED OUT (l.679-690):
  limits_exceeded is never set. We replicate that (i.e. no such e-stop).
"""
from __future__ import annotations

from enum import Enum

import numpy as np
import yaml


class KeyMap:
    R1, L1, start, select, R2, L2, F1, F2 = range(8)
    A, B, X, Y, up, right, down, left = range(8, 16)


class RobotState(Enum):
    ZERO_TORQUE = 0
    MOVE_TO_DEFAULT = 1
    EMERGENCY_STOP = 2
    POLICY = 3


class EmergencyStopPhase(Enum):
    DAMPING = 0
    DISABLE = 1


class MainNodeSim:
    def __init__(self, config_path: str, robot, l1_patch: bool = True):
        """robot: G1MujocoRobot (provides .cmd list and .lowstate)."""
        self.robot = robot
        self.l1_patch = l1_patch
        cfg = yaml.safe_load(open(config_path))
        self.dof2motor_idx = dict(cfg["dof2motor_idx_mapping"])
        self.default_dof_pos = {
            k: float(v) for k, v in cfg["default_joint_angles"].items()
        }
        self.target_dof_pos = dict(self.default_dof_pos)
        self.complete_dof_order = list(cfg["complete_dof_order"])
        # the A-gate below indexes motor[] by list position (faithful to the
        # cpp); that is only correct while the mapping is the identity
        assert all(
            self.dof2motor_idx[n] == i
            for i, n in enumerate(self.complete_dof_order)
        ), "dof2motor_idx is not identity — positional motor indexing invalid"
        self.policy_dof_order = list(cfg["policy_dof_order"])
        self.control_freq = float(cfg["control_freq"])
        self.control_dt = 1.0 / self.control_freq
        jl = cfg["joint_limits"]
        self.joint_position_limits = {
            k: (float(v[0]), float(v[1])) for k, v in jl["position"].items()
        }
        self.joint_effort_limits = {
            k: float(v) for k, v in jl["effort"].items()
        }
        scales = cfg.get("limit_scales", {}) or {}
        self.position_limit_scale = float(scales.get("position", 1.0))
        self.effort_limit_scale = float(scales.get("effort", 1.0))

        # optional Start-behavior arrays (l.258-318)
        self.has_joint_arrays = False
        self.move_to_default_kps: dict[str, float] = {}
        self.move_to_default_kds: dict[str, float] = {}
        self.joint_names_array: list[str] = []
        self.default_position_array: list[float] = []
        if cfg.get("kp") and cfg.get("kd") and cfg.get("joint_names") and cfg.get("default_position"):
            names = list(cfg["joint_names"])
            dpos = [float(x) for x in cfg["default_position"]]
            kp = [float(x) for x in cfg["kp"]]
            kd = [float(x) for x in cfg["kd"]]
            if len(names) == len(dpos) == len(kp) == len(kd):
                self.has_joint_arrays = True
                self.joint_names_array = names
                self.default_position_array = dpos
                for n, p, a, b in zip(names, dpos, kp, kd):
                    self.move_to_default_kps[n] = a
                    self.move_to_default_kds[n] = b
                    self.default_dof_pos[n] = p

        self.kps: dict[str, float] = {}
        self.kds: dict[str, float] = {}
        self.kps_received = False
        self.kds_received = False

        self.current_state = RobotState.ZERO_TORQUE
        self.should_shutdown = False
        self.shutdown_done = False
        self.emergency_stop_phase = EmergencyStopPhase.DAMPING
        self.emergency_stop_time = 0.0
        self.emergency_damping_duration = 2.0
        self.time_ = 0.0
        self.duration_ = 3.0
        self.buttons = [0] * 16
        self.log: list[str] = []

    # ------------------------------------------------- "topic" handlers
    def kps_handler(self, data):
        data = list(data)
        self.kps_received = True
        if len(data) != len(self.policy_dof_order):
            self._estop("kps size mismatch")
            return
        for i, n in enumerate(self.policy_dof_order):
            self.kps[n] = float(data[i])

    def kds_handler(self, data):
        data = list(data)
        self.kds_received = True
        if len(data) != len(self.policy_dof_order):
            self._estop("kds size mismatch")
            return
        for i, n in enumerate(self.policy_dof_order):
            self.kds[n] = float(data[i])

    def policy_action_handler(self, data):
        data = list(data)
        if len(data) != len(self.policy_dof_order):
            self._estop("action size mismatch")
            return
        for i, n in enumerate(self.policy_dof_order):
            pos = float(data[i])
            if n in self.joint_position_limits:
                lo, hi = self.joint_position_limits[n]
                mid = (lo + hi) / 2.0
                half = (hi - lo) / 2.0 * self.position_limit_scale
                pos = float(np.clip(pos, mid - half, mid + half))
            self.target_dof_pos[n] = pos

    def _estop(self, why: str):
        self.log.append(f"EMERGENCY_STOP: {why}")
        self.current_state = RobotState.EMERGENCY_STOP
        self.should_shutdown = True

    # ------------------------------------------------- torque limiting
    def _limit_custom(self, name, q_des, q, dq, kp, kd):
        expected = kp * (q_des - q) + kd * (0.0 - dq)
        a = abs(expected)
        if name in self.joint_effort_limits:
            mx = self.joint_effort_limits[name] * self.effort_limit_scale
            if a > mx and a > 1e-6:
                s = mx / a
                return kp * s, kd * s
        return kp, kd

    # ------------------------------------------------- control tick (500 Hz)
    def control(self):
        """One Control() tick. Reads robot.lowstate, writes robot.cmd."""
        if self.shutdown_done:
            return
        motor = self.robot.lowstate.motor_state
        cmd = self.robot.cmd

        if self.current_state == RobotState.EMERGENCY_STOP:
            self.emergency_stop_time += self.control_dt
            if self.emergency_stop_phase == EmergencyStopPhase.DAMPING:
                for i in range(len(cmd)):
                    c = cmd[i]
                    c.mode = 1
                    c.q = motor[i].q
                    c.dq = 0.0
                    c.kp = 0.0
                    c.kd = 10.0
                    c.tau = 0.0
                if self.emergency_stop_time >= self.emergency_damping_duration:
                    self.emergency_stop_phase = EmergencyStopPhase.DISABLE
                    self.log.append("Damping complete, disabling motors")
            else:
                for c in cmd:
                    c.mode = 0
                    c.q = c.dq = c.kp = c.kd = c.tau = 0.0
                self.shutdown_done = True  # rclcpp::shutdown()
            return

        b = self.buttons
        if b[KeyMap.select] == 1:
            self._estop("select pressed")
            return

        if b[KeyMap.L1] == 1 and self.current_state != RobotState.ZERO_TORQUE:
            if not self.l1_patch:
                # upstream behavior: instant free-fall
                self.log.append("L1: ZERO_TORQUE (upstream)")
                self.current_state = RobotState.ZERO_TORQUE
            elif b[KeyMap.L2] == 1:
                self.log.append("L1+L2: ZERO_TORQUE (bench chord)")
                self.current_state = RobotState.ZERO_TORQUE
            else:
                self._estop("L1 -> damped e-stop (patch)")
                return

        if b[KeyMap.start] == 1 and self.current_state == RobotState.ZERO_TORQUE:
            self.log.append("-> MOVE_TO_DEFAULT")
            self.current_state = RobotState.MOVE_TO_DEFAULT
            self.time_ = 0.0

        if b[KeyMap.A] == 1 and self.current_state == RobotState.MOVE_TO_DEFAULT:
            if self.kps_received and self.kds_received:
                lower = [
                    "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
                    "left_knee", "left_ankle_pitch", "left_ankle_roll",
                    "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
                    "right_knee", "right_ankle_pitch", "right_ankle_roll",
                ]
                ok = True
                for i, name in enumerate(self.complete_dof_order):
                    if name not in lower:
                        continue
                    if abs(motor[i].q - self.default_dof_pos[name]) > 0.4:
                        ok = False
                if ok:
                    self.log.append("-> POLICY")
                    self.current_state = RobotState.POLICY
                    self.time_ = 0.0
                else:
                    self.log.append("A refused: lower-body deviation > 0.4")
            else:
                self.log.append("A refused: kps/kds not received")

        if self.current_state == RobotState.ZERO_TORQUE:
            for c in cmd:
                c.mode = 1
                c.q = c.dq = c.kp = c.kd = c.tau = 0.0
        elif self.current_state == RobotState.MOVE_TO_DEFAULT:
            self._send_move_to_default(motor, cmd)
        elif self.current_state == RobotState.POLICY:
            self._send_policy(motor, cmd)

    def _send_move_to_default(self, motor, cmd):
        self.time_ += self.control_dt
        ratio = float(np.clip(self.time_ / self.duration_, 0.0, 1.0))
        if self.has_joint_arrays:
            names = self.joint_names_array
            finals = self.default_position_array
        else:
            names = self.complete_dof_order
            finals = [self.default_dof_pos[n] for n in names]
        for name, final in zip(names, finals):
            if name not in self.dof2motor_idx:
                continue
            i = self.dof2motor_idx[name]
            q = motor[i].q
            dq = motor[i].dq
            target = (1.0 - ratio) * q + ratio * final
            kp = self.move_to_default_kps.get(name, 50.0)
            kd = self.move_to_default_kds.get(name, 5.0)
            kp, kd = self._limit_custom(name, target, q, dq, kp, kd)
            c = cmd[i]
            c.mode = 1
            c.tau = 0.0
            c.q = target
            c.dq = 0.0
            c.kp = kp
            c.kd = kd

    def _send_policy(self, motor, cmd):
        self.time_ += self.control_dt
        if not (self.kps_received and self.kds_received):
            self._estop("policy params missing in POLICY state")
            return
        for name, target in self.target_dof_pos.items():
            if name not in self.dof2motor_idx:
                continue
            i = self.dof2motor_idx[name]
            c = cmd[i]
            c.mode = 1
            c.tau = 0.0
            c.q = float(target)
            c.dq = 0.0
            # NB: policy gains applied WITHOUT torque limiting (upstream l.612)
            c.kp = self.kps.get(name, 0.0)
            c.kd = self.kds.get(name, 0.0)
