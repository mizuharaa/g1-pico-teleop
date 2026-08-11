"""SimPort — ROS-free stand-in for HumanoidPolicyNode (policy_node_29dof.py).

Instantiates the REAL PolicyRuntime + PolicyObservationEvaluator +
PolicyObsBuilder + VrReference + deployed ONNX checkpoints against a fake
port, mirroring policy_node_29dof.py initialization step by step (line refs
in comments). The goal: the software chain under test in sim is byte-for-byte
the code that runs on the Orin — only ROS transport and the motor bus are
faked.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
HOLOMOTION = REPO / "HoloMotion"
DEPLOY_SRC = HOLOMOTION / "deployment" / "unitree_g1_ros2_29dof" / "src"
SHARE = (
    REPO
    / "artifacts/extracted/opt/holomotion/deployment/unitree_g1_ros2_29dof"
    / "install/humanoid_control/share/humanoid_control"
)
for p in (str(DEPLOY_SRC), str(HOLOMOTION)):
    if p not in sys.path:
        sys.path.insert(0, p)

from omegaconf import OmegaConf  # noqa: E402

from humanoid_policy.observation_evaluator import PolicyObservationEvaluator  # noqa: E402
from humanoid_policy.obs_builder import PolicyObsBuilder  # noqa: E402
from humanoid_policy.onnx_policy import (  # noqa: E402
    load_dual_policy_bundle,
    read_onnx_metadata,
)
from humanoid_policy.policy_runtime import PolicyRuntime  # noqa: E402
from humanoid_policy.utils.remote_controller_filter import RemoteController  # noqa: E402
from humanoid_policy.vr_reference import VrReference  # noqa: E402


class _Logger:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.records: list[tuple[str, str]] = []

    def _log(self, level, msg):
        self.records.append((level, str(msg)))
        # WARN/ERROR always reach the console — a silently refused B-press
        # is indistinguishable from "broken" (2026-08-11 audit finding)
        if self.verbose or level in ("WARN", "ERROR"):
            print(f"[{level}] {msg}", flush=True)

    def info(self, m):
        self._log("INFO", m)

    def warn(self, m):
        self._log("WARN", m)

    def error(self, m):
        self._log("ERROR", m)

    def debug(self, m):
        self._log("DEBUG", m)


class _FakeQueueStats:
    def get_queue_stats(self):
        return {"queue_size": 0, "avg_interval": 0.02, "arrival_freq": 50.0}


def _runtime_state_property(name, default):
    def getter(self):
        return getattr(self.runtime.state, name, default)

    def setter(self, value):
        setattr(self.runtime.state, name, value)

    return property(getter, setter)


class SimPort:
    """Duck-typed policy node. See spec §7 for the construction order."""

    def __init__(self, *, enable_teleop_reference: bool, verbose=False,
                 max_data_age: float = 1.5):
        self.logger = _Logger(verbose)
        # --- plain config surface (node :141-204, profile orin_docker.yaml)
        self.dt = 0.02                       # 1/policy_freq (yaml policy_freq: 50)
        self.max_data_age = max_data_age     # profile :50
        self.enable_teleop_reference = enable_teleop_reference
        self.reference_source = "zmq"        # keeps local-retarget/guard out of path
        self.zmq_jitter_delay_frames = 0     # profile
        self.motion_rope_reset_margin = 64
        self.use_kv_cache = False
        self.motion_kv_cache = None
        self.reference_qpos_dim = 36
        self._reference_buffer = _FakeQueueStats()
        self._ros_reference_buffer = None
        self._npz_replay_frame_index = None
        self._lowstate_msg = None
        self.remote_controller = RemoteController()
        self.gpu_motion_obs_builder = None
        self.all_motion_data = []
        self.motion_file_names = []
        self.n_motion_frames = 0
        self.ref_dof_pos = None
        self.published_targets: list[np.ndarray] = []
        self.published_target = None

        # --- model configs (node :671, :695)
        self.velocity_config = OmegaConf.load(
            SHARE / "models/velocity_tracking_model/config.yaml"
        )
        self.motion_config = OmegaConf.load(
            SHARE / "models/HoloMotion_motion_tracking_model_v1.4.0/config.yaml"
        )

        # --- robot naming (node :789-791)
        self.actions_dim = int(self.velocity_config.robot.actions_dim)
        self.real_dof_names = list(self.velocity_config.robot.dof_names)
        self.dof_names_ref_motion = list(self.velocity_config.robot.dof_names)
        self.num_actions = len(self.dof_names_ref_motion)
        self.n_fut_frames = int(self.motion_config.obs.n_fut_frames)

        # --- runtime + evaluator (node :129, :259)
        self.runtime = PolicyRuntime(self, num_actions=self.num_actions)
        self.observation_evaluator = PolicyObservationEvaluator(self)
        self.actor_place_holder_ndim = (
            self.observation_evaluator._find_actor_place_holder_ndim()
        )

        # --- ONNX sessions (node :560-641 via load_dual_policy_bundle)
        bundle = load_dual_policy_bundle(
            package_share_dir=str(SHARE),
            velocity_model_folder="velocity_tracking_model",
            motion_model_folder="HoloMotion_motion_tracking_model_v1.4.0",
            intra_op_threads=2,
            motion_max_context_len=int(
                self.motion_config.algo.config.num_steps_per_env
            ),
            inference_backend="onnx",
            providers=["CPUExecutionProvider"],  # laptop has no GPU
        )
        self.velocity_policy_session = bundle.velocity_session
        self.motion_policy_session = bundle.motion_session
        self.velocity_onnx_path = bundle.velocity_onnx_path
        self.motion_onnx_path = bundle.motion_onnx_path
        self.velocity_input_name = bundle.velocity_input_name
        self.velocity_output_name = bundle.velocity_output_name
        io = bundle.motion_io
        self.motion_input_name = io.input_name
        self.motion_output_name = io.output_name
        self.motion_kv_input_name = io.kv_input_name
        self.motion_kv_output_name = io.kv_output_name
        self.motion_kv_shape = io.kv_shape
        self.motion_kv_dtype = io.kv_dtype
        self.motion_step_idx_input_name = io.step_idx_input_name
        self.use_kv_cache = bundle.motion_kv_cache.enabled
        self.motion_kv_cache = bundle.motion_kv_cache.cache

        # --- ONNX metadata (node :1167-1225)
        vm = read_onnx_metadata(self.velocity_onnx_path)
        mm = read_onnx_metadata(self.motion_onnx_path)
        self.velocity_dof_names_onnx = vm["joint_names"]
        self.velocity_action_scale_onnx = vm["action_scale"]
        self.velocity_kps_onnx = vm["kps"].astype(np.float32)
        self.velocity_kds_onnx = vm["kds"].astype(np.float32)
        self.velocity_default_angles_onnx = vm["default_joint_pos"].astype(
            np.float32
        )
        self.motion_dof_names_onnx = mm["joint_names"]
        self.motion_action_scale_onnx = mm["action_scale"]
        self.motion_kps_onnx = mm["kps"].astype(np.float32)
        self.motion_kds_onnx = mm["kds"].astype(np.float32)
        self.motion_default_angles_onnx = mm["default_joint_pos"].astype(
            np.float32
        )
        self.motion_rope_max_seq_len = int(mm.get("rope_max_seq_len", 0))

        # --- obs builders (node :483-500)
        self.velocity_obs_builder = PolicyObsBuilder(
            dof_names_onnx=self.velocity_dof_names_onnx,
            default_angles_onnx=self.velocity_default_angles_onnx,
            evaluator=self.observation_evaluator,
            obs_policy_cfg=self.observation_evaluator._get_policy_atomic_obs_list(
                self.velocity_config
            ),
        )
        self.motion_obs_builder = PolicyObsBuilder(
            dof_names_onnx=self.motion_dof_names_onnx,
            default_angles_onnx=self.motion_default_angles_onnx,
            evaluator=self.observation_evaluator,
            obs_policy_cfg=self.observation_evaluator._get_policy_atomic_obs_list(
                self.motion_config
            ),
        )
        self._vr_reference = VrReference(
            n_fut_frames=self.n_fut_frames,
            num_actions=self.num_actions,
            expected_dim=self.reference_qpos_dim,
        )
        self.observation_evaluator.initialize_vr_reference_buffers(
            self.n_fut_frames, self.num_actions
        )
        self.obs_builder = self.velocity_obs_builder

        # --- dof mappings + obs state (node :1227-1259)
        self.velocity_onnx_to_real = [
            self.velocity_dof_names_onnx.index(n) for n in self.real_dof_names
        ]
        self.motion_onnx_to_real = [
            self.motion_dof_names_onnx.index(n) for n in self.real_dof_names
        ]
        self.velocity_kps_real = self.velocity_kps_onnx[
            self.velocity_onnx_to_real
        ].astype(np.float32)
        self.velocity_kds_real = self.velocity_kds_onnx[
            self.velocity_onnx_to_real
        ].astype(np.float32)
        self.motion_kps_real = self.motion_kps_onnx[
            self.motion_onnx_to_real
        ].astype(np.float32)
        self.motion_kds_real = self.motion_kds_onnx[
            self.motion_onnx_to_real
        ].astype(np.float32)
        self.observation_evaluator.initialize_observation_state()
        self.observation_evaluator._init_keybody_indices_cache()
        self._setup_completed = True

        # hooks the FSM sim reads after each policy step
        self.control_param_publishes = 0
        self.last_kps_real = None
        self.last_kds_real = None
        self._publish_control_params()

    # ---------------------------------------------------------------- node API
    def get_logger(self):
        return self.logger

    def _timing_ms(self, t0):
        return (time.perf_counter() - t0) * 1000.0

    def _publish_control_params(self):
        # mirrors node :1261-1291: publish per-mode kps/kds in REAL order
        self.control_param_publishes += 1
        if self.runtime.state.current_policy_mode == "motion":
            self.last_kps_real = self.motion_kps_real
            self.last_kds_real = self.motion_kds_real
        else:
            self.last_kps_real = self.velocity_kps_real
            self.last_kds_real = self.velocity_kds_real

    def _publish_policy_mode(self, force=False):
        pass

    def _publish_action_target(self, target):
        self.published_target = np.asarray(target, dtype=np.float32).copy()
        self.published_targets.append(self.published_target)

    def _publish_reference(self):
        pass

    def _flush_reference_telemetry(self, **kwargs):
        pass

    def _record_timing_sample(self, sample):
        pass

    def _poll_zmq_reference(self):
        pass

    def _poll_local_retarget_reference(self):
        pass

    def _store_vr_reference(self, arr, *, sample_time=None):
        self._vr_reference.store(
            arr, current_time=time.time(), sample_time=sample_time
        )

    def _load_current_motion(self):
        pass

    def _is_vr_ready_for_motion(self):
        return self._vr_reference.is_ready_for_motion(
            enable_teleop_reference=self.enable_teleop_reference,
            delay_frames=self.zmq_jitter_delay_frames,
        )


# runtime-state-backed properties (node :284-475)
for _name, _default in [
    ("policy_enabled", False),
    ("robot_state_ready", False),
    ("current_policy_mode", "velocity"),
    ("reference_stream_active", False),
    ("motion_frame_idx", 0),
    ("motion_step_idx", 0),
    ("motion_in_progress", False),
    ("vx", 0.0),
    ("vy", 0.0),
    ("vyaw", 0.0),
]:
    setattr(SimPort, _name, _runtime_state_property(_name, _default))


def _actions_onnx_get(self):
    return self.runtime.state.actions_onnx


def _target_real_get(self):
    return self.runtime.state.target_dof_pos_real


SimPort.actions_onnx = property(_actions_onnx_get)
SimPort.target_dof_pos_real = property(_target_real_get)
