"""Candidate retarget backends for ik_bench.py.

Each backend: retarget(body_poses[24,7]) -> qpos[36].
Run:  python ik_bench.py --backend backends:warp_cpu
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_HOLOMOTION = str(Path(__file__).resolve().parent.parent / "HoloMotion")
if _HOLOMOTION not in sys.path:
    sys.path.insert(0, _HOLOMOTION)

_warp_runner = None


def _get_warp_runner():
    global _warp_runner
    if _warp_runner is None:
        from holoretarget._engine_impl import HoloRetargetRunner
        from holoretarget.config import DEFAULT_CONFIG as C

        _warp_runner = HoloRetargetRunner(
            asset_root=str(C.resolved_asset_root), robot=C.robot,
            src_human="smplx", offset_to_ground=False,
            newton_iterations=C.newton_iterations, newton_cuda_graph=False,
            newton_joint_limit_weight=C.joint_limit_weight,
            newton_max_joint_step=C.max_joint_step,
            newton_target_table=C.target_table,
            newton_robot_asset=C.robot_asset,
            newton_root_seed_mode=C.root_seed_mode,
            ground_calibration_frames=C.ground_calibration_frames,
            ground_height=C.ground_height,
        )
    return _warp_runner


def warp_cpu(body_poses: np.ndarray) -> np.ndarray:
    """HoloMotion's own Newton LM solver on Warp CPU (dormant upstream path).

    KNOWN PARITY GAP: this path applies only R_x90 to source quaternions;
    the production GPU kernel applies R_x90 (.) q (.) R_y180. Fix before
    using for real teleop or the skeleton is yaw-flipped 180 degrees.
    """
    q = _get_warp_runner().retarget_qpos_from_pico_body_poses(body_poses)
    return np.asarray(q, dtype=np.float32).ravel()
