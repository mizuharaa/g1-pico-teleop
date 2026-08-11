"""Shared online and training API for HoloRetarget."""

from __future__ import annotations

import os
import time

import numpy as np

from ._engine_impl import HoloRetargetRunner
from .config import DEFAULT_CONFIG, HoloRetargetConfig
from .schema import UNITREE_G1_29DOF_NAMES


class HoloRetargeter:
    """HoloMotion production retargeter.

    The only retarget path is ``Pico/XRoboToolkit body_poses[24,7] -> qpos[36]``.
    """

    def __init__(self, config: HoloRetargetConfig | None = None) -> None:
        self.config = config or DEFAULT_CONFIG
        # Training-side HoloSMPL conversion initializes this lazily when needed.
        self._smpl_adapter = None
        self._runner = self._create_runner(src_human="smplx")
        from ._gpu_targets import HoloPicoGpuTargetRunner

        # LOCAL PATCH (2026-08-07): fall back to the CPU retarget path when no
        # CUDA device exists (GPU-less laptop rehearsal). The CPU path in
        # _engine_impl carries the R_y180 parity fix; behavior with a GPU
        # present is unchanged.
        try:
            self._gpu_target_runner = HoloPicoGpuTargetRunner(self)
        except RuntimeError as exc:
            if "CUDA" not in str(exc):
                raise
            self._gpu_target_runner = None
        # LOCAL PATCH (2026-08-07): sanity guard between operator and policy.
        # Disable (not recommended) with HOLORETARGET_GUARD=0.
        from .reference_guard import ReferenceGuard

        self._guard = ReferenceGuard() if os.environ.get(
            "HOLORETARGET_GUARD", "1") != "0" else None
        self._last_device_qpos = None
        self._device_check_counter = 0
        self._device_check_every = 10
        self._device_output_check = os.environ.get(
            "GUARD_DEVICE_OUTPUT_CHECK", "1") != "0"
        self.last_timing: dict[str, float] = {}
        self.last_body_poses: np.ndarray | None = None

    def _create_runner(self, src_human: str) -> HoloRetargetRunner:
        return HoloRetargetRunner(
            asset_root=str(self.config.resolved_asset_root),
            robot=self.config.robot,
            src_human=src_human,
            offset_to_ground=False,
            newton_iterations=self.config.newton_iterations,
            newton_cuda_graph=self.config.use_cuda_graph,
            newton_joint_limit_weight=self.config.joint_limit_weight,
            newton_max_joint_step=self.config.max_joint_step,
            newton_target_table=self.config.target_table,
            newton_robot_asset=self.config.robot_asset,
            newton_root_seed_mode=self.config.root_seed_mode,
            ground_calibration_frames=self.config.ground_calibration_frames,
            ground_height=self.config.ground_height,
            ground_lift_only=self.config.ground_lift_only,
            ground_calibration_mode=self.config.ground_calibration_mode,
            ground_target_scope=self.config.ground_target_scope,
            profile_timing=self.config.profile_timing,
        )

    @property
    def last_ground_offset_z(self) -> float:
        return float(getattr(self._runner, "last_ground_offset_z", 0.0))

    @property
    def dof_names(self) -> tuple[str, ...]:
        """Joint order used by the 29 values at ``qpos[7:]``."""

        return UNITREE_G1_29DOF_NAMES

    def reset_sequence(self) -> None:
        self._runner.reset_sequence()
        if self._gpu_target_runner is not None:
            self._gpu_target_runner.reset_sequence()
        if self._guard is not None:  # LOCAL PATCH
            self._guard.reset()
        self.last_timing = {}
        self.last_body_poses = None

    def retarget_qpos_from_body_poses(
        self,
        body_poses: np.ndarray,
    ) -> np.ndarray | None:
        """Retarget one frame. Returns None when the guard SUPPRESSES the
        reference (sustained impossible input) — the caller must publish
        nothing so the policy's data-age failsafe returns the robot to the
        default standing pose."""
        body_poses = np.asarray(body_poses, dtype=np.float32)
        if body_poses.shape != (24, 7):
            raise ValueError(
                f"body_poses must have shape (24, 7), got {body_poses.shape}"
            )
        t0 = time.perf_counter()
        if self._gpu_target_runner is not None:
            reference_qpos = self._gpu_target_runner.retarget_qpos_from_body_poses(
                body_poses
            )
        else:  # LOCAL PATCH: CPU fallback (no CUDA device)
            reference_qpos = self._runner.retarget_qpos_from_pico_body_poses(
                body_poses
            )
        if self._guard is not None:  # LOCAL PATCH: sanity gate (may suppress)
            reference_qpos = self._guard.gate(body_poses, reference_qpos)
        self.last_body_poses = body_poses.copy()
        self.last_timing = dict(getattr(self._runner, "last_timing", {}))
        self.last_timing["holoretarget.total"] = time.perf_counter() - t0
        return reference_qpos

    def retarget_qpos_device_from_body_poses(self, body_poses: np.ndarray):
        """Retarget into a Warp CUDA array without copying qpos back to CPU."""
        body_poses = np.asarray(body_poses, dtype=np.float32)
        if body_poses.shape != (24, 7):
            raise ValueError(
                f"body_poses must have shape (24, 7), got {body_poses.shape}"
            )
        if self._gpu_target_runner is None:  # LOCAL PATCH
            raise RuntimeError(
                "retarget_qpos_device_from_body_poses requires a CUDA device"
            )
        # LOCAL PATCH: guard for the zero-copy robot path. On a violation we
        # SKIP the solve, so the persistent device buffer still holds the
        # last good reference — an implicit, copy-free hold. Once the hold
        # window is exhausted the guard escalates to SUPPRESS and we return
        # None: the policy node stops receiving references and its data-age
        # failsafe returns the robot to the default standing pose.
        if self._guard is not None:
            reason = self._guard.check_input(body_poses)
            if reason is not None:
                self._device_check_every = 1   # full validation until clean
                if self._guard.register_trip(reason) == "suppress":
                    return None
                return self._last_device_qpos  # None at startup -> no output
        reference_qpos = (
            self._gpu_target_runner.retarget_qpos_device_from_body_poses(
                body_poses
            )
        )
        # Output-side check (root height/tilt/NaN from the solver) — the CPU
        # path gets this via gate(); here we sample the 36-float device qpos
        # (~tiny DtoH copy) every N frames, escalating to every frame while
        # any violation streak is active so the hold window can expire and
        # suppression can fire. The hold is only cleared by a frame whose
        # input AND sampled output both pass. Any failure of the check
        # itself disables the check rather than the pipeline.
        self._device_check_counter += 1
        if self._guard is not None and self._device_output_check:
            if self._device_check_counter % self._device_check_every == 0:
                try:
                    qpos_host = np.asarray(
                        reference_qpos.numpy()
                    ).reshape(-1)
                    reason = self._guard.check_output(qpos_host)
                except Exception as exc:  # never let the check kill the stream
                    self._device_output_check = False
                    print(
                        f"[GUARD] device output check disabled ({exc})",
                        flush=True,
                    )
                    reason = None
                if reason is not None:
                    self._device_check_every = 1
                    if self._guard.register_trip(reason) == "suppress":
                        return None
                    return self._last_device_qpos
                self._device_check_every = 10
                self._guard._mark_good()
        elif self._guard is not None:
            # no output check available: input-clean is full validation
            self._guard._mark_good()
        self._last_device_qpos = reference_qpos
        self.last_body_poses = body_poses.copy()
        self.last_timing = dict(getattr(self._runner, "last_timing", {}))
        return reference_qpos


__all__ = ["HoloRetargeter"]
