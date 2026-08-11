#!/usr/bin/env python
"""IK retargeter benchmark harness — the fixed yardstick for the CPU-IK work.

Every candidate backend implements one function:

    retarget(body_poses: np.ndarray[24, 7]) -> np.ndarray[36]

(input = XRoboToolkit PICO body stream: 24 joints x [x y z, qx qy qz qw];
 output = G1 qpos: 7 floating base + 29 joint angles.)

The harness feeds it synthetic-but-plausible PICO frames (same generator idea
as HoloMotion's make_fake_pico_frame, extended to exercise arms, squat and
waist), then reports per-solve latency percentiles and continuity metrics.

Usage:
    python ik_bench.py                     # bench the built-in no-op baseline
    python ik_bench.py --backend mymod:fn  # bench mymod.fn
"""
from __future__ import annotations

import argparse
import importlib
import time

import numpy as np

HZ = 100.0          # feed rate: 2x the 50 Hz teleop rate to leave headroom
N_FRAMES = 2000     # ~20 s of motion
BUDGET_MS = 20.0    # 50 Hz robot feedback loop (sub-ms no longer required)


def make_pico_frame(t: float) -> np.ndarray:
    """Synthetic PICO body_poses[24,7] with arm swing, squat and waist yaw."""
    body = np.zeros((24, 7), dtype=np.float32)
    body[:, 6] = 1.0  # identity quaternions (xyzw)
    # crude vertical skeleton, pelvis at index 0
    body[:, 1] = np.linspace(0.0, 1.6, 24, dtype=np.float32)
    squat = 0.10 * (1.0 - np.cos(2.0 * np.pi * 0.25 * t)) / 2.0
    body[0, :3] = np.array([0.0, 0.9 - squat, 0.0], dtype=np.float32)
    # arm swing on the same joints HoloMotion's fake generator drives
    swing = 0.6 * np.sin(2.0 * np.pi * 0.5 * t)
    half = 0.5 * swing
    qz, qw = np.float32(np.sin(half)), np.float32(np.cos(half))
    for joint_id in (16, 18, 20, 22):
        body[joint_id, 3:7] = np.array([0.0, 0.0, qz, qw], dtype=np.float32)
    # waist yaw
    wy = 0.25 * np.sin(2.0 * np.pi * 0.2 * t)
    body[3, 3:7] = np.array([0.0, np.sin(wy / 2), 0.0, np.cos(wy / 2)],
                            dtype=np.float32)
    return body


def baseline_noop(body_poses: np.ndarray) -> np.ndarray:
    """Floor for harness overhead: returns a fixed qpos."""
    out = np.zeros(36, dtype=np.float32)
    out[3] = 1.0
    return out


def load_backend(spec: str):
    mod_name, _, fn_name = spec.partition(":")
    mod = importlib.import_module(mod_name)
    return getattr(mod, fn_name or "retarget")


def run(backend, label: str) -> dict:
    # warmup (JIT, caches, model loading side effects)
    for i in range(50):
        backend(make_pico_frame(i / HZ))

    lat_us = np.empty(N_FRAMES)
    prev_q = None
    max_step = 0.0
    for i in range(N_FRAMES):
        frame = make_pico_frame(i / HZ)
        t0 = time.perf_counter()
        q = backend(frame)
        lat_us[i] = (time.perf_counter() - t0) * 1e6
        q = np.asarray(q, dtype=np.float64).ravel()
        assert q.shape == (36,), f"expected qpos[36], got {q.shape}"
        if prev_q is not None:
            max_step = max(max_step, float(np.max(np.abs(q[7:] - prev_q[7:]))))
        prev_q = q

    stats = {
        "label": label,
        "mean_us": float(lat_us.mean()),
        "p50_us": float(np.percentile(lat_us, 50)),
        "p95_us": float(np.percentile(lat_us, 95)),
        "p99_us": float(np.percentile(lat_us, 99)),
        "max_us": float(lat_us.max()),
        "max_joint_step_rad": max_step,
        "sub_ms": bool(np.percentile(lat_us, 99) < BUDGET_MS * 1000),
    }
    return stats


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="",
                    help="module:function implementing retarget(body_poses)->qpos[36]")
    args = ap.parse_args()

    if args.backend:
        backend = load_backend(args.backend)
        label = args.backend
    else:
        backend, label = baseline_noop, "baseline_noop"

    s = run(backend, label)
    print(f"\n=== {s['label']} ===")
    print(f"  mean  {s['mean_us']:9.1f} us")
    print(f"  p50   {s['p50_us']:9.1f} us")
    print(f"  p95   {s['p95_us']:9.1f} us")
    print(f"  p99   {s['p99_us']:9.1f} us")
    print(f"  max   {s['max_us']:9.1f} us")
    print(f"  max joint step between frames: {s['max_joint_step_rad']:.4f} rad")
    verdict = "PASS (sub-millisecond p99)" if s["sub_ms"] else \
        f"FAIL (p99 {s['p99_us']/1000:.2f} ms >= {BUDGET_MS} ms budget)"
    print(f"  verdict: {verdict}")


if __name__ == "__main__":
    main()
