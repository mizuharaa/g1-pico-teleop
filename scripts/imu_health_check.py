#!/usr/bin/env python
"""Post-fall IMU health check — robot STATIONARY (hanging/standing still).
Run: CYCLONEDDS_HOME=~/.local/cyclonedds ~/miniconda3/envs/holomotion_teleop/bin/python imu_health_check.py
PASS: projected gravity ~ [0,0,-1], gyro ~0, no wander. FAIL -> IMU recal
(Unitree app) before blaming policies for tracking-mode phantom errors."""
import sys, time
import numpy as np
sys.path.insert(0, "/home/alois/meta-quest-teleoperate/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
ChannelFactoryInitialize(0, sys.argv[1] if len(sys.argv) > 1 else "enx000ec6c3d44a")
samples = []
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lambda m: samples.append((np.array(m.imu_state.quaternion, float),
                                   np.array(m.imu_state.gyroscope, float))), 10)
t0 = time.time()
while time.time() - t0 < 20 and len(samples) < 900:
    time.sleep(0.05)
qs = np.array([s[0] for s in samples]); gs = np.array([s[1] for s in samples])
w, x, y, z = qs.mean(axis=0)
pg = -np.array([2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)])
print(f"samples={len(samples)}")
print(f"projected gravity [{pg[0]:+.3f} {pg[1]:+.3f} {pg[2]:+.3f}] (want ~[0,0,-1])")
print(f"gyro |mean| {np.abs(gs.mean(axis=0))} (want ~0)  std {gs.std(axis=0)}")
print(f"quat wander(std) {qs.std(axis=0)} (want ~0)")
