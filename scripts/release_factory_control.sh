#!/usr/bin/env bash
# Release the Unitree factory motion service over DDS — the RELIABLE way to
# enter "debug mode" (the remote's L2+R2 chord often doesn't register).
# Run BEFORE `holomotion teleop`, robot damped on the gantry.
# Prints before/after mode; after must show name: ''.
export CYCLONEDDS_HOME="$HOME/.local/cyclonedds"
exec ~/miniconda3/envs/holomotion_teleop/bin/python \
  ~/full-body-teleoperation/scripts/release_factory_control.py
