#!/usr/bin/env bash
# THE ENTER-KEY E-STOP (laptop). Run this in its own terminal on robot days.
# Big picture: ENTER = damp every joint, sent over the WIRED Ethernet link —
# independent of Wi-Fi, the headset, and the teleop stack.
# The Unitree remote (L2+B) remains the primary failsafe.
set -euo pipefail
export CYCLONEDDS_HOME="$HOME/.local/cyclonedds"
IFACE="${1:-enx000ec6c3d44a}"   # laptop's robot-Ethernet interface
exec ~/miniconda3/envs/holomotion_teleop/bin/python \
    ~/full-body-teleoperation/scripts/g1_estop.py --iface "$IFACE"
