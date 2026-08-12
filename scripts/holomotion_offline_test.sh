#!/usr/bin/env bash
# THE bisection test: tracking mode with the BUNDLED offline clip — no VR.
# Twitch here = policy/hardware. Clean here = reference path (VR/retarget).
# REQUIRES: robot tethered + spotter. Toggle on, run ladder (Start->A->B),
# toggle off after. B plays the bundled clip instead of VR.
set -euo pipefail
R="unitree@${2:-192.168.123.164}"
case "${1:-on}" in
  on)
    ssh "$R" "docker exec holomotion_g1 sed -i 's/enable_teleop_reference: *true/enable_teleop_reference: false/' /opt/holomotion/deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml && docker restart holomotion_g1" \
      && echo "OFFLINE MODE ON — app restarting (~60s). Ladder: Start -> A -> B plays the clip." ;;
  off)
    ssh "$R" "docker exec holomotion_g1 sed -i 's/enable_teleop_reference: *false/enable_teleop_reference: true/' /opt/holomotion/deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml && docker restart holomotion_g1" \
      && echo "VR MODE RESTORED — app restarting." ;;
esac
