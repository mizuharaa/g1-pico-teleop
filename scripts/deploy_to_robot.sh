#!/usr/bin/env bash
# ONE-COMMAND robot deployment. Run when the robot is POWERED and the
# Ethernet cable is plugged into the laptop (adapter enx000ec6c3d44a).
#
#   ~/full-body-teleoperation/scripts/deploy_to_robot.sh
#
# Copies: docker image (9.1 GB), installer, patched holoretarget (reference
# guard + R_y180 parity + IK tuning), watchdog, e-stop. Then runs the
# installer on the Orin. SAFE: install + gate check only, no robot motion.
set -euo pipefail
ROBOT_HOST="${ROBOT_HOST:-192.168.123.164}"   # 10.42.0.1 = over the robot AP
R="unitree@$ROBOT_HOST"
D=~/full-body-teleoperation

echo "== 0/4 preflight =="
ping -c2 -W2 "$ROBOT_HOST" >/dev/null || {
  echo "ERROR: robot unreachable at $ROBOT_HOST. Power/cable/Wi-Fi?"; exit 1; }

echo "== 1/4 copy artifacts (9.1 GB image — takes a while on first run) =="
IMAGE=horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
if ssh "$R" "docker image inspect $IMAGE >/dev/null 2>&1 || sudo -n docker image inspect $IMAGE >/dev/null 2>&1"; then
  echo "image already loaded on the Orin — skipping the 9.1 GB tar copy"
else
  scp "$D/artifacts/holomotion_v1.4.0_orin_jp5.1_arm64.tar" "$R":~/ || exit 1
fi
scp "$D/scripts/robot_install.sh" "$D/scripts/joint_watchdog.py" "$D/scripts/g1_estop.py" "$R":~/
# NB: wipe first — scp -r into an EXISTING dir nests instead of replacing
# (2026-08-10: shipped a stale 08-07 holoretarget that way; policy setup
# then failed on the missing stream_gate module)
rsync -a --delete "$D/HoloMotion/holoretarget/" "$R":holoretarget_patched/
# failsafe rework 2026-08-10: suppression handling in the policy source —
# REQUIRED alongside the guard (see robot_install.sh overlay step)
ssh "$R" "mkdir -p ~/humanoid_policy_patched"
scp "$D/HoloMotion/deployment/unitree_g1_ros2_29dof/src/humanoid_policy/local_retarget.py" \
    "$D/HoloMotion/deployment/unitree_g1_ros2_29dof/src/humanoid_policy/policy_runtime.py" \
    "$D/HoloMotion/deployment/unitree_g1_ros2_29dof/src/humanoid_policy/policy_node_29dof.py" \
    "$R":~/humanoid_policy_patched/
scp "$D/HoloMotion/deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml" \
    "$R":~/humanoid_policy_patched/orin_docker.yaml

echo "== 2/4 run installer on the Orin (docker load + patch overlay + gate check) =="
ssh -t "$R" "bash ~/robot_install.sh"

echo "== 3/4 REMINDER: join the Orin to the teleop Wi-Fi and note its IP =="
echo "  ssh $R"
echo "  nmcli device wifi connect \"Tran's iPhone\" password 1234567890"
echo "  hostname -I    # the PICO app points at THIS IP during live sessions"

echo "== 4/4 session-day terminals (laptop) =="
echo "  T1: ~/full-body-teleoperation/scripts/estop_console.sh      # ENTER = damp"
echo "  T2: ssh $R  ->  container  ->  holomotion teleop"
echo "  T3 (on Orin or laptop): python joint_watchdog.py --iface eth0 \\"
echo "        --on-alarm 'python g1_estop.py --iface eth0 --now'"
echo
echo "Gate to proceed: 'HoloMotion Docker check PASSED. No robot action was sent.'"
