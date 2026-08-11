#!/usr/bin/env bash
# Overlay the C++ FSM patch (main_node.cpp: L1 -> damped e-stop) + current
# python patches into the robot image and REBUILD the humanoid_control
# package inside a build container. Ends with a gate check. Sends NO robot
# action.
#
# Why this exists: python files can be docker-cp'd into the install space,
# but main_node.cpp is compiled — it needs `colcon build` inside the image.
# DANGERS handled here:
#   - `clean_workspace` (build_before_launch:true) wipes install/ incl. the
#     model weights -> we do a targeted `colcon build --packages-select
#     humanoid_control` with NO clean, and back up + verify the models dir.
#   - a rebuild regenerates install/ python from src/ -> we overlay the
#     patched python into SRC first so the rebuild installs patched code.
#   - never `docker commit` a running app -> build container is stopped
#     before commit.
set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-192.168.123.164}"
R="unitree@$ROBOT_HOST"
D=~/full-body-teleoperation
SRC=$D/HoloMotion/deployment/unitree_g1_ros2_29dof
IMAGE=horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64

echo "== 1/4 ship patched sources =="
ssh "$R" "mkdir -p ~/humanoid_policy_patched"
scp "$SRC/src/humanoid_policy/policy_runtime.py" \
    "$SRC/src/humanoid_policy/local_retarget.py" \
    "$SRC/src/src/main_node.cpp" \
    "$SRC/launch_profiles/orin_docker.yaml" \
    "$R":~/humanoid_policy_patched/

echo "== 2/4 overlay + rebuild inside a build container (2-4 min) =="
ssh "$R" bash -s <<'EOF'
set -euo pipefail
IMAGE=horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
WS=/opt/holomotion/deployment/unitree_g1_ros2_29dof
docker rm -f holomotion_build >/dev/null 2>&1 || true
docker run -d --name holomotion_build --entrypoint bash "$IMAGE" -c 'sleep 3600' >/dev/null

# overlay python patches into EVERY copy (src + install spaces)
for f in ~/humanoid_policy_patched/policy_runtime.py ~/humanoid_policy_patched/local_retarget.py; do
  NAME=$(basename "$f")
  for p in $(docker exec holomotion_build bash -c "find /opt -name $NAME -path '*humanoid_policy*'"); do
    docker cp "$f" holomotion_build:"$p"; echo "  patched $p"
  done
done
docker cp ~/humanoid_policy_patched/main_node.cpp holomotion_build:$WS/src/src/main_node.cpp
docker cp ~/humanoid_policy_patched/orin_docker.yaml holomotion_build:$WS/launch_profiles/orin_docker.yaml
echo "  patched src/src/main_node.cpp + orin_docker.yaml"

# models backup, targeted build (NO clean_workspace), verify, restore if eaten
docker exec holomotion_build bash -c "
  set -e
  cd $WS
  tar -C install/humanoid_control/share/humanoid_control -cf /tmp/models_backup.tar models
  source /opt/ros/humble/setup.sh
  source /root/unitree_ros2/setup.sh
  colcon build --packages-select humanoid_control 2>&1 | tail -3
  M=install/humanoid_control/share/humanoid_control/models
  if [ ! -d \$M ] || [ -z \"\$(ls \$M 2>/dev/null)\" ]; then
    echo 'models dir wiped by build -> restoring from backup'
    mkdir -p install/humanoid_control/share/humanoid_control
    tar -C install/humanoid_control/share/humanoid_control -xf /tmp/models_backup.tar
  fi
  ls \$M
  rm /tmp/models_backup.tar
"

# verify BOTH patches landed in the final artifacts
docker exec holomotion_build bash -c "
  set -e
  WS=$WS
  strings \$WS/install/humanoid_control/lib/humanoid_control/humanoid_control | grep -q 'L1+L2' \
    && echo 'VERIFY OK: L1 patch in rebuilt binary'
  grep -rq mode_blend \$WS/install/humanoid_control/lib/python*/site-packages/humanoid_policy/policy_runtime.py 2>/dev/null \
    || grep -q mode_blend \$(find \$WS/install -name policy_runtime.py | head -1) \
    && echo 'VERIFY OK: mode-blend in installed python'
"
docker stop -t 5 holomotion_build >/dev/null
docker commit holomotion_build "$IMAGE" >/dev/null
docker rm holomotion_build >/dev/null
echo "committed patched image"
EOF

echo "== 3/4 gate check (no robot action) =="
ssh "$R" "docker run --rm --runtime nvidia --gpus all --privileged --network host \
  --entrypoint bash $IMAGE -c 'holomotion check' 2>&1 | tail -3"

echo "== 4/4 done =="
echo "Gate to proceed: 'HoloMotion Docker check PASSED. No robot action was sent.'"
