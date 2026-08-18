#!/usr/bin/env bash
# HoloMotion install on the G1's Jetson Orin (PC2, 192.168.123.164).
# Run ON THE ROBOT (ssh unitree@192.168.123.164) after copying the image tar over:
#   From the laptop:
#     scp ~/full-body-teleoperation/artifacts/holomotion_v1.4.0_orin_jp5.1_arm64.tar \
#         unitree@192.168.123.164:~/
#     scp ~/full-body-teleoperation/scripts/robot_install.sh unitree@192.168.123.164:~/
#   On the robot:
#     bash ~/robot_install.sh
#
# Safe: this script only installs/loads. It never starts the controller.
set -uo pipefail

TAR=~/holomotion_v1.4.0_orin_jp5.1_arm64.tar
IMAGE=horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64

echo "== 1/5 JetPack / L4T version (HoloMotion image targets stock JetPack 5.1) =="
cat /etc/nv_tegra_release 2>/dev/null || echo "WARN: /etc/nv_tegra_release missing — check JetPack version manually"

echo "== 2/5 Docker + nvidia runtime =="
if ! command -v docker >/dev/null; then
  echo "ERROR: docker not installed on the Orin. Install docker first."; exit 1
fi
docker info 2>/dev/null | grep -i runtime || sudo docker info | grep -i runtime
if ! (docker info 2>/dev/null || sudo docker info 2>/dev/null) | grep -qi nvidia; then
  echo "nvidia runtime missing -> configuring /etc/docker/daemon.json"
  sudo tee /etc/docker/daemon.json >/dev/null <<'EOF'
{
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
  "default-runtime": "nvidia"
}
EOF
  sudo systemctl restart docker
fi

echo "== 3/5 Disk space (need ~25 GB free for docker load) =="
df -h / /var/lib/docker 2>/dev/null | sed -n '1,3p'

echo "== 4/5 Load image =="
if docker image inspect "$IMAGE" >/dev/null 2>&1 || sudo docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Image already loaded."
else
  [ -f "$TAR" ] || { echo "ERROR: $TAR not found — scp it from the laptop first."; exit 1; }
  (docker load -i "$TAR" 2>/dev/null || sudo docker load -i "$TAR") || exit 1
fi

echo "== 4.5/5 Overlay local holoretarget patches into the image =="
# The laptop repo carries local safety/tuning patches the stock image lacks:
#   - reference_guard.py + online.py gate (floating-legs / teleport / tilt holds)
#   - R_y180 CPU parity fix, pinned-memory gating (_engine_impl.py)
#   - tuned wrist/elbow IK weights (smplx_to_g1.json)
# Copy ~/holoretarget_patched (scp'd from the laptop) over the container copy:
#   scp -r ~/full-body-teleoperation/HoloMotion/holoretarget unitree@192.168.123.164:~/holoretarget_patched
if [ -d ~/holoretarget_patched ]; then
  CID=$(docker create "$IMAGE" 2>/dev/null || sudo docker create "$IMAGE")
  (docker cp ~/holoretarget_patched/. "$CID":/opt/holomotion/holoretarget/ 2>/dev/null || \
   sudo docker cp ~/holoretarget_patched/. "$CID":/opt/holomotion/holoretarget/)
  # 2026-08-10 failsafe rework: the guard now SUPPRESSES (returns None) on
  # sustained garbage, and the patched local_retarget.py converts that into
  # "no frame" so the policy's data-age failsafe returns the robot to the
  # default standing pose. The stock local_retarget.py would pass None into
  # store_device and can crash the policy node — the overlay below is
  # MANDATORY whenever the holoretarget overlay is applied. The file can
  # exist at several paths in the image (source + colcon install spaces);
  # patch every copy.
  if [ -f ~/humanoid_policy_patched/local_retarget.py ]; then
    for f in ~/humanoid_policy_patched/*.py; do
      NAME=$(basename "$f")
      # 2026-08-12 TRAP: ros2 launch executes the EXTENSIONLESS entry copy
      # in install/.../lib/humanoid_control/ directly as __main__ — a 5th
      # copy the '*.py' find missed. We shipped guards for weeks that never
      # ran. Patch module copies AND any extensionless entry-script copy.
      STEM="${NAME%.py}"
      FINDCMD="find /opt -name $NAME -path '*humanoid_policy*' 2>/dev/null; find /opt -name $STEM -type f -path '*lib/humanoid_control*' 2>/dev/null"
      TARGETS=$( (docker run --rm --entrypoint bash "$IMAGE" \
          -c "$FINDCMD" 2>/dev/null) || \
        sudo docker run --rm --entrypoint bash "$IMAGE" \
          -c "$FINDCMD" )
      if [ -z "$TARGETS" ]; then
        echo "ERROR: $NAME not found inside the image — cannot overlay the"
        echo "       failsafe patches. NOT safe to run teleop. Aborting."
        (docker rm "$CID" >/dev/null 2>&1 || sudo docker rm "$CID" >/dev/null)
        exit 1
      fi
      for p in $TARGETS; do
        (docker cp "$f" "$CID":"$p" 2>/dev/null || \
         sudo docker cp "$f" "$CID":"$p")
        echo "  patched $p"
      done
    done
  else
    echo "ERROR: ~/humanoid_policy_patched/local_retarget.py missing — the"
    echo "       guard overlay REQUIRES it (suppression handling). scp it"
    echo "       from the laptop (deploy_to_robot.sh does this). Aborting."
    (docker rm "$CID" >/dev/null 2>&1 || sudo docker rm "$CID" >/dev/null)
    exit 1
  fi
  # P1 (2026-08-14): limit_scales 1.0 joint-limit config. TWO copies exist in
  # the image (src/config + install/share); the binary reads the install/share
  # one. Patch BOTH. Runtime proof after container start: main_node must log
  # "Joint limit scales - Position: 1.000000" — if it logs 2.0, the wrong
  # copy was patched (same class of trap as the extensionless entry copy).
  if [ -f ~/humanoid_policy_patched/g1_29dof_holomotion.yaml ]; then
    for p in \
      /opt/holomotion/deployment/unitree_g1_ros2_29dof/src/config/g1_29dof_holomotion.yaml \
      /opt/holomotion/deployment/unitree_g1_ros2_29dof/install/humanoid_control/share/humanoid_control/config/g1_29dof_holomotion.yaml; do
      (docker cp ~/humanoid_policy_patched/g1_29dof_holomotion.yaml "$CID":"$p" 2>/dev/null || \
       sudo docker cp ~/humanoid_policy_patched/g1_29dof_holomotion.yaml "$CID":"$p")
      echo "  patched $p"
    done
  else
    echo "WARN: g1_29dof_holomotion.yaml missing from ~/humanoid_policy_patched —"
    echo "      image keeps whatever limit_scales it already has (stock = 2.0!)"
  fi
  # launch-profile config (max_data_age etc.) ships the same way
  if [ -f ~/humanoid_policy_patched/orin_docker.yaml ]; then
    (docker cp ~/humanoid_policy_patched/orin_docker.yaml \
       "$CID":/opt/holomotion/deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml 2>/dev/null || \
     sudo docker cp ~/humanoid_policy_patched/orin_docker.yaml \
       "$CID":/opt/holomotion/deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml)
    echo "  patched launch_profiles/orin_docker.yaml"
  fi
  (docker commit "$CID" "$IMAGE" >/dev/null 2>&1 || sudo docker commit "$CID" "$IMAGE" >/dev/null)
  (docker rm "$CID" >/dev/null 2>&1 || sudo docker rm "$CID" >/dev/null)
  echo "holoretarget + local_retarget patches overlaid into $IMAGE"
  echo "(guard w/ auto return-to-default + parity + IK tuning)."
else
  echo "WARN: ~/holoretarget_patched not found — image runs STOCK retargeting"
  echo "      (no reference guard!). scp it from the laptop first."
fi

echo "== 5/5 Dry-run check (sends NO robot action) =="
echo "Starting container. Inside it, run:  holomotion check"
echo "Continue to a real session ONLY if it prints:"
echo "  'HoloMotion Docker check PASSED. No robot action was sent.'"
(docker run --rm -it \
  --runtime nvidia --gpus all --privileged --network host \
  --name holomotion_g1 --entrypoint bash "$IMAGE" \
  -c "holomotion check" 2>/dev/null) || \
sudo docker run --rm -it \
  --runtime nvidia --gpus all --privileged --network host \
  --name holomotion_g1 --entrypoint bash "$IMAGE" \
  -c "holomotion check"
