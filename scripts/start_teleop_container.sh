#!/usr/bin/env bash
# Start (or restart) the HoloMotion teleop controller on the Orin — WITHOUT
# --rm, so a crash/e-stop leaves the container and its logs behind for
# forensics (2026-08-10: the fall's logs were erased by --rm + self-shutdown).
# Old container is renamed with a timestamp instead of deleted; the last 3
# are kept. Run from the laptop; works over Ethernet (.164) or Wi-Fi (10.42.0.1).
set -uo pipefail
R="unitree@${1:-192.168.123.164}"
IMAGE=horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
ssh -o BatchMode=yes -o ConnectTimeout=5 "$R" bash -s <<'EOF'
if docker ps -a --format '{{.Names}}' | grep -qx holomotion_g1; then
  docker stop -t 2 holomotion_g1 >/dev/null 2>&1
  docker rename holomotion_g1 "holomotion_g1_$(date +%m%d_%H%M%S)"
fi
# prune old crash containers, keep newest 3
docker ps -a --format '{{.Names}}' | grep '^holomotion_g1_' | sort -r | tail -n +4 \
  | xargs -r docker rm >/dev/null 2>&1
docker run -d --runtime nvidia --gpus all --privileged --network host \
  --name holomotion_g1 --entrypoint bash \
  horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64 -c 'holomotion teleop' >/dev/null \
  && echo CONTAINER-STARTED
EOF
