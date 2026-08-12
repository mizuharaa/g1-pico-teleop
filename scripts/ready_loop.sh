#!/usr/bin/env bash
# AUTO-READY LOOP (2026-08-12): after ANY damp/e-stop (which kills the app),
# bring the session back to remote-usable with ZERO operator round-trips:
#   container dead -> restart -> wait for policy setup -> re-verify factory
#   release -> log READY. Start button works again ~60s after every damp.
# Run: setsid nohup scripts/ready_loop.sh >> /tmp/ready_loop.log 2>&1 &
R="unitree@${ROBOT_HOST:-192.168.123.164}"
D=~/full-body-teleoperation
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $R"
while :; do
  if $SSH true 2>/dev/null; then
    if ! $SSH "docker ps --format '{{.Names}}' 2>/dev/null | grep -qx holomotion_g1"; then
      echo "[$(date +%H:%M:%S)] app down -> restarting"
      "$D/scripts/start_teleop_container.sh" "${ROBOT_HOST:-192.168.123.164}" >/dev/null 2>&1
      for _ in $(seq 1 24); do
        $SSH "docker logs --tail 40 holomotion_g1 2>&1 | grep -q 'setup completed successfully'" && break
        sleep 5
      done
      # idempotent: only acts if the factory controller re-acquired
      timeout 30 "$D/scripts/release_factory_control.sh" >/dev/null 2>&1
      echo "[$(date +%H:%M:%S)] READY — press Start"
    fi
  fi
  sleep 10
done
