#!/usr/bin/env bash
# ONE-COMMAND session bringup. Idempotent: every phase CHECKS first, FIXES
# only what is broken, then RE-CHECKS. Safe to rerun at any point; nothing
# here moves the robot.
#
#   session_up.sh              # full bringup (network + docker + container + safety)
#   session_up.sh --check-only # report status, fix nothing
#   session_up.sh --no-app     # stop after infra (don't start the teleop container)
#
# Exit code 0 = READY. Non-zero = the FIRST failed phase; the fix is printed.
set -uo pipefail

ROBOT_HOST="${ROBOT_HOST:-192.168.123.164}"
R="unitree@$ROBOT_HOST"
D=~/full-body-teleoperation
IMAGE=horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $R"
CHECK_ONLY=0; NO_APP=0
for a in "$@"; do
  case "$a" in
    --check-only) CHECK_ONLY=1 ;;
    --no-app)     NO_APP=1 ;;
    *) echo "unknown flag $a"; exit 2 ;;
  esac
done

PASS=$'\e[32mPASS\e[0m'; FAIL=$'\e[31mFAIL\e[0m'; FIXED=$'\e[33mFIXED\e[0m'
step() { printf '\n== %s ==\n' "$1"; }
die()  { printf '%s %s\n  FIX: %s\n' "$FAIL" "$1" "$2"; exit 1; }
ok()   { printf '%s %s\n' "$PASS" "$1"; }
fixed(){ printf '%s %s\n' "$FIXED" "$1"; }

# ---------------------------------------------------------------- laptop
step "0/6 laptop preflight"
FREE_GB=$(df --output=avail -BG "$HOME" | tail -1 | tr -dc 0-9)
if [ "$FREE_GB" -lt 5 ]; then
  die "disk: only ${FREE_GB}G free (recordings died silently at 0G on 08-10)" \
      "free space in ~/Videos/g1-demos or /tmp before a session"
fi
ok "disk: ${FREE_GB}G free"

IFACE=$(ip -4 -o addr | awk '/ 192\.168\.123\./ {print $2; exit}')
if [ -z "$IFACE" ]; then
  # remediation: try to bring up the known ethernet profile
  if [ "$CHECK_ONLY" = 0 ] && nmcli -t -f NAME con show | grep -qi "robot\|123"; then
    nmcli con up "$(nmcli -t -f NAME con show | grep -i 'robot\|123' | head -1)" >/dev/null 2>&1
    sleep 2
    IFACE=$(ip -4 -o addr | awk '/ 192\.168\.123\./ {print $2; exit}')
    [ -n "$IFACE" ] && fixed "ethernet profile brought up on $IFACE"
  fi
  [ -z "$IFACE" ] || true
fi
[ -z "$IFACE" ] && die "no 192.168.123.x interface" \
  "plug the gigabit cable (adapter enx000ec6c3d44a) / check nmcli con"
ok "wired link on $IFACE"

# ---------------------------------------------------------------- robot reachable
step "1/6 robot reachable"
if ! ping -c2 -W2 "$ROBOT_HOST" >/dev/null 2>&1; then
  die "robot not answering at $ROBOT_HOST" \
      "power the robot on (factory boot), wait ~60 s, keep the cable in; rerun"
fi
ok "ping $ROBOT_HOST"
$SSH true 2>/dev/null || die "ssh to $R failed" \
  "key auth broke? try: ssh $R (password), then ssh-copy-id"
ok "ssh key auth"

# ---------------------------------------------------------------- docker daemon + image
step "2/6 docker on the Orin"
if ! $SSH "docker info >/dev/null 2>&1"; then
  if [ "$CHECK_ONLY" = 0 ]; then
    $SSH "sudo -n systemctl start docker 2>/dev/null || echo 123 | sudo -S systemctl start docker" >/dev/null 2>&1
    sleep 3
    $SSH "docker info >/dev/null 2>&1" && fixed "docker daemon started" \
      || die "docker daemon down and could not start it" \
             "ssh $R; sudo systemctl status docker"
  else
    die "docker daemon down" "sudo systemctl start docker on the Orin"
  fi
else
  ok "docker daemon up"
fi
if ! $SSH "docker image inspect $IMAGE >/dev/null 2>&1"; then
  die "image $IMAGE missing on the Orin" \
      "run: $D/scripts/deploy_to_robot.sh   (ships image + patches + gate check)"
fi
ok "image present"

# yaml sanity INSIDE the image (build_before_launch:true once wiped the models)
YAML_CHECK=$($SSH "docker run --rm --entrypoint cat $IMAGE \
  /opt/holomotion/deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml 2>/dev/null \
  | grep -E 'build_before_launch|motion_observation_backend|max_data_age|reference_source'" 2>/dev/null)
echo "$YAML_CHECK" | grep -q "build_before_launch: false" \
  || die "image yaml is NOT the fixed one ($YAML_CHECK)" \
         "rerun $D/scripts/deploy_to_robot.sh to overlay the fixed orin_docker.yaml"
# 2026-08-11 audit S7: validate the PAIR. cpu backend + pico_local is
# REJECTED at policy-node setup (RuntimeError) — the old gate enforced
# exactly that fatal combination. Valid pairs:
#   reference_source zmq        + backend cpu   (retarget on laptop, wired)
#   reference_source pico_local + backend warp  (unrehearsed 20 Hz path - NO)
if echo "$YAML_CHECK" | grep -q 'motion_observation_backend: "cpu"'; then
  echo "$YAML_CHECK" | grep -q 'reference_source: "zmq"' \
    || die "image yaml has cpu backend + pico_local — policy node RAISES at startup" \
           "rerun $D/scripts/deploy_to_robot.sh (repo yaml now ships zmq+cpu)"
fi
ok "image yaml verified (build_before_launch=false, backend+source pair valid)"

# ---------------------------------------------------------------- robot AP
step "3/6 robot-hosted Wi-Fi AP (headset link)"
AP_ACTIVE=$($SSH "nmcli -t -f NAME con show --active 2>/dev/null | grep -x 'Hotspot-1'" 2>/dev/null || true)
if [ -z "$AP_ACTIVE" ]; then
  if [ "$CHECK_ONLY" = 0 ]; then
    $SSH "sudo -n nmcli con up Hotspot-1 2>/dev/null || echo 123 | sudo -S nmcli device wifi hotspot ifname wlan0 ssid g1-teleop password teleop12345" >/dev/null 2>&1
    sleep 3
    AP_ACTIVE=$($SSH "nmcli -t -f NAME con show --active | grep -x 'Hotspot-1'" 2>/dev/null || true)
    [ -n "$AP_ACTIVE" ] && fixed "AP Hotspot-1 brought up" \
      || die "could not start the robot AP" \
             "ssh $R; sudo nmcli device wifi hotspot ifname wlan0 ssid g1-teleop password teleop12345"
  else
    die "robot AP down" "nmcli con up Hotspot-1 on the Orin"
  fi
else
  ok "AP Hotspot-1 active"
fi
# one-time: make the AP survive reboots (HANDOFF 08-10: autoconnect NOT yet set)
if [ "$CHECK_ONLY" = 0 ]; then
  AUTO=$($SSH "nmcli -t -f connection.autoconnect con show Hotspot-1 2>/dev/null" || true)
  if ! echo "$AUTO" | grep -q "yes"; then
    $SSH "sudo -n nmcli con modify Hotspot-1 connection.autoconnect yes connection.autoconnect-priority 10 2>/dev/null || echo 123 | sudo -S nmcli con modify Hotspot-1 connection.autoconnect yes connection.autoconnect-priority 10" >/dev/null 2>&1 \
      && fixed "AP autoconnect enabled (survives robot reboots now)"
  fi
fi
$SSH "ip -4 -o addr show wlan0 | grep -q 10.42.0.1" \
  && ok "AP address 10.42.0.1 (headset PC-service target)" \
  || die "AP up but wlan0 is not 10.42.0.1" "check: ip addr show wlan0 on the Orin"

# ---------------------------------------------------------------- PC service (port 63901)
step "4/6 XRoboToolkit PC service"
if $SSH "ss -ltn 2>/dev/null | grep -q ':63901 '"; then
  ok "port 63901 listening on the robot"
else
  if [ "$CHECK_ONLY" = 0 ] && $SSH "systemctl list-unit-files 2>/dev/null | grep -q roboticsservice"; then
    $SSH "sudo -n systemctl restart roboticsservice 2>/dev/null || echo 123 | sudo -S systemctl restart roboticsservice" >/dev/null 2>&1
    sleep 3
    $SSH "ss -ltn | grep -q ':63901 '" && fixed "roboticsservice restarted" \
      || echo "WARN: 63901 still not listening — the service may live in the container (starts with the app)"
  else
    echo "WARN: 63901 not listening yet — if it comes up with the container, ignore"
  fi
fi

# ---------------------------------------------------------------- teleop container
step "5/6 teleop container"
if [ "$NO_APP" = 1 ]; then
  echo "SKIP (--no-app): start later with $D/scripts/start_teleop_container.sh"
elif $SSH "docker ps --format '{{.Names}}' | grep -qx holomotion_g1"; then
  ok "holomotion_g1 already running (logs: ssh $R docker logs -f holomotion_g1)"
else
  if [ "$CHECK_ONLY" = 1 ]; then
    die "container not running" "$D/scripts/start_teleop_container.sh $ROBOT_HOST"
  fi
  "$D/scripts/start_teleop_container.sh" "$ROBOT_HOST" | grep -q CONTAINER-STARTED \
    && fixed "container started (first start after a rebuild recompiles TRT ~8-10 min — watch docker logs)" \
    || die "container failed to start" "ssh $R docker logs holomotion_g1"
fi

# ---------------------------------------------------------------- laptop safety rail
step "6/6 safety rail (wired side)"
if [ "$CHECK_ONLY" = 0 ]; then
  pgrep -f 'joint_watchdog\.py' >/dev/null || "$D/scripts/arm_watchdog.sh"
fi
pgrep -f 'joint_watchdog\.py' >/dev/null \
  && ok "watchdog armed (auto-damp on alarm)" \
  || die "watchdog not running" "$D/scripts/arm_watchdog.sh (needs the wired link)"

cat <<EOM

READY. Remaining MANUAL steps (cannot be automated):
  1. Headset: join Wi-Fi 'g1-teleop' (pw teleop12345), app -> Enter 10.42.0.1
     -> Send tick -> Mode: Full-body  (Mode resets to None on EVERY reconnect!)
  2. T1 e-stop console (keep it focused):  $D/scripts/estop_console.sh
  3. Remote in the spotter's hand. Button map under custom control:
       Y  = tracking -> velocity stand (FIRST response, robot balances)
       Select = damped e-stop, 2 s damp then motors OFF (collapse; support first)
       L1 = damped e-stop (patched) / L1+L2 = instant limp (bench only)
EOM
