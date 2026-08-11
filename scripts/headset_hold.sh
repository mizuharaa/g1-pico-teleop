#!/usr/bin/env bash
# headset_hold.sh — pin the PICO 4 so teleop survives idle/sleep/Wi-Fi wander.
# Usage: headset_hold.sh [HEADSET_IP] [--watch] [--pin-wifi]
#   One-shot: applies keep-awake + app foreground.
#   --pin-wifi: ALSO forget every saved network except the teleop AP
#               (prevents "headset flees the AP", but wipes office Wi-Fi etc.)
#   --watch: stays running; re-foregrounds the app and re-applies keep-awake
#            whenever the headset wakes or the app loses focus (10 s poll).
# Reaching the headset from the laptop over the robot AP needs a route:
#   sudo ip route add 10.42.0.0/24 via 192.168.123.164
# or an ssh tunnel: ssh -N -L 15555:<headset-ip>:5555 unitree@192.168.123.164
#   then: headset_hold.sh 127.0.0.1:15555-style addressing won't work; use route.
# PREREQ (verified missing 2026-08-11): wireless adb dies with every headset
# reboot. Re-enable once per boot with a USB-C cable: `adb tcpip 5555`,
# or turn on persistent Wireless Debugging in PICO developer options.
#
# NOTE: the app's "Mode" still resets to None on every PC-service reconnect
# (XRoboToolkit app limitation) — that re-tick stays manual for now.
set -u
ADB="$HOME/full-body-teleoperation/tools/platform-tools/adb"
HEADSET_IP="${1:-10.42.0.209}"
SSID="g1-teleop"
PSK="teleop12345"

say() { echo "[headset_hold] $*"; }

"$ADB" connect "$HEADSET_IP:5555" >/dev/null 2>&1
if ! "$ADB" -s "$HEADSET_IP:5555" shell true >/dev/null 2>&1; then
  say "FATAL: adb cannot reach $HEADSET_IP:5555 (headset on AP? wireless adb enabled?)"
  exit 1
fi
A() { "$ADB" -s "$HEADSET_IP:5555" shell "$@"; }

# Resolve the XRoboToolkit app package once (name differs across builds).
PKG=$(A pm list packages 2>/dev/null | tr -d '\r' | grep -i -m1 -E 'xrobo|robotoolkit' | cut -d: -f2)
[ -n "$PKG" ] && say "app package: $PKG" || say "WARN: XRoboToolkit package not found on device"

apply_keepawake() {
  # Never idle-sleep; screen stays on while powered; long screen timeout.
  A svc power stayon true
  A settings put system screen_off_timeout 1800000
  A settings put secure sleep_timeout 0 2>/dev/null
  # PICO-specific proximity/off-head keys vary by firmware — apply best-effort,
  # verify with: adb shell settings list secure | grep -iE 'proximity|wear|psensor'
  for k in pxr_disable_psensor_sleep disable_proximity_sleep; do
    A settings put secure "$k" 1 2>/dev/null
  done
}

pin_wifi() {
  # Make "headset flees to another SSID" impossible: forget every network
  # except the teleop AP, then (re)add ours.
  local ids
  ids=$(A cmd wifi list-networks 2>/dev/null | tr -d '\r' | awk -v ssid="$SSID" 'NR>1 && $2!=ssid {print $1}')
  for id in $ids; do A cmd wifi forget-network "$id" >/dev/null 2>&1; done
  A cmd wifi connect-network "$SSID" wpa2 "$PSK" >/dev/null 2>&1
  say "wifi pinned to $SSID (others forgotten)"
}

foreground_app() {
  [ -z "$PKG" ] && return 0
  local resumed
  resumed=$(A dumpsys activity activities 2>/dev/null | tr -d '\r' | grep -m1 mResumedActivity || true)
  if ! echo "$resumed" | grep -q "$PKG"; then
    A monkey -p "$PKG" -c android.intent.category.LAUNCHER 1 >/dev/null 2>&1
    say "re-launched $PKG (was: ${resumed:-unknown})"
    return 1
  fi
  return 0
}

apply_keepawake
case " $* " in *" --pin-wifi "*) pin_wifi;; esac
foreground_app
say "one-shot hardening applied"

case " $* " in *" --watch "*) WATCH=1;; *) WATCH=0;; esac
if [ "$WATCH" = 1 ]; then
  say "watch mode: re-asserting every 10 s (Ctrl+C to stop)"
  while true; do
    state=$(A dumpsys power 2>/dev/null | tr -d '\r' | grep -m1 mWakefulness= || true)
    case "$state" in
      *Asleep*|*Dozing*)
        A input keyevent KEYCODE_WAKEUP >/dev/null 2>&1
        say "headset was asleep -> WAKEUP sent"
        apply_keepawake
        ;;
    esac
    foreground_app || true
    sleep 10
  done
fi
