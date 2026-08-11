#!/usr/bin/env bash
# Turn the laptop into the teleop Wi-Fi access point.
# The PICO (and later the robot's Jetson) join THIS hotspot — no external router.
#
#   sudo bash ~/full-body-teleoperation/scripts/laptop_hotspot.sh          # start (5 GHz, falls back to 2.4)
#   sudo bash ~/full-body-teleoperation/scripts/laptop_hotspot.sh stop     # stop, rejoin normal Wi-Fi
#
# NOTE: the laptop has ONE Wi-Fi adapter, so while the hotspot is up the laptop
# has no internet over Wi-Fi. The robot Ethernet link (192.168.123.2) is unaffected.
set -uo pipefail

DEV=${TELEOP_WIFI_DEV:-wlxd037457570db}
CON=g1-teleop-ap
SSID=${TELEOP_SSID:-g1-teleop}
PSK=${TELEOP_PSK:-teleop12345}

if [ "${1:-start}" = "stop" ]; then
  nmcli con down "$CON" 2>/dev/null
  echo "Hotspot down. Rejoining your normal Wi-Fi..."
  nmcli device connect "$DEV"
  exit 0
fi

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo bash $0"; exit 1
fi

bring_up() {  # $1 = band (a = 5 GHz, bg = 2.4 GHz)
  nmcli con delete "$CON" >/dev/null 2>&1
  nmcli con add type wifi ifname "$DEV" con-name "$CON" autoconnect no ssid "$SSID" \
    802-11-wireless.mode ap \
    802-11-wireless.band "$1" \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.group ccmp \
    802-11-wireless-security.psk "$PSK" \
    ipv4.method shared \
    ipv6.method ignore >/dev/null || return 1
  nmcli con up "$CON" >/dev/null 2>&1
}

echo "== Starting hotspot '$SSID' on $DEV (5 GHz) =="
if ! bring_up a; then
  echo "   5 GHz refused by the driver — falling back to 2.4 GHz (higher latency)."
  bring_up bg || { echo "ERROR: could not start the hotspot at all."; exit 1; }
fi

sleep 3
IP=$(ip -4 -o addr show "$DEV" | awk '{print $4}' | cut -d/ -f1)
if [ -z "$IP" ]; then
  echo "ERROR: hotspot has no IP — it did not come up. Check: nmcli con up $CON"
  exit 1
fi

BAND=$(nmcli -g 802-11-wireless.band con show "$CON")
echo
echo "=================================================="
echo " HOTSPOT UP"
echo "   SSID     : $SSID"
echo "   Password : $PSK"
echo "   Band     : $BAND  (a = 5 GHz, bg = 2.4 GHz)"
echo
echo "   >>> In the PICO XRoboToolkit app, PC Service = $IP <<<"
echo "=================================================="
echo
echo "Firewall check (the headset must reach TCP 63901):"
if command -v ufw >/dev/null && ufw status 2>/dev/null | grep -q "Status: active"; then
  echo "  ufw is ACTIVE — opening 63901 on $DEV"
  ufw allow in on "$DEV" to any port 63901 proto tcp
else
  echo "  ufw inactive — nothing to open."
fi
