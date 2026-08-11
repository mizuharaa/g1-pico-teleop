#!/usr/bin/env python
"""Software e-stop for the G1: put every joint into damping over DDS.

This is the SECONDARY failsafe. The PRIMARY failsafe is, and stays, the
human spotter with the Unitree remote (L2+B = damp — same combo as the
g1 dance project; works at firmware level independent of any software).

Two ways to use:
  python g1_estop.py --iface eth0            # panic console: press ENTER to damp
  python g1_estop.py --iface eth0 --now      # damp immediately and exit
                                             #   (this is what the watchdog's
                                             #    --on-alarm hook should call)

Mechanism: unitree_sdk2py LocoClient.Damp() — the same call the remote's
damp combo triggers. Requires unitree_sdk2py (present in the HoloMotion
container on the Orin; on the laptop needs CycloneDDS built).

NOTE: damping DROPS the robot. On the gantry that is safe and is exactly
what you want. Free-standing, a damped robot collapses — the gantry rule
exists for this reason.
"""
from __future__ import annotations

import argparse
import sys
import time


def make_client(iface: str):
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    except ImportError:
        sys.exit(
            "unitree_sdk2py not available. Run this on the Orin (the HoloMotion\n"
            "container ships it), or install it on the laptop with CycloneDDS."
        )
    ChannelFactoryInitialize(0, iface)
    client = LocoClient()
    client.SetTimeout(2.0)
    client.Init()
    return client


def damp(client) -> None:
    t0 = time.time()
    ret = client.Damp()
    dt_ms = (time.time() - t0) * 1000
    if ret in (0, None):
        print(f"DAMP delivered ({dt_ms:.0f} ms). Joints damping. Keep the remote in hand.")
    else:
        print(f"DAMP request FAILED (code {ret}, {dt_ms:.0f} ms) — "
              f"USE THE REMOTE (L2+B) NOW.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="eth0", help="DDS network interface")
    ap.add_argument("--now", action="store_true", help="damp immediately and exit")
    args = ap.parse_args()

    client = make_client(args.iface)
    if args.now:
        damp(client)
        return

    print("=" * 60)
    print(" G1 SOFTWARE E-STOP ARMED")
    print(" Press ENTER to DAMP ALL JOINTS. Ctrl+C to disarm.")
    print(" (The Unitree remote L2+B does the same thing without this")
    print("  script and works even if this machine dies — keep it close.)")
    print("=" * 60)
    try:
        input()
    except KeyboardInterrupt:
        print("\ndisarmed, no action sent")
        return
    damp(client)


if __name__ == "__main__":
    main()
