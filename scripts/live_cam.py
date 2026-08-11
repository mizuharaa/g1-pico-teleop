#!/usr/bin/env python
"""Live external-camera window for session monitoring.

Shows the RealSense D435i RGB stream (/dev/video4) at ~15 fps with a
timestamp overlay. Close the window or Ctrl+C to stop; the rolling
snapshots from monitor_session.sh are independent of this viewer.
"""
import sys
import time

import cv2

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/video4"
MON = "/tmp/session_monitor"
SNAP_EVERY_S = 10.0
SNAP_KEEP = 360   # 1 h of history — the 10-min window pruned the 14:27 fall

# 848x480@60 — the highest 60 fps mode the D435i RGB offers; explicit
# CAP_V4L2 or the resolution/fps request is silently ignored
cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)
if not cap.isOpened():
    sys.exit(f"cannot open {DEV}")
print(f"[live_cam] streaming {DEV} — close window or Ctrl+C to stop", flush=True)

import os
os.makedirs(MON, exist_ok=True)
last_snap = 0.0


def save_snapshot(frame) -> None:
    """This process owns the camera, so it also writes the rolling
    snapshots (UVC devices are single-consumer)."""
    global last_snap
    now = time.time()
    if now - last_snap < SNAP_EVERY_S:
        return
    last_snap = now
    ts = time.strftime("%H%M%S")
    cv2.imwrite(f"{MON}/snap_{ts}.jpg", frame)
    cv2.imwrite(f"{MON}/latest.jpg", frame)
    snaps = sorted(
        (f for f in os.listdir(MON) if f.startswith("snap_")), reverse=True
    )
    for old in snaps[SNAP_KEEP:]:
        try:
            os.remove(os.path.join(MON, old))
        except OSError:
            pass


try:
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.2)
            continue
        save_snapshot(frame)
        cv2.putText(
            frame,
            time.strftime("%H:%M:%S"),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.imshow("EXTERNAL CAM (G1 session)", frame)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
