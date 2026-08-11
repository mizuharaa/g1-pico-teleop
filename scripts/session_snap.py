#!/usr/bin/env python
"""Grab one frame from both session cameras into a single side-by-side JPG.

  operator cam: /dev/video6 (laptop webcam -> Alois)
  robot cam:    /dev/video2 (RealSense view -> G1 on the gantry)

Usage: python session_snap.py [out.jpg]
"""
import sys
import time

import cv2
import numpy as np

CAMS = [("/dev/video4", "EXTERNAL")]   # RealSense D435i RGB on the laptop
H = 480


def grab(dev: str):
    c = cv2.VideoCapture(dev)
    # a few warmup frames helps auto-exposure settle
    frame = None
    for _ in range(5):
        ok, f = c.read()
        if ok:
            frame = f
        time.sleep(0.05)
    c.release()
    return frame


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "session_snap.jpg"
    tiles = []
    for dev, label in CAMS:
        f = grab(dev)
        if f is None:
            f = np.zeros((H, int(H * 4 / 3), 3), np.uint8)
            cv2.putText(f, f"{label}: NO FRAME", (20, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            scale = H / f.shape[0]
            f = cv2.resize(f, (int(f.shape[1] * scale), H))
            cv2.putText(f, label, (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                        1.1, (0, 255, 0), 2)
        tiles.append(f)
    combo = np.hstack(tiles)
    ts = time.strftime("%H:%M:%S")
    cv2.putText(combo, ts, (combo.shape[1] - 130, H - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imwrite(out, combo)
    print(out)


if __name__ == "__main__":
    main()
