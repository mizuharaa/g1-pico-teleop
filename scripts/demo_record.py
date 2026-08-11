#!/usr/bin/env python
"""Dual-camera demo recorder — G1 sessions.

Records BOTH cameras at 1080p30 simultaneously to timestamped MP4s:
  cam A: RealSense D435i RGB  (/dev/video4, YUYV)  — main/wide shot
  cam B: laptop webcam        (/dev/video6, MJPG)  — second angle
                              (--webcam-res 2560x1440 for extra sharpness)

FOV strategy: point the two cameras at the scene from DIFFERENT angles
(e.g. RealSense wide front shot, laptop 45-degree side shot). Together they
cover far more than one lens can; per-camera files let you cut between
angles in an editor, and --preview shows both side by side while filming.

Output: ~/Videos/g1-demos/<stamp>_realsense.mp4 + <stamp>_webcam.mp4
Stop: q in the preview window, or Ctrl+C.

NB: /dev/video4 is shared with live_cam.py — use demo_record.sh, which
parks the viewer during recording and restores it after.
"""
import argparse
import os
import signal
import sys
import threading
import time

import cv2

OUT_DIR = os.path.expanduser("~/Videos/g1-demos")


class CamRecorder(threading.Thread):
    def __init__(self, dev, label, width, height, fps, fourcc_in, out_path):
        super().__init__(daemon=True)
        self.dev, self.label = dev, label
        self.width, self.height, self.fps = width, height, fps
        self.fourcc_in = fourcc_in
        self.out_path = out_path
        self.latest = None
        self.frames = 0
        self.stop_flag = threading.Event()
        self.ok = False

    def run(self):
        # CAP_V4L2 explicitly — the default backend ignores the resolution
        # request and silently falls back to 640x480
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        if self.fourcc_in:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc_in))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if not cap.isOpened():
            print(f"[{self.label}] cannot open {self.dev}", flush=True)
            return
        writer = cv2.VideoWriter(
            self.out_path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.fps,
            (w, h),
        )
        print(f"[{self.label}] recording {w}x{h}@{self.fps} -> {self.out_path}",
              flush=True)
        self.ok = True
        while not self.stop_flag.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            writer.write(frame)
            self.latest = frame
            self.frames += 1
        cap.release()
        writer.release()
        print(f"[{self.label}] saved {self.frames} frames -> {self.out_path}",
              flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--webcam-res", default="1920x1080",
                    help="webcam capture size, e.g. 2560x1440")
    ap.add_argument("--preview", action="store_true",
                    help="show a live side-by-side preview window")
    ap.add_argument("--duration", type=float, default=0,
                    help="stop automatically after N seconds (0 = manual)")
    ap.add_argument("--no-webcam", action="store_true",
                    help="record the RealSense only")
    ap.add_argument("--smooth", action="store_true",
                    help="RealSense at 848x480@60 (smooth fast motion) "
                         "instead of 1280x720@30; webcam stays 1080p30")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    ww, wh = (int(x) for x in args.webcam_res.split("x"))

    # RealSense default 720p30: at YUYV 1080p it only sustains ~16 fps
    # (USB/encode bound) and the fixed-30fps container would play fast.
    # --smooth: 848x480@60, its highest 60 fps mode (webcam has none).
    rs_w, rs_h, rs_fps = (848, 480, 60) if args.smooth else (1280, 720, 30)
    cams = [
        CamRecorder("/dev/video4", "realsense", rs_w, rs_h, rs_fps, None,
                    f"{OUT_DIR}/{stamp}_realsense.mp4"),
    ]
    if not args.no_webcam:
        cams.append(
            CamRecorder("/dev/video6", "webcam", ww, wh, 30, "MJPG",
                        f"{OUT_DIR}/{stamp}_webcam.mp4"))
    for c in cams:
        c.start()

    def stop(*_):
        for c in cams:
            c.stop_flag.set()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    t0 = time.time()
    last_snap = 0.0
    try:
        while any(c.is_alive() for c in cams):
            if args.duration and time.time() - t0 > args.duration:
                stop()
            # keep the session monitor's rolling snapshot alive while this
            # process owns the camera (live_cam is parked during recording)
            now = time.time()
            if now - last_snap > 10 and cams[0].latest is not None:
                last_snap = now
                mon = "/tmp/session_monitor"
                os.makedirs(mon, exist_ok=True)
                cv2.imwrite(f"{mon}/latest.jpg", cams[0].latest)
                cv2.imwrite(f"{mon}/snap_{time.strftime('%H%M%S')}.jpg",
                            cams[0].latest)
            if args.preview:
                tiles = []
                for c in cams:
                    if c.latest is not None:
                        tiles.append(cv2.resize(c.latest, (640, 360)))
                if tiles:
                    view = cv2.hconcat(tiles) if len(tiles) > 1 else tiles[0]
                    cv2.imshow("DEMO REC (q stops)", view)
                    if cv2.waitKey(30) & 0xFF == ord("q"):
                        stop()
            else:
                time.sleep(0.2)
            if all(c.stop_flag.is_set() for c in cams):
                for c in cams:
                    c.join(timeout=3)
                break
    finally:
        stop()
        for c in cams:
            c.join(timeout=3)
        cv2.destroyAllWindows()
    print("done.", flush=True)


if __name__ == "__main__":
    main()
