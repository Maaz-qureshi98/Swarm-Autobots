"""Overhead-camera ground truth for the hardware trials.

Put four ArUco markers (DICT_4X4_50, ids 40-43) flat on the floor at known positions
(the arena corners), and one marker on top of each robot (ids 0-3 = Alpha, Beta-1..3,
marker centred on the robot, marker x-axis pointing forward). Record the trial from above.

    python3 tools/aruco_tracker.py trial.mp4 --corners 0,0 2.44,0 2.44,2.44 0,2.44 --out trial.csv

Output CSV: t, robot, x, y, theta (metres, radians, arena frame). The floor markers fix a
homography, so the camera need not be calibrated or exactly overhead; robot markers must
sit at a known height h above the floor (--marker-height), which is corrected for
using the camera height (--camera-height).
"""
import argparse, csv
import numpy as np
import cv2

FLOOR_IDS = (40, 41, 42, 43)


def detector():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(d, p)


def track(video, corners_m, cam_h=None, marker_h=0.0, every=1):
    det = detector()
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    Hm = None
    rows, k = [], -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        k += 1
        if k % every:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        cs, ids, _ = det.detectMarkers(gray)
        if ids is None:
            continue
        ids = ids.ravel()
        cen = {int(i): c.reshape(4, 2) for i, c in zip(ids, cs)}
        if all(f in cen for f in FLOOR_IDS):      # refresh homography whenever all floor markers are seen
            src = np.array([cen[f].mean(0) for f in FLOOR_IDS], np.float32)
            Hm, _ = cv2.findHomography(src, np.array(corners_m, np.float32))
        if Hm is None:
            continue
        for rid in range(20):
            if rid not in cen:
                continue
            q = cv2.perspectiveTransform(cen[rid][None].astype(np.float32), Hm)[0]   # 4 corners on the floor plane
            c = q.mean(0)
            fwd = (q[1] + q[2]) / 2 - (q[0] + q[3]) / 2                              # marker x-axis
            if cam_h and marker_h:                                                  # parallax: marker above floor
                cxy = np.array(corners_m, float).mean(0)                            # assume camera above arena centre
                c = cxy + (c - cxy) * (1 - marker_h / cam_h)
            rows.append((k / fps, rid, float(c[0]), float(c[1]), float(np.arctan2(fwd[1], fwd[0]))))
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--corners", nargs=4, required=True, help="x,y of floor markers 40..43 in metres")
    ap.add_argument("--camera-height", type=float, default=None)
    ap.add_argument("--marker-height", type=float, default=0.0)
    ap.add_argument("--every", type=int, default=1, help="process every n-th frame")
    ap.add_argument("--out", default="track.csv")
    a = ap.parse_args()
    corners = [tuple(map(float, s.split(","))) for s in a.corners]
    rows = track(a.video, corners, a.camera_height, a.marker_height, a.every)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "robot", "x", "y", "theta"]); w.writerows(rows)
    print(f"{len(rows)} poses written to {a.out}")
