"""Self-test of the ground-truth pipeline on a synthetic video: markers are rendered at known
poses with a tilted (non-overhead) camera, then tracked and compared with the truth."""
import os, sys, tempfile
import numpy as np
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_tracker import track

D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
W, H, PX = 1600, 1200, 400            # canvas: 400 px per metre on the floor plane
ARENA = [(0.0, 0.0), (2.44, 0.0), (2.44, 2.44), (0.0, 2.44)]


def paste(canvas, mid, xy, th, size):
    img = cv2.aruco.generateImageMarker(D, mid, 200)
    img = cv2.copyMakeBorder(img, 25, 25, 25, 25, cv2.BORDER_CONSTANT, value=255)
    s = size * 250 / 200 * PX / 2
    c, si = np.cos(th), np.sin(th)
    # marker image corners: TL, TR, BR, BL; marker x-axis (TL->TR) points along heading
    loc = np.array([[-s, s], [s, s], [s, -s], [-s, -s]])
    R = np.array([[c, -si], [si, c]])
    pts = (loc @ R.T) + np.array(xy) * PX + 100
    pts[:, 1] = H - pts[:, 1]                 # image y down
    src = np.float32([[0, 0], [249, 0], [249, 249], [0, 249]])
    M = cv2.getPerspectiveTransform(src, np.float32(pts))
    warped = cv2.warpPerspective(img, M, (W, H), borderValue=0)
    mask = cv2.warpPerspective(np.full((250, 250), 255, np.uint8), M, (W, H))
    canvas[mask > 0] = warped[mask > 0]


def main():
    tilt = cv2.getPerspectiveTransform(np.float32([[0, 0], [W, 0], [W, H], [0, H]]),
                                       np.float32([[120, 60], [W - 40, 0], [W, H], [0, H - 80]]))
    fn = os.path.join(tempfile.mkdtemp(), "syn.avi")
    vw = cv2.VideoWriter(fn, cv2.VideoWriter_fourcc(*"MJPG"), 10, (W, H))
    truth = []
    for k in range(30):
        canvas = np.full((H, W), 200, np.uint8)
        for fid, xy in zip((40, 41, 42, 43), ARENA):
            paste(canvas, fid, xy, 0.0, 0.12)
        for rid in range(4):
            xy = (0.5 + 0.04 * k + 0.3 * rid, 0.6 + 0.35 * rid + 0.1 * np.sin(0.2 * k + rid))
            th = 0.3 * np.sin(0.1 * k + rid)
            paste(canvas, rid, xy, th, 0.10)
            truth.append((k, rid, *xy, th))
        vw.write(cv2.cvtColor(cv2.warpPerspective(canvas, tilt, (W, H), borderValue=200), cv2.COLOR_GRAY2BGR))
    vw.release()
    rows = track(fn, ARENA)
    T = {(k, r): (x, y, th) for k, r, x, y, th in truth}
    e = [np.hypot(x - T[(round(t * 10), r)][0], y - T[(round(t * 10), r)][1]) for t, r, x, y, th in rows]
    eth = [abs(np.arctan2(np.sin(th - T[(round(t * 10), r)][2]), np.cos(th - T[(round(t * 10), r)][2])))
           for t, r, x, y, th in rows]
    print(f"{len(rows)}/{len(truth)} poses tracked, position error mean {1000*np.mean(e):.1f} mm, "
          f"max {1000*np.max(e):.1f} mm, heading error max {np.degrees(np.max(eth)):.2f} deg")
    ok = len(rows) == len(truth) and np.max(e) < 0.01 and np.degrees(np.max(eth)) < 2.0
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
