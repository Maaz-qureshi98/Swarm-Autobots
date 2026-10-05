"""Fit the Gilbert-Elliott loss model from a receiver log of espnow_loss_test.ino.

    python3 analyze_loss.py receiver_log.csv          # lines "seq,rssi,millis"

Prints the mean loss rate p, the mean burst length L, the burst-length histogram, and the
mean RSSI, i.e. the channel parameters used in the paper's simulator."""
import sys
import numpy as np


def main(fn):
    seq = []
    for line in open(fn):
        parts = line.strip().split(",")
        if len(parts) == 3 and parts[0].isdigit():
            seq.append((int(parts[0]), int(parts[1])))
    if len(seq) < 10:
        print("not enough frames"); return
    s = np.array([q for q, _ in seq]); rssi = np.array([r for _, r in seq])
    s = np.concatenate([[s[0]], s[0] + np.cumsum((np.diff(s) + 65536) % 65536)])   # unwrap uint16
    got = np.zeros(s[-1] - s[0] + 1, bool); got[s - s[0]] = True
    lost = ~got
    p = lost.mean()
    bursts, run = [], 0
    for x in lost:
        if x:
            run += 1
        elif run:
            bursts.append(run); run = 0
    if run:
        bursts.append(run)
    L = float(np.mean(bursts)) if bursts else 0.0
    print(f"frames sent {len(got)}, received {got.sum()}, loss rate p = {p:.3f}, mean burst L = {L:.2f}, "
          f"mean RSSI {rssi.mean():.1f} dBm")
    if bursts:
        h = np.bincount(bursts)
        print("burst length histogram:", {k: int(v) for k, v in enumerate(h) if v})


if __name__ == "__main__":
    main(sys.argv[1])
