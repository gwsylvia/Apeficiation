#!/usr/bin/env python3
"""
Anchor line in:  R,<sweep>,<anchor>,<range_m>,<rx_power_dbm>
Position out:    printed on screen, and sent to Unreal as OSC /tag/position [x, y, z] in cm when --osc-host is given
                 4 anchors answered: full position
                 3 anchors answered: position with the mirror side guessed (see --start)
                 fewer: nothing printed

Install:  pip install pyserial python-osc
Run:      python3 pi_position_unreal.py --osc-host 192.168.219.43     (send to the Unreal PC)
          python3 pi_position_unreal.py --osc-host 192.168.219.43 --start 1 1 1
"""

import argparse
import glob
import math
import os
import queue
import re
import statistics
import threading
import time
from collections import deque

# bench layout (m)
ANCHOR_POS = {
    1: (0.0, 0.0, 0.0),
    2: (0.0, 2.0, 0.0),
    3: (3.0, 0.0, 1.0),
    4: (3.0, 2.0, 3.0),
}

MAX_VALID_RANGE_M = 6.0
MIN_ANCHORS = 3          # fewer valid ranges than this gives no position
MEDIAN_WINDOW = 3        # ranges per anchor in the spike filter
POSITION_ALPHA = 0.4     # 1.0 = no smoothing, smaller = smoother but laggier
SWEEP_CLOSE_S = 0.2      # a sweep still open this long after its first line is closed with the anchors that answered
HEARD_WINDOW_S = 1.5     # an anchor with no line this long counts as silent
RESET_AFTER_S = 1.0      # restart smoothing from a fresh fix after this long without one

# Unreal OSC receiver
OSC_PORT = 8000
OSC_ADDRESS = "/tag/position"
UNREAL_SCALE = 100.0     # convert to centimeters

BAUD = 115200
LINE = re.compile(r"^R,(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+)$")


def det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def sub(a, b):
    return [a[k] - b[k] for k in range(3)]


def dot(a, b):
    return sum(a[k] * b[k] for k in range(3))


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def solve_position(ranges):
    """{anchor: meters} for all four anchors -> (x, y, z) or None. Linearized against anchor 1, Cramer's rule."""
    a0 = ANCHOR_POS[1]
    a0sq = sum(c * c for c in a0)
    rows, rhs = [], []
    for i in (2, 3, 4):
        ai = ANCHOR_POS[i]
        rows.append([2.0 * (ai[k] - a0[k]) for k in range(3)])
        rhs.append(sum(c * c for c in ai) - a0sq - ranges[i] ** 2 + ranges[1] ** 2)
    det = det3(rows)
    if abs(det) < 1e-6:
        return None
    p = []
    for col in range(3):
        m = [[rhs[r] if k == col else rows[r][k] for k in range(3)] for r in range(3)]
        p.append(det3(m) / det)
    return tuple(p)


def solve_three(ranges):
    """{anchor: meters} for exactly three anchors -> the two mirrored positions, or None if the anchors are in a line."""
    ids = sorted(ranges)
    p1, p2, p3 = (ANCHOR_POS[i] for i in ids)
    r1, r2, r3 = (ranges[i] for i in ids)
    d = math.dist(p1, p2)
    ex = [c / d for c in sub(p2, p1)]
    v = sub(p3, p1)
    ix = dot(ex, v)
    t = [v[k] - ix * ex[k] for k in range(3)]
    tn = math.sqrt(dot(t, t))
    if tn < 1e-6:
        return None
    ey = [c / tn for c in t]
    ez = cross(ex, ey)
    jy = dot(ey, v)
    x = (r1 ** 2 - r2 ** 2 + d ** 2) / (2.0 * d)
    y = (r1 ** 2 - r3 ** 2 + ix ** 2 + jy ** 2) / (2.0 * jy) - (ix / jy) * x
    z = math.sqrt(max(r1 ** 2 - x ** 2 - y ** 2, 0.0))  # ranges that do not meet: z = 0, residual shows it
    return [tuple(p1[k] + x * ex[k] + y * ey[k] + s * z * ez[k] for k in range(3)) for s in (1, -1)]


def residual_rms(ranges, p):
    """Gap between each range and the distance from p to that anchor."""
    errs = [math.dist(p, ANCHOR_POS[i]) - ranges[i] for i in ranges]
    return math.sqrt(sum(e * e for e in errs) / len(errs))


class Tracker:
    """Collects anchor ranges into sweeps and turns each sweep into a position."""

    def __init__(self, start):
        self.start = start  # where the tag roughly is at first, picks the mirror side with 3 anchors
        self.sweeps = {}  # sweep id -> {"t": first-seen time, "ranges": {anchor: meters}}
        self.history = {a: deque(maxlen=MEDIAN_WINDOW) for a in ANCHOR_POS}
        self.position = None
        self.position_time = 0.0

    def add(self, now, sweep, anchor, rng):
        """One anchor line in; a fix out when it completes a sweep of all anchors."""
        if anchor not in ANCHOR_POS:
            return None
        entry = self.sweeps.setdefault(sweep, {"t": now, "ranges": {}})
        entry["ranges"][anchor] = rng
        if len(entry["ranges"]) < len(ANCHOR_POS):
            return None
        del self.sweeps[sweep]
        return self.fix(now, entry["ranges"])

    def flush(self, now):
        """Close sweeps older than SWEEP_CLOSE_S, the anchors that answered may still give a position."""
        fixes = []
        for s in [s for s, e in self.sweeps.items() if now - e["t"] > SWEEP_CLOSE_S]:
            fix = self.fix(now, self.sweeps.pop(s)["ranges"])
            if fix:
                fixes.append(fix)
        return fixes

    def fix(self, now, ranges):
        """-> (x, y, z, residual, filtered ranges, mirror guessed) or None."""
        valid = {a: r for a, r in ranges.items() if 0.0 <= r <= MAX_VALID_RANGE_M}
        if len(valid) < MIN_ANCHORS:
            return None
        filtered = {}
        for a, r in valid.items():
            self.history[a].append(r)
            filtered[a] = statistics.median(self.history[a])

        guessed = len(filtered) < len(ANCHOR_POS)
        if guessed:
            candidates = solve_three(filtered)
            if candidates is None:
                return None
            hint = self.position if self.position and now - self.position_time <= RESET_AFTER_S else self.start
            p = min(candidates, key=lambda c: math.dist(c, hint))
        else:
            p = solve_position(filtered)
            if p is None:
                return None

        if self.position is None or now - self.position_time > RESET_AFTER_S:
            self.position = p
        else:
            self.position = tuple(POSITION_ALPHA * n + (1.0 - POSITION_ALPHA) * o
                                  for n, o in zip(p, self.position))
        self.position_time = now
        return self.position + (residual_rms(filtered, p), filtered, guessed)


active = set()  # ports with a reader running
last_error = {}  # port -> last message printed, so a port that stays down is not repeated every second


def read_port(path, out):
    import serial  # pyserial
    try:
        with serial.Serial(path, BAUD, timeout=1) as port:
            print(f"[{path}] reading")
            last_error.pop(path, None)
            while True:
                text = port.readline().decode(errors="ignore").strip()
                m = LINE.match(text)
                if m:
                    out.put((time.monotonic(), int(m.group(1)), int(m.group(2)), float(m.group(3))))
    except Exception as e:  # unplugged cable, permissions, port busy
        if last_error.get(path) != str(e):
            print(f"[{path}] lost: {e}")
            last_error[path] = str(e)
    finally:
        active.discard(path)


def watch_ports(paths, out):
    """Every second start a reader for each port that is present and not already being read."""
    while True:
        for path in (paths or sorted(glob.glob("/dev/ttyACM*"))):
            if path not in active and os.path.exists(path):
                active.add(path)
                threading.Thread(target=read_port, args=(path, out), daemon=True).start()
        time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ports", nargs="*", default=None, help="serial ports (default: every /dev/ttyACM* that appears)")
    ap.add_argument("--osc-host", default=None, help="Unreal PC address, nothing is sent without it")
    ap.add_argument("--osc-port", type=int, default=OSC_PORT)
    ap.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="rough tag position at the start in m, picks the mirror side when only 3 anchors answer")
    args = ap.parse_args()

    client = None
    if args.osc_host:
        from pythonosc.udp_client import SimpleUDPClient
        client = SimpleUDPClient(args.osc_host, args.osc_port)
        print(f"Sending {OSC_ADDRESS} to {args.osc_host}:{args.osc_port}")

    lines = queue.Queue()
    threading.Thread(target=watch_ports, args=(args.ports, lines), daemon=True).start()
    print("Watching for anchor ports, a port that drops and returns is picked up again")

    centroid = tuple(sum(p[k] for p in ANCHOR_POS.values()) / len(ANCHOR_POS) for k in range(3))
    tracker = Tracker(tuple(args.start) if args.start else centroid)
    last_seen = {}  # anchor -> time of its last line
    heard = None    # anchors heard in the last HEARD_WINDOW_S, printed when it changes
    while True:
        fixes = []
        try:
            now, sweep, anchor, rng = lines.get(timeout=0.05)
            last_seen[anchor] = now
            fix = tracker.add(now, sweep, anchor, rng)
            if fix:
                fixes.append(fix)
        except queue.Empty:
            pass
        fixes += tracker.flush(time.monotonic())
        now_heard = sorted(a for a, t in last_seen.items() if time.monotonic() - t < HEARD_WINDOW_S)
        if now_heard != heard:
            print(f"anchors heard: {now_heard}")
            heard = now_heard
        for x, y, z, res, ranges, guessed in fixes:
            print(f"x,y,z (m): {x:.3f}, {y:.3f}, {z:.3f}   residual {res:.3f}   ranges: "
                  + " ".join(f"{a}:{ranges[a]:.2f}" for a in sorted(ranges))
                  + ("   [3 anchors, mirror side guessed]" if guessed else ""))
            if client:
                client.send_message(OSC_ADDRESS, [x * UNREAL_SCALE, y * UNREAL_SCALE, z * UNREAL_SCALE])


if __name__ == "__main__":
    main()
