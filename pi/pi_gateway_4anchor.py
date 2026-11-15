#!/usr/bin/env python3
"""
Anchor line in:  R,<sweep>,<anchor>,<range_m>,<rx_power_dbm>
Position out:    OSC /tag/position [x, y, z] in cm
Touch in:        OSC /tag/touch [id, strength 0..1] on UDP 4210, sent to every anchor as "C,<id>,<0..255>"

Install:  pip install pyserial python-osc
Run:      python3 pi_gateway_4anchor.py                    (all /dev/ttyACM* ports)
          python3 pi_gateway_4anchor.py --dry-run          (print positions, send no position OSC)
"""

import argparse
import glob
import math
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
MEDIAN_WINDOW = 3        # ranges per anchor in the spike filter
POSITION_ALPHA = 0.4     # 1.0 = no smoothing, smaller = smoother but laggier
SWEEP_TIMEOUT_S = 0.5    # an incomplete sweep older than this is discarded
RESET_AFTER_S = 1.0      # restart smoothing from a fresh fix after this long without one

# Unreal OSC receiver (placeholders)
OSC_HOST = "127.0.0.1"
OSC_PORT = 8000
OSC_ADDRESS = "/tag/position"
UNREAL_SCALE = 100.0     # convert to centimeters

# touch commands from Unreal, forwarded to every anchor over USB serial
TOUCH_PORT = 4210
TOUCH_ADDRESS = "/tag/touch"   # [id: int, strength: float 0..1]
CMD_RESEND_S = 0.1       # resend the latest command this often (anchors drop it after 0.5 s of silence)
CMD_TIMEOUT_S = 0.5      # nothing from Unreal this long = send id 0

BAUD = 115200
LINE = re.compile(r"^R,(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+)$")


def det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def solve_position(ranges):
    """{anchor: meters} -> (x, y, z) or None. Linearized against anchor 1, Cramer's rule."""
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


def residual_rms(ranges, p):
    """Gap between each range and the distance from p to that anchor."""
    errs = [math.dist(p, ANCHOR_POS[i]) - ranges[i] for i in ANCHOR_POS]
    return math.sqrt(sum(e * e for e in errs) / len(errs))


class Tracker:
    """Collects anchor ranges into sweeps and turns complete sweeps into positions."""

    def __init__(self):
        self.sweeps = {}  # sweep id -> {"t": first-seen time, "ranges": {anchor: meters}}
        self.history = {a: deque(maxlen=MEDIAN_WINDOW) for a in ANCHOR_POS}
        self.position = None
        self.position_time = 0.0

    def add(self, now, sweep, anchor, rng):
        """One anchor line in; (x, y, z, residual) out when it completes a valid sweep."""
        if anchor not in ANCHOR_POS:
            return None
        self.sweeps = {s: e for s, e in self.sweeps.items() if now - e["t"] <= SWEEP_TIMEOUT_S}
        entry = self.sweeps.setdefault(sweep, {"t": now, "ranges": {}})
        entry["ranges"][anchor] = rng
        if len(entry["ranges"]) < len(ANCHOR_POS):
            return None
        del self.sweeps[sweep]

        if any(not 0.0 <= r <= MAX_VALID_RANGE_M for r in entry["ranges"].values()):
            return None
        filtered = {}
        for a, r in entry["ranges"].items():
            self.history[a].append(r)
            filtered[a] = statistics.median(self.history[a])
        p = solve_position(filtered)
        if p is None:
            return None

        if self.position is None or now - self.position_time > RESET_AFTER_S:
            self.position = p
        else:
            self.position = tuple(POSITION_ALPHA * n + (1.0 - POSITION_ALPHA) * o
                                  for n, o in zip(p, self.position))
        self.position_time = now
        return self.position + (residual_rms(filtered, p),)


open_ports = {}          # path -> open serial port, shared so commands can be written back
port_lock = threading.Lock()
touch = {"cmd": (0, 0, 0.0)}  # latest (id, strength 0..255, time) from Unreal


def read_port(path, out):
    import serial  # pyserial
    try:
        with serial.Serial(path, BAUD, timeout=1) as port:
            open_ports[path] = port
            while True:
                text = port.readline().decode(errors="ignore").strip()
                m = LINE.match(text)
                if m:
                    out.put((time.monotonic(), int(m.group(1)), int(m.group(2)), float(m.group(3))))
    except Exception as e:  # unplugged cable, permissions, port busy
        print(f"[{path}] reader stopped: {e}")
    finally:
        open_ports.pop(path, None)


def on_touch(address, *args):
    """OSC /tag/touch [id, strength 0..1] -> latest command."""
    try:
        touch_id = int(args[0])
        strength = float(args[1]) if len(args) > 1 else 1.0
    except (IndexError, ValueError):
        return
    touch["cmd"] = (max(0, min(255, touch_id)), round(max(0.0, min(1.0, strength)) * 255), time.monotonic())


def send_commands():
    """Every CMD_RESEND_S send the latest command to all anchors; id 0 once Unreal goes quiet."""
    last = None
    while True:
        time.sleep(CMD_RESEND_S)
        touch_id, strength, t = touch["cmd"]
        cmd = (0, 0) if time.monotonic() - t > CMD_TIMEOUT_S else (touch_id, strength)
        if cmd != last:
            print(f"touch id,strength: {cmd[0]}, {cmd[1]}")
            last = cmd
        line = f"C,{cmd[0]},{cmd[1]}\n".encode()
        with port_lock:
            for path, port in list(open_ports.items()):
                try:
                    port.write(line)
                except Exception as e:
                    print(f"[{path}] write failed: {e}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ports", nargs="*", default=None, help="serial ports (default: all /dev/ttyACM*)")
    ap.add_argument("--osc-host", default=OSC_HOST)
    ap.add_argument("--osc-port", type=int, default=OSC_PORT)
    ap.add_argument("--touch-port", type=int, default=TOUCH_PORT, help="UDP port Unreal sends /tag/touch to")
    ap.add_argument("--dry-run", action="store_true", help="print positions, send no OSC")
    args = ap.parse_args()

    ports = args.ports or sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        raise SystemExit("No serial ports found. Plug in the anchors or pass --ports.")

    client = None
    if not args.dry_run:
        from pythonosc.udp_client import SimpleUDPClient
        client = SimpleUDPClient(args.osc_host, args.osc_port)

    lines = queue.Queue()
    for path in ports:
        threading.Thread(target=read_port, args=(path, lines), daemon=True).start()
    print(f"Reading {len(ports)} port(s): {', '.join(ports)}")

    from pythonosc.dispatcher import Dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
    dispatcher = Dispatcher()
    dispatcher.map(TOUCH_ADDRESS, on_touch)
    server = ThreadingOSCUDPServer(("0.0.0.0", args.touch_port), dispatcher)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=send_commands, daemon=True).start()
    print(f"Listening for {TOUCH_ADDRESS} on UDP {args.touch_port}")

    tracker = Tracker()
    while True:
        try:
            now, sweep, anchor, rng = lines.get(timeout=1.0)
        except queue.Empty:
            continue
        result = tracker.add(now, sweep, anchor, rng)
        if result is None:
            continue
        x, y, z, res = result
        print(f"x,y,z (m): {x:.3f}, {y:.3f}, {z:.3f}   residual {res:.3f}")
        if client:
            client.send_message(OSC_ADDRESS, [x * UNREAL_SCALE, y * UNREAL_SCALE, z * UNREAL_SCALE])


if __name__ == "__main__":
    main()
