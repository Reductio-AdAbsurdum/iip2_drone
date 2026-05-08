#!/usr/bin/env python3
"""
UWB tag debug plotter.

Reads the ESP32 tag's USB serial output, parses the raw "[uwb] AT+RANGE=..."
frames, runs the same trilateration as temp.ino, and shows a live top-down
map of the 4 anchors plus the tag's position.

Use this to:
  - Iterate on ANCHOR_XY without reflashing the ESP32.
  - See raw single-frame position vs smoothed -- diagnoses noise vs geometry.
  - Verify your anchor placements match reality (stand at a known spot, see
    if the plot matches).

Setup:
    pip install pyserial matplotlib numpy

Find your serial port:
    ls /dev/cu.usbmodem*       # mac
    ls /dev/ttyUSB* /dev/ttyACM*  # linux

Run:
    python3 uwb_plot.py /dev/cu.usbmodem1101
    (replace with your actual port)

Note: close the Arduino IDE Serial Monitor first -- only one program can
open the USB serial port at a time.
"""

import sys
import re
import collections
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

# ============================================================================
# CONFIG -- edit these to match your setup, then run.
# ============================================================================

# Anchor positions in NED frame (meters). Order MUST match physical UWB_INDEX.
# Tune these here without reflashing -- once the plot matches reality,
# copy the same values into temp.ino's ANCHOR_XY.
ANCHOR_XY = np.array(
    [
        [0.00, 0.00],  # Anchor 0   (X = North, Y = East)
        [2.90, 0.00],  # Anchor 1
        [2.90, 3.00],  # Anchor 2
        [0.00, 3.00],  # Anchor 3
    ],
    dtype=float,
)

# Same as TAG_HEIGHT_M in temp.ino -- vertical separation tag<->anchors.
TAG_HEIGHT_M = 1.0

# Filter knobs (mirror temp.ino so what you see here matches the FC).
DIST_FILT_N = 5
POS_EMA_ALPHA = 0.3

# Number of smoothed positions to keep in the trail.
TRAIL_LEN = 80

# Plot padding around the anchor area (meters).
PLOT_PAD = 2.0

# Serial baud (matches SERIAL_LOG.begin in temp.ino).
SERIAL_BAUD = 115200

# ============================================================================


# Regex pulls out the mask and the (d0,d1,d2,d3,...) tuple from any line
# that contains "AT+RANGE...mask:XX...range:(...)".
RANGE_RE = re.compile(r"AT\+RANGE.*?mask:([0-9A-Fa-f]+).*?range:\(([0-9,\-]+)\)")


def parse_range_line(line):
    """Returns (mask:int, distances_m:list[float]) or None if line isn't a range frame."""
    m = RANGE_RE.search(line)
    if not m:
        return None
    try:
        mask = int(m.group(1), 16)
        cm_vals = [int(x.strip()) for x in m.group(2).split(",")[:4]]
    except ValueError:
        return None
    return mask, [v / 100.0 for v in cm_vals]


def trilaterate_2d(distances, anchor_xy=ANCHOR_XY, h=TAG_HEIGHT_M):
    """
    Same 2D trilateration math as temp.ino. Returns (x_n, y_e) or None
    on singular geometry (e.g. collinear anchors).
    """
    # Slant -> horizontal correction.
    dh = np.zeros(4)
    h_sq = h * h
    for i in range(4):
        d_sq = distances[i] ** 2 - h_sq
        dh[i] = np.sqrt(d_sq) if d_sq > 0 else 0.0

    A = np.zeros((3, 2))
    b = np.zeros(3)
    A0_sq = anchor_xy[0, 0] ** 2 + anchor_xy[0, 1] ** 2
    dh0_sq = dh[0] ** 2
    for i in range(3):
        ai = i + 1
        A[i, 0] = 2.0 * (anchor_xy[ai, 0] - anchor_xy[0, 0])
        A[i, 1] = 2.0 * (anchor_xy[ai, 1] - anchor_xy[0, 1])
        Ai_sq = anchor_xy[ai, 0] ** 2 + anchor_xy[ai, 1] ** 2
        b[i] = dh0_sq - dh[ai] ** 2 + Ai_sq - A0_sq

    ATA = A.T @ A
    ATb = A.T @ b
    det = np.linalg.det(ATA)
    if abs(det) < 1e-6:
        return None
    x = np.linalg.solve(ATA, ATb)
    return float(x[0]), float(x[1])


class UWBState:
    """Holds latest distances + filter state, mirrors what runs on the ESP32."""

    def __init__(self):
        self.dist_buf = collections.deque(maxlen=DIST_FILT_N)
        self.last_mask = 0
        self.last_distances = [0.0, 0.0, 0.0, 0.0]
        self.last_raw_xy = None  # single-frame trilateration (no filter)
        self.last_smooth_xy = None  # median-distance + EMA-position
        self.trail = collections.deque(maxlen=TRAIL_LEN)
        self.frames_seen = 0

    def update(self, mask, distances_m):
        self.last_mask = mask
        self.last_distances = distances_m
        self.frames_seen += 1

        if mask != 0x0F:
            return  # Need all 4 anchors heard for a fix.

        # Single-frame raw trilateration (no smoothing).
        raw = trilaterate_2d(distances_m)
        if raw is not None:
            self.last_raw_xy = raw

        # Median over the last DIST_FILT_N frames, then EMA on position.
        self.dist_buf.append(distances_m)
        if len(self.dist_buf) < DIST_FILT_N:
            return
        arr = np.array(self.dist_buf)
        med = np.median(arr, axis=0).tolist()
        xy = trilaterate_2d(med)
        if xy is None:
            return
        if self.last_smooth_xy is None:
            self.last_smooth_xy = xy
        else:
            a = POS_EMA_ALPHA
            self.last_smooth_xy = (
                a * xy[0] + (1 - a) * self.last_smooth_xy[0],
                a * xy[1] + (1 - a) * self.last_smooth_xy[1],
            )
        self.trail.append(self.last_smooth_xy)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port>")
        print("Find ports with:  ls /dev/cu.usbmodem*")
        sys.exit(1)
    port = sys.argv[1]

    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        print("Hint: close the Arduino IDE Serial Monitor first.")
        sys.exit(1)

    print(f"Listening on {port} at {SERIAL_BAUD} baud. Ctrl+C to quit.")
    state = UWBState()

    # --- Plot setup ---
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal")

    # Anchors as fixed blue squares (note: plot uses Y=East horizontal, X=North vertical).
    ax.scatter(
        ANCHOR_XY[:, 1],
        ANCHOR_XY[:, 0],
        s=240,
        marker="s",
        c="steelblue",
        zorder=3,
        label="Anchors",
    )
    for i, (n, e) in enumerate(ANCHOR_XY):
        ax.annotate(
            f"A{i}",
            (e, n),
            textcoords="offset points",
            xytext=(12, 8),
            fontsize=14,
            fontweight="bold",
            color="steelblue",
        )

    # Tag artists.
    (trail_line,) = ax.plot(
        [],
        [],
        "-",
        color="crimson",
        alpha=0.35,
        linewidth=1.2,
        label="Trail (smoothed)",
    )
    (raw_dot,) = ax.plot(
        [],
        [],
        "+",
        color="crimson",
        markersize=14,
        markeredgewidth=2,
        label="Raw (single frame)",
    )
    (smooth_dot,) = ax.plot(
        [], [], "o", color="forestgreen", markersize=14, label="Smoothed (sent to FC)"
    )

    # Info text panel.
    info_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        fontfamily="monospace",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.7),
    )

    # Bounds.
    xs = ANCHOR_XY[:, 1]  # East -> horizontal
    ys = ANCHOR_XY[:, 0]  # North -> vertical
    ax.set_xlim(xs.min() - PLOT_PAD, xs.max() + PLOT_PAD)
    ax.set_ylim(ys.min() - PLOT_PAD, ys.max() + PLOT_PAD)
    ax.set_xlabel("East  (Y, m)")
    ax.set_ylabel("North (X, m)")
    ax.set_title("UWB tag — live debug plot")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    def drain_serial():
        # Read whatever's queued, parse range frames.
        while ser.in_waiting:
            try:
                raw = ser.readline()
            except Exception:
                break
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace")
            if "AT+RANGE" not in line:
                continue
            parsed = parse_range_line(line)
            if parsed is None:
                continue
            mask, distances = parsed
            state.update(mask, distances)

    def animate(_frame):
        drain_serial()

        if state.trail:
            tarr = np.array(state.trail)
            trail_line.set_data(tarr[:, 1], tarr[:, 0])

        if state.last_raw_xy is not None:
            raw_dot.set_data([state.last_raw_xy[1]], [state.last_raw_xy[0]])

        if state.last_smooth_xy is not None:
            smooth_dot.set_data([state.last_smooth_xy[1]], [state.last_smooth_xy[0]])

        d = state.last_distances
        mask_bin = format(state.last_mask, "04b")
        smooth = state.last_smooth_xy or (float("nan"), float("nan"))
        raw = state.last_raw_xy or (float("nan"), float("nan"))
        info = (
            f"frames: {state.frames_seen}\n"
            f"mask:   0x{state.last_mask:02X} ({mask_bin})\n"
            f"d0={d[0]:5.2f} m   d1={d[1]:5.2f} m\n"
            f"d2={d[2]:5.2f} m   d3={d[3]:5.2f} m\n"
            f"raw:    N={raw[0]:6.2f}  E={raw[1]:6.2f}\n"
            f"smooth: N={smooth[0]:6.2f}  E={smooth[1]:6.2f}"
        )
        info_text.set_text(info)
        return trail_line, raw_dot, smooth_dot, info_text

    ani = animation.FuncAnimation(
        fig, animate, interval=50, blit=False, cache_frame_data=False
    )
    try:
        plt.show()
    finally:
        ser.close()


if __name__ == "__main__":
    main()
