#!/usr/bin/env python3
"""
UWB tag debug plotter (UDP, wireless).

The ESP32 tag broadcasts a CSV line per UWB frame over UDP. This script
listens and shows:

  Top:    top-down map of the cage, with raw single-frame positions
          (red cloud + current +) and the filtered position actually sent
          to the FC (green dot + trail).

  Bottom: one time-series subplot per anchor showing raw vs filtered
          distance over the last ~10 s, with spike frames and NLOS
          frames marked.

Use it to:
  - See multipath / NLOS spikes (red cloud explodes; per-anchor traces
    show which anchor is misbehaving).
  - Sanity-check the median filter + velocity gate.
  - Verify anchor placements (walk the tag on a stick to a known spot
    and see if the plot matches reality).

Setup:
    pip install matplotlib numpy

Connection:
    1. Flash temp.ino on the ESP32 tag.
    2. Join the laptop's WiFi to the SoftAP "DroneTracker_UWB" (pw drone1234).
    3. python3 uwb_plot.py

CSV format emitted by the ESP32 (must match send_debug_udp() in temp.ino):
    UWB,t_ms,mask,fps,d0,d1,d2,d3,d0f,d1f,d2f,d3f,xr,yr,xs,ys,sent,
        std0,std1,std2,std3,spk
"""

import collections
import math
import socket
import sys

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

# ============================================================================
# CONFIG -- match temp.ino's ANCHOR_XY and the SoftAP UDP port.
# ============================================================================

UDP_PORT = 14550

# Anchor positions. X = North, Y = East. Order must match physical UWB_INDEX.
ANCHOR_XY = np.array(
    [
        [0.00, 0.00],  # Anchor 0
        [2.90, 0.00],  # Anchor 1
        [2.90, 3.00],  # Anchor 2
        [0.00, 3.00],  # Anchor 3
    ],
    dtype=float,
)

TRAIL_LEN = 100      # sent-to-FC positions in the trail
RAW_CLOUD_LEN = 80   # raw single-frame positions overlaid as a cloud
TIMESERIES_LEN = 120 # per-anchor samples kept for the time-series subplots
                     # (12 s at 10 Hz)
PLOT_PAD = 1.5

# Colors per anchor (consistent across position cloud + subplot lines).
ANCHOR_COLORS = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd"]

# ============================================================================

# Expected field count after splitting the CSV line on commas. See temp.ino.
# "UWB" + 21 values = 22 fields total.
EXPECTED_FIELDS = 22


def _f(x):
    try:
        return float(x)
    except ValueError:
        return float("nan")


def parse_uwb_line(line):
    """Returns dict with parsed fields, or None if the line isn't a UWB frame."""
    parts = line.strip().split(",")
    if len(parts) != EXPECTED_FIELDS or parts[0] != "UWB":
        return None
    try:
        return {
            "t_ms": int(parts[1]),
            "mask": int(parts[2]),
            "fps": _f(parts[3]),
            "d_raw": [_f(parts[4 + i]) for i in range(4)],
            "d_filt": [_f(parts[8 + i]) for i in range(4)],
            "raw_xy": (_f(parts[12]), _f(parts[13])),
            "sent_xy": (_f(parts[14]), _f(parts[15])),
            "sent": int(parts[16]),
            "std": [_f(parts[17 + i]) for i in range(4)],
            "spk": int(parts[21]),
        }
    except (ValueError, IndexError):
        return None


class UWBState:
    def __init__(self):
        self.last = None
        self.trail = collections.deque(maxlen=TRAIL_LEN)
        self.raw_cloud = collections.deque(maxlen=RAW_CLOUD_LEN)
        self.frames = 0
        self.sent_count = 0
        self.gate_reject_count = 0
        # Per-anchor rolling time series. Each entry is (t_s, d_raw, d_filt,
        # is_spike, was_heard). d_raw is NaN when not heard; d_filt is NaN
        # when the median filter wasn't computed this frame.
        self.anchor_ts = [collections.deque(maxlen=TIMESERIES_LEN) for _ in range(4)]
        # Cumulative spike counts per anchor for the info panel.
        self.spike_total = [0, 0, 0, 0]
        # Cumulative NLOS (mask-cleared) counts per anchor.
        self.nlos_total = [0, 0, 0, 0]
        self.t0_ms = None  # first packet timestamp, for relative time axis
        # Latest AT+GET response per key, populated from "MODINFO,..." UDP
        # lines the tag rebroadcasts every ~5 s.
        self.modinfo = {}
        # Latest position whose UDP packet had sent=1. Kept separately because
        # sent=1 happens at the VPE rate (~10 Hz) while UDP packets arrive at
        # the full UWB frame rate (~66 Hz). Without this, state.last["sent"]
        # would be false most of the time and the green dot would not update.
        self.last_sent_xy = (float("nan"), float("nan"))

    def update(self, pkt):
        self.last = pkt
        self.frames += 1
        if self.t0_ms is None:
            self.t0_ms = pkt["t_ms"]
        t_s = (pkt["t_ms"] - self.t0_ms) / 1000.0

        # Position scatter / trail.
        rx, ry = pkt["raw_xy"]
        if not math.isnan(rx) and not math.isnan(ry):
            self.raw_cloud.append((rx, ry))
        sx, sy = pkt["sent_xy"]
        if pkt["sent"]:
            self.sent_count += 1
            if not math.isnan(sx):
                self.trail.append((sx, sy))
                self.last_sent_xy = (sx, sy)
        else:
            if not math.isnan(sx):
                self.gate_reject_count += 1

        # Per-anchor time-series + cumulative counts.
        for i in range(4):
            heard = bool((pkt["mask"] >> i) & 1) and pkt["d_raw"][i] > 0.0
            is_spike = bool((pkt["spk"] >> i) & 1)
            d_raw = pkt["d_raw"][i] if heard else float("nan")
            d_filt = pkt["d_filt"][i]  # NaN if filter wasn't run this frame
            self.anchor_ts[i].append((t_s, d_raw, d_filt, is_spike, heard))
            if is_spike:
                self.spike_total[i] += 1
            if not heard:
                self.nlos_total[i] += 1


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", UDP_PORT))
    except OSError as e:
        print(f"Cannot bind UDP {UDP_PORT}: {e}")
        sys.exit(1)
    sock.setblocking(False)

    print(f"Listening on UDP 0.0.0.0:{UDP_PORT}")
    print("Connect this laptop to the ESP32 SoftAP 'DroneTracker_UWB' "
          "(pw drone1234) if you haven't already.")

    state = UWBState()

    # ---------- Plot setup ----------
    # Layout (gridspec):
    #   row 0: [ INFO | -------- position plot -------- | MODINFO ]
    #   row 1: [      | A0 | A1 | A2 | A3              |         ]
    # The info/modinfo panels live in their own side columns so they never
    # overlap the position plot or the per-anchor subplots.
    fig = plt.figure(figsize=(16, 10))
    gs = gridspec.GridSpec(
        2, 6,
        height_ratios=[3, 1],
        width_ratios=[1.8, 2.5, 2.5, 2.5, 2.5, 1.8],
        hspace=0.35, wspace=0.4, figure=fig,
    )
    ax_info = fig.add_subplot(gs[0, 0])      # left side panel
    ax_pos = fig.add_subplot(gs[0, 1:5])     # main position plot (4 center cols)
    ax_modinfo = fig.add_subplot(gs[0, 5])   # right side panel
    ax_anchors = [fig.add_subplot(gs[1, 1 + i]) for i in range(4)]

    # The side panels are just text canvases -- strip axis decoration.
    for ax in (ax_info, ax_modinfo):
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)

    # ---------- Position plot ----------
    ax_pos.set_aspect("equal")
    ax_pos.scatter(
        ANCHOR_XY[:, 1], ANCHOR_XY[:, 0],
        s=240, marker="s", c="steelblue", zorder=3, label="Anchors",
    )
    for i, (n, e) in enumerate(ANCHOR_XY):
        ax_pos.annotate(
            f"A{i}", (e, n),
            textcoords="offset points", xytext=(12, 8),
            fontsize=14, fontweight="bold", color="steelblue",
        )
    # Anchor box outline.
    box_n = list(ANCHOR_XY[:, 0]) + [ANCHOR_XY[0, 0]]
    box_e = list(ANCHOR_XY[:, 1]) + [ANCHOR_XY[0, 1]]
    ax_pos.plot(box_e, box_n, "--", color="steelblue", alpha=0.3, linewidth=1)

    (raw_cloud_dots,) = ax_pos.plot(
        [], [], ".", color="crimson", alpha=0.25, markersize=6,
        label=f"Raw (last {RAW_CLOUD_LEN})",
    )
    (raw_now,) = ax_pos.plot(
        [], [], "+", color="crimson", markersize=14, markeredgewidth=2,
        label="Raw (current frame)",
    )
    (trail_line,) = ax_pos.plot(
        [], [], "-", color="forestgreen", alpha=0.5, linewidth=1.4,
        label="Sent to FC (trail)",
    )
    (sent_now,) = ax_pos.plot(
        [], [], "o", color="forestgreen", markersize=14,
        label="Sent to FC (current)",
    )

    # Live stats panel: left side column (outside the position plot).
    info_text = ax_info.text(
        0.0, 1.0, "waiting for first UDP packet...",
        transform=ax_info.transAxes,
        horizontalalignment="left", verticalalignment="top",
        fontfamily="monospace", fontsize=8,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.75),
    )
    # Module info panel: right side column (outside the position plot).
    modinfo_text = ax_modinfo.text(
        0.0, 1.0, "waiting for MODINFO...",
        transform=ax_modinfo.transAxes,
        horizontalalignment="left", verticalalignment="top",
        fontfamily="monospace", fontsize=8,
        bbox=dict(boxstyle="round", facecolor="lightblue", alpha=0.75),
    )

    xs_e = ANCHOR_XY[:, 1]
    ys_n = ANCHOR_XY[:, 0]
    ax_pos.set_xlim(xs_e.min() - PLOT_PAD, xs_e.max() + PLOT_PAD)
    ax_pos.set_ylim(ys_n.min() - PLOT_PAD, ys_n.max() + PLOT_PAD)
    ax_pos.set_xlabel("East  (Y, m)")
    ax_pos.set_ylabel("North (X, m)")
    ax_pos.set_title("UWB tag — live debug (raw vs sent-to-FC)")
    ax_pos.grid(True, alpha=0.3)
    # Side panels are outside the position plot now -- top-right corner is free
    # again and is the conventional place for a legend.
    ax_pos.legend(loc="upper right", fontsize=9, framealpha=0.85)

    # ---------- Per-anchor time-series subplots ----------
    anchor_raw_lines = []
    anchor_filt_lines = []
    anchor_spike_dots = []  # spike markers on raw line
    anchor_nlos_dots = []   # NLOS markers along the bottom of each subplot
    for i, ax in enumerate(ax_anchors):
        color = ANCHOR_COLORS[i]
        (rline,) = ax.plot([], [], "-", color=color, alpha=0.55,
                           linewidth=1.0, label="raw")
        (fline,) = ax.plot([], [], "-", color="black", alpha=0.85,
                           linewidth=1.4, label="filt")
        (sdots,) = ax.plot([], [], "x", color="red", markersize=7,
                           markeredgewidth=1.6, label="spike")
        (ndots,) = ax.plot([], [], "|", color="gray", markersize=10,
                           markeredgewidth=1.0, label="NLOS")
        anchor_raw_lines.append(rline)
        anchor_filt_lines.append(fline)
        anchor_spike_dots.append(sdots)
        anchor_nlos_dots.append(ndots)
        ax.set_title(f"Anchor {i}", color=color, fontsize=10, fontweight="bold")
        ax.set_ylabel("dist (m)", fontsize=8)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc="upper left", fontsize=7, ncol=2)
    ax_anchors[0].set_xlabel("t (s)", fontsize=8)

    def drain_socket():
        while True:
            try:
                data, _addr = sock.recvfrom(2048)
            except (BlockingIOError, OSError):
                break
            if not data:
                break
            for line in data.decode("utf-8", errors="replace").splitlines():
                if line.startswith("MODINFO,"):
                    # Split only the first two commas so any commas inside the
                    # response value are preserved verbatim.
                    parts = line.strip().split(",", 2)
                    if len(parts) == 3:
                        state.modinfo[parts[1]] = parts[2]
                    continue
                pkt = parse_uwb_line(line)
                if pkt is not None:
                    state.update(pkt)

    def animate(_frame):
        drain_socket()

        # ---- Position plot ----
        if state.raw_cloud:
            arr = np.array(state.raw_cloud)
            raw_cloud_dots.set_data(arr[:, 1], arr[:, 0])
        if state.trail:
            tarr = np.array(state.trail)
            trail_line.set_data(tarr[:, 1], tarr[:, 0])
        if state.last is not None:
            rx, ry = state.last["raw_xy"]
            if not math.isnan(rx):
                raw_now.set_data([ry], [rx])
        # Green dot follows the latest sent-true position, not the latest UDP
        # packet (which is usually a sent=0 frame).
        sx, sy = state.last_sent_xy
        if not math.isnan(sx):
            sent_now.set_data([sy], [sx])

        # ---- Per-anchor subplots ----
        for i in range(4):
            ts = state.anchor_ts[i]
            if not ts:
                continue
            arr = np.array(ts, dtype=float)
            t_s = arr[:, 0]
            d_raw = arr[:, 1]
            d_filt = arr[:, 2]
            spk = arr[:, 3].astype(bool)
            heard = arr[:, 4].astype(bool)

            # Raw line: just plot, NaNs at NLOS produce natural gaps.
            anchor_raw_lines[i].set_data(t_s, d_raw)
            # Filtered line: NaN gaps where the filter didn't run.
            anchor_filt_lines[i].set_data(t_s, d_filt)

            # Spike markers on the raw value at spike frames.
            spk_mask = spk & heard
            if spk_mask.any():
                anchor_spike_dots[i].set_data(t_s[spk_mask], d_raw[spk_mask])
            else:
                anchor_spike_dots[i].set_data([], [])

            # NLOS tick marks along the bottom of the subplot.
            nlos_mask = ~heard
            ax = ax_anchors[i]
            if nlos_mask.any():
                # Place the tick near the bottom of the current y-range
                # (recomputed below; use a tiny fixed-low value here to be
                # visible regardless of auto-scaling).
                d_valid = d_raw[heard]
                if d_valid.size:
                    y_bottom = float(np.nanmin(d_valid)) - 0.1
                else:
                    y_bottom = 0.0
                anchor_nlos_dots[i].set_data(t_s[nlos_mask],
                                             np.full(int(nlos_mask.sum()), y_bottom))
            else:
                anchor_nlos_dots[i].set_data([], [])

            # Y-range: tight around the actual data so a 5 cm wiggle is visible.
            finite = arr[:, 1][np.isfinite(arr[:, 1])]
            if finite.size:
                lo = float(np.nanmin(finite)) - 0.20
                hi = float(np.nanmax(finite)) + 0.20
                if hi - lo < 0.3:  # enforce a minimum window
                    mid = 0.5 * (hi + lo)
                    lo, hi = mid - 0.15, mid + 0.15
                ax.set_ylim(lo, hi)
            ax.set_xlim(t_s.min(), max(t_s.max(), t_s.min() + 1.0))

        # ---- Info panel (top-left). Kept narrow so it doesn't collide with
        #     the MODINFO panel in the top-right. ----
        if state.last is not None:
            p = state.last
            d = p["d_raw"]
            df = p["d_filt"]
            s = p["std"]
            rx, ry = p["raw_xy"]
            sx, sy = state.last_sent_xy  # latest sent-true position, not this frame's
            info = (
                f"frames: {state.frames}  fps: {p['fps']:.1f}\n"
                f"sent: {state.sent_count}  rej: {state.gate_reject_count}\n"
                f"mask: 0x{p['mask']:01X}   spk: 0x{p['spk']:01X}\n"
                f"       raw   filt   std    spk   nlos\n"
                f" A0: {d[0]:5.2f}  {df[0]:5.2f}  {s[0]:5.3f}  {state.spike_total[0]:4d}  {state.nlos_total[0]:4d}\n"
                f" A1: {d[1]:5.2f}  {df[1]:5.2f}  {s[1]:5.3f}  {state.spike_total[1]:4d}  {state.nlos_total[1]:4d}\n"
                f" A2: {d[2]:5.2f}  {df[2]:5.2f}  {s[2]:5.3f}  {state.spike_total[2]:4d}  {state.nlos_total[2]:4d}\n"
                f" A3: {d[3]:5.2f}  {df[3]:5.2f}  {s[3]:5.3f}  {state.spike_total[3]:4d}  {state.nlos_total[3]:4d}\n"
                f"raw : ({rx:6.2f}, {ry:6.2f})\n"
                f"sent: ({sx:6.2f}, {sy:6.2f})  {'YES' if p['sent'] else 'no'}"
            )
        else:
            info = "waiting for first UDP packet..."
        info_text.set_text(info)

        # ---- Module info panel ----
        if state.modinfo:
            lines = ["--- MODULE ---"]
            for key in ("ver", "cfg", "ant", "cap", "pow", "rpt"):
                if key in state.modinfo:
                    v = state.modinfo[key]
                    # Trim very long responses for the panel.
                    if len(v) > 60:
                        v = v[:57] + "..."
                    lines.append(f"{key}: {v}")
            modinfo_text.set_text("\n".join(lines))

        return ()  # blit disabled, animator doesn't need this

    ani = animation.FuncAnimation(
        fig, animate, interval=80, blit=False, cache_frame_data=False
    )
    try:
        plt.show()
    finally:
        sock.close()


if __name__ == "__main__":
    main()
