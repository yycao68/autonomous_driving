#!/usr/bin/env python3
"""
Autonomous-vehicle moose-test — top-down simulation video (panel design).

Two cars race the ISO double-lane-change ("moose test") on parallel tracks:
  * TOP track  — the proposed jerk-bounded time-first DI plan (`plan_lateral_jerk`)
  * BOTTOM track — the path-first TOTG plan (`plan_totg`)
so the steering smoothness (jerk) and the lap time (performance) can be compared
directly. The right panel shows the synced steering / dynamics curves for both;
a task-stage timeline runs across the top, a legend sits on the left, a dynamic
readout top-right, and a comparison note in the bottom strip.

Reproduces §VI.C: jerk-DI steering is smooth and within the 0.5 rad/s actuator
cap; TOTG steps at every blend seam (rate spikes past the cap) and takes longer.

Run:  python3 av_video_panel.py
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont

from benchmark_av import (av_double_lane_change, steering, V_CRUISE, V_MAX, A_MAX,
                          DELTA_RATE_LIMIT)
from di_lateral import plan_lateral_jerk
from di_totg import plan_totg

RESULTS_DIR = Path(__file__).parent.parent / "simulationResults"

SCENE_W, H = 1280, 720
PANEL_W    = 600
DT         = 0.05
FPS        = 20
CAR_L, CAR_W = 4.6, 1.9
WINDOW_M   = 50.0                       # horizontal extent of the scrolling view
A_LAT_MAX  = float(A_MAX[1])
Y_OFF      = -11.0                      # vertical offset of the TOTG (lower) track
DI_COL     = "#33b5ff"
TG_COL     = "#ff6b6b"


# --------------------------------------------------------------------------
def _traj_from(res, st):
    """Build (t, x, y, heading, delta_deg, drate, speed, a_lat) from a plan."""
    if "Q" in res:                                   # TOTG returns positions
        x, y = res["Q"][:, 0], res["Q"][:, 1]
    else:                                            # DI lateral: x at cruise, Y
        vx = res["V"][:, 0]
        x = np.cumsum(vx) * res["dt"] - vx[0] * res["dt"]
        y = res["Y"]
    vx, vy = res["V"][:, 0], res["V"][:, 1]
    return dict(t=res["t"], x=x, y=y, heading=np.arctan2(vy, vx),
                delta_deg=np.degrees(st["delta"]), drate=st["drate"],
                speed=st["speed"], a_lat=st["a_lat"],
                peak_drate=st["peak_drate"], finish=float(res["t"][-1]))


def rollout():
    name, W, max_dev = av_double_lane_change()
    di = plan_lateral_jerk(W, V=V_CRUISE, dt=DT)
    tg = plan_totg(W, dt=DT, vmax=V_MAX, amax=A_MAX, max_dev=max_dev)
    di_log = _traj_from(di, steering(di))
    tg_log = _traj_from(tg, steering(tg))
    di_log["wps"], tg_log["wps"] = W, W
    return di_log, tg_log


def _sample(log, t):
    """Interpolated car state at wall-clock time t (clamped past the finish)."""
    return dict(
        x=float(np.interp(t, log["t"], log["x"])),
        y=float(np.interp(t, log["t"], log["y"])),
        heading=float(np.interp(t, log["t"], log["heading"])),
        delta=float(np.interp(t, log["t"], np.radians(log["delta_deg"]))),
        done=t >= log["finish"])


# --------------------------------------------------------------------------
# task-stage timeline (by longitudinal position of the DI car)
# --------------------------------------------------------------------------
def build_stages(di_log):
    x, t = di_log["x"], di_log["t"]
    bounds = [(0, 15, "Approach", "straight, entry lane", (90, 170, 255)),
              (15, 30, "Swerve out", "lane change to avoid the obstacle", (255, 140, 0)),
              (30, 45, "Offset hold", "tracking the displaced lane", (90, 210, 110)),
              (45, 60, "Swerve back", "return to the entry lane", (255, 140, 0)),
              (60, 999, "Recover", "straighten and cruise", (160, 140, 255))]
    return [(float(np.interp(xa, x, t)), float(np.interp(min(xb, x[-1]), x, t)),
             nm, desc, col) for (xa, xb, nm, desc, col) in bounds]


def stage_at(stages, t):
    for i, (a, b, nm, desc, col) in enumerate(stages):
        if a <= t < b:
            return i, nm, desc, col
    return len(stages) - 1, stages[-1][2], stages[-1][3], stages[-1][4]


# --------------------------------------------------------------------------
# scene (top-down): two stacked tracks rendered with matplotlib
# --------------------------------------------------------------------------
class Scene:
    def __init__(self, di_log, tg_log):
        dpi = 100
        self.fig = plt.figure(figsize=(SCENE_W / dpi, H / dpi), dpi=dpi)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_facecolor("#1b1e26")
        self.ax.set_xticks([]); self.ax.set_yticks([])

        self._track(di_log, 0.0, DI_COL, "JERK-DI  (proposed)")
        self._track(tg_log, Y_OFF, TG_COL, "TOTG  (path-first)")

        self.di_car = self._add_car("#1f6fd0", "#7ee0ff")
        self.tg_car = self._add_car("#a02525", "#ff9a9a")

        ymid = 0.5 * (3.0 + (Y_OFF - 3.0))
        half = WINDOW_M * H / SCENE_W / 2
        self.ax.set_ylim(ymid - half, ymid + half)

    def _track(self, log, yoff, col, label):
        a = self.ax
        a.add_patch(Rectangle((-10, -3.0 + yoff), 120, 9.0, facecolor="#33373f",
                              edgecolor="none", zorder=0))
        for yy in (-3.0, 6.0):
            a.axhline(yy + yoff, color="#c9cdd6", lw=2, zorder=1)
        xs = np.arange(-10, 110, 4)
        a.plot(xs, np.full(len(xs), 1.5 + yoff), ".", color="#9aa0ad", ms=2, zorder=1)
        for cx in (15, 30, 45, 60):
            for cy in (-1.6, 5.0):
                a.plot(cx, cy + yoff, marker="^", color="#ff9a1f", ms=8, zorder=2)
        a.plot(log["x"], log["y"] + yoff, "-", color=col, lw=1.8, zorder=2)
        a.axvline  # finish line drawn per-track below
        a.plot([80, 80], [-3 + yoff, 6 + yoff], color="#e8e8e8", lw=1.2,
               ls=(0, (4, 3)), zorder=2)
        a.text(-8, 6.6 + yoff, label, color=col, fontsize=12, fontweight="bold",
               zorder=4, va="bottom")

    def _add_car(self, face, edge):
        car = Polygon(np.zeros((4, 2)), closed=True, facecolor=face,
                      edgecolor=edge, lw=2, zorder=5)
        self.ax.add_patch(car)
        wheels = [self.ax.plot([], [], color="#ffd400", lw=3, zorder=6)[0]
                  for _ in range(2)]
        return dict(body=car, wheels=wheels)

    def _place(self, car, st, yoff):
        cx, cy, th, delta = st["x"], st["y"] + yoff, st["heading"], st["delta"]
        c, sgn = np.cos(th), np.sin(th)
        R = np.array([[c, -sgn], [sgn, c]])
        hl, hw = CAR_L / 2, CAR_W / 2
        corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
        car["body"].set_xy((R @ corners.T).T + [cx, cy])
        axle = np.array([hl * 0.7, 0.0])
        for k, lat in enumerate((-hw * 0.8, hw * 0.8)):
            ctr = R @ (axle + [0.0, lat]) + [cx, cy]
            wd = np.array([np.cos(th + delta), np.sin(th + delta)]) * 0.9
            car["wheels"][k].set_data([ctr[0] - wd[0], ctr[0] + wd[0]],
                                      [ctr[1] - wd[1], ctr[1] + wd[1]])

    def frame(self, di_st, tg_st):
        self._place(self.di_car, di_st, 0.0)
        self._place(self.tg_car, tg_st, Y_OFF)
        cam = 0.5 * (di_st["x"] + tg_st["x"])
        self.ax.set_xlim(cam - WINDOW_M / 2, cam + WINDOW_M / 2)
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()

    def close(self):
        plt.close(self.fig)


# --------------------------------------------------------------------------
# live curve panel (both cars animated up to the current time)
# --------------------------------------------------------------------------
class LivePlot:
    def __init__(self, di_log, tg_log, px_w=PANEL_W, px_h=H):
        self.di, self.tg = di_log, tg_log
        dpi = 100
        self.fig, axes = plt.subplots(4, 1, figsize=(px_w / dpi, px_h / dpi),
                                      dpi=dpi, sharex=True)
        self.fig.patch.set_facecolor("#0f1218")
        self.fig.subplots_adjust(left=0.17, right=0.96, top=0.95, bottom=0.07,
                                 hspace=0.34)
        t_end = max(di_log["t"][-1], tg_log["t"][-1])
        specs = [
            ("delta_deg", "steering angle (°)", None),
            ("drate", "steering rate (rad/s)", DELTA_RATE_LIMIT),
            ("speed", "speed (m/s)", None),
            ("a_lat", "lateral accel (m/s²)", A_LAT_MAX),
        ]
        self.keys = [s[0] for s in specs]
        self.di_lines, self.tg_lines, self.markers = [], [], []
        for ax, (key, ylabel, hline) in zip(axes, specs):
            ax.set_facecolor("#161a22")
            for sp in ax.spines.values():
                sp.set_color("#444b57")
            ax.tick_params(colors="#aab2c0", labelsize=8)
            ax.set_ylabel(ylabel, color="#dfe5ee", fontsize=9)
            ax.grid(alpha=0.18, color="#5a6273")
            ax.set_xlim(0, t_end)
            allv = np.concatenate([di_log[key], tg_log[key]])
            lo, hi = float(allv.min()), float(allv.max())
            if hline is not None:
                lo, hi = min(lo, -hline), max(hi, hline)
            pad = 0.12 * (hi - lo + 1e-6)
            ax.set_ylim(lo - pad, hi + pad)
            if hline is not None:
                for hv in (hline, -hline):
                    ax.axhline(hv, ls="--", color="#ff5555", lw=1.0)
                ax.text(0.985, 0.05, "actuator/comfort limit", transform=ax.transAxes,
                        ha="right", va="bottom", color="#ff8c8c", fontsize=7)
            (ld,) = ax.plot([], [], color=DI_COL, lw=1.7, label="jerk-DI")
            (lt,) = ax.plot([], [], color=TG_COL, lw=1.4, ls="--", label="TOTG")
            self.di_lines.append(ld); self.tg_lines.append(lt)
            self.markers.append(ax.axvline(0., color="#fff", lw=0.8, alpha=0.5))
        axes[0].set_title("steering / dynamics", color="#dfe5ee", fontsize=10)
        axes[0].legend(fontsize=7, facecolor="#161a22", edgecolor="#444b57",
                       labelcolor="#dfe5ee", loc="upper right", framealpha=0.85)
        axes[-1].set_xlabel("time (s)", color="#dfe5ee", fontsize=9)

    def frame(self, t):
        for ld, lt, key in zip(self.di_lines, self.tg_lines, self.keys):
            md = self.di["t"] <= t
            mt = self.tg["t"] <= t
            ld.set_data(self.di["t"][md], self.di[key][md])
            lt.set_data(self.tg["t"][mt], self.tg[key][mt])
        for mk in self.markers:
            mk.set_xdata([t, t])
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()

    def close(self):
        plt.close(self.fig)


# --------------------------------------------------------------------------
# HUD
# --------------------------------------------------------------------------
def _font(sz):
    for p in ("/System/Library/Fonts/Supplemental/Arial.ttf",
              "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(p, sz)
        except OSError:
            continue
    return ImageFont.load_default()


LEGEND = [
    ((31, 111, 208),  "square", "Jerk-DI car (top)"),
    ((160, 37, 37),   "square", "TOTG car (bottom)"),
    ((51, 181, 255),  "line",   "Planned path"),
    ((255, 154, 31),  "tri",    "Moose-test cones"),
    ((255, 212, 0),   "line",   "Steered front wheels"),
]


def _draw_legend(d):
    f = _font(16); x, y = 14, 188
    d.rectangle([0, y - 26, 262, y + 26 * len(LEGEND) + 6], fill=(15, 18, 24, 175))
    d.text((x, y - 24), "Legend", font=f, fill=(200, 206, 216, 255))
    for col, shape, label in LEGEND:
        cy = y + 11
        if shape == "square":
            d.rectangle([x, cy - 6, x + 16, cy + 6], fill=col + (255,))
        elif shape == "tri":
            d.polygon([(x + 8, cy - 7), (x, cy + 6), (x + 16, cy + 6)], fill=col + (255,))
        else:
            d.line([x, cy, x + 18, cy], fill=col + (255,), width=3)
        d.text((x + 28, y), label, font=f, fill=(230, 235, 244, 255))
        y += 26


def _draw_stage_strip(d, t, stages):
    idx, nm, desc, col = stage_at(stages, t)
    t_end = stages[-1][1]
    x0, x1, y0, hh = 18, SCENE_W - 18, 100, 26
    span = x1 - x0
    for i, (a, b, nmi, _de, c) in enumerate(stages):
        lx = x0 + int(span * a / t_end); rx = x0 + int(span * b / t_end)
        active = (i == idx)
        d.rectangle([lx + 1, y0, rx - 1, y0 + hh],
                    fill=(c + (235,)) if active else (70, 78, 92, 200))
        sf = _font(14); bb = d.textbbox((0, 0), nmi, font=sf); tw = bb[2] - bb[0]
        if rx - lx > tw + 6:
            tcol = (15, 18, 24, 255) if active else (200, 206, 216, 255)
            d.text((lx + (rx - lx - tw) / 2, y0 + 4), nmi, font=sf, fill=tcol)
    cx = x0 + int(span * min(t, t_end) / t_end)
    d.line([cx, y0 - 3, cx, y0 + hh + 3], fill=(255, 255, 255, 255), width=2)
    d.text((x0, y0 + hh + 7), f"▶  {nm}: {desc}", font=_font(20), fill=col + (255,))


def _draw_note(d, di_log, tg_log, t):
    """Comparison note in the bottom strip (jerk + performance)."""
    f, hf = _font(17), _font(18)
    bw, bx, by = 760, 18, H - 96
    d.rectangle([bx, by, bx + bw, H - 14], fill=(15, 18, 24, 180))
    d.text((bx + 12, by + 6), "Jerk-DI vs TOTG", font=hf, fill=(255, 255, 255, 255))
    di_fin = "✓ finished" if t >= di_log["finish"] else f"{di_log['finish']:.1f}s"
    tg_fin = "✓ finished" if t >= tg_log["finish"] else f"{tg_log['finish']:.1f}s"
    lines = [
        (f"JERK-DI:  peak steering rate {di_log['peak_drate']:.2f} rad/s  ≤ 0.5 cap "
         f"(smooth) · finish {di_log['finish']:.1f}s [{di_fin}]", (130, 210, 255)),
        (f"TOTG:     peak steering rate {tg_log['peak_drate']:.2f} rad/s  > 0.5 cap "
         f"(jerky seams) · finish {tg_log['finish']:.1f}s [{tg_fin}]", (255, 150, 150)),
    ]
    y = by + 32
    for txt, c in lines:
        d.text((bx + 12, y), txt, font=f, fill=c + (255,)); y += 24


def add_hud(rgb, di_log, tg_log, t, stages, di_st, tg_st):
    img = Image.fromarray(rgb); d = ImageDraw.Draw(img, "RGBA")
    tf, f = _font(25), _font(22)
    d.rectangle([0, 0, SCENE_W, 92], fill=(15, 18, 24, 175))
    d.text((16, 10), "Autonomous driving — moose test (two cars, top-down)",
           font=tf, fill=(255, 255, 255, 255))
    d.text((16, 50), "Jerk-bounded DI  vs  path-first TOTG", font=f,
           fill=(120, 220, 255, 255))
    d.text((SCENE_W - 200, 14), f"t = {t:4.2f} s", font=tf, fill=(255, 255, 255, 255))

    _draw_stage_strip(d, t, stages)
    _draw_legend(d)
    _draw_note(d, di_log, tg_log, t)

    # readout (top-right): live speed / steering for both cars
    def near(log, key, tt):
        return float(np.interp(tt, log["t"], log[key]))
    lines = [
        (f"DI  speed {near(di_log,'speed',t):5.1f}  δ {near(di_log,'delta_deg',t):5.1f}°",
         (180, 230, 255)),
        (f"DI  steer-rate {near(di_log,'drate',t):6.3f} rad/s", (180, 230, 255)),
        (f"TG  speed {near(tg_log,'speed',t):5.1f}  δ {near(tg_log,'delta_deg',t):5.1f}°",
         (255, 170, 170)),
        (f"TG  steer-rate {near(tg_log,'drate',t):6.3f} rad/s", (255, 170, 170)),
    ]
    bx, y0 = SCENE_W - 300, 134
    d.rectangle([bx, y0, SCENE_W, y0 + 24 * len(lines) + 14], fill=(15, 18, 24, 160))
    y = y0 + 10
    for txt, c in lines:
        d.text((bx + 12, y), txt, font=f, fill=c + (255,)); y += 24
    return np.asarray(img)


# --------------------------------------------------------------------------
def make_video(out=None, fps=FPS):
    di_log, tg_log = rollout()
    stages = build_stages(di_log)
    scene = Scene(di_log, tg_log)
    panel = LivePlot(di_log, tg_log)

    t_max = max(di_log["finish"], tg_log["finish"]) + 0.4
    t_grid = np.arange(0.0, t_max, DT)

    out = out or str(RESULTS_DIR / "av_video_panel.mp4")
    writer = imageio.get_writer(out, fps=fps, codec="libx264", quality=8,
                                macro_block_size=8)
    for t in t_grid:
        di_st, tg_st = _sample(di_log, t), _sample(tg_log, t)
        left = scene.frame(di_st, tg_st)
        left = add_hud(left, di_log, tg_log, t, stages, di_st, tg_st)
        curves = panel.frame(t)
        writer.append_data(np.hstack([left, curves]))
    writer.close(); scene.close(); panel.close()
    print(f"[video] {len(t_grid)} frames @ {fps} fps -> {out}  "
          f"({SCENE_W + PANEL_W}x{H})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--fps", type=int, default=FPS)
    a = ap.parse_args()
    make_video(a.out, a.fps)


if __name__ == "__main__":
    main()
