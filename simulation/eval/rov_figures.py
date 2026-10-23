"""Figures for the pool vehicle: course tracks by pilot mode, and target identification by range."""

from __future__ import annotations

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
from isonavi.pool import PoolArena          # noqa: E402
from isonavi.pilot import CourseRun         # noqa: E402
from isonavi.optics import Camera, WaterConfig   # noqa: E402

FIG = os.path.join(ROOT, "results", "figures")
LOGS = os.path.join(ROOT, "results", "logs")
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({"figure.dpi": 130, "savefig.dpi": 145, "font.size": 9,
                     "axes.spines.top": False, "axes.spines.right": False})
MODE_COL = {"manual": "#c0392b", "assist": "#e08e0b", "auto": "#1f6fb2", "shared": "#1e8449"}


def draw_course(ax, a):
    c = a.cfg
    ax.add_patch(Rectangle((0, -c.width / 2), c.length, c.width, fill=False, lw=0.8, ec="#888"))
    for name, _ in a.elements:
        prim = a.scene.by_name(name)
        prim = prim[0] if isinstance(prim, list) else prim
        if name.startswith("balloon"):
            t = a.targets[name]
            ax.add_patch(Circle(t["pos"][:2], t["radius"], color=name.split("_")[1], alpha=0.85))
        elif name.startswith(("gate_post", "obstacle")):
            ax.add_patch(Circle((prim.cx, prim.cy), c.pipe_radius * 2,
                                color="#333"))
    x0 = c.maze_x0
    x1 = a.sections["passage"][1]
    for s in (1, -1):
        y = c.maze_y + s * (c.maze_width / 2 + c.wall_thickness / 2)
        ax.plot([x0, x1], [y, y], color="#555", lw=2)
    px0, px1 = a.sections["passage"]
    ax.add_patch(Rectangle((px0, c.maze_y - c.maze_width / 2), px1 - px0, c.maze_width,
                           color="#999", alpha=0.25, lw=0))
    for name, (s0, s1) in a.sections.items():
        ax.text((s0 + s1) / 2, -2.45, name, ha="center", fontsize=7, color="#555")


def course_figure():
    a = PoolArena()
    fig, axes = plt.subplots(2, 1, figsize=(9.0, 4.6), sharex=True)
    for ax, b in zip(axes, (0.4, 1.0)):
        draw_course(ax, a)
        for mode in MODE_COL:
            r = CourseRun(a, mode=mode, seed=3, turbidity=b).run()
            e = r.log["eta"]
            ax.plot(e[:, 0], e[:, 1], color=MODE_COL[mode], lw=1.1,
                    label=f"{mode}: {r.time_s:.0f} s, {r.contacts} contacts")
            for tc, name in r.contact_list:
                k = int(np.argmin(np.abs(r.log["t"] - tc)))
                ax.plot(e[k, 0], e[k, 1], "x", color=MODE_COL[mode], ms=6, mew=1.6)
        ax.set_xlim(0, 20)
        ax.set_ylim(-2.6, 1.4)
        ax.set_aspect("equal")
        ax.set_ylabel("y, m")
        ax.set_title(f"water b = {b} /m, seed 3", fontsize=9, loc="left")
        ax.legend(loc="upper left", fontsize=7, frameon=False, ncol=4)
    axes[-1].set_xlabel("x along the pool, m")
    fig.tight_layout()
    fig.savefig(f"{FIG}/r1_course_tracks.png")
    plt.close(fig)


def vision_figure():
    d = json.load(open(os.path.join(LOGS, "vision_metrics.json")))["summary"]
    rows = list(d.values())
    ranges = sorted({r["range"] for r in rows})
    fig, axes = plt.subplots(1, 3, figsize=(9.6, 2.9), sharey=True)
    for ax, b in zip(axes, sorted({r["b"] for r in rows})):
        for sep, ls in ((0.10, ":"), (0.18, "-"), (0.30, "--")):
            sel = sorted([r for r in rows if r["b"] == b and r["sep"] == sep], key=lambda r: r["range"])
            ax.plot([r["range"] for r in sel], [r["id_corrected"] for r in sel], ls, color="#1f6fb2",
                    label=f"lamps {sep:.2f} m off axis")
        sel = sorted([r for r in rows if r["b"] == b and r["sep"] == 0.18], key=lambda r: r["range"])
        ax.plot([r["range"] for r in sel], [r["id_uncorrected"] for r in sel], "-", color="#c0392b",
                lw=0.9, label="0.18 m, no water correction")
        ax.set_title(f"b = {b} /m", fontsize=9, loc="left")
        ax.set_xlabel("range to target, m")
        ax.set_xticks(ranges[::2] + [ranges[-1]] if ranges[-1] not in ranges[::2] else ranges[::2])
        ax.set_ylim(-0.03, 1.05)
    for ax in axes:
        ax.axhline(0.25, color="#999", lw=0.7, ls=":")
        ax.text(ranges[-1], 0.27, "chance", ha="right", fontsize=6.5, color="#777")
    axes[0].set_ylabel("colour identified correctly")
    axes[0].legend(fontsize=6.5, frameon=False, loc="lower left")
    fig.tight_layout()
    fig.savefig(f"{FIG}/r2_target_identification.png")
    plt.close(fig)


def camera_figure():
    a = PoolArena()
    tgt = a.targets["balloon_red"]["pos"]
    fig, axes = plt.subplots(1, 3, figsize=(9.6, 2.5))
    for ax, b in zip(axes, (0.05, 0.4, 1.0)):
        cam = Camera(water=WaterConfig(scattering=b), seed=4)
        eta = np.array([tgt[0] - 1.8 - cam.cfg.pos[0], tgt[1] + 0.5, tgt[2] - cam.cfg.pos[2], 0, 0, 0])
        img, _, _ = cam.render(a.scene, eta)
        rng = np.hypot(1.8, 0.5)
        ax.imshow(img)
        ax.set_title(f"b = {b} /m, red target at {rng:.1f} m", fontsize=8, loc="left", pad=6)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(f"{FIG}/r3_camera_views.png")
    plt.close(fig)


if __name__ == "__main__":
    course_figure()
    vision_figure()
    camera_figure()
    print("figures written to", os.path.abspath(FIG))
