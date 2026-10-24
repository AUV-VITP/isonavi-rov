"""Views of isonavi-rov for the documents: a perspective render and a dimensioned three view.

The geometry is the assembly rov_cad.py builds from the solved budget, and the
shading and projection are the isonavi-1 renderer's, so the two vehicles are
drawn the same way.
"""

from __future__ import annotations

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import render_views as RV
import rov as R
import rov_cad as C

HERE = os.path.dirname(os.path.abspath(__file__))
MM = 1000.0


def pad(ax, d):
    """Room for dimension lines that sit outside the part."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_xlim(x0 - d, x1 + d)
    ax.set_ylim(y0 - d, y1 + d)


def draw_sealed(ax, tris, norms, cols, elev, azim, zoom):
    """RV.draw, with each triangle stroked in its own colour.

    Unstroked polygons leave hairline seams along shared edges, which on the
    large flat plates of this frame read as cracks.
    """
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    tris, norms, cols = RV.depth_sort(tris, norms, cols, elev, azim)
    fc = RV.shade(norms, cols, elev, azim)
    ax.add_collection3d(Poly3DCollection(tris, facecolors=fc, edgecolors=fc, linewidths=0.35))
    allv = tris.reshape(-1, 3)
    ctr = allv.mean(0)
    rng = (allv.max(0) - allv.min(0)).max() / 2 * zoom
    ax.set_xlim(ctr[0] - rng, ctr[0] + rng)
    ax.set_ylim(ctr[1] - rng, ctr[1] + rng)
    ax.set_zlim(ctr[2] - rng, ctr[2] + rng)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def hero(tris, norms, cols):
    fig = plt.figure(figsize=(12.0, 7.6), facecolor="white")
    ax = fig.add_subplot(111, projection="3d", facecolor="white")
    draw_sealed(ax, tris, norms, cols, 24, -58, zoom=0.72)
    out = os.path.join(HERE, "isonavi_rov_hero.png")
    plt.savefig(out, dpi=150, facecolor="white", bbox_inches="tight")
    plt.close()
    RV.trim_png(out)
    return out


def three_view(tris, norms, cols, b):
    L, B, H = R.FRAME_L * MM, R.FRAME_B * MM, R.FRAME_H * MM
    tip, tail = R.PROBE_TIP_X * MM, R.TETHER_END_X * MM
    fig = plt.figure(figsize=(13.0, 8.4), facecolor="white")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1.0], hspace=0.10, wspace=0.06)

    ax = fig.add_subplot(gs[0, 0])
    RV.ortho(ax, tris, norms, cols, screen=[0, 2], view=(0, 1, 0))
    RV.dim2(ax, (tail, -H / 2), (tip, -H / 2), f"{tip - tail:.0f} overall", -70)
    RV.dim2(ax, (-L / 2, H / 2), (L / 2, H / 2), f"{L:.0f} frame", 45)
    RV.dim2(ax, (-L / 2, -H / 2), (-L / 2, H / 2), f"{H:.0f}", -80, vertical=True)
    pad(ax, 60)
    ax.set_title("side elevation", fontsize=11, loc="left", color=RV.INK)

    ax = fig.add_subplot(gs[1, 0])
    RV.ortho(ax, tris, norms, cols, screen=[0, 1], view=(0, 0, 1))
    RV.dim2(ax, (-L / 2, -B / 2), (-L / 2, B / 2), f"{B:.0f}", -80, vertical=True)
    lx, ly, vx, vy = R.ARMS
    RV.dim2(ax, (-lx * MM, -B / 2), (lx * MM, -B / 2), f"{2 * lx * MM:.0f} horizontal stations", -55)
    ax.annotate("horizontal thrusters canted 45 degrees", xy=(lx * MM, ly * MM),
                xytext=(lx * MM + 40, B / 2 + 70), fontsize=8.5, color=RV.INK, ha="center",
                arrowprops=dict(arrowstyle="->", lw=0.8, color=RV.GREY))
    pad(ax, 60)
    ax.set_title("plan", fontsize=11, loc="left", color=RV.INK)

    ax = fig.add_subplot(gs[0, 1])
    RV.ortho(ax, tris, norms, cols, screen=[1, 2], view=(1, 0, 0))
    RV.dim2(ax, (-B / 2, -H / 2), (B / 2, -H / 2), f"{B:.0f}", -55)
    RV.dim2(ax, (-C.LAMP_Y * MM, C.LAMP_Z * MM), (C.LAMP_Y * MM, C.LAMP_Z * MM),
            f"lamps {2 * C.LAMP_Y * MM:.0f} apart", 0)
    pad(ax, 60)
    ax.set_title("front", fontsize=11, loc="left", color=RV.INK)

    ax = fig.add_subplot(gs[1, 1])
    ax.axis("off")
    rows = [("mass in air", f"{b['mass_kg']:.2f} kg"),
            ("displaced volume", f"{b['displaced_m3'] * 1000:.2f} L"),
            ("net buoyancy", f"{b['net_buoyancy_n']:+.2f} N"),
            ("centre of gravity below buoyancy", f"{b['bg_m'] * 1000:.0f} mm"),
            ("foam, ballast", f"{b['foam_l']:.2f} L, {b['ballast_kg']:.2f} kg"),
            ("thrusters", f"8 x {b['thrust_max_n']:.0f} N, vectored"),
            ("surge, all units / one lost", f"{b['envelope_n']['surge']['full']:.0f} N / {b['envelope_n']['surge']['one_lost']:.0f} N"),
            ("power", f"{b['tether']['supply_v']:.0f} V tether, {b['tether']['peak_input_w']:.0f} W governed")]
    ax.text(0.02, 0.97, "isonavi-rov", fontsize=15, fontweight="bold", color=RV.BLUE, va="top")
    for i, (k, v) in enumerate(rows):
        y = 0.82 - i * 0.095
        ax.text(0.02, y, k, fontsize=9.5, color=RV.GREY, va="center")
        ax.text(0.98, y, v, fontsize=9.5, color=RV.INK, va="center", ha="right")
    ax.text(0.02, 0.02, "millimetres", fontsize=8, color=RV.GREY)

    out = os.path.join(HERE, "isonavi_rov_views.png")
    plt.savefig(out, dpi=150, facecolor="white", bbox_inches="tight")
    plt.close()
    RV.trim_png(out)
    return out


def main():
    assy, named, b = C.build()
    # isonavi-1's crease angle suits a revolved hull; on flat plates it blends
    # normals across box corners and splits faces along their diagonals.
    RV.COS_CREASE = float(np.cos(np.radians(12.0)))
    tris, norms, cols = RV.tessellate(assy, tol=0.4, ang=0.12)
    print("wrote", hero(tris, norms, cols))
    print("wrote", three_view(tris, norms, cols, b))


if __name__ == "__main__":
    main()
