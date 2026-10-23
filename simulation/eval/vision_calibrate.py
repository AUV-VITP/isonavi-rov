"""Calibrate and evaluate the course estimators on rendered frames.

Corridor: frames from random poses inside the maze of random layouts, so
corridor width and post placement vary. The two parameter wall model, gain
on the log brightness ratio and a heading term, is fitted per water clarity
on half the frames and its error is reported on the other half.

Gate and target: frames at known range and offset; reported as how often a
measurement comes back and how far it is from the truth. The gate frames come
from random layouts and the fit is called exactly as the closed loop calls
it, searched about the planned gate line, so the error carries what lies
beyond the gate as well as the gate itself.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isonavi.pool import PoolArena, random_arena, ArenaConfig      # noqa: E402
from isonavi.optics import Camera, CameraConfig, WaterConfig       # noqa: E402
from isonavi.vision import CourseEstimator                         # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "logs")
TURB = (0.05, 0.2, 0.4, 0.7, 1.0)
N_CORRIDOR = 400
RANGES = (0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0)
N_PER_RANGE = 24
CAM_X = CameraConfig().pos[0]


def corridor_case(args):
    b, i = args
    rng = np.random.default_rng([int(b * 100), i, 7])
    cfg = random_arena(1000 + i)
    a = PoolArena(cfg)
    sec = [a.sections["straight"], a.sections["obstacles"], a.sections["passage"]]
    k = int(rng.integers(3))
    x0, x1 = sec[k]
    xc = rng.uniform(max(x0, cfg.maze_x0 + 0.3), x1 - 0.2)
    half = cfg.maze_width / 2 - 0.22
    y = rng.uniform(-half, half)
    yaw = rng.normal(0, np.radians(4))
    z = rng.uniform(-0.85, -0.60)
    eta = np.array([xc - CAM_X, cfg.maze_y + y - CAM_X * np.sin(yaw), z,
                    rng.normal(0, 0.02), rng.normal(0, 0.02), yaw])
    img, _, _ = Camera(water=WaterConfig(scattering=b), seed=int(rng.integers(1 << 30))).render(a.scene, eta)
    est = CourseEstimator(scattering=b)
    s = est.wall_ratio(img)
    post = None
    if k == 1:
        posts = [(e.cx, e.cy) for e in a.scene.primitives if getattr(e, "name", "").startswith("obstacle")]
        yaw_m = yaw + rng.normal(0, np.radians(1.0))
        post = (img, yaw_m, xc, posts, cfg.maze_y, cfg.maze_width, cfg.maze_y + y)
    return {"b": b, "section": ["straight", "obstacles", "passage"][k], "s": s,
            "yaw": float(yaw), "y_cam": float(y), "w": cfg.maze_width, "post_err": post}


def gate_case(args):
    b, r, i = args
    rng = np.random.default_rng([int(b * 100), int(r * 100), i, 11])
    cfg = random_arena(2000 + i)
    a = PoolArena(cfg)
    y = cfg.gate_y + rng.uniform(-0.3, 0.3)
    yaw = rng.normal(0, np.radians(4))
    xc = cfg.gate_x - r
    eta = np.array([xc - CAM_X, y - CAM_X * np.sin(yaw), -0.55, 0, 0, yaw])
    img, _, _ = Camera(water=WaterConfig(scattering=b), seed=int(rng.integers(1 << 30))).render(a.scene, eta)
    yaw_m = yaw + rng.normal(0, np.radians(1.0))
    est = CourseEstimator(scattering=b).gate(img, yaw_m, xc, cfg.gate_x, cfg.gate_y,
                                             cfg.gate_width / 2 + cfg.pipe_radius, cfg.gate_y,
                                             span=0.5)
    return {"b": b, "range": r, "ok": est is not None,
            "err": None if est is None else float(est - y)}


def target_case(args):
    b, r, i = args
    rng = np.random.default_rng([int(b * 100), int(r * 100), i, 13])
    cfg = ArenaConfig()
    a = PoolArena(cfg)
    tg = a.targets[f"balloon_{cfg.target_colour}"]["pos"]
    y = tg[1] + rng.uniform(-0.3, 0.3)
    yaw = rng.normal(0, np.radians(4))
    eta = np.array([tg[0] - r - CAM_X, y - CAM_X * np.sin(yaw), tg[2] - CameraConfig().pos[2], 0, 0, yaw])
    img, _, _ = Camera(water=WaterConfig(scattering=b), seed=int(rng.integers(1 << 30))).render(a.scene, eta)
    yaw_m = yaw + rng.normal(0, np.radians(1.0))
    est = CourseEstimator(scattering=b).target(img, yaw_m, tg, cfg.target_colour)
    return {"b": b, "range": r, "ok": est is not None,
            "err": None if est is None else float(est - y)}


def _cell(c):
    rms = "-" if c["rms"] is None else f"{c['rms'] * 1000:.0f}"
    return f"{c['range']:.2f}:{c['available']:.2f}/{rms}"


def model(p, s, yaw, w):
    g, k_yaw, bias = p
    return (w / 2) * np.tanh(g * s / 4.0) + k_yaw * yaw + bias


def main():
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        cor = pool.map(corridor_case, [(b, i) for b in TURB for i in range(N_CORRIDOR)], chunksize=8)
        gate = pool.map(gate_case, [(b, r, i) for b in TURB for r in RANGES for i in range(N_PER_RANGE)], chunksize=8)
        tgt = pool.map(target_case, [(b, r, i) for b in TURB for r in RANGES for i in range(N_PER_RANGE)], chunksize=8)

    fits, corridor_err = {}, {}
    for b in TURB:
        rows = [r for r in cor if r["b"] == b]
        s = np.array([r["s"] for r in rows]); yaw = np.array([r["yaw"] for r in rows])
        y = np.array([r["y_cam"] for r in rows]); w = np.array([r["w"] for r in rows])
        sec = np.array([r["section"] for r in rows])
        tr = np.arange(len(rows)) % 2 == 0
        res = least_squares(lambda p: model(p, s[tr], yaw[tr], w[tr]) - y[tr], [1.0, 0.0, 0.0])
        e = model(res.x, s[~tr], yaw[~tr], w[~tr]) - y[~tr]
        fits[str(b)] = {"g": float(res.x[0]), "k_yaw": float(res.x[1]), "bias": float(res.x[2])}
        corridor_err[str(b)] = {"rms": float(np.sqrt(np.mean(e ** 2))),
                                "p95": float(np.percentile(np.abs(e), 95)),
                                **{f"rms_{k}": float(np.sqrt(np.mean(e[sec[~tr] == k] ** 2)))
                                   for k in ("straight", "obstacles", "passage")}}

    def summarise(rows):
        out = {}
        for b in TURB:
            for r in RANGES:
                sel = [x for x in rows if x["b"] == b and x["range"] == r]
                e = np.array([x["err"] for x in sel if x["ok"]])
                out[f"{b}_{r}"] = {"b": b, "range": r, "available": len(e) / len(sel),
                                   "rms": float(np.sqrt(np.mean(e ** 2))) if len(e) else None}
        return out

    # Posts, as the camera mode uses them: anchored on the fitted wall estimate of
    # the same frame, and dropped when they stray more than 0.15 m from it.
    for r in cor:
        if r["post_err"] is None:
            continue
        img, yaw_m, xc, posts, my, w, truth = r["post_err"]
        est = CourseEstimator(scattering=r["b"], calib=fits[str(r["b"])])
        y_wall = est.corridor(img, yaw_m, my, w)
        p = est.posts(img, yaw_m, xc, posts, y_wall)
        r["post_err"] = float(p - truth) if p is not None and abs(p - y_wall) < 0.15 else None
    for b in TURB:
        sel = [r for r in cor if r["b"] == b and r["section"] == "obstacles"]
        e = np.array([r["post_err"] for r in sel if r["post_err"] is not None])
        corridor_err[str(b)]["posts_available"] = len(e) / max(len(sel), 1)
        corridor_err[str(b)]["posts_rms"] = float(np.sqrt(np.mean(e ** 2))) if len(e) else None
    out = {"fits": fits, "corridor": corridor_err, "gate": summarise(gate),
           "target": summarise(tgt), "n_corridor": N_CORRIDOR, "n_per_range": N_PER_RANGE}
    os.makedirs(OUT, exist_ok=True)
    json.dump(out, open(os.path.join(OUT, "vision_calibration.json"), "w"), indent=1)
    print("corridor fit and held-out error")
    for b in TURB:
        f, e = fits[str(b)], corridor_err[str(b)]
        print(f"  b {b:4.2f}  g {f['g']:.3f}  k_yaw {f['k_yaw']:+.3f}  bias {f['bias']*1000:+.1f} mm"
              f"   rms {e['rms']*1000:.1f} mm  p95 {e['p95']*1000:.1f} mm"
              f"  (straight {e['rms_straight']*1000:.0f}, obstacles {e['rms_obstacles']*1000:.0f},"
              f" passage {e['rms_passage']*1000:.0f})  posts {e['posts_available']:.2f} available, rms "
              f"{'-' if e['posts_rms'] is None else round(e['posts_rms'] * 1000)} mm")
    for name in ("gate", "target"):
        print(f"{name}: available / rms mm by range")
        for b in TURB:
            print(f"  b {b:4.2f}  " + "  ".join(_cell(out[name][f"{b}_{r}"]) for r in RANGES))


if __name__ == "__main__":
    main()
