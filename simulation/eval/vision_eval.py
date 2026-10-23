"""Target colour identification against range, water clarity and lamp placement.

Frames are rendered by the camera model looking at each balloon from a
jittered pose, and the target finder is asked which colour is in the middle
of the frame. Run twice, with and without the attenuation correction, so the
value of undoing the water is measured rather than asserted.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isonavi.pool import PoolArena                                  # noqa: E402
from isonavi.optics import Camera, CameraConfig, Lamp, WaterConfig  # noqa: E402
from isonavi.vision import TargetFinder, COLOURS                    # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "logs")
RANGES = (0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0)
TURB = (0.05, 0.4, 1.0)
SEPARATION = (0.10, 0.18, 0.30)       # lamp offset either side of the lens, m; 0.18 as built
JITTER = 8


def frame_case(args):
    rng_m, b, sep, colour, j = args
    a = PoolArena()
    rs = np.random.default_rng([int(rng_m * 100), int(b * 100), int(sep * 100), COLOURS.index(colour), j])
    tgt = a.targets[f"balloon_{colour}"]["pos"]
    cfg = CameraConfig(lamps=(Lamp((0.07, sep, 0.0)), Lamp((0.07, -sep, 0.0))))
    cam = Camera(cfg, WaterConfig(scattering=b), seed=int(rs.integers(1 << 30)))
    yaw = rs.normal(0, np.radians(4))
    cx = tgt[0] - rng_m * np.cos(yaw) - cfg.pos[0]
    cy = tgt[1] - rng_m * np.sin(yaw)
    eta = np.array([cx, cy + rs.normal(0, 0.03), tgt[2] - cfg.pos[2] + rs.normal(0, 0.03),
                    rs.normal(0, 0.02), rs.normal(0, 0.02), yaw])
    img, _, _ = cam.render(a.scene, eta)
    res = {}
    for corr in (True, False):
        d = TargetFinder(scattering=b, correct=corr).nearest_to_centre(img)
        res["corrected" if corr else "uncorrected"] = None if d is None else d.colour
    d = TargetFinder(scattering=b).nearest_to_centre(img)
    return {"range": rng_m, "b": b, "sep": sep, "colour": colour, "jitter": j,
            "detected": d is not None,
            "range_est": None if d is None else d.range_m,
            "corrected": res["corrected"], "uncorrected": res["uncorrected"]}


def main():
    jobs = [(r, b, s, c, j) for r in RANGES for b in TURB for s in SEPARATION
            for c in COLOURS for j in range(JITTER)]
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        rows = pool.map(frame_case, jobs, chunksize=8)
    summ = {}
    for b in TURB:
        for s in SEPARATION:
            for r in RANGES:
                sel = [x for x in rows if x["b"] == b and x["sep"] == s and x["range"] == r]
                n = len(sel)
                summ[f"{b}_{s}_{r}"] = {
                    "b": b, "sep": s, "range": r, "n": n,
                    "detect": sum(x["detected"] for x in sel) / n,
                    "id_corrected": sum(x["corrected"] == x["colour"] for x in sel) / n,
                    "id_uncorrected": sum(x["uncorrected"] == x["colour"] for x in sel) / n,
                    "range_err": float(np.mean([abs(x["range_est"] - x["range"])
                                                for x in sel if x["range_est"] is not None]
                                               or [np.nan])),
                }
    os.makedirs(OUT, exist_ok=True)
    json.dump({"rows": rows, "summary": summ}, open(os.path.join(OUT, "vision_metrics.json"), "w"),
              indent=1)
    for b in TURB:
        print(f"b = {b} /m   (detect / id corrected / id raw), by lamp separation")
        print("  range " + "".join(f"{f'sep {s:.2f} m':>20s}" for s in SEPARATION))
        for r in RANGES:
            cells = []
            for s in SEPARATION:
                v = summ[f"{b}_{s}_{r}"]
                cells.append(f"{v['detect']:.2f}/{v['id_corrected']:.2f}/{v['id_uncorrected']:.2f}")
            print(f"  {r:5.2f} " + "".join(f"{c:>20s}" for c in cells))


if __name__ == "__main__":
    main()
