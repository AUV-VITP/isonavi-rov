"""Scored course runs across pilot modes, water clarity, seeds and course layouts.

Each run flies the gate, the target touch and the three section maze with the
pilot model in isonavi/pilot.py, and is scored on what an observer at the
poolside sees: a clean gate, the right balloon touched, contacts as events,
and time to the maze exit.

Two experiments. The reference layout across 20 seeds and three water
clarities isolates the pilot mode. Forty random layouts at two clarities
check that nothing depends on one arrangement of the course.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isonavi.pool import PoolArena, random_arena   # noqa: E402
from isonavi.pilot import CourseRun                # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "logs")
MODES = ("manual", "assist", "auto", "shared")
TURBIDITY = (0.05, 0.4, 1.0)
SEEDS = range(1, 21)
LAYOUTS = range(1, 41)
LAYOUT_TURBIDITY = (0.4, 1.0)
CONTACT_PENALTY = 5
REF_NAMES = {1: "gate", 2: "target", 3: "corridor"}


def one(args):
    exp, mode, turb, seed, layout = args
    arena = PoolArena(random_arena(layout)) if layout else PoolArena()
    r = CourseRun(arena, mode=mode, seed=seed, turbidity=turb).run()
    vis = {}
    v = r.log.get("vision")
    if v is not None and len(v):
        for ref, name in REF_NAMES.items():
            e = v[v[:, 1] == ref, 2] - v[v[:, 1] == ref, 3]
            vis[name] = {"n": int(len(e)), "rms": float(np.sqrt(np.mean(e ** 2))) if len(e) else None}
    return {"exp": exp, "mode": mode, "turbidity": turb, "seed": seed, "layout": layout,
            "time_s": r.time_s, "finished": r.finished, "gate_clean": r.gate_clean,
            "balloon": r.balloon_touched, "wrong_balloon": r.wrong_balloon,
            "contacts": r.contacts, "contact_list": r.contact_list,
            "min_clearance": r.min_clearance, "depth_rms": r.depth_rms, "vision": vis}


def summarise(rows, exp, turbs):
    out = {}
    for mode in MODES:
        for turb in turbs:
            sel = [r for r in rows if r["exp"] == exp and r["mode"] == mode and r["turbidity"] == turb]
            c = np.array([r["contacts"] for r in sel], float)
            t = np.array([r["time_s"] for r in sel if r["finished"]], float)
            cell = {
                "n": len(sel),
                "finished": sum(r["finished"] for r in sel),
                "gate_clean": sum(r["gate_clean"] for r in sel),
                "balloon": sum(r["balloon"] for r in sel),
                "wrong_balloon": sum(r["wrong_balloon"] for r in sel),
                "contacts_mean": float(c.mean()), "contacts_std": float(c.std()),
                "contacts_max": int(c.max()),
                "clean_runs": int((c == 0).sum()),
                "penalty_mean": float(CONTACT_PENALTY * c.mean()),
                "time_mean": float(t.mean()) if len(t) else None,
                "time_std": float(t.std()) if len(t) else None,
                "depth_rms_mean": float(np.mean([r["depth_rms"] for r in sel])),
            }
            if mode in ("auto", "shared"):
                for name in REF_NAMES.values():
                    vals = [r["vision"][name]["rms"] for r in sel
                            if name in r["vision"] and r["vision"][name]["rms"] is not None]
                    cell[f"vision_rms_{name}"] = float(np.mean(vals)) if vals else None
            out[f"{mode}_{turb}"] = cell
    return out


def main():
    jobs = [("reference", m, tb, s, 0) for m in MODES for tb in TURBIDITY for s in SEEDS]
    jobs += [("layouts", m, tb, k, k) for m in MODES for tb in LAYOUT_TURBIDITY for k in LAYOUTS]
    # Rendered runs are the slow ones; start them first so the pool ends together.
    jobs.sort(key=lambda j: j[1] not in ("auto", "shared"))
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        rows = pool.map(one, jobs, chunksize=1)
    summ = {"reference": summarise(rows, "reference", TURBIDITY),
            "layouts": summarise(rows, "layouts", LAYOUT_TURBIDITY)}
    os.makedirs(OUT, exist_ok=True)
    json.dump({"runs": rows, "summary": summ, "contact_penalty": CONTACT_PENALTY},
              open(os.path.join(OUT, "course_trials.json"), "w"), indent=1)
    for exp, block in summ.items():
        print(exp)
        print(f"  {'mode, b':16s}{'done':>7}{'gate':>6}{'hit':>5}{'wrong':>6}{'contacts':>15}{'clean':>7}{'time s':>14}")
        for k, v in block.items():
            tm = f"{v['time_mean']:.1f}+/-{v['time_std']:.1f}" if v["time_mean"] else "-"
            extra = ""
            if k.startswith(("auto", "shared")):
                extra = "  vision rms mm " + " ".join(
                    f"{n[0]}{'-' if v.get(f'vision_rms_{n}') is None else round(v[f'vision_rms_{n}'] * 1000)}"
                    for n in REF_NAMES.values())
            print(f"  {k:16s}{v['finished']:>4}/{v['n']:<3}{v['gate_clean']:>5}{v['balloon']:>5}"
                  f"{v['wrong_balloon']:>6}{v['contacts_mean']:>8.2f}+/-{v['contacts_std']:.2f}"
                  f"{v['clean_runs']:>7}{tm:>14}{extra}")


if __name__ == "__main__":
    main()
