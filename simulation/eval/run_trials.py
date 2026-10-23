"""Scored course runs across pilot modes, water clarity and seeds.

Each run flies the gate, the target touch and the three section maze with the
pilot model in isonavi/pilot.py, and is scored on what an observer at the poolside sees: a clean gate,
the right balloon touched, contacts as events, and time to the maze exit.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isonavi.pool import PoolArena          # noqa: E402
from isonavi.pilot import CourseRun         # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "logs")
MODES = ("manual", "assist", "auto")
TURBIDITY = (0.05, 0.4, 1.0)
SEEDS = range(1, 13)
CONTACT_PENALTY = 5


def one(args):
    mode, turb, seed = args
    r = CourseRun(PoolArena(), mode=mode, seed=seed, turbidity=turb).run()
    return {"mode": mode, "turbidity": turb, "seed": seed, "time_s": r.time_s,
            "finished": r.finished, "gate_clean": r.gate_clean,
            "balloon": r.balloon_touched, "wrong_balloon": r.wrong_balloon,
            "contacts": r.contacts, "contact_list": r.contact_list,
            "min_clearance": r.min_clearance, "depth_rms": r.depth_rms}


def summarise(rows):
    out = {}
    for mode in MODES:
        for turb in TURBIDITY:
            sel = [r for r in rows if r["mode"] == mode and r["turbidity"] == turb]
            c = np.array([r["contacts"] for r in sel], float)
            t = np.array([r["time_s"] for r in sel if r["finished"]], float)
            out[f"{mode}_{turb}"] = {
                "n": len(sel),
                "finished": sum(r["finished"] for r in sel),
                "gate_clean": sum(r["gate_clean"] for r in sel),
                "balloon": sum(r["balloon"] for r in sel),
                "wrong_balloon": sum(r["wrong_balloon"] for r in sel),
                "contacts_mean": float(c.mean()), "contacts_std": float(c.std()),
                "clean_runs": int((c == 0).sum()),
                "penalty_mean": float(CONTACT_PENALTY * c.mean()),
                "time_mean": float(t.mean()) if len(t) else None,
                "time_std": float(t.std()) if len(t) else None,
                "depth_rms_mean": float(np.mean([r["depth_rms"] for r in sel])),
                "min_clearance_mean": float(np.mean([r["min_clearance"] for r in sel])),
            }
    return out


def main():
    jobs = [(m, tb, s) for m in MODES for tb in TURBIDITY for s in SEEDS]
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        rows = pool.map(one, jobs)
    summ = summarise(rows)
    os.makedirs(OUT, exist_ok=True)
    json.dump({"runs": rows, "summary": summ, "contact_penalty": CONTACT_PENALTY},
              open(os.path.join(OUT, "course_trials.json"), "w"), indent=1)
    print(f"{'mode, b':16s}{'done':>6}{'gate':>6}{'hit':>5}{'wrong':>6}{'contacts':>14}{'clean':>7}{'time s':>14}")
    for k, v in summ.items():
        tm = f"{v['time_mean']:.1f}+/-{v['time_std']:.1f}" if v["time_mean"] else "-"
        print(f"{k:16s}{v['finished']:>4}/{v['n']}{v['gate_clean']:>6}{v['balloon']:>5}"
              f"{v['wrong_balloon']:>6}{v['contacts_mean']:>8.2f}+/-{v['contacts_std']:.2f}"
              f"{v['clean_runs']:>7}{tm:>14}")


if __name__ == "__main__":
    main()
