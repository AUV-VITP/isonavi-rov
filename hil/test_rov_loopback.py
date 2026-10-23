"""isonavi-rov bench over localhost: host plant and pilot against the board code, no hardware.

Three checks.
  1. equivalence: an assisted course flown through the link reproduces the
     pure simulation run with the same seed;
  2. link loss: with pilot packets withheld, the board goes to HOLD within the
     supervisor timeout, thrust goes to zero, and the vehicle rises;
  3. kill: the kill input latches, and pilot traffic does not re-arm it.
"""

from __future__ import annotations

import json
import os
import sys
import threading

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (HERE, os.path.join(HERE, "host"), os.path.join(HERE, "board"),
          os.path.join(HERE, "common"), os.path.join(HERE, "..", "simulation")):
    sys.path.insert(0, p)

import rov_plant                            # noqa: E402
import rov_computer                         # noqa: E402
from isonavi.pool import PoolArena          # noqa: E402
from isonavi.pilot import CourseRun         # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        FAILS.append(name)


def bench(port, **kw):
    out, ready = {}, threading.Event()
    th = threading.Thread(target=lambda: out.update(host=rov_plant.serve(port, ready=ready, **kw)))
    th.start()
    ready.wait(10)
    board = rov_computer.RovComputer("127.0.0.1", port, esp_dev=None)
    rep = board.run(kw.get("max_time", 420.0))
    th.join(60)
    return out["host"], rep


def main():
    results = {}
    print("1. equivalence with the pure simulation, assisted, seed 2")
    ref = CourseRun(PoolArena(), mode="assist", seed=2).run()
    (res, trace, wall, crc), rep = bench(5611, mode="assist", seed=2)
    n = min(len(ref.log["eta"]), len(res.log["eta"]))
    dev = float(np.max(np.linalg.norm(ref.log["eta"][:n, :3] - res.log["eta"][:n, :3], axis=1)))
    check("course finished through the link", res.finished and res.balloon_touched,
          f"{res.time_s:.2f} s against {ref.time_s:.2f} s")
    check("same score", res.contacts == ref.contacts and res.gate_clean == ref.gate_clean,
          f"{res.contacts} contacts against {ref.contacts}")
    check("same trajectory", dev < 1e-6, f"largest position difference {dev:.2e} m")
    check("no frame errors", crc == 0 and rep["host_crc_errors"] == 0)
    results["equivalence"] = {"time_hil": res.time_s, "time_sim": ref.time_s, "contacts_hil": res.contacts,
                              "contacts_sim": ref.contacts, "max_dev_m": dev, "ticks": rep["ticks"],
                              "compute_ms_mean": rep["compute_ms_mean"], "wall_s": wall}

    print("2. pilot link lost from 30 s to 40 s")
    (res, trace, wall, crc), rep = bench(5612, mode="assist", seed=2, drop=(30.0, 40.0))
    m = min(len(trace["t"]), len(trace["state"]))
    t = np.array(trace["t"][:m])                 # host time after each step
    tb = np.array(trace["state_t"][:m])          # board time of the same tick
    st = np.array(trace["state"][:m])
    thr = np.array(trace["thrust_abs"][:m])
    z = np.array(trace["z"][:m])
    pk = np.array(trace["packet_t"])
    last = float(pk[pk < 30.0].max())
    hold = tb[st == "HOLD"]
    t_hold = float(hold[0] - last) if len(hold) else None
    timeout, dt = 0.5, 0.02
    in_hold = st == "HOLD"
    rise = float(z[in_hold][-1] - z[in_hold][0]) if in_hold.any() else 0.0
    rearmed = bool((st[(tb > 40.0 + dt) & (tb < 45)] == "ARMED").all())
    check("HOLD one tick after the timeout", t_hold is not None and t_hold <= timeout + dt + 1e-9,
          "" if t_hold is None else f"{t_hold:.2f} s after the last packet, timeout {timeout} s")
    check("zero thrust while held", bool(in_hold.any()) and float(thr[in_hold].max()) == 0.0)
    check("vehicle rises while held", rise > 0.05,
          f"{rise:.2f} m in {tb[in_hold][-1] - tb[in_hold][0]:.1f} s")
    check("re-arms when packets return", rearmed)
    results["link_loss"] = {"t_to_hold_s": t_hold, "timeout_s": timeout, "rise_m": rise,
                            "hold_s": float(tb[in_hold][-1] - tb[in_hold][0]), "rearmed": rearmed}

    print("3. kill at 20 s")
    (res, trace, wall, crc), rep = bench(5613, mode="assist", seed=2, kill_at=20.0, max_time=30.0)
    m = min(len(trace["t"]), len(trace["state"]))
    tb = np.array(trace["state_t"][:m])
    st = np.array(trace["state"][:m])
    thr = np.array(trace["thrust_abs"][:m])
    after = tb >= 20.0
    first = tb[st == "KILLED"]
    latched = bool((st[after] == "KILLED").all()) and float(thr[after].max()) == 0.0
    check("killed on the tick it arrives, and latched", latched and len(first) and first[0] < 20.0 + 0.021,
          f"{int(after.sum())} ticks after the kill, all zero thrust")
    results["kill"] = {"latched": latched, "ticks_after": int(after.sum())}

    out = os.path.join(HERE, "results", "rov_loopback.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(results, open(out, "w"), indent=1)
    print()
    print("loopback complete" if not FAILS else f"{len(FAILS)} failed")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
