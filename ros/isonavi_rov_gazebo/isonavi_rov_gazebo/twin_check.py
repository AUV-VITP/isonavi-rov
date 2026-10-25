"""Checks the Gazebo model against the python dynamics the course results come from.

Each case flies the vehicle open loop with the same thruster forces in both:
Gazebo through its thruster plugins, stepped at 1 ms by a test fixture, and
simulation/isonavi/dynamics.py from the same start. The python thrusters
apply their command at once here, as Gazebo's do; in the course runs they
lag by 0.1 s. The python dynamics has no water surface, so every case stays
submerged.

  rise     zero thrust: the vehicle rises on its +5 N
  heave    the verticals push down
  surge    the allocation's pure surge, which needs the verticals to hold
           pitch against the horizontals sitting below the CG
  roll     released at 20 degrees of roll
  tether   surge with the tether drag on

Writes simulation/results/logs/gazebo_twin.json and the comparison figure.
Run from the repository root with ROS 2 Jazzy and Gazebo Harmonic sourced.
"""

import dataclasses
import json
import math
import multiprocessing
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(PKG, "..", ".."))
for p in (os.path.join(ROOT, "simulation"), PKG):
    if p not in sys.path:
        sys.path.insert(0, p)
os.environ.setdefault("GZ_SIM_RESOURCE_PATH", os.path.join(ROOT, "ros", "isonavi_rov_description", "models"))

import gz.math7                                   # noqa: E402,F401  registers the math types
from gz.msgs10.double_pb2 import Double          # noqa: E402
from gz.sim8 import Link, Model, TestFixture, World, world_entity   # noqa: E402
from gz.transport13 import Node                  # noqa: E402

import make_world                                # noqa: E402
import tether_system                             # noqa: E402
from isonavi.dynamics import VehicleDynamics, isonavi_rov, rot_body_to_world   # noqa: E402
from isonavi.pilot import TetherConfig, tether_wrench   # noqa: E402
from isonavi.pool import PoolArena               # noqa: E402

DT_PY = 0.002
LOG_HZ = 50


def neutral():
    """Vertical thrust that cancels the net buoyancy, so a level vehicle holds depth."""
    p = isonavi_rov
    f = np.zeros(8)
    f[4:] = -(p.buoyancy - p.weight) / 4
    return f


def surge(n):
    """The allocation's thruster forces for a pure surge force of n newtons."""
    d = VehicleDynamics(isonavi_rov)
    return d.B_pinv @ np.array([n, 0, 0, 0, 0, 0])


CASES = [
    # name, spawn x y z roll pitch yaw, thruster forces, seconds, tether
    ("rise", (15.0, 3.0, -1.2, 0, 0, 0), np.zeros(8), 4.0, False),
    ("heave", (18.0, 3.0, -0.4, 0, 0, 0), np.r_[np.zeros(4), -8.0 * np.ones(4)], 3.0, False),
    ("surge", (3.0, 3.0, -0.45, 0, 0, 0), surge(40.0) + neutral(), 6.0, False),
    ("roll", (15.0, 3.0, -1.0, math.radians(20), 0, 0), neutral(), 4.0, False),
    ("tether", (1.5, 3.0, -0.45, 0, 0, 0), surge(40.0) + neutral(), 7.0, True),
]


def run_gazebo(spawn, forces, seconds, tether):
    arena = PoolArena()
    path = os.path.join(tempfile.mkdtemp(), "twin.sdf")
    # Unthrottled: a physics run need not keep to the wall clock.
    # The tether system is stepped from here rather than by Gazebo's python
    # loader, which cannot start inside a python process; it is the same code.
    open(path, "w").write(make_world.world(arena, list(spawn), sensors=False, tether=False, rtf=0))
    rec, state = [], {}
    drag = tether_system.TetherDrag() if tether else None

    def pre(info, ecm):
        if "link" not in state:
            entity = World(world_entity(ecm)).model_by_name(ecm, "isonavi_rov")
            state["link"] = Link(Model(entity).link_by_name(ecm, "base_link"))
            state["link"].enable_velocity_checks(ecm, True)
            if drag:
                drag.configure(entity, None, ecm, None)
        if drag:
            drag.pre_update(info, ecm)

    def post(info, ecm):
        link = state.get("link")
        pose = link.world_pose(ecm) if link else None
        v, w = (link.world_linear_velocity(ecm), link.world_angular_velocity(ecm)) if link else (None, None)
        if pose is None or v is None or w is None:
            return
        t = info.sim_time.total_seconds() if hasattr(info.sim_time, "total_seconds") else float(info.sim_time)
        p, rpy = pose.pos(), pose.rot().euler()
        eta = np.array([p.x(), p.y(), p.z(), rpy.x(), rpy.y(), rpy.z()])
        R = rot_body_to_world(*eta[3:])
        nu = np.concatenate([R.T @ np.array([v.x(), v.y(), v.z()]), R.T @ np.array([w.x(), w.y(), w.z()])])
        rec.append(np.concatenate([[t], eta, nu]))

    fx = TestFixture(path)
    fx.on_pre_update(pre)
    fx.on_post_update(post)
    fx.finalize()
    server = fx.server()
    server.run(True, 1, False)
    node = Node()
    pubs = [node.advertise(f"/model/isonavi_rov/joint/thruster_{i + 1}_joint/cmd_thrust", Double) for i in range(8)]
    time.sleep(1.0)
    for pub, fi in zip(pubs, forces):
        msg = Double()
        msg.data = float(fi)
        pub.publish(msg)
    time.sleep(1.0)
    rec.clear()
    server.run(True, int(round(seconds / 0.001)), False)
    a = np.array(rec)
    a[:, 0] -= a[0, 0]
    return a


def run_python(spawn, forces, seconds, tether):
    p = dataclasses.replace(isonavi_rov, thruster_tau=1e-6)
    d = VehicleDynamics(p)
    d.reset(eta=list(spawn))
    tau = d.B @ np.asarray(forces, float)
    tc = TetherConfig()
    out, t = [], 0.0
    for _ in range(int(round(seconds / DT_PY))):
        ext = tether_wrench(d.eta, d.nu, tc, p.tow_point) if tether else None
        eta, nu = d.step(tau, DT_PY, tau_ext=ext)
        t += DT_PY
        out.append(np.concatenate([[t], eta, nu]))
    return np.array(out)


def resample(a, hz=LOG_HZ):
    t = np.arange(0.0, a[-1, 0] + 1e-9, 1.0 / hz)
    return np.column_stack([t] + [np.interp(t, a[:, 0], a[:, k]) for k in range(1, a.shape[1])])


# RMS difference allowed between the two, by quantity: metres, m/s, degrees, deg/s.
TOLERANCE = {"x": 0.01, "z": 0.01, "u": 0.01, "w": 0.01, "roll": 0.5, "pitch": 0.5, "p": 1.5}
COLS = {"x": 1, "y": 2, "z": 3, "roll": 4, "pitch": 5, "yaw": 6, "u": 7, "v": 8, "w": 9, "p": 10, "q": 11, "r": 12}
WATCH = {"rise": ("z", "w"), "heave": ("z", "w"), "surge": ("x", "u", "pitch", "z"), "roll": ("roll", "p"),
         "tether": ("x", "u")}


def compare(name, g, py):
    n = min(len(g), len(py))
    g, py = g[:n], py[:n]
    out = {"t": g[:, 0].round(3).tolist()}
    for q in WATCH[name]:
        k = COLS[q]
        scale = 180 / math.pi if q in ("roll", "pitch", "yaw", "p", "q", "r") else 1.0
        dg, dp = g[:, k] * scale, py[:, k] * scale
        out[q] = {"gazebo": dg.round(5).tolist(), "python": dp.round(5).tolist(),
                  "rms": float(np.sqrt(np.mean((dg - dp) ** 2))), "max": float(np.max(np.abs(dg - dp))),
                  "end_gazebo": float(dg[-1]), "end_python": float(dp[-1]),
                  "span": float(np.ptp(dp))}
    return out


def figure(results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    panels = [("rise", "z", "depth, m", -1), ("heave", "z", "depth, m", -1), ("surge", "u", "surge speed, m/s", 1),
              ("surge", "pitch", "pitch, deg", 1), ("roll", "roll", "roll, deg", 1), ("tether", "u", "surge speed, m/s", 1)]
    fig, axes = plt.subplots(2, 3, figsize=(10.5, 5.2))
    for ax, (case, q, label, sgn) in zip(axes.flat, panels):
        r = results[case]
        ax.plot(r["t"], sgn * np.array(r[q]["python"]), color="#1f6fb2", lw=1.6, label="python dynamics")
        ax.plot(r["t"], sgn * np.array(r[q]["gazebo"]), color="#c0392b", lw=1.0, ls="--", label="Gazebo")
        ax.set_title(f"{case}: {label}", fontsize=9, loc="left")
        ax.set_xlabel("time, s", fontsize=8)
        ax.tick_params(labelsize=8)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[0, 0].legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def gazebo_case(case):
    """One Gazebo case in a fresh process: a second server in the same process does not start cleanly.

    Each gets its own transport partition; the cases run side by side, and
    without it every server would hear every case's thruster commands.
    """
    name, spawn, forces, seconds, tether = case
    os.environ["GZ_PARTITION"] = f"twin_{name}_{os.getpid()}"
    return resample(run_gazebo(spawn, forces, seconds, tether))


def main():
    results = {}
    with multiprocessing.get_context("spawn").Pool(len(CASES)) as pool:
        gz_runs = pool.map(gazebo_case, CASES)
    for (name, spawn, forces, seconds, tether), g in zip(CASES, gz_runs):
        py = resample(run_python(spawn, forces, seconds, tether))
        results[name] = compare(name, g, py)
        results[name]["forces_n"] = [round(float(v), 4) for v in forces]
        msg = ", ".join(f"{q} rms {r['rms']:.4f} (end {r['end_gazebo']:+.3f} vs {r['end_python']:+.3f})"
                        for q, r in ((q, results[name][q]) for q in WATCH[name]))
        print(f"{name:7s} {msg}", flush=True)
    fails = [f"{name} {q} rms {r[q]['rms']:.4f} over {TOLERANCE[q]}" for name, r in results.items()
             for q in WATCH[name] if r[q]["rms"] > TOLERANCE[q]]
    results["tolerance"] = TOLERANCE
    results["passed"] = not fails
    logs = os.path.join(ROOT, "simulation", "results", "logs")
    json.dump(results, open(os.path.join(logs, "gazebo_twin.json"), "w"), indent=1)
    figure(results, os.path.join(ROOT, "simulation", "results", "figures", "r4_gazebo_twin.png"))
    for msg in fails:
        print("FAIL", msg)
    print("Gazebo model agrees with the python dynamics" if not fails else f"{len(fails)} quantities out of tolerance")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
