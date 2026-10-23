"""Checks for the pool arena, camera, target finder, pilot run and safety chain."""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isonavi.pool import PoolArena                              # noqa: E402
from isonavi.optics import Camera, WaterConfig                  # noqa: E402
from isonavi.vision import CourseEstimator, TargetFinder, load_errors   # noqa: E402
from isonavi.pilot import CourseRun, PilotConfig, blend_weight, read_error   # noqa: E402
from isonavi.safety import PowerGovernor, Supervisor            # noqa: E402
from isonavi.dynamics import VehicleDynamics, isonavi_rov       # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        FAILS.append(name)


a = PoolArena()

# Arena: ray cast floor against the analytic slope, gate clearance.
xs = np.array([0.5, 10.0, 19.5])
h = a.scene.intersect(np.stack([xs, 0 * xs, -0.1 + 0 * xs], 1), np.tile([0, 0, -1.0], (3, 1)))
check("floor depth matches the slope", np.allclose(0.1 + h.t, -a.floor_z(xs), atol=1e-6))
dims = CourseRun.DIMS
t0, c0 = a.contacts(np.array([4.0, 0.0, -0.55, 0, 0, 0]), dims)
t1, _ = a.contacts(np.array([4.0, 0.32, -0.55, 0, 0, 0]), dims)
check("centred in the gate is clear", not t0 and c0 > 0.05, f"clearance {c0:.3f} m")
check("offset into a post is a contact", "gate_post_left" in t1)

# Camera and target finder: the target colour is read at close range.
cam = Camera(water=WaterConfig(scattering=0.4), seed=3)
tgt = a.targets["balloon_red"]["pos"]
img, _, _ = cam.render(a.scene, np.array([tgt[0] - 1.3, tgt[1], tgt[2], 0, 0, 0]))
d = TargetFinder(scattering=0.4).nearest_to_centre(img)
check("red balloon identified at 1 m", d is not None and d.colour == "red",
      "" if d is None else f"range estimate {d.range_m:.2f} m")

# Gate fit on synthetic frames: two post columns give the offset; a lone
# column, near the gate or far from it, is not taken for a post.
est = CourseEstimator(scattering=0.4)
hs = a.cfg.gate_width / 2 + a.cfg.pipe_radius


def frame(bearings):
    img = np.full((120, 160, 3), 20, np.uint8)
    for phi in bearings:
        u = int(round(80 - est.f * np.tan(phi)))
        img[:, u - 1:u + 2] = 200
    return img


g2 = est.gate(frame([np.arctan2(hs - 0.1, 2.0), np.arctan2(-hs - 0.1, 2.0)]), 0.0, 2.0, 4.0, 0.0, hs, 0.0,
              span=0.5)
lone = [est.gate(frame([np.radians(-22)]), 0.0, 4.0 - r, 4.0, 0.0, hs, 0.0, span=0.5) for r in (0.6, 2.0)]
check("gate fit from two posts", g2 is not None and abs(g2 - 0.1) < 0.02,
      "" if g2 is None else f"{g2:+.3f} m for +0.100 m")
check("gate fit refuses a lone column", lone == [None, None])

# Shared control: the camera's share follows the errors, and grows with murk.
pc = PilotConfig()
w = [blend_weight(read_error(pc, 1.5, b), load_errors(b).walls["straight"]) for b in (0.05, 1.0)]
check("camera share grows with turbidity", 0.0 < w[0] < w[1] < 1.0,
      f"{w[0]:.2f} at b 0.05, {w[1]:.2f} at b 1.0, 1.5 m")
check("equal errors split the command", abs(blend_weight(0.05, 0.05) - 0.5) < 1e-12)

# Pilot run, assisted, completes the course.
r = CourseRun(a, mode="assist", seed=2).run()
check("assisted run finishes and touches the target",
      r.finished and r.balloon_touched and not r.wrong_balloon,
      f"{r.time_s:.0f} s, {r.contacts} contacts")

# Governor: meets the cap exactly and never scales a feasible demand.
g = PowerGovernor()
full = np.full(8, 30.0)
f, k = g.limit(full)
check("governor holds input at the cap", abs(g.input_power(f) - 800.0) < 0.5,
      f"{g.input_power(full):.0f} W demanded, {g.input_power(f):.0f} W after, k {k:.3f}")
f2, k2 = g.limit(np.full(8, 8.0))
check("governor leaves a cruise demand alone", k2 == 1.0)

# Supervisor: link loss holds, recovers; kill latches.
s = Supervisor(0.5)
s.pilot_packet(0.0)
check("armed with a live link", s.update(0.3) == "ARMED")
check("link loss zeroes thrust", s.update(0.8) == "HOLD" and not s.gate(full).any())
s.pilot_packet(0.9)
check("link recovery re-arms", s.update(0.95) == "ARMED")
s.update(1.0, kill=True)
s.pilot_packet(1.1)
check("kill latches through pilot traffic", s.update(1.2) == "KILLED")

# Zero thrust from the deepest point: the vehicle rises on its own buoyancy.
dyn = VehicleDynamics(isonavi_rov)
dyn.reset(eta=[18.0, 0, -1.6, 0, 0, 0])
t = 0.0
while dyn.eta[2] < -0.2 and t < 120:
    dyn.step(np.zeros(6), 0.02)
    t += 0.02
check("unpowered vehicle surfaces from 1.6 m", dyn.eta[2] >= -0.2, f"{t:.0f} s")

print()
print("smoke test complete" if not FAILS else f"{len(FAILS)} failed")
sys.exit(1 if FAILS else 0)
