"""One camera-mode run, with every lateral measurement against the truth along the course."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isonavi.pool import PoolArena, random_arena   # noqa: E402
from isonavi.pilot import CourseRun  # noqa: E402

b = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
mode = sys.argv[2] if len(sys.argv) > 2 else "auto"
layout = int(sys.argv[3]) if len(sys.argv) > 3 else 0
arena = PoolArena(random_arena(layout)) if layout else PoolArena()
r = CourseRun(arena, mode=mode, seed=layout or 3, turbidity=b).run()
v = r.log["vision"]
names = {1: "gate", 2: "target", 3: "walls", 4: "posts"}
print(f"b {b} {mode} layout {layout}: {r.contacts} contacts, {r.time_s:.1f} s, contacts at {r.contact_list[:12]}")
for code, name in names.items():
    e = v[v[:, 1] == code, 2] - v[v[:, 1] == code, 3]
    if len(e):
        print(f"  {name:6s} n {len(e):4d}  rms {np.sqrt(np.mean(e ** 2)) * 1000:7.0f} mm  max {np.abs(e).max() * 1000:7.0f} mm")
print("  x, estimator, estimate, truth, calibrated error (every error over 0.15 m, first 40)")
bad = v[np.abs(v[:, 2] - v[:, 3]) > 0.15]
for row in bad[:40]:
    print(f"    x {row[4]:6.2f}  {names[int(row[1])]:6s}  est {row[2]:+.3f}  true {row[3]:+.3f}"
          f"  cal {row[5] * 1000:4.0f} mm")
if "--gate" in sys.argv:
    print("  gate measurements: x, estimate, truth, calibrated error")
    for row in v[v[:, 1] == 1]:
        print(f"    x {row[4]:6.2f}  est {row[2]:+.3f}  true {row[3]:+.3f}  cal {row[5] * 1000:4.0f} mm")
