"""Operator console video of one course run: onboard camera beside the track.

The camera panel is what the pilot sees, rendered by the same camera model
the vision results use; the right panel is the track over the course with
the live mode, time, depth and contact count.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from multiprocessing import Pool

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
from isonavi.pool import PoolArena                  # noqa: E402
from isonavi.pilot import CourseRun                 # noqa: E402
from isonavi.optics import Camera, CameraConfig, WaterConfig   # noqa: E402
from eval.rov_figures import draw_course            # noqa: E402

OUT = os.path.join(ROOT, "results", "video")
FRAMES = os.path.join(OUT, "rov_frames")

_state = {}


def _init(mode, b, seed):
    a = PoolArena()
    r = CourseRun(a, mode=mode, seed=seed, turbidity=b).run()
    _state.update(a=a, r=r, b=b, mode=mode,
                  cam=Camera(CameraConfig(width=320, height=240), WaterConfig(scattering=b), seed=seed))


def frame(k):
    a, r, cam = _state["a"], _state["r"], _state["cam"]
    t, eta = r.log["t"][k], r.log["eta"][k]
    img, _, _ = cam.render(a.scene, eta)
    fig = plt.figure(figsize=(12.8, 4.8), dpi=100)
    ax0 = fig.add_axes([0.01, 0.06, 0.40, 0.86])
    ax0.imshow(img)
    ax0.axis("off")
    ax0.set_title("onboard camera", fontsize=10, loc="left")
    ax1 = fig.add_axes([0.45, 0.12, 0.53, 0.78])
    draw_course(ax1, a)
    e = r.log["eta"][: k + 1]
    ax1.plot(e[:, 0], e[:, 1], color="#1f6fb2", lw=1.2)
    ax1.plot(eta[0], eta[1], "o", color="#1f6fb2", ms=5)
    yaw = eta[5]
    ax1.arrow(eta[0], eta[1], 0.5 * np.cos(yaw), 0.5 * np.sin(yaw), width=0.03,
              color="#1f6fb2", length_includes_head=True)
    n_c = int(r.log["contacts"][k])
    for tc, _ in r.contact_list:
        if tc <= t:
            j = int(np.argmin(np.abs(r.log["t"] - tc)))
            ax1.plot(r.log["eta"][j, 0], r.log["eta"][j, 1], "x", color="#c0392b", ms=7, mew=2)
    ax1.set_xlim(0, 20)
    ax1.set_ylim(-2.6, 1.4)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x along the pool, m")
    ax1.set_title(f"mode {_state['mode']}   water b = {_state['b']} /m   t {t:5.1f} s   "
                  f"depth {-eta[2]:.2f} m   contacts {n_c}", fontsize=10, loc="left")
    fig.savefig(os.path.join(FRAMES, f"f{k:05d}.png"))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="shared")
    ap.add_argument("--b", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--speed", type=float, default=2.0, help="playback speed")
    args = ap.parse_args()
    shutil.rmtree(FRAMES, ignore_errors=True)
    os.makedirs(FRAMES)
    _init(args.mode, args.b, args.seed)
    n = len(_state["r"].log["t"])          # log is at 10 Hz
    with Pool(max(1, (os.cpu_count() or 2) - 1), initializer=_init,
              initargs=(args.mode, args.b, args.seed)) as pool:
        pool.map(frame, range(n), chunksize=8)
    fps = int(round(10 * args.speed))
    out = os.path.join(OUT, f"rov_course_{args.mode}.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps),
                    "-i", os.path.join(FRAMES, "f%05d.png"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "23", out], check=True)
    shutil.rmtree(FRAMES, ignore_errors=True)
    print(out, f"{n / fps:.0f} s")


if __name__ == "__main__":
    main()
