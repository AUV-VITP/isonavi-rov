"""Coloured target identification from the onboard camera, without training data.

Water changes a balloon's colour before it hides it: red is absorbed several
times faster than blue, and the lamps' backscatter lays a grey veil over the
frame. A fixed colour threshold therefore works at one range in one water and
nowhere else. This pipeline undoes the water instead, using the one thing
known in advance about the target, its size.

  1. veil      the backscatter veil is estimated per channel as a robust
               background level of the frame and removed;
  2. blobs     what remains above the noise floor is segmented into
               connected regions, kept if roughly round;
  3. range     range follows from apparent size against the known radius;
  4. colour    each channel is multiplied by exp(2 c r) to undo the two way
               attenuation at that range, and the corrected chromaticity is
               matched to the nearest reference albedo.

The attenuation coefficient comes from the same water model the camera uses.
On the vehicle it is identified once per pool from a reference target of
known albedo at a measured range, which is a two minute calibration.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from .optics import WATER_ABSORPTION
from .pool import OPTICAL_MATERIALS

COLOURS = ("red", "green", "blue", "yellow")
REF = {m.name.split("_")[1]: np.array(m.albedo) for m in OPTICAL_MATERIALS
       if m.name.startswith("balloon_")}


def _chroma(rgb):
    rgb = np.clip(np.asarray(rgb, float), 1e-6, None)
    return rgb / rgb.sum()


@dataclass
class Detection:
    colour: str
    centroid: tuple          # pixel u, v
    radius_px: float
    range_m: float
    score: float             # chromaticity distance to the best match, lower is better
    raw_colour: str          # what a fixed threshold on the uncorrected pixels would say


class TargetFinder:
    def __init__(self, width=160, hfov_deg=80.0, target_radius=0.12,
                 scattering=0.4, min_px=6, correct=True, max_range=3.5):
        self.f = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
        self.R = target_radius
        self.c = WATER_ABSORPTION + scattering
        self.min_px = min_px
        self.correct = correct
        # A blob whose size implies a range the lamps cannot light is speckle.
        # Left in, the attenuation correction at that range turns it any colour.
        self.max_range = max_range

    @staticmethod
    def linear(img_u8):
        return (img_u8.astype(float) / 255.0) ** 2.2

    def find(self, img_u8):
        lin = self.linear(img_u8)
        veil = np.percentile(lin.reshape(-1, 3), 40, axis=0)
        sig = np.clip(lin - veil, 0, None)
        lum = sig.sum(axis=2)
        noise = np.percentile(lum, 60) + 1e-4
        mask = ndimage.binary_opening(lum > 6.0 * noise, iterations=1)
        lab, n = ndimage.label(mask)
        out = []
        for k in range(1, n + 1):
            ys, xs = np.nonzero(lab == k)
            if len(xs) < self.min_px:
                continue
            w, h = np.ptp(xs) + 1, np.ptp(ys) + 1
            fill = len(xs) / (w * h)
            if not (0.5 < w / h < 2.0 and fill > 0.5):
                continue
            r_px = 0.5 * np.sqrt(w * h)
            rng = self.f * self.R / max(r_px, 1e-6)
            if rng > self.max_range:
                continue
            rgb = sig[ys, xs].mean(axis=0)
            raw = min(COLOURS, key=lambda c: np.linalg.norm(_chroma(rgb) - _chroma(REF[c])))
            if self.correct:
                rgb = rgb * np.exp(2.0 * self.c * rng)
            ch = _chroma(rgb)
            d = {c: float(np.linalg.norm(ch - _chroma(REF[c]))) for c in COLOURS}
            best = min(d, key=d.get)
            out.append(Detection(best, (float(xs.mean()), float(ys.mean())), float(r_px),
                                 float(rng), d[best], raw))
        return out

    def nearest_to_centre(self, img_u8):
        dets = self.find(img_u8)
        if not dets:
            return None
        h, w = img_u8.shape[:2]
        return min(dets, key=lambda d: np.hypot(d.centroid[0] - w / 2, d.centroid[1] - h / 2))


# ------------------------------------------------------------ course references
class CourseEstimator:
    """Lateral position against the course from one camera frame.

    gate     the posts are found as narrow bright columns; with the distance to
             the gate known, each post's bearing, rotated by the IMU heading,
             gives the lateral offset on its own.
    target   bearing and range-from-size of the detection of the target colour.
    posts    bearings to the known obstacle posts ahead, each matched to the post
             whose predicted bearing is nearest.
    corridor a lamp-lit wall dims as the square of its distance, so the log
             ratio of left to right wall brightness is s = 2 ln((w/2+y)/(w/2-y))
             for a camera y off the centreline. Inverted, y = (w/2) tanh(g s / 4)
             plus a heading term; g and the heading gain are fitted on rendered
             frames per water clarity (eval/vision_calibrate.py).

    Every method returns the camera's world y, or None when the frame does not
    support a measurement.
    """

    BAND = (0.30, 0.70)        # rows used for the column profile and the wall bands
    EDGE = 0.18                # fraction of the width read as each wall band

    def __init__(self, width=160, height=120, hfov_deg=80.0, scattering=0.4, calib=None):
        self.w, self.h = width, height
        self.f = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
        self.finder = TargetFinder(width=width, hfov_deg=hfov_deg, scattering=scattering)
        self.calib = calib or {"g": 1.0, "k_yaw": 0.0, "bias": 0.0}

    def bearing(self, u):
        return float(np.arctan2(-(u - self.w / 2), self.f))

    # ---------------------------------------------------------------- gate
    def post_columns(self, img_u8, max_width_px=44, bg_px=81):
        """Bearings of vertical bright bars, as pixel columns.

        A post is bright down its whole height and speckle is not, so each row
        is high-passed against a running median wider than any post, and a
        column scores by its median over the rows. Walls are wider than the
        filter and fold into the background. The width limit is a post's
        apparent width at the closest range it is used from.
        """
        lin = TargetFinder.linear(img_u8).sum(axis=2)
        r0, r1 = int(self.BAND[0] * self.h), int(self.BAND[1] * self.h)
        band = lin[r0:r1]
        bg = ndimage.median_filter(band, size=(1, bg_px), mode="nearest")
        score = np.median(band - bg, axis=0)
        mad = np.median(np.abs(score - np.median(score))) + 1e-6
        mask = score > max(5.0 * mad, 0.002)
        lab, n = ndimage.label(mask)
        cols = []
        for k in range(1, n + 1):
            us = np.nonzero(lab == k)[0]
            if 1 <= len(us) <= max_width_px:
                cols.append(float(np.average(us, weights=score[us])))
        return cols

    def gate(self, img_u8, yaw, x_cam, gate_x, gate_y, half_span, prior_y, span=0.25):
        """Camera y from the gate posts, or None.

        With the distance to the gate known, each post is a bearing at a known
        range, so one post is enough and a narrow object elsewhere in the frame
        is rejected by its bearing rather than paired with a post.
        """
        return self.posts(img_u8, yaw, x_cam,
                          [(gate_x, gate_y + half_span), (gate_x, gate_y - half_span)], prior_y,
                          span=span)

    # -------------------------------------------------------------- target
    def target(self, img_u8, yaw, target_pos, colour, max_range=1.6, max_bearing_deg=25.0):
        # Beyond about 1.6 m, or near the edge of the frame where the lamp beam
        # falls off, the blob shrinks and range from size reads long.
        dets = [d for d in self.finder.find(img_u8) if d.colour == colour
                and d.range_m <= max_range
                and abs(self.bearing(d.centroid[0])) <= np.radians(max_bearing_deg)]
        if not dets:
            return None
        d = min(dets, key=lambda d: d.range_m)            # the nearest, largest return
        phi = yaw + self.bearing(d.centroid[0])
        return float(target_pos[1] - d.range_m * np.sin(phi))

    # ----------------------------------------------------------- obstacles
    def posts(self, img_u8, yaw, x_cam, posts_xy, prior_y, span=0.25, cap_deg=4.0,
              accept_deg=1.5, prior_weight=0.05):
        """Camera y from bearings to known posts ahead, or None.

        A joint fit rather than post-by-post matching: for each candidate y near
        the prior, every visible post has a predicted bearing, and the candidate
        is scored by how well those bearings explain all the bright columns at
        once, each column's miss capped so a stray column cannot dominate. With
        several posts at different ranges, matching each column to its nearest
        prediction on its own pairs a far post with a near one; the joint fit
        cannot, because one y has to explain them all.
        """
        cols = self.post_columns(img_u8)
        if not cols:
            return None
        phi_c = np.array([yaw + self.bearing(u) for u in cols])
        ahead = [(xp, yp) for xp, yp in posts_xy if xp - x_cam >= 0.15]
        if not ahead:
            return None
        xp = np.array([p[0] for p in ahead]) - x_cam
        yp = np.array([p[1] for p in ahead])
        ys = prior_y + np.linspace(-span, span, 201)
        pred = np.arctan2(yp[None, :] - ys[:, None], xp[None, :])       # (candidates, posts)
        half = np.radians(40.0)
        visible = np.abs(pred - yaw) < half
        miss = np.abs(phi_c[None, :, None] - pred[:, None, :])          # (cand, cols, posts)
        miss = np.where(visible[:, None, :], miss, np.inf).min(axis=2)
        cap = np.radians(cap_deg)
        cost = np.minimum(miss, cap) ** 2
        cost = cost.sum(axis=1) + prior_weight * cap ** 2 * ((ys - prior_y) / span) ** 2
        k = int(np.argmin(cost))
        explained = miss[k] < np.radians(accept_deg)
        if not explained.any():
            return None
        return float(ys[k])

    # ------------------------------------------------------------ corridor
    def wall_ratio(self, img_u8):
        lin = TargetFinder.linear(img_u8).sum(axis=2)
        r0, r1 = int(self.BAND[0] * self.h), int(self.BAND[1] * self.h)
        e = int(self.EDGE * self.w)
        left = lin[r0:r1, :e].mean()
        right = lin[r0:r1, -e:].mean()
        return float(np.log((left + 1e-5) / (right + 1e-5)))

    def corridor(self, img_u8, yaw, maze_y, maze_width):
        s = self.wall_ratio(img_u8)
        c = self.calib
        off = (maze_width / 2) * np.tanh(c["g"] * s / 4.0) + c["k_yaw"] * yaw + c["bias"]
        return float(maze_y + off)


def load_calibration(scattering):
    """Corridor estimator calibration nearest in water clarity, if it has been run."""
    import json
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "logs",
                        "vision_calibration.json")
    if not os.path.exists(path):
        return None
    cal = json.load(open(path))["fits"]
    key = min(cal, key=lambda k: abs(float(k) - scattering))
    return cal[key]
