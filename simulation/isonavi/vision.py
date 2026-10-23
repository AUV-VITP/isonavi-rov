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
                 scattering=0.4, min_px=6, correct=True):
        self.f = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
        self.R = target_radius
        self.c = WATER_ABSORPTION + scattering
        self.min_px = min_px
        self.correct = correct

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
