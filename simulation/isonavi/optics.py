"""Physics-based underwater camera image formation under the vehicle's own lamps.

In an unlit pool the only light is what the vehicle carries, so the image is
set by three things the design controls: lamp power, lamp beam, and where the
lamps sit relative to the lens. Per pixel and per colour channel:

    signal       L = rho / pi * sum_lamps I0 b(theta) cos(i) exp(-c r_l) / r_l^2 * exp(-c t)
    backscatter  B = integral_0^t beta * sum_lamps I0 b(theta_s) exp(-c r_l(s)) / r_l(s)^2 * exp(-c s) ds

c is the beam attenuation coefficient (absorption plus scattering), beta the
volume backscatter coefficient, rho the surface albedo, t the range to the
first surface along the camera ray and r_l the lamp to point range. Red is
absorbed roughly five times faster than blue in water, which is why colour
identification degrades with range before detection does. Backscatter is
integrated numerically along every ray, so lamp to lens separation shows up
in the image the way it does in a real one: lamps next to the lens light the
water in front of it.

The sensor adds shot and read noise and an 8 bit, gamma 2.2 output.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .dynamics import rot_body_to_world
from .pool import ALBEDO

# Pure water absorption at the RGB channel centres (approx. 600, 530, 460 nm), 1/m.
WATER_ABSORPTION = np.array([0.25, 0.05, 0.015])


@dataclass
class WaterConfig:
    """Optical water properties. Turbidity adds wavelength-flat scattering."""

    scattering: float = 0.4          # b, 1/m; clear pool ~0.05, murky ~1.0
    backscatter_fraction: float = 0.02

    @property
    def c(self) -> np.ndarray:
        return WATER_ABSORPTION + self.scattering

    @property
    def beta(self) -> float:
        # Volume backscatter per steradian, from the backscatter fraction of b.
        return self.backscatter_fraction * self.scattering / (2 * np.pi)


@dataclass
class Lamp:
    pos: tuple                       # body frame, metres from the camera
    intensity: float = 1.0           # on-axis radiant intensity, arbitrary units
    beam_exp: float = 4.0            # cos^k beam pattern
    axis: tuple = (1.0, 0.0, 0.0)


@dataclass
class CameraConfig:
    width: int = 160
    height: int = 120
    hfov_deg: float = 80.0           # in water, behind a flat port
    pos: tuple = (0.30, 0.0, 0.0)    # body frame
    exposure: float = 2.0            # gain from radiance to full scale
    read_noise: float = 0.004
    full_well: float = 4000.0        # electrons at full scale, sets shot noise
    backscatter_samples: int = 24
    lamps: tuple = field(default_factory=lambda: (
        Lamp((0.0, 0.18, 0.0)), Lamp((0.0, -0.18, 0.0))))


class Camera:
    def __init__(self, cfg: CameraConfig | None = None,
                 water: WaterConfig | None = None, seed=0):
        self.cfg = cfg or CameraConfig()
        self.water = water or WaterConfig()
        self.rng = np.random.default_rng(seed)
        c = self.cfg
        f = (c.width / 2) / np.tan(np.radians(c.hfov_deg) / 2)
        u, v = np.meshgrid(np.arange(c.width) + 0.5, np.arange(c.height) + 0.5)
        d = np.stack([np.full(u.size, f), -(u.ravel() - c.width / 2),
                      -(v.ravel() - c.height / 2)], axis=1)
        self.rays_body = d / np.linalg.norm(d, axis=1, keepdims=True)

    def _irradiance(self, pts, lamps_w, axes_w, normals=None):
        """Lamp light arriving at points, per channel, after attenuation."""
        c = self.water.c
        E = np.zeros((pts.shape[0], 3))
        for lamp, lp, ax in zip(self.cfg.lamps, lamps_w, axes_w):
            v = pts - lp
            r = np.maximum(np.linalg.norm(v, axis=1), 1e-3)
            u = v / r[:, None]
            cos_b = np.clip(u @ ax, 0.0, 1.0)
            w = lamp.intensity * cos_b ** lamp.beam_exp / r ** 2
            if normals is not None:
                w = w * np.clip(-(u * normals).sum(axis=1), 0.0, 1.0)
            E += w[:, None] * np.exp(-np.outer(r, c))
        return E

    def render(self, scene, eta, noise=True):
        """Return (image uint8 HxWx3, linear radiance, range map)."""
        cfg, water = self.cfg, self.water
        R = rot_body_to_world(*eta[3:6])
        o_cam = np.asarray(eta[:3], float) + R @ np.asarray(cfg.pos, float)
        d = self.rays_body @ R.T
        o = np.broadcast_to(o_cam, d.shape).copy()
        hit = scene.intersect(o, d)
        t = np.where(np.isfinite(hit.t), hit.t, 30.0)
        lamps_w = [o_cam + R @ np.asarray(l.pos, float) for l in cfg.lamps]
        axes_w = [R @ (np.asarray(l.axis, float) / np.linalg.norm(l.axis)) for l in cfg.lamps]

        c = water.c
        L = np.zeros((d.shape[0], 3))
        ok = np.isfinite(hit.t)
        if ok.any():
            p = o[ok] + d[ok] * t[ok, None]
            rho = ALBEDO[np.clip(hit.material[ok], 0, len(ALBEDO) - 1)]
            E = self._irradiance(p, lamps_w, axes_w, normals=hit.normal[ok])
            L[ok] = rho / np.pi * E * np.exp(-np.outer(t[ok], c))

        # Backscatter, midpoint rule along each camera ray.
        n = cfg.backscatter_samples
        B = np.zeros_like(L)
        ds = t / n
        for k in range(n):
            s = (k + 0.5) * ds
            p = o + d * s[:, None]
            B += water.beta * self._irradiance(p, lamps_w, axes_w) \
                * np.exp(-np.outer(s, c)) * ds[:, None]
        radiance = L + B

        img = radiance * cfg.exposure
        if noise:
            e = np.clip(img, 0, None) * cfg.full_well
            img = self.rng.poisson(e) / cfg.full_well
            img = img + self.rng.normal(0, cfg.read_noise, img.shape)
        img = np.clip(img, 0.0, 1.0) ** (1 / 2.2)
        shape = (cfg.height, cfg.width, 3)
        return ((img * 255).astype(np.uint8).reshape(shape),
                radiance.reshape(shape), t.reshape(shape[:2]))
