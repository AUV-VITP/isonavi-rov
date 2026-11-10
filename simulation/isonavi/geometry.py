"""Vectorised ray casting against analytic primitives and a heightfield.

All intersection routines take ray origins ``o`` of shape (N, 3) and unit
directions ``d`` of shape (N, 3) and return a ``Hit`` describing the nearest
intersection along each ray. Everything is written against NumPy so a full
camera frame (thousands of rays) is evaluated in a single vectorised call.

Coordinate convention: right handed, ``z`` positive up, ``z = 0`` at the water
surface, so submerged geometry has negative ``z``.
"""

from __future__ import annotations

import numpy as np

INF = np.inf


class Hit:
    """Nearest-intersection record for a batch of rays."""

    __slots__ = ("t", "normal", "material")

    def __init__(self, n: int):
        self.t = np.full(n, INF, dtype=np.float64)
        self.normal = np.zeros((n, 3), dtype=np.float64)
        self.material = np.full(n, -1, dtype=np.int32)

    def merge(self, t, normal, material) -> None:
        """Keep whichever of self / candidate is closer, per ray."""
        better = t < self.t
        if not np.any(better):
            return
        self.t[better] = t[better]
        self.normal[better] = normal[better]
        self.material[better] = material

    @property
    def valid(self):
        return np.isfinite(self.t)


def _normalise(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


class Primitive:
    """Base class. ``material`` indexes into the acoustic material table."""

    def __init__(self, material: int, name: str = ""):
        self.material = int(material)
        self.name = name

    def intersect(self, o, d, hit: Hit) -> None:
        raise NotImplementedError


class Plane(Primitive):
    """Infinite plane through ``point`` with unit ``normal``."""

    def __init__(self, point, normal, material, name=""):
        super().__init__(material, name)
        self.point = np.asarray(point, dtype=np.float64)
        self.normal = _normalise(np.asarray(normal, dtype=np.float64))

    def intersect(self, o, d, hit):
        denom = d @ self.normal
        ok = np.abs(denom) > 1e-9
        t = np.full(len(o), INF)
        t[ok] = ((self.point - o[ok]) @ self.normal) / denom[ok]
        t[t <= 1e-6] = INF
        nrm = np.broadcast_to(self.normal, (len(o), 3)).copy()
        flip = denom > 0
        nrm[flip] = -nrm[flip]
        hit.merge(t, nrm, self.material)


class Box(Primitive):
    """Oriented box: centre, half-extents, and a yaw about ``z``."""

    def __init__(self, center, half_extents, material, yaw=0.0, name=""):
        super().__init__(material, name)
        self.center = np.asarray(center, dtype=np.float64)
        self.half = np.asarray(half_extents, dtype=np.float64)
        self.yaw = float(yaw)
        c, s = np.cos(yaw), np.sin(yaw)
        self.R = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])

    def intersect(self, o, d, hit):
        n = len(o)
        ol = (o - self.center) @ self.R.T
        dl = d @ self.R.T
        inv = 1.0 / np.where(np.abs(dl) < 1e-12, 1e-12, dl)
        t0 = (-self.half - ol) * inv
        t1 = (self.half - ol) * inv
        tmin = np.minimum(t0, t1)
        tmax = np.maximum(t0, t1)
        tnear = tmin.max(axis=1)
        tfar = tmax.min(axis=1)
        ok = (tnear <= tfar) & (tfar > 1e-6)
        t = np.where(ok, np.where(tnear > 1e-6, tnear, tfar), INF)
        axis = np.argmax(tmin, axis=1)
        rows = np.arange(n)
        nl = np.zeros((n, 3))
        nl[rows, axis] = -np.sign(dl[rows, axis])
        nrm = nl @ self.R
        hit.merge(t, nrm, self.material)


class Cylinder(Primitive):
    """Finite cylinder aligned with ``z``, used for bridge piers."""

    def __init__(self, center_xy, radius, z_min, z_max, material, name=""):
        super().__init__(material, name)
        self.cx, self.cy = float(center_xy[0]), float(center_xy[1])
        self.r = float(radius)
        self.z0, self.z1 = float(z_min), float(z_max)

    def intersect(self, o, d, hit):
        ox = o[:, 0] - self.cx
        oy = o[:, 1] - self.cy
        dx, dy = d[:, 0], d[:, 1]
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - self.r * self.r
        disc = b * b - 4 * a * c
        t = np.full(len(o), INF)
        ok = (disc > 0) & (a > 1e-12)
        if np.any(ok):
            sq = np.sqrt(disc[ok])
            aa = a[ok]
            t0 = (-b[ok] - sq) / (2 * aa)
            t1 = (-b[ok] + sq) / (2 * aa)
            z0 = o[ok, 2] + t0 * d[ok, 2]
            z1 = o[ok, 2] + t1 * d[ok, 2]
            cand = np.where(
                (t0 > 1e-6) & (z0 >= self.z0) & (z0 <= self.z1),
                t0,
                np.where((t1 > 1e-6) & (z1 >= self.z0) & (z1 <= self.z1), t1, INF),
            )
            t[ok] = cand
        p = o + np.where(np.isfinite(t), t, 0.0)[:, None] * d
        nrm = np.zeros((len(o), 3))
        fin = np.isfinite(t)
        if np.any(fin):
            nrm[fin, 0] = p[fin, 0] - self.cx
            nrm[fin, 1] = p[fin, 1] - self.cy
            nrm[fin] = _normalise(nrm[fin])
        hit.merge(t, nrm, self.material)


class Sphere(Primitive):
    """Boulders and rounded rubble."""

    def __init__(self, center, radius, material, name=""):
        super().__init__(material, name)
        self.c = np.asarray(center, dtype=np.float64)
        self.r = float(radius)

    def intersect(self, o, d, hit):
        oc = o - self.c
        b = 2.0 * np.einsum("ij,ij->i", oc, d)
        c = np.einsum("ij,ij->i", oc, oc) - self.r * self.r
        disc = b * b - 4 * c
        t = np.full(len(o), INF)
        ok = disc > 0
        if np.any(ok):
            sq = np.sqrt(disc[ok])
            t0 = (-b[ok] - sq) / 2.0
            t1 = (-b[ok] + sq) / 2.0
            t[ok] = np.where(t0 > 1e-6, t0, np.where(t1 > 1e-6, t1, INF))
        nrm = np.zeros((len(o), 3))
        fin = np.isfinite(t)
        if np.any(fin):
            nrm[fin] = _normalise(o[fin] + t[fin, None] * d[fin] - self.c)
        hit.merge(t, nrm, self.material)


class Heightfield(Primitive):
    """Bathymetry as z = H(x, y), sampled on a regular grid.

    Intersection uses fixed-step ray marching followed by bisection refinement.
    This is what lets the simulator represent scour holes around bridge piers,
    which is the quantity the inspection mission ultimately measures.
    """

    def __init__(self, x0, y0, dx, dy, H, material, name="riverbed",
                 max_range=120.0, step=0.35, refine=12):
        super().__init__(material, name)
        self.x0, self.y0 = float(x0), float(y0)
        self.dx, self.dy = float(dx), float(dy)
        self.H = np.asarray(H, dtype=np.float64)
        self.ny, self.nx = self.H.shape
        self.max_range = float(max_range)
        self.step = float(step)
        self.refine = int(refine)

    def height(self, x, y):
        """Bilinear sample of the height grid, clamped at the borders."""
        fx = np.clip((np.asarray(x) - self.x0) / self.dx, 0, self.nx - 1.001)
        fy = np.clip((np.asarray(y) - self.y0) / self.dy, 0, self.ny - 1.001)
        i0 = fx.astype(np.int32)
        j0 = fy.astype(np.int32)
        tx = fx - i0
        ty = fy - j0
        h00 = self.H[j0, i0]
        h10 = self.H[j0, i0 + 1]
        h01 = self.H[j0 + 1, i0]
        h11 = self.H[j0 + 1, i0 + 1]
        return (h00 * (1 - tx) * (1 - ty) + h10 * tx * (1 - ty)
                + h01 * (1 - tx) * ty + h11 * tx * ty)

    def normal_at(self, x, y, eps=0.25):
        """Central-difference surface normal."""
        hx = (self.height(x + eps, y) - self.height(x - eps, y)) / (2 * eps)
        hy = (self.height(x, y + eps) - self.height(x, y - eps)) / (2 * eps)
        n = np.stack([-hx, -hy, np.ones_like(hx)], axis=-1)
        return _normalise(n)

    def intersect(self, o, d, hit):
        n = len(o)
        t = np.full(n, INF)
        t_cur = np.full(n, self.step)
        prev_t = np.zeros(n)
        prev_above = (o[:, 2] - self.height(o[:, 0], o[:, 1])) > 0
        active = np.ones(n, dtype=bool)
        found = np.zeros(n, dtype=bool)
        nsteps = int(self.max_range / self.step)

        for _ in range(nsteps):
            if not np.any(active):
                break
            idx = np.where(active)[0]
            p = o[idx] + t_cur[idx, None] * d[idx]
            above = (p[:, 2] - self.height(p[:, 0], p[:, 1])) > 0
            crossed = prev_above[idx] & (~above)
            if np.any(crossed):
                ci = idx[crossed]
                lo = prev_t[ci].copy()
                hi = t_cur[ci].copy()
                for _ in range(self.refine):
                    mid = 0.5 * (lo + hi)
                    pm = o[ci] + mid[:, None] * d[ci]
                    am = (pm[:, 2] - self.height(pm[:, 0], pm[:, 1])) > 0
                    lo = np.where(am, mid, lo)
                    hi = np.where(am, hi, mid)
                t[ci] = 0.5 * (lo + hi)
                found[ci] = True
                active[ci] = False
            prev_above[idx] = above
            prev_t[idx] = t_cur[idx]
            t_cur[idx] += self.step
            active &= t_cur < self.max_range

        nrm = np.zeros((n, 3))
        if np.any(found):
            p = o[found] + t[found, None] * d[found]
            nrm[found] = self.normal_at(p[:, 0], p[:, 1])
        hit.merge(t, nrm, self.material)


class Scene:
    """A collection of primitives sharing one acoustic material table."""

    def __init__(self, primitives=None):
        self.primitives = list(primitives or [])

    def add(self, prim: Primitive) -> Primitive:
        self.primitives.append(prim)
        return prim

    def by_name(self, name):
        return [p for p in self.primitives if p.name == name]

    def intersect(self, o, d) -> Hit:
        o = np.ascontiguousarray(o, dtype=np.float64)
        d = _normalise(np.ascontiguousarray(d, dtype=np.float64))
        hit = Hit(len(o))
        for p in self.primitives:
            p.intersect(o, d, hit)
        return hit
