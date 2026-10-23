"""Pool arena: a submerged gate, coloured balloon targets and a three section maze.

The pool is 20 m by 10 m with the floor sloping from 0.6 m deep at the start
end to 1.8 m at the far end. Course elements are thin PVC structures and
tethered balloons. Their placement is parametric, because the layout is what
changes between events while the element sizes do not: a 1 m gate, a
straight corridor, an obstacle section and a covered passage with walls up to
1.5 m high.

Besides the ray-castable Scene, the arena answers one question the dynamics
cannot: whether the vehicle hull is touching anything. Each element carries a
signed distance function, and the hull is sampled as points on its bounding
box, so a contact is any sample at or inside an element's surface.

Frame convention matches the rest of the package: x along the pool from the
start wall, y across it, z up with z = 0 at the water surface.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import Scene, Box, Cylinder, Sphere, Plane


@dataclass(frozen=True)
class OpticalMaterial:
    name: str
    albedo: tuple          # diffuse reflectance, linear RGB


OPTICAL_MATERIALS = (
    OpticalMaterial("pool_tile", (0.55, 0.62, 0.66)),
    OpticalMaterial("pvc_white", (0.80, 0.80, 0.78)),
    OpticalMaterial("pvc_grey", (0.35, 0.36, 0.37)),
    OpticalMaterial("balloon_red", (0.70, 0.06, 0.05)),
    OpticalMaterial("balloon_green", (0.06, 0.55, 0.12)),
    OpticalMaterial("balloon_blue", (0.05, 0.12, 0.65)),
    OpticalMaterial("balloon_yellow", (0.78, 0.68, 0.05)),
)
OPT_INDEX = {m.name: i for i, m in enumerate(OPTICAL_MATERIALS)}
ALBEDO = np.array([m.albedo for m in OPTICAL_MATERIALS])


@dataclass
class ArenaConfig:
    """Pool and course geometry, metres."""

    length: float = 20.0
    width: float = 10.0
    depth_start: float = 0.6
    depth_end: float = 1.8

    start: tuple = (1.2, 0.0)            # launch point, x and y

    gate_x: float = 4.0
    gate_y: float = 0.0
    gate_width: float = 1.0              # clear opening between post faces
    gate_top_depth: float = 0.25         # crossbar below the surface
    pipe_radius: float = 0.025           # 50 mm PVC

    balloon_x: float = 7.5
    balloon_y: float = 0.0               # centre of the target line
    balloon_radius: float = 0.12
    balloon_depth: float = 0.55          # centre below the surface
    balloon_spacing: float = 1.2
    balloon_colours: tuple = ("red", "green", "blue", "yellow")
    target_colour: str = "red"

    maze_x0: float = 10.0
    maze_width: float = 1.0              # corridor clear width
    section_length: tuple = (2.5, 3.0, 2.5)   # straight, obstacles, passage
    wall_height: float = 1.5             # above the local floor, capped below the surface
    wall_thickness: float = 0.05
    obstacle_offsets: tuple = (0.25, -0.25, 0.25)   # posts alternate across the corridor
    passage_roof_depth: float = 0.45     # cover over the last section
    maze_y: float = 0.0


def random_arena(seed) -> ArenaConfig:
    """A course layout drawn from the range an event could plausibly set.

    Element sizes stay fixed; placement, the order of the colours, which one
    is the target and the corridor width vary. Every draw leaves the vehicle
    a passable line, so a failure is the pilot's or the vehicle's, not the
    layout's.
    """
    rng = np.random.default_rng(seed)
    cols = tuple(rng.permutation(["red", "green", "blue", "yellow"]))
    sign = rng.choice([-1.0, 1.0])
    mag = rng.uniform(0.20, 0.30, 3)
    return ArenaConfig(
        gate_x=float(rng.uniform(3.0, 5.0)),
        gate_y=float(rng.uniform(-1.0, 1.0)),
        balloon_x=float(rng.uniform(7.0, 8.5)),
        balloon_y=float(rng.uniform(-0.8, 0.8)),
        balloon_spacing=float(rng.uniform(1.0, 1.5)),
        balloon_depth=float(rng.uniform(0.45, 0.70)),
        balloon_colours=cols,
        target_colour=str(rng.choice(cols)),
        maze_width=float(rng.uniform(0.90, 1.20)),
        obstacle_offsets=tuple(float(sign * m * (-1) ** k) for k, m in enumerate(mag)),
        passage_roof_depth=float(rng.uniform(0.40, 0.55)),
    )


class PoolArena:
    """Builds the Scene and the contact model for one course layout."""

    def __init__(self, cfg: ArenaConfig | None = None):
        self.cfg = cfg or ArenaConfig()
        self.scene = Scene()
        self.elements: list = []          # (name, sdf) for contact checks
        self.targets: dict[str, dict] = {}
        self.sections: dict[str, tuple] = {}
        self._build()

    # ------------------------------------------------------------------ floor
    def floor_z(self, x):
        c = self.cfg
        s = np.clip(np.asarray(x, float) / c.length, 0.0, 1.0)
        return -(c.depth_start + s * (c.depth_end - c.depth_start))

    # ------------------------------------------------------------------ build
    def _add_box(self, name, centre, half, mat, yaw=0.0):
        self.scene.add(Box(centre, half, OPT_INDEX[mat], yaw=yaw, name=name))
        centre, half = np.asarray(centre, float), np.asarray(half, float)
        cy, sy = np.cos(yaw), np.sin(yaw)

        def sdf(p):
            d = p - centre
            local = np.stack([cy * d[:, 0] + sy * d[:, 1],
                              -sy * d[:, 0] + cy * d[:, 1], d[:, 2]], axis=1)
            q = np.abs(local) - half
            out = np.linalg.norm(np.maximum(q, 0.0), axis=1)
            return out + np.minimum(np.max(q, axis=1), 0.0)
        self.elements.append((name, sdf))

    def _add_post(self, name, x, y, z0, z1, mat="pvc_white"):
        r = self.cfg.pipe_radius
        self.scene.add(Cylinder((x, y), r, z0, z1, OPT_INDEX[mat], name=name))

        def sdf(p):
            radial = np.hypot(p[:, 0] - x, p[:, 1] - y) - r
            axial = np.maximum(z0 - p[:, 2], p[:, 2] - z1)
            out = np.hypot(np.maximum(radial, 0), np.maximum(axial, 0))
            return out + np.minimum(np.maximum(radial, axial), 0.0)
        self.elements.append((name, sdf))

    def _add_balloon(self, name, centre, colour):
        r = self.cfg.balloon_radius
        centre = np.asarray(centre, float)
        self.scene.add(Sphere(centre, r, OPT_INDEX[f"balloon_{colour}"], name=name))
        self.elements.append((name, lambda p: np.linalg.norm(p - centre, axis=1) - r))
        self.targets[name] = {"pos": centre, "colour": colour, "radius": r}

    def _build(self):
        c = self.cfg
        tile = OPT_INDEX["pool_tile"]
        slope = (c.depth_end - c.depth_start) / c.length
        self.scene.add(Plane((0, 0, -c.depth_start), (slope, 0, 1), tile, "floor"))
        self.scene.add(Plane((0, 0, 0), (1, 0, 0), tile, "wall_start"))
        self.scene.add(Plane((c.length, 0, 0), (-1, 0, 0), tile, "wall_end"))
        self.scene.add(Plane((0, -c.width / 2, 0), (0, 1, 0), tile, "wall_right"))
        self.scene.add(Plane((0, c.width / 2, 0), (0, -1, 0), tile, "wall_left"))

        # Gate: two posts and a crossbar, opening measured between post faces.
        r = c.pipe_radius
        zf = float(self.floor_z(c.gate_x))
        zt = -c.gate_top_depth
        yp = c.gate_width / 2 + r
        self._add_post("gate_post_left", c.gate_x, c.gate_y + yp, zf, zt)
        self._add_post("gate_post_right", c.gate_x, c.gate_y - yp, zf, zt)
        self._add_box("gate_crossbar", (c.gate_x, c.gate_y, zt + r), (r, yp + r, r), "pvc_white")
        self.sections["gate"] = (c.gate_x - 0.5, c.gate_x + 0.5)

        # Balloons on a line across the pool, each on a short tether from the floor.
        n = len(c.balloon_colours)
        for i, col in enumerate(c.balloon_colours):
            y = c.balloon_y + (i - (n - 1) / 2) * c.balloon_spacing
            self._add_balloon(f"balloon_{col}", (c.balloon_x, y, -c.balloon_depth), col)
        self.sections["targets"] = (c.balloon_x - 1.0, c.balloon_x + 1.0)

        # Maze: one corridor in three sections.
        x = c.maze_x0
        names = ("straight", "obstacles", "passage")
        for name, ln in zip(names, c.section_length):
            self.sections[name] = (x, x + ln)
            x += ln
        x0, x1 = c.maze_x0, x
        zf0 = float(self.floor_z(x1))
        top = min(zf0 + c.wall_height, -0.05)
        hx, ht = (x1 - x0) / 2, c.wall_thickness / 2
        zc, hz = (zf0 + top) / 2, (top - zf0) / 2
        for side, sgn in (("left", 1), ("right", -1)):
            yc = c.maze_y + sgn * (c.maze_width / 2 + ht)
            self._add_box(f"maze_wall_{side}", ((x0 + x1) / 2, yc, zc), (hx, ht, hz), "pvc_grey")

        ox0, ox1 = self.sections["obstacles"]
        for k, off in enumerate(c.obstacle_offsets):
            ox = ox0 + (k + 1) * (ox1 - ox0) / (len(c.obstacle_offsets) + 1)
            self._add_post(f"obstacle_{k + 1}", ox, c.maze_y + off,
                           float(self.floor_z(ox)), top, "pvc_white")

        px0, px1 = self.sections["passage"]
        self._add_box("passage_roof", ((px0 + px1) / 2, c.maze_y, -c.passage_roof_depth),
                      ((px1 - px0) / 2, c.maze_width / 2 + c.wall_thickness, r), "pvc_grey")

    # ---------------------------------------------------------------- contact
    # Two tests, because neither covers both shapes. Hull sample points against
    # each element's distance function catch the vehicle meeting a wall or a
    # roof, where a corner or edge arrives first. A post or a crossbar is thinner
    # than the spacing of those points, so it is sampled along its own axis and
    # tested against the vehicle box instead.

    @staticmethod
    def hull_body_points(dims, n=4):
        L, B, H = dims
        g = np.linspace(-0.5, 0.5, n)
        pts = np.array([(a, b, c) for a in g for b in g for c in g
                        if max(abs(a), abs(b), abs(c)) == 0.5])
        return pts * np.array([L, B, H])

    @staticmethod
    def hull_points(eta, dims, n=4):
        """Sample points on the surface of the vehicle bounding box, in the world.

        eta is the pose (x, y, z, roll, pitch, yaw); dims are (L, B, H).
        """
        from .dynamics import rot_body_to_world
        R = rot_body_to_world(*eta[3:6])
        return PoolArena.hull_body_points(dims, n) @ R.T + np.asarray(eta[:3], float)

    def _rods(self):
        """Axis samples and radius of every post and thin bar, built once."""
        if hasattr(self, "_rod_cache"):
            return self._rod_cache
        rods = []
        for prim in self.scene.primitives:
            name = getattr(prim, "name", "")
            if isinstance(prim, Cylinder):
                z = np.arange(prim.z0, prim.z1 + 1e-9, 0.02)
                rods.append((name, np.stack([np.full_like(z, prim.cx), np.full_like(z, prim.cy), z], 1),
                             prim.r))
            elif isinstance(prim, Box) and name in ("gate_crossbar",):
                c, h = np.asarray(prim.center, float), np.asarray(prim.half, float)
                y = np.arange(c[1] - h[1], c[1] + h[1] + 1e-9, 0.02)
                rods.append((name, np.stack([np.full_like(y, c[0]), y, np.full_like(y, c[2])], 1),
                             float(max(h[0], h[2]))))
        self._rod_cache = rods
        return rods

    @staticmethod
    def _box_sdf(p_body, half):
        q = np.abs(p_body) - half
        return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(np.max(q, axis=1), 0.0)

    def _probe(self, eta, dims, extra):
        from .dynamics import rot_body_to_world
        R = rot_body_to_world(*eta[3:6])
        pb = self.hull_body_points(dims)
        if extra is not None:
            pb = np.vstack([pb, np.asarray(extra, float)])
        return R, pb, pb @ R.T + np.asarray(eta[:3], float)

    def contacts(self, eta, dims, margin=0.0, skip=None, extra=None):
        """Names of course elements the vehicle is touching, and the minimum clearance.

        Elements whose name starts with ``skip`` are left out, so a target the
        vehicle is meant to touch does not count against its clearance.
        extra are body-frame points outside the box, such as a probe tip.
        """
        R, _, p = self._probe(eta, dims, extra)
        touched, clearance = [], np.inf
        rod_names = {n for n, _, _ in self._rods()}
        for name, sdf in self.elements:
            if (skip and name.startswith(skip)) or name in rod_names:
                continue
            d = float(np.min(sdf(p)))
            clearance = min(clearance, d)
            if d <= margin:
                touched.append(name)
        half = np.asarray(dims, float) / 2
        for name, pts, r in self._rods():
            pb = (pts - np.asarray(eta[:3], float)) @ R
            d = float(np.min(self._box_sdf(pb, half))) - r
            if extra is not None:
                # The probe can strike a post too.
                ex = np.asarray(extra, float) @ R.T + np.asarray(eta[:3], float)
                dx = np.linalg.norm(ex[:, None, :2] - pts[None, :, :2], axis=2) - r
                inz = (ex[:, None, 2] >= pts[None, :, 2].min()) & (ex[:, None, 2] <= pts[None, :, 2].max())
                if inz.any():
                    d = min(d, float(np.min(np.where(inz, dx, np.inf))))
            clearance = min(clearance, d)
            if d <= margin:
                touched.append(name)
        return touched, clearance

    def contact_wrench(self, eta, nu, dims, extra=None, skip="balloon", k=3000.0, c=60.0,
                       k_line=15000.0, c_line=300.0):
        """Body wrench the course exerts on a vehicle pressing into it.

        A penalty spring and damper along the outward normal, so a wall pushes
        the vehicle back rather than letting it through. Damping acts only
        while the vehicle moves into the surface. k and c are per hull point;
        posts and bars carry a stiffness per metre of length, so the force does
        not depend on how finely they are sampled.

        The normal for a wall is taken at the vehicle's centre rather than at
        the contact point: a wall is thin, and past half its thickness the
        nearest way out of it is the far face, which would push the vehicle
        through instead of back.
        """
        R, pb, p = self._probe(eta, dims, extra)
        vb, wb = np.asarray(nu[:3], float), np.asarray(nu[3:], float)
        F = np.zeros(3)
        M = np.zeros(3)
        rod_names = {n for n, _, _ in self._rods()}
        eps = 1e-4
        for name, sdf in self.elements:
            if (skip and name.startswith(skip)) or name in rod_names:
                continue
            d = sdf(p)
            ins = d < 0.0
            if not ins.any():
                continue
            q = p[ins]
            ctr = np.asarray(eta[:3], float)[None, :]
            base = ctr if sdf(ctr)[0] > 0 else q
            grad = np.stack([(sdf(base + e) - sdf(base - e)) / (2 * eps)
                             for e in np.eye(3) * eps], axis=1)
            n_w = grad / np.maximum(np.linalg.norm(grad, axis=1, keepdims=True), 1e-9)
            n_w = np.broadcast_to(n_w, q.shape)
            n_b = n_w @ R                              # into the body frame
            r_b = pb[ins]
            v_pt = vb + np.cross(wb, r_b)
            vn = np.einsum("ij,ij->i", v_pt, n_b)
            mag = k * (-d[ins]) - c * np.minimum(vn, 0.0)
            f_b = n_b * np.maximum(mag, 0.0)[:, None]
            F += f_b.sum(axis=0)
            M += np.cross(r_b, f_b).sum(axis=0)
        half = np.asarray(dims, float) / 2
        for name, pts, r in self._rods():
            pbr = (pts - np.asarray(eta[:3], float)) @ R
            d = self._box_sdf(pbr, half) - r
            ins = d < 0.0
            if not ins.any():
                continue
            q = pbr[ins]
            grad = np.stack([(self._box_sdf(q + e, half) - self._box_sdf(q - e, half)) / (2 * eps)
                             for e in np.eye(3) * eps], axis=1)
            # The gradient points out of the vehicle towards the rod; the rod pushes the other way.
            n_b = -grad / np.maximum(np.linalg.norm(grad, axis=1, keepdims=True), 1e-9)
            v_pt = vb + np.cross(wb, q)
            vn = np.einsum("ij,ij->i", v_pt, n_b)
            ds = 0.02                                  # rod sample spacing, see _rods
            mag = k_line * ds * (-d[ins]) - c_line * ds * np.minimum(vn, 0.0)
            f_b = n_b * np.maximum(mag, 0.0)[:, None]
            F += f_b.sum(axis=0)
            M += np.cross(q, f_b).sum(axis=0)
        return np.concatenate([F, M])
