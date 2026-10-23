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
    gate_width: float = 1.0              # clear opening between post faces
    gate_top_depth: float = 0.25         # crossbar below the surface
    pipe_radius: float = 0.025           # 50 mm PVC

    balloon_x: float = 7.5
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
        self._add_post("gate_post_left", c.gate_x, yp, zf, zt)
        self._add_post("gate_post_right", c.gate_x, -yp, zf, zt)
        self._add_box("gate_crossbar", (c.gate_x, 0, zt + r), (r, yp + r, r), "pvc_white")
        self.sections["gate"] = (c.gate_x - 0.5, c.gate_x + 0.5)

        # Balloons on a line across the pool, each on a short tether from the floor.
        n = len(c.balloon_colours)
        for i, col in enumerate(c.balloon_colours):
            y = (i - (n - 1) / 2) * c.balloon_spacing
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
    @staticmethod
    def hull_points(eta, dims, n=3):
        """Sample points on the surface of the vehicle bounding box.

        eta is the pose (x, y, z, roll, pitch, yaw); dims are (L, B, H).
        """
        from .dynamics import rot_body_to_world
        L, B, H = dims
        g = np.linspace(-0.5, 0.5, n)
        pts = np.array([(a, b, c) for a in g for b in g for c in g
                        if max(abs(a), abs(b), abs(c)) == 0.5])
        pts = pts * np.array([L, B, H])
        R = rot_body_to_world(*eta[3:6])
        return pts @ R.T + np.asarray(eta[:3], float)

    def contacts(self, eta, dims, margin=0.0):
        """Names of course elements the hull is touching, and the minimum clearance."""
        p = self.hull_points(eta, dims)
        touched, clearance = [], np.inf
        for name, sdf in self.elements:
            d = float(np.min(sdf(p)))
            clearance = min(clearance, d)
            if d <= margin:
                touched.append(name)
        return touched, clearance
