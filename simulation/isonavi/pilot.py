"""Pilot in the loop: a human operator model, assist modes and a scored course run.

The vehicle is flown by a person looking at the camera feed, so the question
the simulation has to answer is not whether a controller can follow a path but
what a pilot with a reaction time, video latency and an imperfect read of the
picture achieves, and how much the onboard assist changes that.

Operator model. A crossover-model pilot: stick commands are a gain on the
perceived error plus a lead term, delayed by reaction time plus video
latency, with motor noise on the stick. The pilot reads lateral offset and
heading against the course from the camera, with an error that grows with
the range to the next reference and with the two way attenuation of the
vehicle's own light over that range.

Modes.
  manual  sticks map straight to thrust. The pilot also flies depth and
          attitude, which on a positively buoyant vehicle is constant work.
  assist  depth hold from the pressure sensor, heading and attitude hold from
          the IMU. Sticks command surge and sway thrust and a yaw rate; the
          pilot only steers the course in the horizontal plane.
  auto    the assist loops plus vision in the lateral loop. The camera is
          rendered at 5 Hz and the vehicle's position across the course is
          measured from the frame: triangulated from the gate posts, from
          the bearing and range of the target colour, from bearings to the
          obstacle posts, and from the wall brightness ratio elsewhere in
          the corridor (vision.CourseEstimator). Open
          water legs, where there is nothing to see or hit, are held on
          heading. Progress along the course is taken as known.

A run is scored on what an observer would record: contacts counted as events, not
time in contact, gate passed clean or not, balloon touched with the probe or
not, and the time to the end of the maze.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .control import wrap
from .dynamics import VehicleDynamics, rot_body_to_world, isonavi_rov
from .sensors import IMU, DepthCell
from .optics import WATER_ABSORPTION

RHO = 1000.0


@dataclass
class PilotConfig:
    reaction_s: float = 0.25
    video_latency_s: float = 0.15
    k_lat: float = 55.0          # N per metre of perceived lateral error
    k_lat_d: float = 1.1         # s, lead on lateral error
    k_yaw: float = 1.6           # rad/s per rad, as a rate demand
    k_depth: float = 140.0       # N per metre, manual depth
    k_depth_d: float = 1.0
    k_att: float = 6.0           # N m per rad, manual attitude, coarse
    stick_noise: float = 0.06    # fraction of full stick
    read_sigma0: float = 0.03    # m, lateral read error at close range
    read_sigma_per_m: float = 0.025
    read_filter: float = 0.03    # per tick; the read error wanders rather than jitters
    cruise_n: float = 26.0       # surge thrust the pilot holds in the open
    slow_n: float = 13.0         # surge thrust near elements


def read_sigma(pc, rng, turbidity):
    """Scale of the pilot's lateral read error at a range, lit by the vehicle's lamps.

    Contrast falls as exp(-2 c r) over the two way path, so the read error
    grows by the inverse of it.
    """
    loss = np.exp(2.0 * (WATER_ABSORPTION[1] + turbidity) * rng)
    return pc.read_sigma0 + pc.read_sigma_per_m * rng * loss


def read_error(pc, rng, turbidity):
    """Spread of the pilot's read: the low-passed error, sigma * sqrt(k / (2 - k))."""
    k = pc.read_filter
    return read_sigma(pc, rng, turbidity) * np.sqrt(k / (2.0 - k))


def blend_weight(err_pilot, err_camera):
    """Share of the lateral command given to the camera, by inverse error variance."""
    return err_pilot ** 2 / (err_pilot ** 2 + err_camera ** 2)


@dataclass
class TetherConfig:
    entry: tuple = (0.0, 0.0)    # where the tether leaves the deck, xy
    diameter: float = 0.016
    cd: float = 1.2
    slack_fraction: float = 0.35  # trailing length as a fraction of the range from the entry


@dataclass
class CourseResult:
    mode: str
    seed: int
    time_s: float
    finished: bool
    gate_clean: bool
    balloon_touched: bool
    wrong_balloon: bool
    contacts: int
    contact_list: list
    min_clearance: float
    depth_rms: float
    log: dict


def course_path(arena, target_colour=None):
    """Course line as waypoints (x, y, z, speed class, heading, reference) from launch to the maze exit.

    Heading is NaN where the pilot points along the leg. Inside the maze it is
    the corridor axis: the vectored layout lets the vehicle sway through the
    slalom without yawing, which keeps its swept width at its beam.

    Reference is what the camera can measure the leg against: 0 nothing,
    1 the gate posts, 2 the target, 3 the corridor walls.
    """
    c = arena.cfg
    tgt = arena.targets[f"balloon_{target_colour or c.target_colour}"]
    tx, ty, tz = tgt["pos"]
    nan = np.nan
    wps = [
        (c.start[0], c.start[1], -0.40, 0, nan, 0),
        (c.gate_x - 1.2, c.gate_y, -0.55, 1, 0.0, 1),
        (c.gate_x + 0.8, c.gate_y, -0.55, 1, 0.0, 1),
        (tx - 1.4, ty, tz, 1, 0.0, 2),
        (tx - CourseRun.PROBE_X + 0.05, ty, tz, 2, 0.0, 2),   # probe tip just through the centre
        (tx - 1.2, ty, tz, 1, 0.0, 2),                          # back off
        (c.maze_x0 - 1.2, c.maze_y, -0.70, 1, nan, 0),
        (c.maze_x0 - 0.2, c.maze_y, -0.70, 1, 0.0, 0),
    ]
    x0, x1 = arena.sections["obstacles"]
    r, w = c.pipe_radius, c.maze_width / 2
    for k, off in enumerate(c.obstacle_offsets):
        ox = x0 + (k + 1) * (x1 - x0) / (len(c.obstacle_offsets) + 1)
        sg = np.sign(off)
        gap_mid = (off - sg * r - sg * w) / 2
        wps.append((ox, c.maze_y + gap_mid, -0.75, 1, 0.0, 3))
    px0, px1 = arena.sections["passage"]
    wps += [(px0 + 0.2, c.maze_y, -0.85, 1, 0.0, 3), (px1 + 0.6, c.maze_y, -0.85, 1, 0.0, 3)]
    return np.array(wps, float)


class AssistController:
    """Depth, heading and attitude hold from the pressure sensor and the IMU only.

    Vertical speed is not measured, so it is taken from the depth signal
    through a first order filtered derivative. This is the same object the
    ROS 2 node runs on the vehicle.
    """

    def __init__(self, net_buoyancy_n, pc_yaw=1.6, kz=260.0, kvz=120.0,
                 kyaw=9.0, katt=40.0, katt_d=12.0, vz_tau=0.15):
        self.buoy = net_buoyancy_n
        self.k = (kz, kvz, kyaw, katt, katt_d, pc_yaw)
        self.vz_tau = vz_tau
        self.z_prev = None
        self.vz = 0.0

    def wrench(self, z_ref, z_meas, att, gyro, hdg_err, dt):
        kz, kvz, kyaw, katt, katt_d, kpy = self.k
        if self.z_prev is None:
            self.z_prev = z_meas
        a = dt / (self.vz_tau + dt)
        self.vz += a * ((z_meas - self.z_prev) / dt - self.vz)
        self.z_prev = z_meas
        tau = np.zeros(6)
        tau[2] = kz * (z_ref - z_meas) - kvz * self.vz - self.buoy
        yaw_rate_ref = float(np.clip(kpy * hdg_err, -0.8, 0.8))
        tau[5] = kyaw * (yaw_rate_ref - gyro[2])
        tau[3] = -katt * att[0] - katt_d * gyro[0]
        tau[4] = -katt * att[1] - katt_d * gyro[1]
        return tau


class CourseRun:
    DIMS = (0.600, 0.440, 0.330)
    PROBE_X = 0.36              # probe tip ahead of the frame centre
    PROBE_PTS = ((0.33, 0.0, -0.06), (0.36, 0.0, -0.06))   # the probe can strike a post too

    def __init__(self, arena, mode="assist", seed=0, params=isonavi_rov,
                 pilot=None, tether=None, turbidity=0.4, dt=0.02, max_time=420.0):
        self.arena, self.mode, self.seed = arena, mode, seed
        self.rng = np.random.default_rng(seed)
        self.p = params
        self.pc = pilot or PilotConfig()
        self.tc = tether or TetherConfig()
        self.turb = turbidity
        self.dt, self.max_time = dt, max_time
        self.dyn = VehicleDynamics(params)
        c = arena.cfg
        self.dyn.reset(eta=[c.start[0], c.start[1], -0.25, 0, 0, 0])
        self.imu = IMU(rng=np.random.default_rng(seed + 1))
        self.depth = DepthCell(rng=np.random.default_rng(seed + 2))
        self.path = course_path(arena)
        self.wi = 1
        delay = self.pc.reaction_s + self.pc.video_latency_s
        if mode == "auto":
            delay = 0.12                       # camera to command, measured on the board
        self.buf = deque(maxlen=max(1, int(round(delay / dt))))
        self.prev_lat = 0.0
        self.prev_dz = 0.0
        self.yaw_ref = 0.0
        self.read_bias = 0.0
        self.assist = AssistController(params.buoyancy - params.weight, pc_yaw=self.pc.k_yaw)
        self.vis = None                         # (time, vehicle y, error) from the last usable frame
        self.vis_log = []                       # (time, reference, estimate, truth, x, error)
        self.cam_lat = None                     # shared mode: camera lateral error, blend weight
        self.prev_cam_lat = None
        if mode in ("auto", "shared"):
            from .optics import Camera, WaterConfig
            from .vision import CourseEstimator, load_calibration, load_errors
            self.cam = Camera(water=WaterConfig(scattering=turbidity), seed=seed + 5)
            self.est = CourseEstimator(scattering=turbidity, calib=load_calibration(turbidity))
            self.err = load_errors(turbidity)
            self.next_frame = 0.0
            self.posts_xy = [(e.cx, e.cy) for e in self.arena.scene.primitives
                             if getattr(e, "name", "").startswith("obstacle")]

    # ----------------------------------------------------------------- helpers
    def _tether_wrench(self, eta, nu):
        R = rot_body_to_world(*eta[3:6])
        v = R @ nu[:3]
        speed = np.linalg.norm(v[:2])
        if speed < 1e-6:
            return np.zeros(6)
        trail = self.tc.slack_fraction * np.hypot(eta[0] - self.tc.entry[0], eta[1] - self.tc.entry[1])
        f = -0.5 * RHO * self.tc.cd * self.tc.diameter * trail * speed * v
        f[2] = 0.0
        fb = R.T @ f
        r = np.asarray(self.p.tow_point)        # the termination, from the solved budget
        return np.concatenate([fb, np.cross(r, fb)])

    def _measure(self, t, eta, yaw_meas):
        """One camera frame, reduced to the vehicle's y against the current reference."""
        c = self.arena.cfg
        ref = int(self.path[self.wi, 5])
        if ref == 0:
            return
        img, _, _ = self.cam.render(self.arena.scene, eta)
        y_cam = None
        tag = ref
        x_cam = eta[0] + self.cam.cfg.pos[0] * np.cos(yaw_meas)
        if ref == 1:
            # Anchored on the planned gate line, not the last estimate: there is
            # no wall here to break a loop in which one bad fit pulls the next.
            y_cam = self.est.gate(img, yaw_meas, x_cam, c.gate_x, c.gate_y,
                                  c.gate_width / 2 + c.pipe_radius, c.gate_y, span=0.5)
        elif ref == 2:
            tg = self.arena.targets[f"balloon_{c.target_colour}"]
            y_cam = self.est.target(img, yaw_meas, tg["pos"], c.target_colour)
        elif ref == 3 and eta[0] + self.cam.cfg.pos[0] > c.maze_x0 + 0.2:
            ox0, ox1 = self.arena.sections["obstacles"]
            # The wall estimate is bounded and comes from this frame alone, so it
            # anchors the post association; a post estimate that strays from it
            # is a wrong match, not a better measurement.
            y_wall = self.est.corridor(img, yaw_meas, c.maze_y, c.maze_width)
            y_cam, tag = y_wall, 3
            if ox0 - 1.0 < x_cam < ox1:
                y_post = self.est.posts(img, yaw_meas, x_cam, self.posts_xy, y_wall)
                if y_post is not None and abs(y_post - y_wall) < 0.15:
                    y_cam, tag = y_post, 4
        if y_cam is None:
            return
        y = y_cam - self.cam.cfg.pos[0] * np.sin(yaw_meas)
        self.vis = (t, y, self._vis_error(tag, x_cam))
        self.vis_log.append((t, tag, y, float(eta[1]), float(eta[0]), self.vis[2]))

    def _vis_error(self, tag, x_cam):
        """The estimator's held-out RMS error for this measurement, from the calibration."""
        e, c = self.err, self.arena.cfg
        if e is None:
            return 0.05
        if tag == 1:
            return e.at("gate", c.gate_x - x_cam)
        if tag == 2:
            return e.at("target", c.balloon_x - x_cam)
        if tag == 4:
            return e.posts
        for name in ("straight", "obstacles", "passage"):
            x0, x1 = self.arena.sections[name]
            if x0 <= x_cam < x1:
                return e.walls[name]
        return e.walls["passage"]

    def external_wrench(self, eta, nu):
        """Everything that is not the thrusters: the tether's drag and the course pushing back."""
        return self._tether_wrench(eta, nu) + self.arena.contact_wrench(
            eta, nu, self.DIMS, extra=self.PROBE_PTS)

    def _perceive(self, eta, now=0.0):
        """Lateral and heading error against the current course leg, as the pilot reads it."""
        a, b = self.path[self.wi - 1, :2], self.path[self.wi, :2]
        d = b - a
        L = np.linalg.norm(d)
        t = d / max(L, 1e-9)
        n = np.array([-t[1], t[0]])
        rel = eta[:2] - a
        lat = float(rel @ n)
        along = float(rel @ t)
        rng_ref = float(np.clip(L - along, 0.3, 3.0))
        sigma = read_sigma(self.pc, rng_ref, self.turb)
        if self.mode == "auto":
            loss = np.exp(2.0 * (WATER_ABSORPTION[1] + self.turb) * rng_ref)
            sigma = 0.02 + 0.01 * rng_ref * loss
        k = self.pc.read_filter
        self.read_bias = (1.0 - k) * self.read_bias + k * self.rng.normal(0, sigma)
        lat_read = lat + self.read_bias
        fresh = self.path[self.wi, 5] > 0 and self.vis is not None and now - self.vis[0] <= 0.45
        if self.mode == "auto" and fresh:
            lat_read = float((np.array([eta[0], self.vis[1]]) - a) @ n)
        if self.mode == "shared":
            self.cam_lat = None
            if fresh:
                # Weight the camera against the pilot by the error each has here.
                w = blend_weight(read_error(self.pc, rng_ref, self.turb), self.vis[2])
                self.cam_lat = (float((np.array([eta[0], self.vis[1]]) - a) @ n), float(w))
        heading = float(np.arctan2(t[1], t[0]))
        if np.isfinite(self.path[self.wi, 4]):
            heading = float(self.path[self.wi, 4])
        return lat_read, heading, along, L, t

    # ------------------------------------------------------------------- step
    # The loop is split the way the hardware is: pilot() is the surface side,
    # vehicle_wrench() is what the flight computer runs on the vehicle, and
    # score() is the observer. The bench in hil/ calls the same three.
    def pilot(self, t, eta, att, gyro, z_meas):
        """Pilot command for this tick; None when the course is done, "advance" on a waypoint change."""
        pc, dt = self.pc, self.dt
        if self.mode in ("auto", "shared") and t >= self.next_frame:
            self._measure(t, eta, att[2])
            self.next_frame = t + 0.2
        lat, heading, along, L, tdir = self._perceive(eta, t)
        if along >= L - 0.15:
            self.wi += 1
            return None if self.wi >= len(self.path) else "advance"
        slow = self.path[self.wi, 3]
        z_ref = self.path[self.wi, 2]

        dlat = (lat - self.prev_lat) / dt
        self.prev_lat = lat
        hdg_err = wrap(heading - att[2])
        surge = pc.cruise_n if slow == 0 else pc.slow_n
        if slow == 2:
            surge = 9.0
        sway = -pc.k_lat * (lat + pc.k_lat_d * dlat)
        # Turn before going on, the way a pilot would.
        surge *= float(np.clip(1.0 - abs(hdg_err) / 0.6, 0.0, 1.0))
        # Slow down when the line is off, rather than drive into a post.
        surge *= float(np.clip(1.0 - abs(lat) / 0.25, 0.2, 1.0)) if slow else 1.0
        cmd = np.array([surge, sway, hdg_err])
        if self.mode != "auto":
            cmd = cmd + self.rng.normal(0, pc.stick_noise, 3) * np.array([30.0, 30.0, 0.3])
        self.buf.append(cmd)
        surge, sway, hdg_err = self.buf[0]
        if self.mode == "shared":
            # The pilot's stick arrives through the reaction and video delay; the
            # camera's correction is formed aboard and does not wait for it.
            if self.cam_lat is None:
                self.prev_cam_lat = None
            else:
                y_c, w = self.cam_lat
                d_c = 0.0 if self.prev_cam_lat is None else (y_c - self.prev_cam_lat) / dt
                self.prev_cam_lat = y_c
                sway = (1.0 - w) * sway - w * pc.k_lat * (y_c + pc.k_lat_d * d_c)
        sway = float(np.clip(sway, -40, 40))

        # Sticks act in the body frame; the pilot resolves the leg into it.
        n = np.array([-tdir[1], tdir[0]])
        f_w = surge * tdir + sway * n
        psi = eta[5]
        tau = np.zeros(6)
        tau[0] = np.cos(psi) * f_w[0] + np.sin(psi) * f_w[1]
        tau[1] = -np.sin(psi) * f_w[0] + np.cos(psi) * f_w[1]
        if self.mode == "manual":
            dz = z_ref - z_meas
            ddz = (dz - self.prev_dz) / dt
            self.prev_dz = dz
            # The pilot reads depth off the overlay, through the same delay.
            tau[2] = pc.k_depth * dz + pc.k_depth * pc.k_depth_d * ddz * 0.3
            tau[5] = 2.2 * pc.k_yaw * hdg_err - 1.2 * gyro[2]
            tau[3] = -pc.k_att * att[0]
            tau[4] = -pc.k_att * att[1]
            tau[2] = float(np.clip(tau[2], -60, 60))
        self._z_ref = z_ref
        return {"tau": tau, "z_ref": float(z_ref), "hdg_err": float(hdg_err),
                "manual": self.mode == "manual"}

    def vehicle_wrench(self, cmd, z_meas, att, gyro):
        """What the flight computer does with a pilot command: hold depth, heading and attitude."""
        tau = np.array(cmd["tau"], float).copy()
        if not cmd["manual"]:
            tau[2:] = self.assist.wrench(cmd["z_ref"], z_meas, att, gyro, cmd["hdg_err"], self.dt)[2:]
        return tau

    def _reset_score(self):
        self._contacts, self._touching = [], set()
        self._min_clear = np.inf
        self._gate_clean, self._gate_passed = True, False
        self._balloon, self._wrong = False, False
        self._depth_err = []
        self._log = {k: [] for k in ("t", "eta", "wi", "contacts")}

    def score(self, t, eta):
        ar = self.arena
        self._depth_err.append(eta[2] - self._z_ref)
        touched, clear = ar.contacts(eta, self.DIMS, skip="balloon", extra=self.PROBE_PTS)
        self._min_clear = min(self._min_clear, clear)
        now = set(n for n in touched if not n.startswith("balloon"))
        for n in now - self._touching:
            self._contacts.append((round(t, 2), n))
            if n.startswith("gate"):
                self._gate_clean = False
        self._touching = now
        # Probe tip against the balloons.
        R = rot_body_to_world(*eta[3:6])
        tip = eta[:3] + R @ np.array([self.PROBE_X, 0.0, -0.06])
        tgt_name = f"balloon_{ar.cfg.target_colour}"
        for name, tg in ar.targets.items():
            if np.linalg.norm(tip - tg["pos"]) <= tg["radius"] + 0.01:
                if name == tgt_name:
                    self._balloon = True
                else:
                    self._wrong = True
        if eta[0] > ar.cfg.gate_x + 0.4:
            self._gate_passed = True
        log = self._log
        if len(log["t"]) == 0 or t - log["t"][-1] >= 0.1:
            log["t"].append(t)
            log["eta"].append(eta.copy())
            log["wi"].append(self.wi)
            log["contacts"].append(len(self._contacts))

    def run(self):
        dt = self.dt
        self._reset_score()
        t = 0.0
        finished = False
        while t < self.max_time:
            eta, nu = self.dyn.eta, self.dyn.nu
            gyro, att = self.imu.measure(eta, nu, dt)
            z_meas = self.depth.measure(eta)
            cmd = self.pilot(t, eta, att, gyro, z_meas)
            if cmd is None:
                finished = True
                break
            if isinstance(cmd, str):
                continue
            tau = self.vehicle_wrench(cmd, z_meas, att, gyro)
            self.dyn.step(tau, dt, tau_ext=self.external_wrench(eta, nu))
            t += dt
            self.score(t, self.dyn.eta)
        return self.result(t, finished)

    def result(self, t, finished):
        contacts, log, depth_err = self._contacts, self._log, self._depth_err
        min_clear, balloon, wrong = self._min_clear, self._balloon, self._wrong
        gate_clean, gate_passed = self._gate_clean, self._gate_passed
        return CourseResult(
            mode=self.mode, seed=self.seed, time_s=t, finished=finished,
            gate_clean=gate_clean and gate_passed, balloon_touched=balloon,
            wrong_balloon=wrong, contacts=len(contacts), contact_list=contacts,
            min_clearance=float(min_clear),
            depth_rms=float(np.sqrt(np.mean(np.square(depth_err)))) if depth_err else 0.0,
            log={**{k: np.array(v) for k, v in log.items()},
                 "vision": np.array(self.vis_log).reshape(-1, 6)},
        )
