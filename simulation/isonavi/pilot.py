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
  auto    the assist loops plus vision: the course line is taken from the
          detected gate posts, corridor walls and target, which removes the
          operator delay from the lateral loop. This is the mode the ROS 2
          layer runs autonomously.

A run is scored the way it would be judged: contacts counted as events, not
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
    cruise_n: float = 26.0       # surge thrust the pilot holds in the open
    slow_n: float = 13.0         # surge thrust near elements


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
    """Course line as waypoints (x, y, z, speed class, heading) from launch to the maze exit.

    Heading is NaN where the pilot points along the leg. Inside the maze it is
    the corridor axis: the vectored layout lets the vehicle sway through the
    slalom without yawing, which keeps its swept width at its beam.
    """
    c = arena.cfg
    tgt = arena.targets[f"balloon_{target_colour or c.target_colour}"]
    tx, ty, tz = tgt["pos"]
    nan = np.nan
    wps = [
        (c.start[0], c.start[1], -0.40, 0, nan),
        (c.gate_x - 1.2, 0.0, -0.55, 1, 0.0),
        (c.gate_x + 0.8, 0.0, -0.55, 1, 0.0),
        (tx - 1.4, ty, tz, 1, 0.0),
        (tx - CourseRun.PROBE_X + 0.05, ty, tz, 2, 0.0),   # probe tip just through the centre
        (tx - 1.2, ty, tz, 1, 0.0),                          # back off
        (c.maze_x0 - 1.2, c.maze_y, -0.70, 1, nan),
        (c.maze_x0 - 0.2, c.maze_y, -0.70, 1, 0.0),
    ]
    x0, x1 = arena.sections["obstacles"]
    r, w = c.pipe_radius, c.maze_width / 2
    for k, off in enumerate(c.obstacle_offsets):
        ox = x0 + (k + 1) * (x1 - x0) / (len(c.obstacle_offsets) + 1)
        sg = np.sign(off)
        gap_mid = (off - sg * r - sg * w) / 2
        wps.append((ox, c.maze_y + gap_mid, -0.75, 1, 0.0))
    px0, px1 = arena.sections["passage"]
    wps += [(px0 + 0.2, c.maze_y, -0.85, 1, 0.0), (px1 + 0.6, c.maze_y, -0.85, 1, 0.0)]
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
        r = np.array([-0.29, 0.0, 0.05])        # termination point on the frame
        return np.concatenate([fb, np.cross(r, fb)])

    def _perceive(self, eta):
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
        # Lit by the vehicle's own lamps, contrast falls as exp(-2 c r) over the
        # two way path, so the read error grows by the inverse of it.
        c_att = WATER_ABSORPTION[1] + self.turb
        loss = np.exp(2.0 * c_att * rng_ref)
        sigma = self.pc.read_sigma0 + self.pc.read_sigma_per_m * rng_ref * loss
        if self.mode == "auto":
            sigma = 0.02 + 0.01 * rng_ref * loss
        self.read_bias = 0.97 * self.read_bias + 0.03 * self.rng.normal(0, sigma)
        lat_read = lat + self.read_bias
        heading = float(np.arctan2(t[1], t[0]))
        if np.isfinite(self.path[self.wi, 4]):
            heading = float(self.path[self.wi, 4])
        return lat_read, heading, along, L, t

    # ------------------------------------------------------------------- step
    def run(self):
        p, pc, dt = self.p, self.pc, self.dt
        ar = self.arena
        t = 0.0
        contacts, touching = [], set()
        min_clear = np.inf
        gate_clean, gate_passed = True, False
        balloon, wrong = False, False
        depth_err = []
        log = {k: [] for k in ("t", "eta", "wi", "contacts")}
        tgt_name = f"balloon_{ar.cfg.target_colour}"
        finished = False
        while t < self.max_time:
            eta, nu = self.dyn.eta, self.dyn.nu
            gyro, att = self.imu.measure(eta, nu, dt)
            z_meas = self.depth.measure(eta)

            lat, heading, along, L, tdir = self._perceive(eta)
            if along >= L - 0.15:
                self.wi += 1
                if self.wi >= len(self.path):
                    finished = True
                    break
                continue
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
                # Manual depth is read through the delay as well.
                tau[2] = float(np.clip(tau[2], -60, 60))
            else:
                tau[2:] = self.assist.wrench(z_ref, z_meas, att, gyro, hdg_err, dt)[2:]
            depth_err.append(eta[2] - z_ref)

            self.dyn.step(tau, dt, tau_ext=self._tether_wrench(eta, nu))
            eta = self.dyn.eta
            t += dt

            touched, clear = ar.contacts(eta, self.DIMS, skip="balloon")
            min_clear = min(min_clear, clear)
            now = set(n for n in touched if not n.startswith("balloon"))
            for n in now - touching:
                contacts.append((round(t, 2), n))
                if n.startswith("gate"):
                    gate_clean = False
            touching = now
            # Probe tip against the balloons.
            R = rot_body_to_world(*eta[3:6])
            tip = eta[:3] + R @ np.array([self.PROBE_X, 0.0, -0.06])
            for name, tg in ar.targets.items():
                if np.linalg.norm(tip - tg["pos"]) <= tg["radius"] + 0.01:
                    if name == tgt_name:
                        balloon = True
                    else:
                        wrong = True
            if eta[0] > ar.cfg.gate_x + 0.4:
                gate_passed = True
            if len(log["t"]) == 0 or t - log["t"][-1] >= 0.1:
                log["t"].append(t)
                log["eta"].append(eta.copy())
                log["wi"].append(self.wi)
                log["contacts"].append(len(contacts))

        return CourseResult(
            mode=self.mode, seed=self.seed, time_s=t, finished=finished,
            gate_clean=gate_clean and gate_passed, balloon_touched=balloon,
            wrong_balloon=wrong, contacts=len(contacts), contact_list=contacts,
            min_clearance=float(min_clear),
            depth_rms=float(np.sqrt(np.mean(np.square(depth_err)))) if depth_err else 0.0,
            log={k: np.array(v) for k, v in log.items()},
        )
