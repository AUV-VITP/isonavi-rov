"""Six degree of freedom vehicle dynamics for a hovering underwater robot.

Standard Fossen formulation. In the body frame, with nu the body velocity
vector and eta the world pose,

    M nu_dot + C(nu_r) nu_r + D(nu_r) nu_r + g(eta) = tau + tau_dist
    eta_dot = J(eta) nu

M combines rigid-body and added mass, C the Coriolis and centripetal terms,
D linear plus quadratic drag, and g the restoring wrench from the offset
between centre of gravity and centre of buoyancy.

Hydrodynamic damping and Coriolis act on the velocity *relative to the water*,
nu_r = nu - nu_current. That distinction is the entire reason the vehicle gets
pushed downstream when the controller is open loop, and it is what the
station-keeping controller has to reject.

Numerical parameters are for a BlueROV2-class vectored hovering platform and
follow the identification widely used in the marine robotics literature. They
are stated as the design baseline; the flight-model vehicle is intended to be
indigenously manufactured to the same envelope.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

G = 9.81
RHO = 1000.0


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def rot_body_to_world(roll, pitch, yaw):
    """ZYX Euler rotation, ROS REP-103 body frame (x forward, y left, z up)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def euler_rate_matrix(roll, pitch):
    """Maps body angular rates to Euler angle rates."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cp = np.sign(cp) * max(abs(cp), 1e-6)
    return np.array([
        [1.0, sr * sp / cp, cr * sp / cp],
        [0.0, cr, -sr],
        [0.0, sr / cp, cr / cp],
    ])


@dataclass
class VehicleParams:
    """Mass, hydrodynamic and geometric properties."""

    name: str = "isonavi-1"
    mass: float = 13.5
    volume: float = 0.0136          # gives slight positive buoyancy
    r_g: tuple = (0.0, 0.0, 0.0)    # centre of gravity in body frame
    r_b: tuple = (0.0, 0.0, 0.02)   # centre of buoyancy, above CG for stability
    tow_point: tuple = (0.0, 0.0, 0.0)   # where a tether pulls, in body frame about the CG
    inertia: tuple = (0.26, 0.23, 0.37)

    # Added mass, diagonal approximation.
    added_mass: tuple = (6.36, 7.12, 18.68, 0.189, 0.135, 0.222)
    # Linear damping.
    lin_damp: tuple = (13.7, 0.0, 33.0, 0.0, 0.8, 0.0)
    # Quadratic damping.
    quad_damp: tuple = (141.0, 217.0, 190.0, 1.19, 0.47, 1.50)

    max_thrust_n: float = 51.0      # per thruster, T200 class at full throttle
    thruster_tau: float = 0.12      # first order response time constant

    # Fixed stabilising fins. A faired hull has a large added-mass asymmetry
    # between its axial and transverse directions, which produces a
    # destabilising Munk moment proportional to (Zw_dot - Xu_dot) u_r w_r
    # whenever it moves at an angle of attack. Buoyancy restoring alone is far
    # too weak to counter it, so a streamlined vehicle needs lifting surfaces
    # aft. fin_coeff is the resulting restoring moment per unit u_r w_r; it
    # must exceed the added-mass difference for the hull to be stable.
    # An open-frame vehicle is nearly isotropic and needs none.
    fin_coeff: float = 0.0

    # Thruster geometry, metres. Horizontal pair arms (lx, ly) and vertical
    # pair arms (vx, vy). Larger arms buy yaw and pitch authority for the same
    # thrust, which matters because torque and force share one thrust budget.
    arms: tuple = (0.156, 0.111, 0.120, 0.218)

    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def buoyancy(self) -> float:
        return RHO * self.volume * G


# Baseline commercial platform: open frame, T200-class thrusters. Included as
# the comparison point, because it is the vehicle most proposals in this space
# assume, and the analysis in docs/ shows it cannot hold the design current.
BLUEROV2_HEAVY = VehicleParams(
    name="BlueROV2 Heavy (COTS baseline)",
    mass=13.5, volume=0.0136,
    inertia=(0.26, 0.23, 0.37),
    added_mass=(6.36, 7.12, 18.68, 0.189, 0.135, 0.222),
    lin_damp=(13.7, 0.0, 33.0, 0.0, 0.8, 0.0),
    quad_damp=(141.0, 217.0, 190.0, 1.19, 0.47, 1.50),
    max_thrust_n=51.0, thruster_tau=0.12,
)

# Purpose-designed platform for high-current, zero-visibility flood work.
# Two changes drive the entire capability difference: a faired hull, which cuts
# axial quadratic drag by roughly a factor of four, and larger thrusters. The
# drag asymmetry is deliberate and characteristic of a streamlined body: low
# along the axis, high broadside, which also gives useful passive weathervaning
# into the flow.
isonavi_1 = VehicleParams(
    name="isonavi-1 (faired, high-thrust)",
    mass=28.0, volume=0.0282,
    r_b=(0.0, 0.0, 0.0273),   # from the CAD mass budget, see cad/isonavi_layout.py
    inertia=(0.55, 1.60, 1.70),
    added_mass=(12.0, 42.0, 45.0, 0.60, 3.20, 3.40),
    r_g=(0.0, 0.0, 0.0),
    lin_damp=(22.0, 48.0, 52.0, 6.0, 18.0, 16.0),
    quad_damp=(32.0, 210.0, 225.0, 8.0, 60.0, 52.0),
    max_thrust_n=120.0, thruster_tau=0.15,
    fin_coeff=110.0,
    arms=(0.42, 0.30, 0.38, 0.26),
)



def _rov_params():
    """isonavi-rov, read from the solved vehicle budget so the two cannot drift."""
    import json
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "cad", "rov_budget.json")
    d = json.load(open(path))
    k = d["dynamics"]
    return VehicleParams(
        name="isonavi-rov (open frame, pool)",
        mass=k["mass"], volume=k["volume"], r_b=tuple(k["r_b"]), tow_point=tuple(k["tow_point"]),
        inertia=tuple(k["inertia"]), added_mass=tuple(k["added_mass"]),
        lin_damp=tuple(k["lin_damp"]), quad_damp=tuple(k["quad_damp"]),
        max_thrust_n=d["thrust_max_n"], thruster_tau=0.10,
        arms=tuple(d["arms"]),
    )


isonavi_rov = _rov_params()

VEHICLES = {"bluerov2": BLUEROV2_HEAVY, "isonavi": isonavi_1, "rov": isonavi_rov}


def vehicle(name: str) -> VehicleParams:
    key = name.lower().split()[0]
    key = "rov" if key.endswith("-rov") else key.split("-")[0]
    if key not in VEHICLES:
        raise KeyError(f"unknown vehicle {name!r}, have {sorted(VEHICLES)}")
    return VEHICLES[key]


def max_holdable_current(p: VehicleParams) -> float:
    """Steady current at which surge drag equals the saturated surge thrust.

    Solves Xuu v^2 + Xu v = F_max for v, where F_max is the largest pure surge
    wrench the allocation can produce before any single thruster saturates.
    """
    B = vectored_allocation(p.arms)
    f_unit = np.linalg.pinv(B) @ np.array([1.0, 0, 0, 0, 0, 0])
    F_max = p.max_thrust_n / np.max(np.abs(f_unit))
    Xu, Xuu = p.lin_damp[0], p.quad_damp[0]
    return float((-Xu + np.sqrt(Xu ** 2 + 4 * Xuu * F_max)) / (2 * Xuu))


def vectored_allocation(arms=(0.156, 0.111, 0.120, 0.218)) -> np.ndarray:
    """Thrust allocation for an eight thruster vectored configuration.

    Four horizontal thrusters at 45 degrees give surge, sway and yaw; four
    vertical thrusters give heave, roll and pitch. Returns the 6x8 matrix B
    such that tau = B f.
    """
    a = np.radians(45.0)
    lx, ly, vx, vy = arms
    cols = []
    # Horizontal: (x, y, direction sign) laid out fore-port, fore-stbd, aft-port, aft-stbd.
    horiz = [(+lx, +ly, +1), (+lx, -ly, -1), (-lx, +ly, -1), (-lx, -ly, +1)]
    for (px, py, s) in horiz:
        fx, fy = np.cos(a), s * np.sin(a)
        cols.append([fx, fy, 0.0, 0.0, 0.0, px * fy - py * fx])
    # Vertical thrusters, all thrusting along +z.
    vert = [(+vx, +vy), (+vx, -vy), (-vx, +vy), (-vx, -vy)]
    for (px, py) in vert:
        cols.append([0.0, 0.0, 1.0, py * 1.0, -px * 1.0, 0.0])
    return np.array(cols).T


# Retained name for the layout, which both vehicles share.
bluerov_heavy_allocation = vectored_allocation


class VehicleDynamics:
    """Integrates the 6-DOF equations of motion."""

    def __init__(self, params: VehicleParams | None = None, current_fn=None):
        self.p = params or VehicleParams()
        self.current_fn = current_fn or (lambda pos: np.zeros(3))
        self.B = vectored_allocation(self.p.arms)
        self.B_pinv = np.linalg.pinv(self.B)
        self._build_matrices()
        self.reset()

    def _build_matrices(self):
        p = self.p
        Ix, Iy, Iz = p.inertia
        m = p.mass
        rg = np.asarray(p.r_g)
        M_rb = np.zeros((6, 6))
        M_rb[:3, :3] = m * np.eye(3)
        M_rb[:3, 3:] = -m * skew(rg)
        M_rb[3:, :3] = m * skew(rg)
        M_rb[3:, 3:] = np.diag([Ix, Iy, Iz])
        M_a = np.diag(p.added_mass)
        self.M = M_rb + M_a
        self.M_inv = np.linalg.inv(self.M)
        self.M_rb, self.M_a = M_rb, M_a
        self.D_lin = np.diag(p.lin_damp)
        self.D_quad = np.diag(p.quad_damp)

    def reset(self, eta=None, nu=None):
        self.eta = np.zeros(6) if eta is None else np.asarray(eta, float).copy()
        self.nu = np.zeros(6) if nu is None else np.asarray(nu, float).copy()
        self.thrust = np.zeros(self.B.shape[1])
        self.torque_scale = 1.0
        self.t = 0.0
        self.last_tau = np.zeros(6)

    # ------------------------------------------------------------------ terms
    def _coriolis(self, nu_r):
        """Coriolis and centripetal matrix for rigid body plus added mass."""
        m = self.p.mass
        v1, v2 = nu_r[:3], nu_r[3:]
        Ib = np.diag(self.p.inertia)
        C = np.zeros((6, 6))
        C[:3, 3:] = -m * skew(v1)
        C[3:, :3] = -m * skew(v1)
        C[3:, 3:] = -skew(Ib @ v2)
        A = self.p.added_mass
        a1 = np.array([A[0] * v1[0], A[1] * v1[1], A[2] * v1[2]])
        a2 = np.array([A[3] * v2[0], A[4] * v2[1], A[5] * v2[2]])
        Ca = np.zeros((6, 6))
        Ca[:3, 3:] = -skew(a1)
        Ca[3:, :3] = -skew(a1)
        Ca[3:, 3:] = -skew(a2)
        return C + Ca

    def _damping(self, nu_r):
        return self.D_lin + self.D_quad * np.abs(np.diag(nu_r))

    def _fin_moment(self, nu_r):
        """Restoring moment from fixed aft fins.

        A lifting surface at angle of attack alpha generates a moment that
        opposes it, growing with the square of the through-water speed. To
        first order in alpha this is -k u_r w_r in pitch and -k u_r v_r in yaw,
        the same functional form as the destabilising Munk moment, so the two
        can be compared coefficient against coefficient.
        """
        k = self.p.fin_coeff
        if k <= 0.0:
            return np.zeros(6)
        u, v, w = nu_r[0], nu_r[1], nu_r[2]
        tau = np.zeros(6)
        # Fins turn the hull until its axis lines up with the direction it is
        # travelling through the water. In this z-up frame a positive pitch is
        # nose-down, so aligning with a downward velocity (-w) needs a positive
        # moment, while aligning with a leftward velocity (+v) needs a positive
        # yaw. Hence the opposite signs.
        tau[4] = -k * u * w      # pitch
        tau[5] = +k * u * v      # yaw
        # Flying backwards puts the fins ahead of the centre of pressure, which
        # is destabilising. The sign of u_r already captures that: the moments
        # reverse and drive the vehicle to swap ends, which is why the mission
        # layer always orients the vehicle into the flow.
        return tau

    def _restoring(self, eta):
        """Gravity and buoyancy wrench expressed in the body frame."""
        p = self.p
        R = rot_body_to_world(eta[3], eta[4], eta[5])
        W = p.weight
        Bo = p.buoyancy
        # World-frame force, rotated into the body frame. z is up, so weight is
        # negative z and buoyancy positive z.
        f_world = np.array([0.0, 0.0, Bo - W])
        f_body = R.T @ f_world
        rg, rb = np.asarray(p.r_g), np.asarray(p.r_b)
        m_body = (np.cross(rb, R.T @ np.array([0.0, 0.0, Bo]))
                  + np.cross(rg, R.T @ np.array([0.0, 0.0, -W])))
        return -np.concatenate([f_body, m_body])

    # ------------------------------------------------------------------ step
    def allocate(self, tau_cmd):
        """Map a desired wrench to per-thruster forces, respecting saturation.

        Force and torque compete for one thrust budget. When the demand cannot
        be met, translational force is preserved and the attitude torque is
        scaled back, because losing station in a 2 m/s current is
        unrecoverable whereas a slow attitude response is not. Naive uniform
        scaling of the whole wrench does the opposite: a large yaw demand
        crushes the surge thrust holding the vehicle against the flow.
        """
        tau = np.asarray(tau_cmd, float)
        lim = self.p.max_thrust_n
        f_force = self.B_pinv @ np.concatenate([tau[:3], np.zeros(3)])
        f_torque = self.B_pinv @ np.concatenate([np.zeros(3), tau[3:]])

        # Force first. If force alone saturates, nothing else fits.
        peak_f = np.max(np.abs(f_force))
        if peak_f > lim:
            self.torque_scale = 0.0
            return f_force * (lim / peak_f)

        # Largest torque scale that still fits inside the per-thruster limit.
        lo, hi = 0.0, 1.0
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if np.max(np.abs(f_force + mid * f_torque)) <= lim:
                lo = mid
            else:
                hi = mid
        self.torque_scale = lo
        return f_force + lo * f_torque

    def step(self, tau_cmd, dt: float, tau_ext=None):
        """Advance one timestep under a commanded body wrench.

        ``tau_ext`` is an optional external body wrench that does not pass
        through the thrusters, such as the pull of a tether.
        """
        p = self.p
        f_cmd = self.allocate(np.asarray(tau_cmd, float))
        # First order thruster lag.
        alpha = dt / max(p.thruster_tau, 1e-6)
        alpha = min(alpha, 1.0)
        self.thrust += alpha * (f_cmd - self.thrust)
        tau = self.B @ self.thrust
        self.last_tau = tau

        v_c_world = np.asarray(self.current_fn(self.eta[:3]), float).reshape(3)
        R = rot_body_to_world(self.eta[3], self.eta[4], self.eta[5])
        v_c_body = R.T @ v_c_world
        nu_c = np.concatenate([v_c_body, np.zeros(3)])
        nu_r = self.nu - nu_c

        C = self._coriolis(nu_r)
        D = self._damping(nu_r)
        g = self._restoring(self.eta)
        tau_fin = self._fin_moment(nu_r)
        if tau_ext is not None:
            tau_fin = tau_fin + np.asarray(tau_ext, float)
        nu_dot = self.M_inv @ (tau + tau_fin - C @ nu_r - D @ nu_r - g)

        # Semi-implicit Euler keeps this stable at the timesteps used here.
        self.nu = self.nu + nu_dot * dt
        J = np.zeros((6, 6))
        J[:3, :3] = R
        J[3:, 3:] = euler_rate_matrix(self.eta[3], self.eta[4])
        self.eta = self.eta + (J @ self.nu) * dt
        self.eta[3:] = np.arctan2(np.sin(self.eta[3:]), np.cos(self.eta[3:]))
        self.t += dt
        return self.eta.copy(), self.nu.copy()

    @property
    def world_velocity(self):
        R = rot_body_to_world(self.eta[3], self.eta[4], self.eta[5])
        return R @ self.nu[:3]
