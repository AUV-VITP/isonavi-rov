"""Vehicle budget for isonavi-rov: mass, buoyancy, trim, envelope, power and tether.

isonavi-rov keeps the isonavi-1 propulsion layout, eight thrusters in the
vectored arrangement, and the same discipline: every component is listed
with a mass, a displaced volume and a station, and the free variables are
solved rather than chosen.

What changes is the job. A pool vehicle has no current to fight and never
goes deeper than 2 m, so the faired hull, the fins and the 50 m pressure
rating all go, and what matters instead is a small box, low mass, full
holonomic control for threading a gate and a maze, and a camera with its own
light. Power comes down the tether from a mains supply, so there is no
battery aboard.

Solved:
  1. foam volume, so the vehicle floats slightly positive and surfaces on a
     power or tether failure;
  2. ballast mass, so the centre of gravity sits a set distance below the
     centre of buoyancy and the vehicle is self righting;
  3. foam station, so it floats level.

Then checked: the size box, the weight against the two thresholds, the
thrust envelope with any one thruster lost, the tether voltage drop and the
fuse rating.
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linprog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "simulation"))
from isonavi.dynamics import vectored_allocation, BLUEROV2_HEAVY   # noqa: E402

RHO, G = 1000.0, 9.81

# Envelope, metres. Frame plates set L and H, the thruster ducts set B.
FRAME_L, FRAME_B, FRAME_H = 0.600, 0.440, 0.330
SIZE_TIER = (0.75, 0.6501, 0.6001)          # bonus tier, L B H strictly below
MASS_LIMIT, MASS_BONUS = 35.0, 20.0

# Design targets for the solve.
NET_BUOYANCY_N = 2.0       # positive, so it surfaces when power is lost
BG_TARGET = 0.045          # m, centre of gravity below centre of buoyancy

# Densities, kg/m3.
HDPE, ACRYLIC, AL, LEAD, FOAM = 955.0, 1190.0, 2700.0, 11340.0, 110.0

# Thrusters: flooded outrunner in a printed duct, 76 mm propeller.
N_THR = 8
THRUST_MAX_N = 30.0        # forward, at 16 V, the bench test acceptance figure; reverse held to the same
PROP_D = 0.076
DRIVE_ETA = 0.45           # motor, controller and printed propeller together
ARMS = (0.215, 0.150, 0.130, 0.170)   # lx, ly, vx, vy as in vectored_allocation

# Power path: mains to a surface supply, down the tether, bucked aboard.
V_SUPPLY = 48.0
TETHER_LEN = 50.0
CORE_MM2 = 2.5             # flexible copper, two cores per leg in parallel
CORES_PER_LEG = 2
RHO_CU = 0.0172            # ohm mm2 / m at 20 C
BUCK_ETA = 0.94
BUCK_VIN_MIN = 36.0        # converter input floor
HOTEL_W = {
    "flight computer and camera": 9.0,
    "actuator interface and sensors": 2.0,
    "two lamps": 30.0,
    "ethernet and standby": 3.0,
}
P_CAP_W = 800.0            # input power governor, enforced in the thrust allocator
OPS_SPEED = 0.35           # m/s, the speed the course is flown at


@dataclass
class Part:
    name: str
    mass: float            # kg in air
    vol: float             # m3 displaced
    pos: tuple             # x forward, z up, m from the frame centre
    group: str


def tube(od, wall, length):
    return math.pi / 4 * (od ** 2 - (od - 2 * wall) ** 2) * length


def fixed_parts():
    P = []
    a = P.append
    # Frame: two side plates and four cross members, 10 mm HDPE, cut in house.
    side = FRAME_L * FRAME_H * 0.010 * 0.62       # lightened by pocketing
    for s in (+1, -1):
        a(Part(f"side plate {'port' if s > 0 else 'stbd'}", side * HDPE, side,
               (0.0, 0.0), "frame"))
    cross = 0.40 * 0.040 * 0.010
    for x, z in ((0.22, 0.12), (-0.22, 0.12), (0.22, -0.13), (-0.22, -0.13)):
        a(Part("cross member", cross * HDPE, cross, (x, z), "frame"))
    a(Part("fasteners, A4 stainless", 0.35, 0.35 / 8000, (0.0, 0.0), "frame"))

    # Main hull: cast acrylic tube, machined aluminium end caps, flat front port.
    od, wall, ln = 0.114, 0.006, 0.300
    hz, hx = 0.040, 0.030
    a(Part("hull tube, cast acrylic", tube(od, wall, ln) * ACRYLIC,
           math.pi / 4 * od ** 2 * ln, (hx, hz), "hull"))
    cap = math.pi / 4 * 0.124 ** 2 * 0.016 * 0.55   # pocketed
    a(Part("end cap, rear, 8 penetrators", cap * AL, cap, (hx - 0.158, hz), "hull"))
    a(Part("end cap, front, port ring", cap * AL * 0.7, cap * 0.7, (hx + 0.158, hz), "hull"))
    a(Part("front port, 10 mm acrylic", 0.10, 0.10 / ACRYLIC, (hx + 0.168, hz), "hull"))
    a(Part("o-rings, penetrators, vent", 0.18, 0.0, (hx - 0.16, hz), "hull"))

    # Inside the hull: the hull displacement already counts their volume.
    for name, m, dx in (("flight computer, Raspberry Pi 5", 0.07, 0.05),
                        ("actuator interface, ESP32", 0.02, -0.02),
                        ("IMU", 0.01, 0.0),
                        ("camera module and lens", 0.03, 0.13),
                        ("48 to 16 V converters, two", 0.52, -0.08),
                        ("ESCs, eight", 0.24, -0.04),
                        ("power distribution, fuses, relay", 0.20, -0.10),
                        ("ethernet interface and wiring", 0.25, 0.0),
                        ("tray, printed", 0.15, 0.0)):
        a(Part(name, m, 0.0, (hx + dx, hz - 0.015), "electronics"))

    # Thrusters: four horizontal at the corners, four vertical inboard.
    lx, ly, vx, vy = ARMS
    for x in (+lx, -lx):
        for _ in (0, 1):
            a(Part("thruster, horizontal", 0.17, 0.000085, (x, -0.06), "propulsion"))
    for x in (+vx, -vx):
        for _ in (0, 1):
            a(Part("thruster, vertical", 0.17, 0.000085, (x, 0.10), "propulsion"))

    a(Part("lamp, 15 W LED, two", 0.24, 0.00016, (0.27, -0.02), "payload"))
    a(Part("depth and temperature sensor", 0.02, 0.00001, (-0.10, 0.0), "payload"))
    a(Part("touch probe and bumper", 0.12, 0.00008, (0.29, -0.06), "payload"))
    a(Part("external kill switch", 0.05, 0.00002, (-0.20, 0.15), "safety"))
    a(Part("tether termination, strain relief", 0.15, 0.00005, (-0.29, 0.05), "tether"))
    a(Part("thruster cables, eight", 0.26, 0.00012, (0.0, 0.0), "propulsion"))
    a(Part("skids, HDPE", 0.18, 0.18 / HDPE, (0.0, -0.16), "frame"))
    return P


def budget(parts):
    M = sum(p.mass for p in parts)
    V = sum(p.vol for p in parts)
    cg = np.array([sum(p.mass * p.pos[i] for p in parts) / M for i in (0, 1)])
    cb = np.array([sum(p.vol * p.pos[i] for p in parts) / V for i in (0, 1)])
    return M, V, cg, cb


def solve():
    """Foam volume, ballast mass, foam station. Nested fixed point, converges in a few passes."""
    base = fixed_parts()
    z_foam, z_lead, x_lead = 0.145, -0.150, 0.0
    v_foam, m_lead, x_foam = 0.004, 0.5, 0.0
    for _ in range(200):
        foam = Part("buoyancy foam, closed cell", v_foam * FOAM, v_foam, (x_foam, z_foam), "buoyancy")
        lead = Part("trim ballast, lead", m_lead, m_lead / LEAD, (x_lead, z_lead), "buoyancy")
        parts = base + [foam, lead]
        M, V, cg, cb = budget(parts)
        net = RHO * V * G - M * G
        # Foam volume closes buoyancy, ballast closes BG, foam station closes trim.
        v_foam += (NET_BUOYANCY_N - net) / (G * (RHO - FOAM))
        bg = cb[1] - cg[1]
        m_lead = max(0.0, m_lead + (BG_TARGET - bg) * M / (0.30))
        x_foam = float(np.clip(x_foam + (cg[0] - cb[0]) * V / max(v_foam, 1e-6) * 0.8, -0.25, 0.25))
    M, V, cg, cb = budget(parts)
    return parts, M, V, cg, cb


# ------------------------------------------------------------------ hydrodynamics
def coefficients(M, V):
    """Damping and added mass scaled from the open frame baseline by projected area and volume.

    The baseline is the same open frame vehicle isonavi used as its reference,
    so the coefficients carry its tank-identified values, scaled to this box.
    """
    ref_box = (0.457, 0.575, 0.254)                  # BlueROV2 Heavy, L B H
    box = (FRAME_L, FRAME_B, FRAME_H)
    areas = lambda b: (b[1] * b[2], b[0] * b[2], b[0] * b[1])   # noqa: E731
    ra = np.array(areas(box)) / np.array(areas(ref_box))
    rv = V / BLUEROV2_HEAVY.volume
    p = BLUEROV2_HEAVY
    lin = np.array(p.lin_damp) * np.r_[ra, ra.mean() ** 1.5 * np.ones(3)]
    quad = np.array(p.quad_damp) * np.r_[ra, ra.mean() ** 1.5 * np.ones(3)]
    added = np.array(p.added_mass) * np.r_[rv * np.ones(3), rv ** (5 / 3) * np.ones(3)]
    L, B, H = box
    inertia = (M / 12 * (B ** 2 + H ** 2) * 0.7, M / 12 * (L ** 2 + H ** 2) * 0.7,
               M / 12 * (L ** 2 + B ** 2) * 0.7)
    return lin, quad, added, inertia


def max_along(B, axis, limit, alive=None):
    n = B.shape[1]
    bounds = [(-limit, limit) if (alive is None or alive[i]) else (0, 0) for i in range(n)]
    rows = [i for i in range(6) if i != axis]
    res = linprog(c=-B[axis], A_eq=B[rows], b_eq=np.zeros(5), bounds=bounds, method="highs")
    return float(B[axis] @ res.x) if res.success else 0.0


def speed_for(force, lin, quad):
    return float((-lin + math.sqrt(lin ** 2 + 4 * quad * force)) / (2 * quad))


def prop_power(t):
    a = math.pi * (PROP_D / 2) ** 2
    return abs(t) ** 1.5 / math.sqrt(2 * RHO * a) / DRIVE_ETA


def tether():
    r_leg = RHO_CU * TETHER_LEN / (CORE_MM2 * CORES_PER_LEG)
    r_loop = 2 * r_leg
    hotel = sum(HOTEL_W.values())
    peak_mech = N_THR * prop_power(THRUST_MAX_N)
    p_load_peak = (peak_mech + hotel) / BUCK_ETA
    p_load = min(p_load_peak, P_CAP_W)
    # Thrust every unit can still make when all eight run together at the cap.
    t_capped = THRUST_MAX_N * ((p_load * BUCK_ETA - hotel) / peak_mech) ** (2 / 3)

    def solve_i(p):
        # P = (Vs - I R) I  ->  I = (Vs - sqrt(Vs^2 - 4 R P)) / 2R
        disc = V_SUPPLY ** 2 - 4 * r_loop * p
        if disc < 0:
            return None
        return (V_SUPPLY - math.sqrt(disc)) / (2 * r_loop)
    # Find the largest load the tether can carry with the converters still above their floor.
    p_lim = (V_SUPPLY - BUCK_VIN_MIN) * BUCK_VIN_MIN / r_loop
    i_peak = solve_i(min(p_load, p_lim))
    return {
        "loop_resistance_ohm": r_loop,
        "hotel_w": hotel,
        "peak_thruster_w": peak_mech,
        "uncapped_input_w": p_load_peak,
        "peak_input_w": p_load,
        "thrust_all_eight_at_cap_n": t_capped,
        "deliverable_w_at_floor": p_lim,
        "peak_current_a": i_peak,
        "peak_vin_v": V_SUPPLY - i_peak * r_loop,
        "peak_tether_loss_w": i_peak ** 2 * r_loop,
        "fuse_a": int(math.ceil(i_peak * 1.25 / 5.0) * 5),
        "supply_a": V_SUPPLY and int(math.ceil(i_peak * 1.15)),
        "supply_w": int(math.ceil(V_SUPPLY * i_peak * 1.2 / 100) * 100),
        "ethernet_ok": bool(TETHER_LEN + 5 <= 100),
    }


def tether_drag(trail_m, v, d=0.016, cd=1.2):
    """Drag of tether trailed broadside behind the vehicle."""
    return 0.5 * RHO * cd * d * trail_m * v * v


def main(write=True):
    parts, M, V, cg, cb = solve()
    lin, quad, added, inertia = coefficients(M, V)
    B = vectored_allocation(ARMS)

    env = {}
    for ax, name in ((0, "surge"), (1, "sway"), (2, "heave"), (5, "yaw")):
        full = max_along(B, ax, THRUST_MAX_N)
        worst = min(max_along(B, ax, THRUST_MAX_N, alive=[j != i for j in range(N_THR)])
                    for i in range(N_THR))
        env[name] = {"full": full, "one_lost": worst}

    trail = 8.0
    need = lin[0] * OPS_SPEED + quad[0] * OPS_SPEED ** 2 + tether_drag(trail, OPS_SPEED)
    v_full = speed_for(env["surge"]["full"], lin[0], quad[0])
    v_lost = speed_for(env["surge"]["one_lost"], lin[0], quad[0])
    sway_v = speed_for(env["sway"]["full"], lin[1], quad[1])
    heave_v = speed_for(env["heave"]["full"], lin[2], quad[2])

    tt = tether()
    size_ok = FRAME_L < SIZE_TIER[0] and FRAME_B < SIZE_TIER[1] and FRAME_H < SIZE_TIER[2]

    out = {
        "mass_kg": M, "displaced_m3": V,
        "net_buoyancy_n": RHO * V * G - M * G,
        "cg_m": cg.tolist(), "cb_m": cb.tolist(), "bg_m": float(cb[1] - cg[1]),
        "trim_offset_m": float(cg[0] - cb[0]),
        "foam_l": next(p.vol for p in parts if p.group == "buoyancy" and "foam" in p.name) * 1000,
        "ballast_kg": next(p.mass for p in parts if "lead" in p.name),
        "size_m": [FRAME_L, FRAME_B, FRAME_H], "size_bonus": bool(size_ok),
        "mass_limit_ok": bool(M <= MASS_LIMIT), "mass_bonus": bool(M < MASS_BONUS),
        "thrust_max_n": THRUST_MAX_N, "arms": list(ARMS),
        "envelope_n": env,
        "surge_speed_full": v_full, "surge_speed_one_lost": v_lost,
        "sway_speed_full": sway_v, "heave_speed_full": heave_v,
        "ops_speed": OPS_SPEED, "ops_force_n": need, "tether_trail_m": trail,
        "ops_margin_one_lost": env["surge"]["one_lost"] / need,
        "tether": tt,
        "dynamics": {"mass": M, "volume": V, "r_b": [0.0, 0.0, float(cb[1] - cg[1])],
                     "inertia": list(inertia), "added_mass": added.tolist(),
                     "lin_damp": lin.tolist(), "quad_damp": quad.tolist()},
        "groups": {},
        "parts": [{"name": p.name, "mass": p.mass, "vol": p.vol, "x": p.pos[0],
                   "z": p.pos[1], "group": p.group} for p in parts],
    }
    for p in parts:
        out["groups"][p.group] = out["groups"].get(p.group, 0.0) + p.mass

    print("isonavi-rov vehicle budget")
    print(f"  mass {M:6.2f} kg   limit {MASS_LIMIT:.0f} ({'ok' if M <= MASS_LIMIT else 'OVER'}), "
          f"bonus below {MASS_BONUS:.0f} ({'yes' if M < MASS_BONUS else 'no'})")
    print(f"  size {FRAME_L*100:.0f} x {FRAME_B*100:.0f} x {FRAME_H*100:.0f} cm, bonus tier "
          f"{'met' if size_ok else 'missed'}")
    print(f"  displaced {V*1000:.2f} L, net buoyancy {out['net_buoyancy_n']:+.2f} N, "
          f"BG {out['bg_m']*1000:.1f} mm, trim offset {out['trim_offset_m']*1000:.2f} mm")
    print(f"  foam {out['foam_l']:.2f} L, ballast {out['ballast_kg']:.2f} kg")
    for g_, m in sorted(out["groups"].items(), key=lambda kv: -kv[1]):
        print(f"    {g_:12s}{m:7.2f} kg")
    print("  envelope, newtons (full / one thruster lost):")
    for k, v in env.items():
        print(f"    {k:6s}{v['full']:8.1f}{v['one_lost']:8.1f}")
    print(f"  speeds: surge {v_full:.2f} m/s ({v_lost:.2f} one lost), sway {sway_v:.2f}, heave {heave_v:.2f}")
    print(f"  at {OPS_SPEED} m/s with {trail:.0f} m of tether trailing: {need:.1f} N needed, "
          f"margin {out['ops_margin_one_lost']:.1f}x with a thruster lost")
    print(f"  tether loop {tt['loop_resistance_ohm']:.3f} ohm, peak {tt['peak_input_w']:.0f} W in, "
          f"{tt['peak_current_a']:.1f} A, {tt['peak_vin_v']:.1f} V aboard, loss {tt['peak_tether_loss_w']:.0f} W")
    print(f"  governor {P_CAP_W:.0f} W against {tt['uncapped_input_w']:.0f} W uncapped: all eight together "
          f"still make {tt['thrust_all_eight_at_cap_n']:.1f} N each")
    print(f"  surface supply {tt['supply_w']} W at {V_SUPPLY:.0f} V, fuse {tt['fuse_a']} A, "
          f"ethernet over the tether {'ok' if tt['ethernet_ok'] else 'NO'}")
    if write:
        json.dump(out, open(os.path.join(HERE, "rov_budget.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
