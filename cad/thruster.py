"""Motor and propeller match for the in house thruster.

A hobby outrunner is sold by KV, and the obvious choice is the wrong one. A
76 mm ducted propeller makes 30 N at under 3000 rpm, far below where a
1000 KV motor on 16 V wants to run, so the motor works at low speed and high
torque: phase current is torque over the torque constant, and copper loss
goes as its square. This picks the KV from the values the 3536 frame is
actually sold in, by phase current and loss at the bench target.

  thrust   T = Ct rho n^2 D^4          (ducted, Ct from the duct and blade)
  shaft    P = T^(3/2) / sqrt(2 rho A) / eta_prop
  torque   Q = P / (2 pi n)
  current  I = Q / Kt,  Kt = 60 / (2 pi KV)   N m per A
  loss     I^2 Rm
"""

from __future__ import annotations

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RHO = 1000.0
D = 0.076
A = math.pi * (D / 2) ** 2
CT = 0.42                  # ducted three blade with stator flow straighteners, bollard
ETA_PROP = 0.74            # propeller, duct and anti-vortex stator recovery, bollard
V_RAIL = 16.0
ESC_A = 30.0
T_BENCH = 30.0
# 3536 class windings on sale, with typical phase resistance, ohm.
OPTIONS = {750: 0.085, 910: 0.062, 1000: 0.052, 1250: 0.036, 1450: 0.028}


def operating_point(thrust, kv, rm):
    n = math.sqrt(thrust / (CT * RHO * D ** 4))        # rev/s
    p_shaft = thrust ** 1.5 / math.sqrt(2 * RHO * A) / ETA_PROP
    q = p_shaft / (2 * math.pi * n)
    kt = 60.0 / (2 * math.pi * kv)
    i = q / kt
    loss = i * i * rm
    v_needed = (2 * math.pi * n) * kt + i * rm         # back EMF plus IR drop
    p_in = p_shaft + loss
    return {"kv": kv, "rpm": n * 60, "shaft_w": p_shaft, "torque_nm": q, "phase_a": i,
            "copper_w": loss, "input_w": p_in, "eta_motor": p_shaft / p_in,
            "duty": v_needed / V_RAIL, "esc_ok": i <= 0.8 * ESC_A}


def main(write=True):
    rows = [operating_point(T_BENCH, kv, rm) for kv, rm in OPTIONS.items()]
    ok = [r for r in rows if r["esc_ok"] and r["duty"] <= 0.9]
    best = min(ok, key=lambda r: r["input_w"]) if ok else None
    print(f"bench point {T_BENCH:.0f} N on a {D*1000:.0f} mm ducted propeller, {V_RAIL:.0f} V rail")
    print(f"  {'KV':>6}{'rpm':>7}{'phase A':>9}{'copper W':>10}{'input W':>9}{'motor eff':>11}{'duty':>6}  ESC")
    for r in rows:
        mark = "  <- chosen" if best and r["kv"] == best["kv"] else ""
        print(f"  {r['kv']:>6}{r['rpm']:>7.0f}{r['phase_a']:>9.1f}{r['copper_w']:>10.1f}"
              f"{r['input_w']:>9.1f}{r['eta_motor']:>11.2f}{r['duty']:>6.2f}  "
              f"{'ok' if r['esc_ok'] else 'over 80 %'}{mark}")
    if write:
        json.dump({"rows": rows, "chosen_kv": best["kv"] if best else None,
                   "bench_n": T_BENCH, "prop_d": D, "ct": CT},
                  open(os.path.join(HERE, "rov_thruster.json"), "w"), indent=1)
    return best


if __name__ == "__main__":
    main()
