"""Every number in the isonavi-rov report, generated from the result files.

Writes docs/rov_metrics.tex as LaTeX macros, and returns the same values so
docs/check_rov.py can confirm that the README quotes them unchanged. Nothing
in the report is typed by hand, so it cannot drift from the experiments.
"""

from __future__ import annotations

import json
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
LOGS = os.path.join(ROOT, "simulation", "results", "logs")
CAD = os.path.join(ROOT, "cad")
sys.path.insert(0, os.path.join(ROOT, "simulation"))
from isonavi.pool import ArenaConfig   # noqa: E402


def load(*p):
    return json.load(open(os.path.join(*p)))


def inr(v):
    """Indian digit grouping: 137092 -> 1,37,092."""
    s = str(int(round(v)))
    if len(s) <= 3:
        return s
    head, tail = s[:-3], s[-3:]
    parts = []
    while len(head) > 2:
        parts.insert(0, head[-2:])
        head = head[:-2]
    if head:
        parts.insert(0, head)
    return ",".join(parts) + "," + tail


def collect():
    b = load(CAD, "rov_budget.json")
    bom = load(CAD, "rov_bom.json")
    th = load(CAD, "rov_thruster.json")
    cc = load(CAD, "rov_cad_check.json")
    tr = load(LOGS, "course_trials.json")["summary"]
    vm = load(LOGS, "vision_metrics.json")["summary"]
    vc = load(LOGS, "vision_calibration.json")
    hl = load(ROOT, "hil", "results", "rov_loopback.json")
    m = {}

    # Vehicle.
    t = b["tether"]
    env = b["envelope_n"]
    m.update({
        "mass": f"{b['mass_kg']:.2f}", "displaced": f"{b['displaced_m3'] * 1000:.2f}",
        "netbuoy": f"{b['net_buoyancy_n']:+.2f}", "bg": f"{b['bg_m'] * 1000:.0f}",
        "trim": f"{abs(b['trim_offset_m']) * 1000:.2f}", "foam": f"{b['foam_l']:.2f}",
        "ballast": f"{b['ballast_kg']:.2f}",
        "lenoverall": f"{b['overall_l_m'] * 100:.0f}", "lenframe": f"{b['size_m'][0] * 100:.0f}",
        "beam": f"{b['size_m'][1] * 100:.0f}", "height": f"{b['size_m'][2] * 100:.0f}",
        "thrustmax": f"{b['thrust_max_n']:.0f}",
        "surgefull": f"{env['surge']['full']:.1f}", "surgelost": f"{env['surge']['one_lost']:.1f}",
        "swayfull": f"{env['sway']['full']:.1f}", "heavefull": f"{env['heave']['full']:.1f}",
        "heavelost": f"{env['heave']['one_lost']:.1f}", "yawfull": f"{env['yaw']['full']:.1f}",
        "speedfull": f"{b['surge_speed_full']:.2f}", "speedlost": f"{b['surge_speed_one_lost']:.2f}",
        "swayspeed": f"{b['sway_speed_full']:.2f}", "heavespeed": f"{b['heave_speed_full']:.2f}",
        "opsspeed": f"{b['ops_speed']:.2f}", "opsforce": f"{b['ops_force_n']:.1f}",
        "trail": f"{b['tether_trail_m']:.0f}", "opsmargin": f"{b['ops_margin_one_lost']:.1f}",
        "loopohm": f"{t['loop_resistance_ohm']:.3f}", "hotel": f"{t['hotel_w']:.0f}",
        "uncapped": f"{t['uncapped_input_w']:.0f}", "capw": f"{t['peak_input_w']:.0f}",
        "capthrust": f"{t['thrust_all_eight_at_cap_n']:.1f}", "peakamps": f"{t['peak_current_a']:.1f}",
        "vinpeak": f"{t['peak_vin_v']:.1f}", "tetherloss": f"{t['peak_tether_loss_w']:.0f}",
        "fuse": f"{t['fuse_a']}", "supplyw": f"{t['supply_w']}",
    })
    # Drag split at operating speed: frame against tether.
    lin, quad = b["dynamics"]["lin_damp"][0], b["dynamics"]["quad_damp"][0]
    v = b["ops_speed"]
    frame = lin * v + quad * v * v
    m["framedrag"] = f"{frame:.1f}"
    m["tetherdrag"] = f"{b['ops_force_n'] - frame:.1f}"

    # Slalom: the gap beside each post, and what the beam leaves of it a side.
    ac = ArenaConfig()
    gap = ac.maze_width / 2 + min(abs(o) for o in ac.obstacle_offsets) - ac.pipe_radius
    m["slalomgap"] = f"{gap * 100:.1f}"
    m["slalomside"] = f"{(gap - b['size_m'][1]) / 2 * 1000:.0f}"

    # Thruster match.
    rows = {r["kv"]: r for r in th["rows"]}
    ch = rows[th["chosen_kv"]]
    m.update({"kvchosen": f"{th['chosen_kv']}", "kvphase": f"{ch['phase_a']:.1f}",
              "kvinput": f"{ch['input_w']:.0f}", "kvrpm": f"{ch['rpm']:.0f}",
              "kvmotoreff": f"{ch['eta_motor']:.2f}",
              "kvthousandphase": f"{rows[1000]['phase_a']:.1f}"})

    # CAD closure.
    vol = cc["volume"]
    m.update({"cadparts": f"{cc['n_parts']}", "cadclash": f"{len(cc['interference'])}",
              "cadplate": f"{cc['min_plate_mm']:.1f}", "cadhull": f"{cc['min_hull_mm']:.1f}",
              "cadoutflow": f"{cc['max_outflow_blocked_l'] * 1000:.0f}",
              "cadbuoybuilt": f"{vol['buoyancy']['built_l']:.3f}",
              "cadhullbuilt": f"{vol['hull']['built_l']:.3f}",
              "cadenvl": f"{cc['envelope_m'][0] * 100:.1f}", "cadenvb": f"{cc['envelope_m'][1] * 100:.1f}",
              "cadenvh": f"{cc['envelope_m'][2] * 100:.1f}"})

    # Cost.
    g = bom["groups"]
    m.update({"costvehicle": inr(bom["vehicle_inr"]), "costbuild": inr(bom["build_inr"]),
              "costtotal": inr(bom["total_inr"]), "costcont": inr(bom["contingency_inr"]),
              "costinhouse": inr(bom["in_house_inr"]), "inhouselines": f"{bom['in_house_items']}",
              "bomlines": f"{bom['n_lines']}"})
    for k, val in g.items():
        m[f"cost{k}"] = inr(val)

    # Course trials.
    words = {"0.05": "clear", "0.4": "mid", "1.0": "murky"}
    for exp, block in tr.items():
        for key, c in block.items():
            mode, bb = key.split("_")
            p = f"{'ref' if exp == 'reference' else 'lay'}{mode}{words[bb]}"
            m[p + "contacts"] = f"{c['contacts_mean']:.2f}"
            m[p + "contactsd"] = f"{c['contacts_std']:.2f}"
            m[p + "clean"] = f"{c['clean_runs']}"
            m[p + "n"] = f"{c['n']}"
            m[p + "done"] = f"{c['finished']}"
            m[p + "gate"] = f"{c['gate_clean']}"
            m[p + "hit"] = f"{c['balloon']}"
            m[p + "wrong"] = f"{c['wrong_balloon']}"
            m[p + "time"] = "-" if c["time_mean"] is None else f"{c['time_mean']:.1f}"
            m[p + "penalty"] = f"{c['penalty_mean']:.1f}"
            for ref in ("gate", "target", "corridor"):
                r = c.get(f"vision_rms_{ref}")
                if r is not None:
                    m[p + "vis" + ref] = f"{r * 1000:.0f}"

    # Target identification, lamps as built.
    sep = "0.18"
    for bb, w in words.items():
        for r in ("0.75", "1.0", "1.5", "2.0", "2.5"):
            k = f"{bb}_{sep}_{r}"
            if k in vm:
                rw = {"0.75": "near", "1.0": "one", "1.5": "onehalf", "2.0": "two", "2.5": "twohalf"}[r]
                m[f"id{w}{rw}"] = f"{vm[k]['id_corrected'] * 100:.0f}"
                m[f"idraw{w}{rw}"] = f"{vm[k]['id_uncorrected'] * 100:.0f}"
    for bb, w in words.items():
        k10, k18 = f"{bb}_0.1_1.5", f"{bb}_0.18_1.5"
        if k10 in vm and k18 in vm:
            m[f"lampnarrow{w}"] = f"{vm[k10]['id_corrected'] * 100:.0f}"
            m[f"lampbuilt{w}"] = f"{vm[k18]['id_corrected'] * 100:.0f}"

    # Estimator calibration.
    cw = {"0.05": "clear", "0.4": "mid", "1.0": "murky"}
    for bb, w in cw.items():
        c = vc["corridor"][bb]
        m[f"wall{w}straight"] = f"{c['rms_straight'] * 1000:.0f}"
        m[f"wall{w}obstacles"] = f"{c['rms_obstacles'] * 1000:.0f}"
        m[f"wall{w}passage"] = f"{c['rms_passage'] * 1000:.0f}"
        m[f"posts{w}"] = "-" if c["posts_rms"] is None else f"{c['posts_rms'] * 1000:.0f}"
        m[f"posts{w}avail"] = f"{c['posts_available'] * 100:.0f}"
        f = vc["fits"][bb]
        m[f"wallgain{w}"] = f"{f['g']:.2f}"
        m[f"wallyaw{w}"] = f"{f['k_yaw']:.2f}"
        for r, rw in (("1.0", "one"), ("1.5", "onehalf"), ("2.0", "two")):
            gk, tk = vc["gate"][f"{bb}_{r}"], vc["target"][f"{bb}_{r}"]
            m[f"gate{w}{rw}"] = "-" if gk["rms"] is None else f"{gk['rms'] * 1000:.0f}"
            m[f"gate{w}{rw}avail"] = f"{gk['available'] * 100:.0f}"
            m[f"tgt{w}{rw}"] = "-" if tk["rms"] is None else f"{tk['rms'] * 1000:.0f}"
            m[f"tgt{w}{rw}avail"] = f"{tk['available'] * 100:.0f}"

    # Bench.
    e, ll = hl["equivalence"], hl["link_loss"]
    m.update({"hiltime": f"{e['time_hil']:.2f}", "simtime": f"{e['time_sim']:.2f}",
              "hilcontacts": f"{e['contacts_hil']}", "hildev": f"{e['max_dev_m']:.1e}",
              "hilticks": f"{e['ticks']}", "hilcompute": f"{e['compute_ms_mean']:.2f}",
              "holdtime": f"{ll['t_to_hold_s']:.2f}", "holdtimeout": f"{ll['timeout_s']:.1f}",
              "holdrise": f"{ll['rise_m']:.2f}", "holdspan": f"{ll['hold_s']:.1f}",
              "killticks": f"{hl['kill']['ticks_after']}"})
    return m


def write(m):
    out = os.path.join(ROOT, "docs", "rov_metrics.tex")
    with open(out, "w") as f:
        f.write("% Generated by simulation/eval/make_rov_metrics.py. Do not edit.\n")
        for k, v in sorted(m.items()):
            assert k.isalpha(), k
            f.write(f"\\newcommand{{\\m{k}}}{{{v}}}\n")
    return out


if __name__ == "__main__":
    m = collect()
    print("wrote", write(m), f"({len(m)} values)")
