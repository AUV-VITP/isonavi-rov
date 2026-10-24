"""Parametric CAD of isonavi-rov, built with CadQuery from the solved budget.

Every station and size comes from rov.py: the frame box, the thruster arms,
the hull, and the foam and ballast the solve returned. The model then checks
the budget rather than illustrating it:

  volume     the displaced volume of each group as built, against the budget;
  envelope   the bounding box of the assembly, against the frame box;
  clearance  every thruster duct against the side plates and the hull, and
             the outflow of every vertical thruster against the foam.

Outputs a STEP assembly and the views used in the documents.
"""

from __future__ import annotations

import json
import math
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rov as R                         # noqa: E402

MM = 1000.0
PLATE_T = 0.010
DUCT_OD, DUCT_ID, DUCT_L = 0.090, 0.080, 0.050
POD_D, POD_L = 0.040, 0.046
HULL_OD, HULL_L, HULL_X, HULL_Z = 0.114, 0.300, 0.030, 0.040
CAP_D, CAP_T = 0.124, 0.016
LAMP_D, LAMP_L, LAMP_Y = 0.036, 0.060, 0.18
LAMP_X, LAMP_Z = 0.230, 0.040            # beside the hull front, at lens height
FOAM_W, FOAM_H, FOAM_Z = 0.240, 0.032, 0.145
POCKET_FRACTION = 0.38                  # material removed from each side plate, as budgeted

C_FRAME = cq.Color(0.20, 0.22, 0.27, 1.0)
C_HULL = cq.Color(0.70, 0.82, 0.90, 1.0)
C_CAP = cq.Color(0.62, 0.64, 0.68, 1.0)
C_DUCT = cq.Color(0.86, 0.55, 0.16, 1.0)
C_POD = cq.Color(0.16, 0.17, 0.20, 1.0)
C_FOAM = cq.Color(0.93, 0.80, 0.25, 1.0)
C_LEAD = cq.Color(0.34, 0.35, 0.40, 1.0)
C_LAMP = cq.Color(0.95, 0.95, 0.85, 1.0)
C_RED = cq.Color(0.75, 0.12, 0.10, 1.0)


def mm(v):
    return v * MM


def box(lx, ly, lz, c):
    return cq.Workplane("XY").box(mm(lx), mm(ly), mm(lz)).translate(tuple(mm(x) for x in c))


def cyl_x(d, length, c):
    return (cq.Workplane("YZ").circle(mm(d) / 2).extrude(mm(length))
            .translate((mm(c[0] - length / 2), mm(c[1]), mm(c[2]))))


def side_plate(y):
    L, H = R.FRAME_L, R.FRAME_H
    p = cq.Workplane("XZ").rect(mm(L), mm(H)).extrude(mm(PLATE_T))
    # Three lightening pockets sized so the plate loses the budgeted fraction.
    area = L * H * POCKET_FRACTION
    pw = 0.150
    ph = area / (3 * pw)
    p = p.faces(">Y").workplane().pushPoints([(mm(x), 0) for x in (-0.19, 0.0, 0.19)]) \
        .rect(mm(pw), mm(ph)).cutThruAll()
    # An XZ workplane extrudes along -Y, so the plate spans [y, y + PLATE_T] after this.
    return p.translate((0, mm(y + PLATE_T), 0))


def thruster(c, axis):
    """Duct, motor pod and three struts, built along +Z then turned onto axis."""
    duct = cq.Workplane("XY").circle(mm(DUCT_OD) / 2).circle(mm(DUCT_ID) / 2).extrude(mm(DUCT_L))
    pod = cq.Workplane("XY").circle(mm(POD_D) / 2).extrude(mm(POD_L)).translate((0, 0, mm(0.002)))
    struts = None
    for k in range(3):
        a = 2 * math.pi * k / 3
        s = (cq.Workplane("XY").box(mm(DUCT_ID / 2 - POD_D / 2), mm(0.004), mm(0.012))
             .translate((mm((DUCT_ID + POD_D) / 4), 0, mm(DUCT_L / 2)))
             .rotate((0, 0, 0), (0, 0, 1), math.degrees(a)))
        struts = s if struts is None else struts.union(s)
    parts = []
    for solid, col in ((duct, C_DUCT), (pod.union(struts), C_POD)):
        s = solid.translate((0, 0, -mm(DUCT_L) / 2))
        ax = [float(v) for v in axis]
        n = math.sqrt(sum(v * v for v in ax))
        ax = [v / n for v in ax]
        rot_axis = (-ax[1], ax[0], 0.0)
        ang = math.degrees(math.acos(max(-1.0, min(1.0, ax[2]))))
        if math.hypot(rot_axis[0], rot_axis[1]) > 1e-9:
            s = s.rotate((0, 0, 0), rot_axis, ang)
        parts.append((s.translate(tuple(mm(v) for v in c)), col))
    return parts


def build():
    b = json.load(open(os.path.join(HERE, "rov_budget.json")))
    lx, ly, vx, vy = R.ARMS
    foam = next(p for p in b["parts"] if "foam" in p["name"])
    lead = next(p for p in b["parts"] if "lead" in p["name"])
    assy = cq.Assembly(name="isonavi-rov")
    named = {}

    def add(name, wp, col, group):
        assy.add(wp, name=name, color=col)
        named[name] = (wp, group)

    yi = R.FRAME_B / 2 - PLATE_T
    add("side plate port", side_plate(yi), C_FRAME, "frame")
    add("side plate stbd", side_plate(-yi - PLATE_T), C_FRAME, "frame")
    for i, (x, z) in enumerate(((0.22, 0.12), (-0.22, 0.12), (0.22, -0.13), (-0.22, -0.13))):
        add(f"cross member {i + 1}", box(0.040, 2 * yi, 0.010, (x, 0, z)), C_FRAME, "frame")
    for s in (1, -1):
        add(f"skid {'port' if s > 0 else 'stbd'}", box(R.FRAME_L * 0.95, 0.020, 0.020, (0, s * 0.15, -0.155)),
            C_FRAME, "frame")

    add("hull tube", cyl_x(HULL_OD, HULL_L, (HULL_X, 0, HULL_Z)), C_HULL, "hull")
    add("end cap rear", cyl_x(CAP_D, CAP_T, (HULL_X - HULL_L / 2 - CAP_T / 2, 0, HULL_Z)), C_CAP, "hull")
    add("end cap front", cyl_x(CAP_D, CAP_T, (HULL_X + HULL_L / 2 + CAP_T / 2, 0, HULL_Z)), C_CAP, "hull")

    k = 0
    for x in (+lx, -lx):
        for y in (+ly, -ly):
            # Horizontal units at 45 degrees, as in vectored_allocation.
            sgn = 1 if (x > 0) == (y > 0) else -1
            axis = (math.cos(math.radians(45)), sgn * math.sin(math.radians(45)), 0.0)
            k += 1
            for j, (wp, col) in enumerate(thruster((x, y, -0.06), axis)):
                add(f"thruster h{k} {'duct' if j == 0 else 'pod'}", wp, col, "propulsion")
    k = 0
    for x in (+vx, -vx):
        for y in (+vy, -vy):
            k += 1
            for j, (wp, col) in enumerate(thruster((x, y, 0.10), (0, 0, 1))):
                add(f"thruster v{k} {'duct' if j == 0 else 'pod'}", wp, col, "propulsion")

    v_foam = foam["vol"]
    lf = v_foam / (FOAM_W * FOAM_H)
    add("buoyancy foam", box(lf, FOAM_W, FOAM_H, (foam["x"], 0, FOAM_Z)), C_FOAM, "buoyancy")
    v_lead = lead["vol"]
    add("trim ballast", box(0.150, 0.060, v_lead / (0.150 * 0.060), (lead["x"], 0, -0.150)), C_LEAD, "buoyancy")

    for s in (1, -1):
        add(f"lamp {'port' if s > 0 else 'stbd'}", cyl_x(LAMP_D, LAMP_L, (LAMP_X, s * LAMP_Y, LAMP_Z)), C_LAMP, "payload")
        add(f"lamp bracket {'port' if s > 0 else 'stbd'}",
            box(0.030, R.FRAME_B / 2 - PLATE_T - LAMP_Y - LAMP_D / 2, 0.006,
                (LAMP_X, s * (LAMP_Y + LAMP_D / 2 + (R.FRAME_B / 2 - PLATE_T - LAMP_Y - LAMP_D / 2) / 2), LAMP_Z)),
            C_FRAME, "payload")
    probe_l = R.PROBE_TIP_X - 0.24
    add("touch probe", cyl_x(0.010, probe_l, (R.PROBE_TIP_X - probe_l / 2, 0, -0.06)), C_RED, "payload")
    add("probe bracket", box(0.020, 0.020, 0.059, (0.25, 0, -0.0945)), C_FRAME, "payload")
    add("kill switch", cq.Workplane("XY").circle(mm(0.015)).extrude(mm(0.020))
        .translate((mm(-0.22), mm(0.15), mm(0.125))), C_RED, "safety")   # rear cross member, outboard of the foam
    add("tether termination", cyl_x(0.030, 0.040, (R.TETHER_END_X + 0.020, 0, R.TETHER_END_Z)), C_POD, "tether")
    # Tow post between the rear cross members, and the bracket out to the termination.
    add("tow post", box(0.030, 0.010, 0.240, (-0.220, 0, -0.005)), C_FRAME, "frame")
    add("tow bracket", box(0.025, 0.020, 0.020, (-0.2475, 0, R.TETHER_END_Z)), C_FRAME, "frame")
    return assy, named, b


def solid_of(wp):
    return wp.val() if isinstance(wp, cq.Workplane) else wp


def verify(named, b):
    report = {}
    # Volume by group, as built.
    built = {}
    for name, (wp, g) in named.items():
        built[g] = built.get(g, 0.0) + solid_of(wp).Volume() / MM ** 3
    budget = {}
    for p in b["parts"]:
        budget[p["group"]] = budget.get(p["group"], 0.0) + p["vol"]
    report["volume"] = {g: {"built_l": built.get(g, 0) * 1000, "budget_l": budget.get(g, 0) * 1000}
                        for g in sorted(set(built) | set(budget))}
    # Envelope.
    bb = None
    for wp, _ in named.values():
        s = solid_of(wp).BoundingBox()
        bb = s if bb is None else bb.add(s)
    ext = (bb.xlen / MM, bb.ylen / MM, bb.zlen / MM)
    report["envelope_m"] = ext
    report["envelope_ok"] = all(e <= f + 1e-6 for e, f in zip(ext, (R.OVERALL_L, R.FRAME_B, R.FRAME_H)))
    # Clearances: every duct against plates and hull; vertical outflow against foam.
    plates = [solid_of(named[n][0]) for n in ("side plate port", "side plate stbd")]
    hull = [solid_of(named[n][0]) for n in ("hull tube", "end cap rear", "end cap front")]
    foam = solid_of(named["buoyancy foam"][0])
    clear = {}
    for name, (wp, g) in named.items():
        if not name.endswith("duct"):
            continue
        s = solid_of(wp)
        d_plate = min(s.distance(p) for p in plates) / MM
        d_hull = min(s.distance(h) for h in hull) / MM
        clear[name] = {"plate_mm": d_plate * 1000, "hull_mm": d_hull * 1000}
        if " v" in name:
            # The column the vertical unit pushes water through, up to the top of the box.
            c = s.BoundingBox().center
            col = (cq.Workplane("XY").circle(mm(DUCT_ID) / 2).extrude(mm(R.FRAME_H / 2) - c.z)
                   .translate((c.x, c.y, c.z)).val())
            clear[name]["outflow_blocked_l"] = col.intersect(foam).Volume() / MM ** 3 * 1000
    report["clearance"] = clear
    report["min_plate_mm"] = min(v["plate_mm"] for v in clear.values())
    report["min_hull_mm"] = min(v["hull_mm"] for v in clear.values())
    report["max_outflow_blocked_l"] = max(v.get("outflow_blocked_l", 0.0) for v in clear.values())
    # Every pair of parts, for overlap. Parts that are meant to bear on each
    # other (a cross member on a plate, a strut on its duct) touch without
    # overlapping, so any shared volume is a real clash.
    names = list(named)
    solids = [solid_of(named[n][0]) for n in names]
    boxes = [s.BoundingBox() for s in solids]
    clashes = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, c = boxes[i], boxes[j]
            if (a.xmax < c.xmin or c.xmax < a.xmin or a.ymax < c.ymin or c.ymax < a.ymin
                    or a.zmax < c.zmin or c.zmax < a.zmin):
                continue
            v = solids[i].intersect(solids[j]).Volume()
            if v > 50.0:                                   # mm3
                clashes.append({"a": names[i], "b": names[j], "overlap_cm3": v / 1000.0})
    report["interference"] = clashes
    report["n_parts"] = len(named)
    return report


def main():
    assy, named, b = build()
    rep = verify(named, b)
    assy.save(os.path.join(HERE, "isonavi_rov.step"))
    json.dump(rep, open(os.path.join(HERE, "rov_cad_check.json"), "w"), indent=1)
    print(f"isonavi-rov CAD, {rep['n_parts']} parts")
    print("  volume by group, built against budget, litres")
    for g, v in rep["volume"].items():
        print(f"    {g:12s}{v['built_l']:8.3f}{v['budget_l']:8.3f}")
    e = rep["envelope_m"]
    print(f"  envelope {e[0]*100:.1f} x {e[1]*100:.1f} x {e[2]*100:.1f} cm "
          f"({'inside' if rep['envelope_ok'] else 'OUTSIDE'} the budgeted box)")
    print(f"  closest duct to a side plate {rep['min_plate_mm']:.1f} mm, to the hull {rep['min_hull_mm']:.1f} mm")
    print(f"  vertical outflow blocked by foam, worst unit {rep['max_outflow_blocked_l']:.3f} L")
    print(f"  part pairs overlapping: {len(rep['interference'])}")
    for c in rep["interference"]:
        print(f"    CLASH {c['a']} with {c['b']}: {c['overlap_cm3']:.2f} cm3")
    for n, v in rep["clearance"].items():
        if v["plate_mm"] < 2.0 or v["hull_mm"] < 2.0:
            print(f"    CLASH {n}: plate {v['plate_mm']:.1f} mm, hull {v['hull_mm']:.1f} mm")


if __name__ == "__main__":
    main()
