"""Bill of materials for one isonavi-rov, in rupees including GST.

Every line carries a quantity, a unit price, the basis that price rests on,
and whether the part is made in house. Basis is one of:

  market     an Indian retail listing for the item or its direct equivalent
  converted  a distributor price in USD at the rate below, before import
  estimate   a material or service price from local suppliers, to be quoted

The in house share is reported against the build cost, because making the
thrusters, the hull and the frame is where the design work sits and where
most of the cost saving comes from.

Lines that depend on the solved vehicle, the foam, the ballast, the supply
voltage and the fuse, are written from rov_budget.json, so run rov.py first.
"""

from __future__ import annotations

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
USD_INR = 95.64            # mid-market, 24 August 2026, as in the isonavi budget
BUDGET = json.load(open(os.path.join(HERE, "rov_budget.json")))
TT = BUDGET["tether"]

# group, item, qty, unit INR, basis, made in house, reason
LINES = [
    # Propulsion: eight thrusters built in house around a hobby outrunner.
    ("propulsion", "BLDC outrunner, 3536 class, 750 KV", 8, 1690, "market", False,
     "KV matched to the propeller in thruster.py; runs flooded, potted stator"),
    ("propulsion", "bidirectional ESC, 30 A, 2 to 4S", 8, 844, "market", False,
     "reversible thrust for the vectored layout"),
    ("propulsion", "duct, propeller and mount, printed PETG", 8, 300, "estimate", True,
     "Kort nozzle duct and 76 mm three blade propeller, 2 kg of filament across eight"),
    ("propulsion", "shaft, bearings, potting epoxy", 8, 350, "estimate", True,
     "corrosion proofing for open water running"),
    # Hull and frame.
    ("hull", "cast acrylic tube, 114 mm OD, 6 mm wall, 300 mm", 1, 3500, "estimate", False,
     "the one pressure housing; clear so a leak is visible on deck"),
    ("hull", "end caps, machined 6061 aluminium", 2, 1100, "estimate", True,
     "bar stock plus in house turning; double radial o-ring seal"),
    ("hull", "front port, 10 mm cast acrylic", 1, 400, "estimate", True, "flat camera window"),
    ("hull", "o-rings, cable penetrators, vent plug", 1, 2500, "market", False,
     "ten IP68 penetrators, nitrile o-ring kit, vent for the vacuum test"),
    ("frame", "HDPE sheet, 10 mm, 1220 x 610 mm", 1, 4500, "estimate", True,
     "side plates, cross members, skids and the tow post, cut in house"),
    ("frame", "A4 stainless fasteners", 1, 1200, "market", False, "no galvanic pairs with the aluminium"),
    ("frame", "closed cell foam, 3 L, epoxy coated", 1, 1500, "estimate", True,
     f"{BUDGET['foam_l']:.2f} L needed for {BUDGET['net_buoyancy_n']:+.0f} N net buoyancy, see rov.py"),
    ("frame", f"lead trim ballast, {BUDGET['ballast_kg']:.2f} kg", 1, 450, "estimate", True,
     f"sets {BUDGET['bg_m'] * 1000:.0f} mm BG, see rov.py"),
    # Electronics.
    ("electronics", "Raspberry Pi 5, 4 GB, with cooler and card", 1, 7600, "market", False,
     "flight computer: camera, vision, supervisor, ROS 2"),
    ("electronics", "ESP32 devkit", 1, 450, "market", False,
     "actuator interface; the board validated in the loop"),
    ("electronics", "camera module, wide angle", 1, 3200, "market", False, "pilot feed and vision"),
    ("electronics", "IMU with onboard fusion", 1, 2500, "market", False, "attitude and heading hold"),
    ("electronics", "depth sensor, MS5837-30BA module", 1, 2800, "market", False, "depth hold"),
    ("electronics", "DC-DC converter, 36 to 75 V in, 16 V out, 400 W", 2, 3500, "estimate", False,
     "thruster rail; two share the 800 W governor cap; the input range covers the supply with margin"),
    ("electronics", "DC-DC converter, 36 to 75 V in, 5 V out, 5 A", 1, 600, "market", False, "computer rail"),
    ("electronics", "kill switch: reed input and 100 A high side switch", 1, 1800, "estimate", True,
     "external magnet switch opens the thruster rail"),
    ("electronics", "fuses, holders, distribution board", 1, 1500, "market", False,
     "one fuse per converter, one per thruster pair"),
    ("electronics", "leak probes", 2, 150, "market", False, "end caps; trips the supervisor"),
    ("electronics", "thermal bridge, 6061 plate and pads to the rear end cap", 1, 850, "estimate", True,
     "conducts the converter losses to the water-cooled cap instead of the still air in the tube"),
    ("electronics", "lamp, 15 W COB LED and driver, potted housing", 2, 1200, "estimate", True,
     "placed 0.2 m either side of the lens, see vision_eval.py"),
    ("electronics", "wire, connectors, heat shrink", 1, 2000, "market", False, "silicone wire, XT60, JST"),
    # Tether and surface station.
    ("tether", "power cable, 4 core 2.5 mm2 flexible, 55 m", 55, 110, "estimate", False,
     "two cores per leg, 0.34 ohm loop, see rov.py"),
    ("tether", "Cat5e, 55 m", 55, 25, "market", False,
     "plain Ethernet; 50 m is inside the 100 m limit, no modem needed"),
    ("tether", "floats, braid sleeve, strain relief", 1, 1500, "estimate", True,
     "makes the bundle neutrally buoyant, so it does not drag on the floor"),
    ("surface", f"power supply, Mean Well RSP-1000-48, set to {TT['supply_v']:.0f} V", 1, round(268 * USD_INR),
     "converted", False, "mains input 90 to 264 V AC, so 230 V 50 Hz compatible; output adjusts 43 to 55 V"),
    ("surface", f"emergency stop, {TT['fuse_a']} A DC breaker, main fuse", 1, 1800, "estimate", False,
     "fuse sized at 1.25 times the governed peak current"),
    ("surface", "enclosure and mains inlet", 1, 1500, "estimate", True, "keeps the supply off a wet deck"),
    ("surface", "gamepad", 1, 2500, "market", False, "pilot input"),
    # Test equipment.
    ("test", "thrust rig: 10 kg load cell and amplifier", 1, 900, "market", True,
     "bench thrust curve for every thruster before it is fitted"),
    ("test", "hand vacuum pump with gauge", 1, 1500, "market", False,
     "hull leak test before every dive"),
    ("test", "practice pool hire, four sessions", 4, 2000, "estimate", False,
     "trimming, pilot practice and a course mock up"),
]
CONTINGENCY = 0.10


def main(write=True):
    rows, groups = [], {}
    for g, item, q, u, basis, made, why in LINES:
        cost = q * u
        rows.append({"group": g, "item": item, "qty": q, "unit_inr": u, "cost_inr": cost,
                     "basis": basis, "in_house": made, "reason": why})
        groups[g] = groups.get(g, 0) + cost
    build = sum(v for k, v in groups.items() if k not in ("test",))
    vehicle = sum(v for k, v in groups.items() if k not in ("test", "surface", "tether"))
    made = sum(r["cost_inr"] for r in rows if r["in_house"] and r["group"] not in ("test",))
    sub = sum(groups.values())
    cont = round(sub * CONTINGENCY)
    out = {"rows": rows, "groups": groups, "vehicle_inr": vehicle, "build_inr": build,
           "subtotal_inr": sub, "contingency_pct": CONTINGENCY * 100, "contingency_inr": cont,
           "total_inr": sub + cont, "in_house_inr": made,
           "in_house_items": sum(1 for r in rows if r["in_house"]),
           "n_lines": len(rows), "usd_inr": USD_INR}
    print(f"isonavi-rov bill of materials, {len(rows)} lines, rupees incl. GST")
    for g, v in groups.items():
        print(f"  {g:12s}{v:>10,d}")
    print(f"  vehicle only        {vehicle:>10,d}")
    print(f"  build incl. tether and surface {build:>10,d}")
    print(f"  contingency {CONTINGENCY*100:.0f} %     {cont:>10,d}")
    print(f"  total               {sub + cont:>10,d}")
    print(f"  made in house: {out['in_house_items']} lines, Rs {made:,d} of parts cost")
    if write:
        json.dump(out, open(os.path.join(HERE, "rov_bom.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
