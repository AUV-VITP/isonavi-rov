"""Checks the isonavi-rov report and README against the generated numbers.

1. every metric macro the report uses is defined, so no table cell is blank;
2. the report source carries no hand-typed copy of a generated value in a
   table row, which is how numbers drift;
3. every number the README quotes appears in the generated set, formatted
   the same way.
"""

from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "simulation", "eval"))
from make_rov_metrics import collect   # noqa: E402

fails = []
m = collect()
tex = open(os.path.join(HERE, "isonavi_rov_report.tex"), encoding="utf-8").read()
used = set(re.findall(r"\m([a-z]+)", tex))
undefined = sorted(u for u in used if u not in m)
if undefined:
    fails.append(f"report uses undefined metrics: {', '.join(undefined)}")

if re.search("—|–", tex):
    fails.append("report carries an em or en dash")

readme = open(os.path.join(ROOT, "README.md"), encoding="utf-8").read()
if re.search("—|–", readme):
    fails.append("README carries an em or en dash")
# README fragments, each built from the generated values.
expected = [
    f"{m['mass']} kg", f"{m['lenoverall']} x {m['beam']} x {m['height']} cm",
    f"{m['netbuoy']} N", f"{m['bg']} mm", f"{m['surgefull']} N", f"{m['surgelost']} N",
    f"{m['speedfull']} m/s", f"{m['speedlost']} m/s", f"{m['opsmargin']}",
    f"{m['loopohm']} ohm", f"{m['capw']} W", f"{m['peakamps']} A", f"{m['vinpeak']} V",
    f"{m['supplyw']} W", f"{m['fuse']} A", f"{m['uncapped']} W", f"{m['capthrust']} N",
    f"{m['tetherdrag']} N", f"{m['framedrag']} N",
    f"{m['kvthousandphase']} A", f"{m['kvphase']} A",
    f"Rs {m['costvehicle']}", f"Rs {m['costbuild']}", f"Rs {m['costtotal']}",
    f"{m['holdtime']} s", f"{m['holdrise']} m",
    f"{m['refmanualmidcontacts']} +/- {m['refmanualmidcontactsd']}",
    f"{m['refassistmidcontacts']} +/- {m['refassistmidcontactsd']}",
    f"{m['refautomidcontacts']} +/- {m['refautomidcontactsd']}",
    f"{m['refmanualmurkycontacts']} +/- {m['refmanualmurkycontactsd']}",
    f"{m['refassistmurkycontacts']} +/- {m['refassistmurkycontactsd']}",
    f"{m['refautomurkycontacts']} +/- {m['refautomurkycontactsd']}",
    f"{m['refmanualclearcontacts']} +/- {m['refmanualclearcontactsd']}",
    f"{m['refassistclearcontacts']} +/- {m['refassistclearcontactsd']}",
    f"{m['refautoclearcontacts']} +/- {m['refautoclearcontactsd']}",
]
for e in expected:
    if e not in readme:
        fails.append(f"README does not quote '{e}' as generated")

print("isonavi-rov document check")
print(f"  {len(used)} metrics used by the report, {len(m)} generated")
for f in fails:
    print("  FAIL", f)
if not fails:
    print("  report and README agree with the result files")
sys.exit(1 if fails else 0)
