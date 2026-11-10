#!/usr/bin/env bash
# Regenerate the vehicle budget and the report's numbers, then build the report
# and check that the README quotes the same values.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"

echo "regenerating the vehicle budget, thruster match and bill of materials"
( cd "$repo/simulation/vehicle" && python3 rov.py > /tmp/rov_budget.log 2>&1 \
  && python3 thruster.py > /tmp/rov_thruster.log 2>&1 && python3 rov_bom.py > /tmp/rov_bom.log 2>&1 )

echo "regenerating metrics from the result files"
( cd "$repo/simulation" && PYTHONPATH=. python3 eval/make_rov_metrics.py )

doc=isonavi_rov_report
echo "building $doc"
( cd "$repo/docs" && pdflatex -interaction=nonstopmode "$doc.tex" > "/tmp/$doc.1.log" 2>&1 \
  && pdflatex -interaction=nonstopmode "$doc.tex" > "/tmp/$doc.2.log" 2>&1 )
if grep -qE '^!' "/tmp/$doc.2.log"; then
  echo "  FAILED:"; grep -E '^!' "/tmp/$doc.2.log" | head -5; exit 1
fi
grep -o 'Output written.*' "/tmp/$doc.2.log" | sed 's/^/  /'
python3 "$here/check_rov.py"

echo "done"
