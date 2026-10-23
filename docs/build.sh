#!/usr/bin/env bash
# Regenerate the numbers, then both documents.
#
# The brief and the report share metrics.tex, so they cannot quote different
# values for the same quantity. Building them separately is what would let
# them drift, which is why this exists rather than a note in a README.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"

echo "regenerating bill of materials and programme budget"
( cd "$repo/cad" && python3 bom.py > /tmp/isonavi_bom.log 2>&1 )

echo "regenerating metrics from the measured result files"
( cd "$repo/simulation" && PYTHONPATH=. python3 eval/make_metrics_tex.py )

for doc in isonavi_report isonavi_brief; do
  echo "building $doc"
  ( cd "$repo/docs" \
    && pdflatex -interaction=nonstopmode "$doc.tex" > "/tmp/$doc.1.log" 2>&1 \
    && pdflatex -interaction=nonstopmode "$doc.tex" > "/tmp/$doc.2.log" 2>&1 )
  if grep -qE '^!' "/tmp/$doc.2.log"; then
    echo "  FAILED:"; grep -E '^!' "/tmp/$doc.2.log" | head -5; exit 1
  fi
  grep -o 'Output written.*' "/tmp/$doc.2.log" | sed 's/^/  /'
  if grep -qi 'undefined control sequence' "/tmp/$doc.2.log"; then
    echo "  WARNING: undefined macro, a result file is probably missing"
  fi
done

# Nothing in these documents should carry a dash character that is not a hyphen.
if grep -rlP '\xe2\x80\x94|\xe2\x80\x93' "$repo/docs"/*.tex >/dev/null 2>&1; then
  echo "  WARNING: em or en dash found in a source file"
fi

echo "regenerating the isonavi-rov budget, thruster match, bill of materials and metrics"
( cd "$repo/cad" && python3 rov.py > /tmp/rov_budget.log 2>&1 && python3 thruster.py > /tmp/rov_thruster.log 2>&1   && python3 rov_bom.py > /tmp/rov_bom.log 2>&1 )
( cd "$repo/simulation" && PYTHONPATH=. python3 eval/make_rov_metrics.py )
doc=isonavi_rov_report
echo "building $doc"
( cd "$repo/docs"   && pdflatex -interaction=nonstopmode "$doc.tex" > "/tmp/$doc.1.log" 2>&1   && pdflatex -interaction=nonstopmode "$doc.tex" > "/tmp/$doc.2.log" 2>&1 )
if grep -qE '^!' "/tmp/$doc.2.log"; then
  echo "  FAILED:"; grep -E '^!' "/tmp/$doc.2.log" | head -5; exit 1
fi
grep -o 'Output written.*' "/tmp/$doc.2.log" | sed 's/^/  /'
python3 "$here/check_rov.py"

echo "done"
