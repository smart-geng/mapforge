#!/bin/bash
# Before/after views of the two roads with lane swaps (0711 road 11 right, 0411 road 13 left): main (gen-pg) vs this
# change (gen-sw). Writes into the evidence staging folder given as $1.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
E=$1
B=$S/runs/gen-pg; A=$S/runs/gen-sw
J11=shp-2023071109582525264.xodr; J04=shp-2023041111160971268.xodr
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
cd $S/sw-tree
ST11=$(python3 -c "print(','.join(str(x / 2) for x in range(1, 156)))")
ST04=$(python3 -c "print(','.join(str(60 + x / 2) for x in range(0, 165)))")
for v in before:$B after:$A; do
  n=${v%%:*}; d=${v#*:}
  $PY $S/bnd/cross_sections.py $d/$J11 11 $ST11 $E/0711-road11-xsec-$n.json > /dev/null
  $PY $S/bnd/cross_sections.py $d/$J04 13 $ST04 $E/0411-road13-xsec-$n.json both > /dev/null
  $PY $S/bnd/road_edges.py $d/$J11 11 $E/0711-road11-edges-$n.json > /dev/null
  $PY $S/bnd/road_edges.py $d/$J04 13 $E/0411-road13-edges-$n.json > /dev/null
  $PY $S/bnd/plot_st.py $d/$J11 11 0 78.2 $E/0711-road11-st-$n.png right
  $PY $S/bnd/plot_st.py $d/$J04 13 55 145 $E/0411-road13-st-$n.png left
done
$PY $S/bnd/runs_lanes.py $E/boundary-runs-by-lane.json 0.05 0711=$B/$J11,$A/$J11 0411=$B/$J04,$A/$J04
{ echo "0711 road 11, right side: largest |curvature| of each lane edge per laneSection, main -> this change"
  $PY $S/bnd/edge_curv_where.py $B/$J11 $A/$J11 11 right
  echo
  echo "0411 road 13, left side"
  $PY $S/bnd/edge_curv_where.py $B/$J04 $A/$J04 13 left; } > $E/edge-curvature.txt
$PY $S/bnd/plot_before_after2.py $B/$J11 $A/$J11 11 50 78.2 $E/0711-road11-s50-78-before-after.png main lane-swap
$PY $S/bnd/plot_before_after2.py $B/$J11 $A/$J11 11 0 12 $E/0711-road11-s0-12-before-after.png main lane-swap
$PY $S/bnd/plot_before_after2.py $B/$J04 $A/$J04 13 60 95 $E/0411-road13-s60-95-before-after.png main lane-swap
echo analyzed
