#!/bin/bash
# Before/after analysis of the junctions this change writes differently: main's conversions (scoreboard sb-sw,
# generalization gen-sw, 0707104 gen-slow-sw, 061310 gen-slow-base13) vs the final version's (sb-bw4, gen-bw4,
# gen-slow-bw4). Writes into the evidence staging folder $1.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
E=$1
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
SB0=$S/runs/sb-sw; SB1=$S/runs/sb-bw4; G0=$S/runs/gen-sw; G1=$S/runs/gen-bw4; W1=$S/runs/gen-slow-bw4
J27=shp-2023062713394062899; J30=shp-2023063015350832057; J04=shp-2023070710412621654; J13=shp-2023061310244790027
pairs="shp-node3=$SB0/shp-node3,$SB1/shp-node3 shp-node4=$SB0/shp-node4,$SB1/shp-node4
 shp-node16=$SB0/shp-node16,$SB1/shp-node16 shp-node17=$SB0/shp-node17,$SB1/shp-node17
 shp-node18=$SB0/shp-node18,$SB1/shp-node18 0627=$G0/$J27,$G1/$J27 0630=$G0/$J30,$G1/$J30
 0707104=$S/runs/gen-slow-sw/$J04,$W1/$J04 061310=$S/runs/gen-slow-base13/$J13,$W1/$J13"
cd $S/bw4-tree
# roads that differ in the written XODRs
for p in $pairs; do
  n=${p%%=*}; ab=${p#*=}; a=${ab%%,*}; b=${ab#*,}
  echo "== $n"; $PY $S/bnd/roads_changed.py $a.xodr $b.xodr
done > $E/roads-changed.txt
# G8 and delivery decisions
$PY $E/g8_codes.py $(for p in $pairs; do n=${p%%=*}; ab=${p#*=}; a=${ab%%,*}; b=${ab#*,}
  echo "$n=$(dirname $a),$(dirname $b),$(basename $a)"; done) > $E/g8-codes.txt
# lane-edge curvature near each new event on the written XODRs
( $PY $E/event_curv.py $SB0 $SB1 .xodr shp-node16:11:right:158.27 shp-node17:10:right:81.01 shp-node18:13:left:38.11 \
    shp-node3:13:left:131.45 shp-node3:13:right:69.00 shp-node4:10:left:60.00 shp-node4:13:left:152.24
  $PY $E/event_curv.py $G0 $G1 .xodr $J27:12:left:128.76 $J30:11:right:108.62 $J30:12:left:98.23
  $PY $E/event_curv.py $S/runs/gen-slow-sw $W1 .xodr $J04:10:right:78.85 $J04:13:right:65.07
  $PY $E/event_curv.py $S/runs/gen-slow-base13 $W1 .xodr $J13:12:left:88.35 ) > $E/event-curvature.txt
# boundary_inside_runs by source lane (runs_lanes.py, ../prune-guard-20261013/): lanes that changed or are over 0.1 m
$PY $S/bnd/runs_lanes.py $E/boundary-runs-by-lane.json 0.1 $(for p in $pairs; do n=${p%%=*}; ab=${p#*=}
  a=${ab%%,*}; b=${ab#*,}; echo "$n=$a.xodr,$b.xodr"; done) > $E/boundary-runs-by-lane.txt
# lane-centre fairness peaks (fair_where.py, ../lane-swap-20261014/)
PYTHONPATH=$PWD $PY $S/cloud-verify/mapforge/handoff_assets/research/lane-swap-20261014/fair_where.py $(for p in $pairs; do ab=${p#*=}
  a=${ab%%,*}; b=${ab#*,}; echo "$a.xodr $b.xodr"; done) > $E/fairness-peaks.txt
# (s, t) views of three changed roads (plot_st.py, ../hold-width-retry-20261013/): 0630 road 12 left, 0627 road 12
# left, shp-node16 road 11 right
for v in before:$G0:$SB0 after:$G1:$SB1; do
  n=${v%%:*}; r=${v#*:}; g=${r%%:*}; sb=${r#*:}
  $PY $S/bnd/plot_st.py $g/$J30.xodr 12 85 115 $E/0630-road12-st-$n.png left
  $PY $S/bnd/plot_st.py $g/$J27.xodr 12 115 150 $E/0627-road12-st-$n.png left
  $PY $S/bnd/plot_st.py $sb/shp-node16.xodr 11 145 175 $E/node16-road11-st-$n.png right
done
