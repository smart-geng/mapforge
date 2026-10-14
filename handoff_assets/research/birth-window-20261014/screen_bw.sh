#!/bin/bash
# Screening of the birth-window round on the 19 lane_refit inputs of the earlier rounds (main conversions' *.raw.xodr,
# ../prune-guard-20261013/stage_raw.py): main = sw-tree (df7455b), this change = bw-tree (the committed files);
# BLAS single-thread. Main's lane_refit stage of the 19 inputs is the lane-swap round's (../lane-swap-20261014/run_st2.sh,
# $S/swap/st2).
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
E=$1                                  # evidence staging folder
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
# 1. lane_refit.advance_births of both trees on all 19 inputs (report, hash of the advanced tree, own_way)
(cd $S/sw-tree && $PY $E/adv_report.py $E/adv-main.json $S/next/raw/*.raw.xodr) > $E/adv-main.log 2>&1 &
(cd $S/bw-tree && $PY $E/adv_report.py $E/adv-window.json $S/next/raw/*.raw.xodr) > $E/adv-window.log 2>&1 &
wait
# 2. the events only the window measures and advances (9 of the 19 inputs have some)
cd $S/bw-tree && $PY $E/new_events.py $E/adv-main.json $E/adv-window.json $S/next/raw > $E/new-events.txt
# 3. every birth/death of the 19 inputs by whether its slope is measured (own section / window / not)
cd $S/bw-tree && $PY $E/window_census.py $E/window-census.json $S/next/raw/*.raw.xodr > $E/window-census.txt
# 4. the lane_refit stage (c_stage_hold.py, ../hold-width-retry-20261013/) of all 19 inputs with this change, 4 at a time
run() { f=$1; cd $S/bw-tree && timeout 2400 $PY $S/bnd/c_stage_hold.py $S/next/raw/$f.raw.xodr $S/bw/new2/$f.stage.xodr $S/bw/new2/$f.stage.json > $S/bw/new2/$f.stage.log 2>&1; echo "$f rc=$?"; }
export -f run; export S PY
mkdir -p $S/bw/new2
ls $S/next/raw/*.raw.xodr | xargs -n1 basename | sed 's/.raw.xodr//' | xargs -P 4 -I{} bash -c 'run {}'
# 5. roads that differ from main's stage (stage_diff.py, ../prune-guard-20261013/), source boundary deviation near each
#    new event, lane-edge curvature of the changed roads and near each new event, source and written widths of two
#    event lanes
all=$(ls $S/next/raw/*.raw.xodr | xargs -n1 basename | sed 's/.raw.xodr//')
cd $S/bw-tree && $PY $S/bnd/stage_diff.py $E/stage-diff.json $(for f in $all; do echo $S/swap/st2/$f.stage.xodr=$S/bw/new2/$f.stage.xodr; done)
files="shp-2023061310244790027 shp-2023062713394062899 shp-2023063015350832057 shp-2023070710412621654 shp-node16 shp-node17 shp-node18 shp-node3 shp-node4"
cd $S/sw-tree && $PY $E/event_dev.py $E/adv-main.json $E/adv-window.json $S/swap/st2 $S/bw/new2 $files > $E/event-dev.txt
cd $S/bw-tree
ec() { echo "### $1 road $2 $3"; $PY $S/bnd/edge_curv_where.py $S/swap/st2/$1.stage.xodr $S/bw/new2/$1.stage.xodr $2 $3 0.01; }
( ec shp-2023061310244790027 12 left; ec shp-2023062713394062899 12 left; ec shp-2023063015350832057 11 right
  ec shp-2023063015350832057 12 left; ec shp-2023070710412621654 10 right; ec shp-2023070710412621654 13 right
  ec shp-node16 11 right; ec shp-node17 10 right; ec shp-node18 13 left; ec shp-node3 13 left; ec shp-node3 13 right
  ec shp-node4 10 left; ec shp-node4 13 left ) > $E/stage-edge-curvature.txt
EVS="shp-2023061310244790027:12:left:88.35 shp-2023062713394062899:12:left:128.76 shp-2023063015350832057:11:right:108.62
 shp-2023063015350832057:12:left:98.23 shp-2023070710412621654:10:right:78.85 shp-2023070710412621654:13:right:65.07
 shp-node16:11:right:158.27 shp-node17:10:right:81.01 shp-node18:13:left:38.11 shp-node3:13:left:131.45
 shp-node3:13:right:69.00 shp-node4:10:left:60.00 shp-node4:13:left:152.24"
$PY $E/event_curv.py $S/swap/st2 $S/bw/new2 .stage.xodr $EVS > $E/event-curvature-stage.txt
J=shp-2023063015350832057
( echo "== 0630 road 11 right -3 (birth 108.62)"
  $PY $E/event_widths.py $S/next/raw/$J.raw.xodr $S/swap/st2/$J.stage.xodr $S/bw/new2/$J.stage.xodr 11 right -3 103 116
  echo "== 0630 road 12 left 4 (birth 98.23)"
  $PY $E/event_widths.py $S/next/raw/$J.raw.xodr $S/swap/st2/$J.stage.xodr $S/bw/new2/$J.stage.xodr 12 left 4 92 106 ) > $E/event-widths.txt
