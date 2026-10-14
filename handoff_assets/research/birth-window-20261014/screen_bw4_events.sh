#!/bin/bash
# The per-event analyses of screen_bw.sh (step 5) and the whole-road deviation (road_dev.py) again, on the final
# version's lane_refit stage ($S/bw/new4, screen_bw4.sh) against main's ($S/swap/st2). The third version's results
# (screen_bw3_events.sh, on $S/bw/new3) were renamed *-shoulder-guard.txt before this ran. BLAS single-thread.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
E=$1
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
files="shp-2023061310244790027 shp-2023062713394062899 shp-2023063015350832057 shp-2023070710412621654 shp-node16 shp-node17 shp-node18 shp-node3 shp-node4"
cd $S/sw-tree && $PY $E/event_dev.py $E/adv-main.json $E/adv-window.json $S/swap/st2 $S/bw/new4 $files > $E/event-dev.txt
cd $S/bw4-tree
ec() { echo "### $1 road $2 $3"; $PY $S/bnd/edge_curv_where.py $S/swap/st2/$1.stage.xodr $S/bw/new4/$1.stage.xodr $2 $3 0.01; }
( ec shp-2023061310244790027 12 left; ec shp-2023062713394062899 12 left; ec shp-2023063015350832057 11 right
  ec shp-2023063015350832057 12 left; ec shp-2023070710412621654 10 right; ec shp-2023070710412621654 13 right
  ec shp-node16 11 right; ec shp-node17 10 right; ec shp-node18 13 left; ec shp-node3 13 left; ec shp-node3 13 right
  ec shp-node4 10 left; ec shp-node4 13 left ) > $E/stage-edge-curvature.txt
EVS="shp-2023061310244790027:12:left:88.35 shp-2023062713394062899:12:left:128.76 shp-2023063015350832057:11:right:108.62
 shp-2023063015350832057:12:left:98.23 shp-2023070710412621654:10:right:78.85 shp-2023070710412621654:13:right:65.07
 shp-node16:11:right:158.27 shp-node17:10:right:81.01 shp-node18:13:left:38.11 shp-node3:13:left:131.45
 shp-node3:13:right:69.00 shp-node4:10:left:60.00 shp-node4:13:left:152.24"
$PY $E/event_curv.py $S/swap/st2 $S/bw/new4 .stage.xodr $EVS > $E/event-curvature-stage.txt
J=shp-2023063015350832057
( echo "== 0630 road 11 right -3 (birth 108.62)"
  $PY $E/event_widths.py $S/next/raw/$J.raw.xodr $S/swap/st2/$J.stage.xodr $S/bw/new4/$J.stage.xodr 11 right -3 103 116
  echo "== 0630 road 12 left 4 (birth 98.23)"
  $PY $E/event_widths.py $S/next/raw/$J.raw.xodr $S/swap/st2/$J.stage.xodr $S/bw/new4/$J.stage.xodr 12 left 4 92 106 ) > $E/event-widths.txt
args=""
for spec in shp-2023061310244790027:12:88.35 shp-2023062713394062899:12:128.76 shp-2023063015350832057:11:108.62 \
    shp-2023063015350832057:12:98.23 shp-2023070710412621654:13:65.07 shp-node16:11:158.27 shp-node17:10:81.01 \
    shp-node18:13:38.11 shp-node3:13:131.45,69.00 shp-node4:10:60.00 shp-node4:13:152.24; do
  n=${spec%%:*}; r=${spec#*:}; rid=${r%%:*}; ev=${r#*:}; args="$args $n:$rid:left:$ev $n:$rid:right:$ev"
done
$PY $E/road_dev.py $S/swap/st2 $S/bw/new4 .stage.xodr $args > $E/road-dev-stage.txt 2>&1
