#!/bin/bash
# Screening of the final birth-window code (keep_own_way: the curb-shoulder guard added) on the 19 lane_refit inputs:
# the lane_refit stage (c_stage_hold.py, ../hold-width-retry-20261013/) of every input with bw3-tree, 2 at a time
# (the second validation's conversions were still running), then compared road by road (stage_diff.py,
# ../prune-guard-20261013/) with main's stage ($S/swap/st2, ../lane-swap-20261014/run_st2.sh) and with the stage of
# the version without the guard ($S/bw/new2, screen_bw.sh). BLAS single-thread.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
E=$1
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
run() { f=$1; cd $S/bw3-tree && timeout 2400 $PY $S/bnd/c_stage_hold.py $S/next/raw/$f.raw.xodr $S/bw/new3/$f.stage.xodr $S/bw/new3/$f.stage.json > $S/bw/new3/$f.stage.log 2>&1; echo "$f rc=$?"; }
export -f run; export S PY
mkdir -p $S/bw/new3
ls $S/next/raw/*.raw.xodr | xargs -n1 basename | sed 's/.raw.xodr//' | xargs -P 2 -I{} bash -c 'run {}'
all=$(ls $S/next/raw/*.raw.xodr | xargs -n1 basename | sed 's/.raw.xodr//')
cd $S/bw3-tree
$PY $S/bnd/stage_diff.py $E/stage-diff-guard-vs-main.json $(for f in $all; do echo $S/swap/st2/$f.stage.xodr=$S/bw/new3/$f.stage.xodr; done)
$PY $S/bnd/stage_diff.py $E/stage-diff-guard-vs-noguard.json $(for f in $all; do echo $S/bw/new2/$f.stage.xodr=$S/bw/new3/$f.stage.xodr; done)
