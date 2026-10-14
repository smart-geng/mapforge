#!/bin/bash
# lane_refit stage (c_stage_hold.py) of all 19 raw inputs with the lane-swap tree, BLAS single-thread.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
cd $S/sw-tree
for x in $S/next/raw/*.raw.xodr; do
  n=$(basename $x .raw.xodr)
  $PY $S/bnd/c_stage_hold.py $x $S/swap/st2/$n.stage.xodr $S/swap/st2/$n.stage.json > $S/swap/st2/$n.stage.log 2>&1
  echo "$n $?"
done
echo done
