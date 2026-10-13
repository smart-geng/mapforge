#!/bin/bash
# lane_refit stage of every raw input, in the hold-width tree (rw) and the prune-guard tree (pg)
cd $1
for raw in /tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/next/raw/*.raw.xodr; do
  n=$(basename $raw .raw.xodr)
  OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8 timeout 1800 /tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/cloud-verify/mapforge/.venv/bin/python /tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/bnd/c_stage_hold.py $raw $2/$n.stage.xodr $2/$n.stage.json > $2/$n.stage.log 2>&1
  echo "$n exit $?"
done
