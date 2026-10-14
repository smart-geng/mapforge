#!/bin/bash
# Detached conversions of the lane-swap tree (sw-tree), BLAS single-thread; each job writes <name>.done with its exit code.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
T=$S/sw-tree
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
mkdir -p $S/runs/sb-sw $S/runs/gen-sw $S/runs/gen-slow-sw
GEN=2023041111160971268,2023062713394062899,2023063015350832057,2023080118313036830,2023041216222042093,2023062110304177600,2023070118182048481,2023080810023203229,2023062913464138453,2023072914272536839,2023041810414040830,2023070710324132429,2023081110220126408,2023071109582525264

cd $T
setsid nohup bash -c "date; $PY $S/bnd/sb_gen.py $S/runs/sb-sw shp,map 2; echo \$? > $S/runs/sb-sw.done; date" \
  > $S/runs/sb-sw.log 2>&1 < /dev/null &
setsid nohup bash -c "date; $PY $S/bnd/gen_part.py $S/runs/gen-sw $GEN 2 all; echo \$? > $S/runs/gen-sw.done; date" \
  > $S/runs/gen-sw.log 2>&1 < /dev/null &
setsid nohup bash -c "date; $PY -m mapforge.cli convert $T/shp_0222-0326 --at 106.31411600801903,29.586641338727574 --to xodr -o $S/runs/gen-slow-sw/shp-2023070710412621654.xodr > $S/runs/gen-slow-sw/shp-2023070710412621654.convert.log 2>&1; echo \$? > $S/runs/gen-slow-sw.done; date" \
  > $S/runs/gen-slow-sw.log 2>&1 < /dev/null &
sleep 2
pgrep -af "sb_gen.py|gen_part.py|mapforge.cli convert" | grep -v pgrep | cut -c1-200
