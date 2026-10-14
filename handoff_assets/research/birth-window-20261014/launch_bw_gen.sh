#!/bin/bash
# Detached generalization conversion of the birth-window tree (bw-tree), BLAS single-thread, started after
# launch_bw.sh (the scoreboard and the slow junctions were still running); writes gen-bw.done with its exit code.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
T=$S/bw-tree
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
mkdir -p $S/runs/gen-bw
GEN=2023041111160971268,2023062713394062899,2023063015350832057,2023080118313036830,2023041216222042093,2023062110304177600,2023070118182048481,2023080810023203229,2023062913464138453,2023072914272536839,2023041810414040830,2023070710324132429,2023081110220126408,2023071109582525264

cd $T
setsid nohup bash -c "date; $PY $S/bnd/gen_part.py $S/runs/gen-bw $GEN 2 all; echo \$? > $S/runs/gen-bw.done; date" \
  > $S/runs/gen-bw.log 2>&1 < /dev/null &
sleep 2
pgrep -af "sb_gen.py|gen_part.py|mapforge.cli convert" | grep -v pgrep | cut -c1-200
