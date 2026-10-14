#!/bin/bash
# Detached conversions of the final birth-window tree (bw-tree with redo_skipped), BLAS single-thread; each job writes
# <name>.done with its exit code. Started after the last change of the three committed files. The first launch
# (launch_bw.sh, launch_bw_gen.sh) ran the version without redo_skipped and was stopped; its 061310 base conversion
# with main's code (gen-slow-base13) had finished and is kept as the base.
# Correction after the run: not the final version. The curb-shoulder guard (launch_bw3.sh) and the source-deviation
# guard (launch_bw4.sh) followed, each after another change of the three files. Only this launch's slow junctions
# ran to the end; its scoreboard and generalization runs were stopped part way.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
PY=$S/cloud-verify/mapforge/.venv/bin/python
T=$S/bw-tree
export OPENBLAS_NUM_THREADS=1 PYTHONIOENCODING=utf-8
mkdir -p $S/runs/sb-bw2 $S/runs/gen-bw2 $S/runs/gen-slow-bw2
GEN=2023041111160971268,2023062713394062899,2023063015350832057,2023080118313036830,2023041216222042093,2023062110304177600,2023070118182048481,2023080810023203229,2023062913464138453,2023072914272536839,2023041810414040830,2023070710324132429,2023081110220126408,2023071109582525264
AT0707104=106.31411600801903,29.586641338727574
AT061310=106.31586320301415,29.527374102902616

cd $T
setsid nohup bash -c "date; $PY $S/bnd/sb_gen.py $S/runs/sb-bw2 shp,map 2; echo \$? > $S/runs/sb-bw2.done; date" \
  > $S/runs/sb-bw2.log 2>&1 < /dev/null &
setsid nohup bash -c "date; $PY $S/bnd/gen_part.py $S/runs/gen-bw2 $GEN 2 all; echo \$? > $S/runs/gen-bw2.done; date" \
  > $S/runs/gen-bw2.log 2>&1 < /dev/null &
setsid nohup bash -c "date; $PY -m mapforge.cli convert $T/shp_0222-0326 --at $AT0707104 --to xodr -o $S/runs/gen-slow-bw2/shp-2023070710412621654.xodr > $S/runs/gen-slow-bw2/shp-2023070710412621654.convert.log 2>&1; echo \$? > $S/runs/gen-slow-bw2.0707104.done; date" \
  > $S/runs/gen-slow-bw2.0707104.log 2>&1 < /dev/null &
setsid nohup bash -c "date; $PY -m mapforge.cli convert $T/shp_0222-0326 --at $AT061310 --to xodr -o $S/runs/gen-slow-bw2/shp-2023061310244790027.xodr > $S/runs/gen-slow-bw2/shp-2023061310244790027.convert.log 2>&1; echo \$? > $S/runs/gen-slow-bw2.061310.done; date" \
  > $S/runs/gen-slow-bw2.061310.log 2>&1 < /dev/null &
sleep 2
pgrep -af "sb_gen.py|gen_part.py|mapforge.cli convert" | grep -v pgrep | cut -c1-200
