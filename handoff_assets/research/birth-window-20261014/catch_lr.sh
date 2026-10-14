#!/bin/bash
# Copy every transient *.lane-refit.xodr of the running conversions (lane_refit.apply_with_mouths deletes it after the
# mouth step) into $S/bw/lrcheck, once it has not changed for 3 s; stop when all four runs are done.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
while true; do
  for f in $S/runs/sb-bw2/*.lane-refit.xodr $S/runs/gen-bw2/*.lane-refit.xodr $S/runs/gen-slow-bw2/*.lane-refit.xodr; do
    [ -f "$f" ] || continue
    age=$(( $(date +%s) - $(stat -c %Y "$f" 2>/dev/null || date +%s) ))
    [ $age -ge 3 ] || continue
    d=$S/bw/lrcheck/$(basename "$f")
    [ -f "$d" ] && cmp -s "$f" "$d" && continue
    cp "$f" "$d.tmp" 2>/dev/null && mv "$d.tmp" "$d" && echo "$(date +%T) $(basename $f)"
  done
  [ -f $S/runs/sb-bw2.done ] && [ -f $S/runs/gen-bw2.done ] && [ -f $S/runs/gen-slow-bw2.0707104.done ] && [ -f $S/runs/gen-slow-bw2.061310.done ] && break
  sleep 2
done
echo finished
