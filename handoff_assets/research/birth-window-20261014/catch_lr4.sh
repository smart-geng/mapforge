#!/bin/bash
# Copy every transient shp-*.lane-refit.xodr of the final validation's conversions (lane_refit.apply_with_mouths deletes
# it after the mouth step) into $S/bw/lrcheck4, once it has not changed for 3 s; stop when all four runs are done.
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
mkdir -p $S/bw/lrcheck4
while true; do
  for f in $S/runs/sb-bw4/shp-*.lane-refit.xodr $S/runs/gen-bw4/shp-*.lane-refit.xodr $S/runs/gen-slow-bw4/shp-*.lane-refit.xodr; do
    [ -f "$f" ] || continue
    age=$(( $(date +%s) - $(stat -c %Y "$f" 2>/dev/null || date +%s) ))
    [ $age -ge 3 ] || continue
    d=$S/bw/lrcheck4/$(basename "$f")
    [ -f "$d" ] && cmp -s "$f" "$d" && continue
    cp "$f" "$d.tmp" 2>/dev/null && mv "$d.tmp" "$d" && echo "$(date +%T) $(basename $f)"
  done
  [ -f $S/runs/sb-bw4.done ] && [ -f $S/runs/gen-bw4.done ] && [ -f $S/runs/gen-slow-bw4.0707104.done ] && [ -f $S/runs/gen-slow-bw4.061310.done ] && break
  sleep 2
done
echo finished
