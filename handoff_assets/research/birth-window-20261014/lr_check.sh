#!/bin/bash
# The lane_refit stage of the full conversions (the transient shp-*.lane-refit.xodr, copied by catch_lr4.sh into
# $S/bw/lrcheck4) byte for byte against the screening's lane_refit stage of the same input (screen_bw4.sh, $S/bw/new4);
# then the same for the third version's copies (catch_lr3.sh, $S/bw/lrcheck3) against its screening ($S/bw/new3) and
# for the second version's (catch_lr.sh, $S/bw/lrcheck) against its screening ($S/bw/new2).
S=/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad
for v in "final:lrcheck4:new4" "third version:lrcheck3:new3" "second version:lrcheck:new2"; do
  name=${v%%:*}; r=${v#*:}; got=${r%%:*}; st=${r#*:}
  echo "== $name: $S/bw/$got/*.lane-refit.xodr vs $S/bw/$st/*.stage.xodr"
  for f in $S/bw/$got/shp-*.lane-refit.xodr; do
    b=$(basename $f .lane-refit.xodr)
    if [ ! -f $S/bw/$st/$b.stage.xodr ]; then echo "$b  no screening stage"
    elif cmp -s $f $S/bw/$st/$b.stage.xodr; then echo "$b  identical  $(sha256sum < $f | cut -c1-16)"
    else echo "$b  DIFFERENT"; fi
  done
done
