"""Rerun the lane_refit stage (ks-tree, MF_KS_MODE=ks-nonstraight, logging) on finished outputs with the refit
marker removed, and list anchored flare zones whose run into the mouth is steeper than MOUTH_FLARE_MAX."""
import json, os, sys, tempfile
from pathlib import Path
from lxml import etree
from mapforge.ops import lane_refit as LR
params = dict(LR.VARIANTS["g2-k04-c2"]); params.pop("edge_joins")
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
tmp = Path(tempfile.mkdtemp())
for path in sys.argv[1:]:
    log = tmp / (Path(path).stem + ".jsonl")
    os.environ["MF_ANCHOR_LOG"] = str(log)
    tree = etree.parse(path)
    for u in list(tree.getroot().iter("userData")):
        if u.get("code") == LR.CODE:
            u.getparent().remove(u)
    src = tmp / "in.xodr"; tree.write(str(src))
    LR.apply(src, tmp / "out.xodr", params)
    hits = set()
    if log.exists():
        for line in open(log):
            d = json.loads(line)
            if d["anchor"] is not None and not d["straight"] and abs(d["zone_slope"]) > LR.MOUTH_FLARE_MAX:
                hits.add((d["which"], round(d["a"], 2), round(d["b"], 2), round(d["zone_slope"], 3), round(d["kept_slope"], 3)))
    print(Path(path).name, sorted(hits), flush=True)
