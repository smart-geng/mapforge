"""lane_refit stage on finished outputs (refit marker removed, birth-advance markers kept), logging advanced events
whose station would move to the source taper's zero point (exp2-tree, MF_EVENT_SOURCE=1)."""
import os, sys, tempfile
from pathlib import Path
from lxml import etree
from mapforge.ops import lane_refit as LR
params = dict(LR.VARIANTS["g2-k04-c2"]); params.pop("edge_joins")
for k in ("mouth_blend_kappa", "kappa_bound_scale", "mouth_blend_pick", "source_guided", "width_local_slopes",
          "match_end_curvature", "monotone_turns", "aligned_frame", "source_fit", "turn_end_zone"):
    params.pop(k, None)
tmp = Path(tempfile.mkdtemp())
for path in sys.argv[1:]:
    log = tmp / (Path(path).stem + ".log")
    os.environ["MF_EVENT_LOG"] = str(log)
    tree = etree.parse(path)
    for u in list(tree.getroot().iter("userData")):
        if u.get("code") == LR.CODE:
            u.getparent().remove(u)
    src = tmp / "in.xodr"; tree.write(str(src))
    LR.apply(src, tmp / "out.xodr", params)
    moves = sorted({l.strip() for l in open(log) if l.startswith("road") and "->" in l} if log.exists() else set())
    moves = [m for m in moves if m.split("x=")[1].split(" -> ")[0] != m.split(" -> ")[1].split(" ")[0]]
    print(Path(path).name, moves, flush=True)
