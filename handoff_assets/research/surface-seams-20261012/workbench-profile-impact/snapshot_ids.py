"""Workbench source snapshot identities (snapshot_id, content_hash) for the registered junctions, built from this
tree's Profile; cwd = code tree. Prints JSON."""
import json, sys, time
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from mapforge.workbench.sources import SourceCatalog
root = Path.cwd()
t = time.time()
cat = SourceCatalog(root / "shp_0222-0326", root / "profiles/shp/ibd-smarteditor-v1.yaml")
out = {"profile_sha256": cat.profile_hash, "snapshot_id": cat.snapshot_id, "load_seconds": round(time.time() - t, 1),
       "layers": sorted(cat.rows), "load_issues": cat.load_issues}
for jid in ("2023061509381992034", "2023062110304177600"):
    snap = cat.snapshot(jid)
    out[jid] = {"snapshot_id": snap["snapshot_id"], "content_hash": snap.get("content_hash"),
                "objects": len(snap["objects"]), "roles": sorted({o["role"] for o in snap["objects"]})}
print(json.dumps(out, ensure_ascii=False, indent=1))
