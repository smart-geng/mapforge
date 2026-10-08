"""Copy a saved workbench project and rebind identical source data at a new location.

The destination workspace must be new. Historical evidence stays unchanged;
old candidates and checks become stale and must be recomputed in the workbench.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mapforge.workbench.project_relocation import RelocationRejected, relocate_project


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True, help="Saved project ID directory")
    parser.add_argument("--workspace", type=Path, required=True, help="New destination workspace")
    parser.add_argument("--source", type=Path, required=True, help="Identical source data at its new location")
    parser.add_argument("--profile", type=Path, required=True, help="Identical SHP profile")
    args = parser.parse_args()
    try:
        result = relocate_project(args.project, args.workspace, args.source, args.profile)
    except RelocationRejected as exc:
        print(json.dumps({"status": "FAILED", "code": exc.code, "message": str(exc),
                          "pending_directory": getattr(exc, "pending_directory", None)}, ensure_ascii=False))
        return 1
    print(json.dumps({key: result[key] for key in
                      ("status", "project_directory", "relocation_id", "formal_delivery")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
