"""Opt-in MAP joint solve using elastic trust-region feasibility restoration."""
import argparse
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spikes import road_boundary_family as family, joint_mouth_candidate as joint
from spikes.clarabel_joint_candidate import interior_qp


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('source', type=Path)
    parser.add_argument('target', type=Path); args = parser.parse_args()
    def solve(*a, **kw):
        return family.solve_road(*a, **kw, restore_dynamics=True)
    with patch.object(family, '_convex_qp', interior_qp), patch.object(joint, 'solve_road', solve):
        ok = joint.run(args.source, args.target)
    raise SystemExit(0 if ok else 2)
