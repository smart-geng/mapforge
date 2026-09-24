"""Read final XODR using separated acceptance; retain immutable old stress files."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.validate.separated_acceptance import audit_file, sha256, POLICY
from mapforge.repair_web.model import atomic, json_bytes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xodr', type=Path, required=True)
    parser.add_argument('--source-manifest', type=Path, required=True)
    parser.add_argument('--casebook', type=Path)
    parser.add_argument('--historical-stress', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    dest = args.out.resolve()
    if dest == ROOT/'out' or not dest.is_relative_to(ROOT/'out'):
        parser.error('output must be a new child of workspace out/')
    dest.mkdir(exist_ok=False)
    inputs = [args.xodr, args.source_manifest, POLICY,
              ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml',
              ROOT/'mapforge/validate/separated_acceptance.py', ROOT/'mapforge/validate/g11.py',
              ROOT/'mapforge/validate/dynamics_speed.py', ROOT/'mapforge/validate/smoothness.py',
              ROOT/'scripts/internal_edge_jets.py', Path(__file__)]
    inputs += [p for p in (args.casebook, args.historical_stress) if p is not None]
    binding = {str(p.resolve()): sha256(p.read_bytes()) for p in inputs}
    report, legacy, domains = audit_file(args.xodr, args.source_manifest,
        casebook=args.casebook, historical_stress=args.historical_stress)
    if any(sha256(Path(p).read_bytes()) != digest for p, digest in binding.items()):
        raise ValueError('inputs or checker code changed during run')
    atomic(dest/'binding.json', json_bytes(binding))
    atomic(dest/'report.json', json_bytes(report))
    atomic(dest/'legacy-g11.json', json_bytes(legacy))
    atomic(dest/'required-domains.json', json_bytes(dict(xodr_sha256=report['xodr_sha256'], domains=domains)))
    print(report['status'], 'static:', report['static_map']['status'],
          'operating:', report['operating_dynamics']['reason'],
          'old limit stress:', report['legacy_limit_stress']['status'], flush=True)
    return 0 if report['map_accepted'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
