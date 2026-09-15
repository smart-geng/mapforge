"""Read-only audit of cached MAP source inventories against the seven raw XMLs."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import CASES
from mapforge.validate.map_source import audit_map_source_manifest


def main():
    raw = [ROOT/'v2x_map_xml'/name for _, name in CASES]
    rows = []
    for label, _ in CASES:
        path = ROOT/'out/m2x'/(label+'.source-lanes.json')
        manifest = json.loads(path.read_text(encoding='utf-8'))
        report = audit_map_source_manifest(manifest, raw)
        rows.append({'case': label, 'manifest': str(path), **report})
        print(label, report['status'], len(report['failure_reasons']))
    target = ROOT/'out/map-source-inventory-audit-v139.json'
    target.write_text(json.dumps({'scope': 'cached inventory fidelity, not full conversion acceptance',
                                  'rows': rows}, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
