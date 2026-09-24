"""Read-only inventory of written/source ownership for an outer-edge repair."""
import json
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.model import intervals, lanes
from scripts.internal_edge_jets import audit


def main():
    root = ET.parse(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr').getroot()
    packet = json.loads((ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json').read_text())
    for road in root.findall('road'):
        if road.get('id') != '11':
            continue
        for sec, lo, hi in intervals(road):
            row = []
            for lid, lane in lanes(sec).items():
                ids = [e.get('value') for e in lane.findall('userData') if e.get('code') == 'mapforge.source_lane']
                row.append((lid, ids, lane.find('width').attrib))
            print(lo, hi, row)
        print('ORIGINAL LANES')
        ids = sorted({o['source_lane_id'] for o in packet['occurrences'] if o['road'] == '11'})
        for sid in ids:
            obs = packet['observations'][sid]
            print(sid, [(r['declared_side'], r['boundary_key']) for r in obs['boundary_relations']],
                  'widths', obs['start_width_mm'], obs['end_width_mm'])
    print('FAILURES', audit(root)['failures'])


if __name__ == '__main__':
    main()
