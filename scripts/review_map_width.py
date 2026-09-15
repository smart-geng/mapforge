"""Independent raw laneWidth audit for research MAP artifacts; no XODR edits."""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import CASES
from scripts.recheck_speed_contract import dump,sha
from mapforge.validate.map_width import audit


def run(paths,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    rows=[];raw=[ROOT/'v2x_map_xml'/name for _,name in CASES]
    for path in map(Path,paths):
        manifest=json.loads(path.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
        result=audit(ET.parse(path).getroot(),manifest,raw)
        result.update(xodr=str(path.resolve()),xodr_sha256=sha(path),
                      manifest_sha256=sha(path.with_suffix('.source-lanes.json')),
                      acceptance='BLOCKED' if result['status']!='PASS' else 'other gates still required')
        label=path.parent.name+'-'+path.stem;dump(output/(label+'.json'),result)
        row=dict(case=label,status=result['status'],max_error_m=result.get('max_error_m'),
                 failures=len(result['failures']),intervals=len(result['lanes']))
        rows.append(row);print(row,flush=True)
    dump(output/'summary.json',rows)
    return 2 if any(r['status']!='PASS' for r in rows) else 0


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('maps',nargs='+',type=Path);p.add_argument('--out',required=True,type=Path)
    a=p.parse_args();raise SystemExit(run(a.maps,a.out))
