"""Raw MAP connectivity evidence; never rewrites operational IDs or topology."""
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all


def inspect(path):
    rows=[]
    for node in parse_map_xml_all(str(path)):
        for link in node.links:
            for lane in link.lanes:
                for c in lane.connects:
                    rows.append({'owner':[node.region,node.node_id],'link':link.name,'lane':lane.lane_id,
                                 'upstream':list(link.upstream),'remote':[c.region,c.node],
                                 'target_lane':c.lane,'maneuver_raw':c.maneuver,
                                 'same_upstream_ref':link.upstream==(c.region,c.node)})
    return {'source':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'scope':'raw field equality; not a finding that a U-turn is necessarily wrong',
            'same_upstream_ref_count':sum(r['same_upstream_ref'] for r in rows),'rows':rows}


if __name__=='__main__':
    results=[inspect(path) for path in sorted((ROOT/'v2x_map_xml').glob('map*.xml'))]
    out=ROOT/'out/map-raw-connection-semantics-v138.json'
    out.write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([{k:v for k,v in r.items() if k!='rows'} for r in results],ensure_ascii=False))
