"""Read back selected roundtrip witnesses using independent esmini RoadManager."""
import argparse
import ctypes
import json
import math
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.esmini_rm_check import DLL, RMPos, _bind
from scripts.check_editor_roundtrip import sha


def run(comparison, output):
    data=json.loads(comparison.read_text(encoding='utf-8'))
    probes=[]
    for row in data['rows']:
        if row['name']=='junction_paving' or row['edge_max_m'] is None:
            continue
        w=row['worst_edge_witness']
        if w['error_m']<1:
            continue
        probes.append(dict(road=int(row['road']), lane=w['lane'],
                           s=max(0.,w['s_m']-(1e-4 if w['left_limit'] else 0.)),
                           side=w['edge']))
    rm=ctypes.CDLL(str(DLL)); _bind(rm)
    result=dict(scope='selected worst-edge witnesses, not full consumer or controller acceptance',
                comparison_sha256=sha(comparison), stages=[], map_accepted=False)
    for stage in ('source','target'):
        path=Path(data[stage]); assert sha(path)==data[stage+'_sha256']
        code=rm.RM_Init(str(path).encode('utf-8'))
        entry=dict(stage=stage,sha256=sha(path),load_code=code,samples=[])
        if code==0:
            try:
                handle=rm.RM_CreatePosition()
                for probe in probes:
                    rid,lid,s=probe['road'],probe['lane'],probe['s']
                    width=ctypes.c_double()
                    wc=rm.RM_GetLaneWidthByRoadId(rid,lid,s,ctypes.byref(width))
                    offset=width.value/2*(1 if lid>0 else -1)*(1 if probe['side']==1 else -1)
                    rc=rm.RM_SetLanePosition(handle,rid,lid,offset,s,True)
                    pos=RMPos(); pc=rm.RM_GetPositionData(handle,ctypes.byref(pos))
                    entry['samples'].append(dict(**probe,width_code=wc,width_m=width.value,
                        position_code=rc,read_code=pc,xy=[pos.x,pos.y],
                        returned_road=pos.roadId,returned_lane=pos.laneId,
                        identity_preserved=pos.roadId==rid and pos.laneId==lid,
                        positive_width=width.value>0))
            finally: rm.RM_Close()
        result['stages'].append(entry)
    output.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('comparison',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    run(a.comparison,a.output)
