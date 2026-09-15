"""Independent XSD + esmini point readback of written candidate's changed roads.

Runs in a child process (RoadManager is process-global). This is deliberately
NOT a whole-map/source-fidelity/dynamics acceptance gate.
"""
import argparse
import ctypes
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from lxml import etree
from mapforge.repair_web.model import parse, digest, intervals, lanes, atomic, json_bytes
from scripts.internal_edge_jets import states
from scripts.esmini_rm_check import DLL, RMPos, _bind


def validate(path, report_path, output):
    report = json.loads(report_path.read_text(encoding='utf8'))
    data = path.read_bytes()
    if digest(data) != report['xodr_sha256']:
        raise ValueError('Export file hash mismatch')
    root = parse(data)
    result = {'scope':'changed roads driving lane edge/center positions only; not routes or dynamics',
              'xodr_sha256':digest(data),'xsd':'NOT_RUN','esmini':'NOT_RUN','map_accepted':False}
    if root.find('header').get('revMinor')=='5':
        schema=etree.XMLSchema(etree.parse(str(ROOT/'OpenDRIVE_1.5M.xsd')))
        result['xsd']='PASS' if schema.validate(etree.fromstring(data)) else 'FAIL'
        result['xsd_errors']=[str(e) for e in schema.error_log]
    if not DLL.exists():
        result['esmini']='UNAVAILABLE'
        atomic(output,json_bytes(result)); return result
    rm=ctypes.CDLL(str(DLL));_bind(rm)
    result['dll_sha256']=digest(DLL.read_bytes())
    result['load_code']=rm.RM_Init(str(path.resolve()).encode('utf8'))
    if result['load_code']!=0:
        result['esmini']='LOAD_FAILED';atomic(output,json_bytes(result));return result
    rows=[];skipped=[];failures=[]
    try:
        handle=rm.RM_CreatePosition()
        for road in root.findall('road'):
            if road.get('id') not in report['guard']['changed_roads']:continue
            rid=int(road.get('id'))
            for sec,lo,hi in intervals(road):
                ss=set(float(v) for v in np.linspace(lo+1e-5,hi-1e-5,max(2,math.ceil(hi-lo)+1)))
                for lane in lanes(sec).values():
                    for w in lane.findall('width'):
                        cut=lo+float(w.get('sOffset'))
                        ss.update(s for s in (cut-1e-5,cut+1e-5) if lo<s<hi)
                for lid,lane in lanes(sec).items():
                    if lane.get('type')!='driving':continue
                    for s in sorted(ss):
                        pair=states(road,lid,s,False)
                        width=ctypes.c_double()
                        wc=rm.RM_GetLaneWidthByRoadId(rid,lid,s,ctypes.byref(width))
                        if width.value < 1e-5:
                            skipped.append({'road':rid,'lane':lid,'s':s,'reason':'zero/nonpositive width'});continue
                        for edge,expected in [('inner',pair[0][:2]),('center',np.mean(np.array(pair)[:,:2],axis=0)),('outer',pair[1][:2])]:
                            direction={'inner':-1,'center':0,'outer':1}[edge]
                            offset=width.value/2*(1 if lid>0 else -1)*direction
                            rc=rm.RM_SetLanePosition(handle,rid,lid,offset,s,True)
                            pos=RMPos();pc=rm.RM_GetPositionData(handle,ctypes.byref(pos))
                            error=math.hypot(pos.x-expected[0],pos.y-expected[1])
                            row={'road':rid,'lane':lid,'s':s,'edge':edge,'error_m':error,
                                 'width_m':width.value,'xy':[pos.x,pos.y],'xml_xy':list(expected),
                                 'identity_ok':pos.roadId==rid and pos.laneId==lid,
                                 'codes':[wc,rc,pc]}
                            rows.append(row)
                            if not row['identity_ok'] or any(v<0 for v in row['codes']) or error>1e-5:
                                failures.append(row)
    finally:rm.RM_Close()
    result.update(esmini='FAIL' if failures else ('PASS' if rows else 'NO_CHANGED_ROADS'),
                  checked_positions=len(rows),skipped=skipped,failures=failures,
                  max_position_error_m=max((r['error_m'] for r in rows),default=None),
                  witnesses=sorted(rows,key=lambda r:r['error_m'],reverse=True)[:10])
    atomic(output,json_bytes(result))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('xodr',type=Path);p.add_argument('report',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args(); print(json.dumps(validate(a.xodr,a.report,a.output),ensure_ascii=True))
