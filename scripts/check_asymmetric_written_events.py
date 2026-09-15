"""Exact XML jets at original zero tips, source speed events, and hash sealing."""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from scripts.internal_edge_jets import states
from mapforge.ops.source_speed_events import speed_records


def run(folder):
    folder=Path(folder).resolve();trial=json.loads((folder/'report.json').read_text(encoding='utf8'))
    unchanged(trial['source_hashes'])
    path=Path(trial['artifact'])
    if _sha256(path)!=trial['sha256']:raise ValueError('review artifact changed')
    component=Path(trial['component'])
    compiled=json.loads((component/'report.json').read_text(encoding='utf8'))['compiled']
    state=json.loads((component/'shared-state.json').read_text(encoding='utf8'))
    contacts=json.loads((Path(state['source_directory'])/'contact-model.json').read_text(encoding='utf8'))
    road=ET.parse(path).getroot().find(f"road[@id='{state['road']}']")
    geometry=road.findall('planView/geometry')
    if len(geometry)!=1 or geometry[0].find('line') is None:raise ValueError('this direct event checker requires one Line')
    g=geometry[0];h=float(g.get('hdg'));e=np.array([math.cos(h),math.sin(h)]);p=np.array([float(g.get('x')),float(g.get('y'))])
    sections=road.findall('lanes/laneSection');cuts=np.array([float(sec.get('s')) for sec in sections]+[float(road.get('length'))])
    chains={c['key']:c for c in compiled['chains']};rows=[]
    for event in contacts['source_endpoint_inventory']:
        if not event['width_known'] or event['width_mm']!=0:continue
        cs=[c for c in chains.values() if event['source_lane_id'] in c['source_ids']]
        if not cs:continue
        if len(cs)!=1:raise ValueError('ambiguous original chain')
        tip=contacts['boundary_endpoints'][event['boundary_endpoints']['left']]
        projected=float((np.asarray(tip['xy'])-p)@e)
        nearest=float(cuts[np.argmin(abs(cuts-projected))]);s=nearest if abs(nearest-projected)<=1e-7 else projected
        matches=[r for r in compiled['lane_ledger'] if r['source_chain']==cs[0]['key'] and
            r['chart_s'][0]-compiled['chart_start_m']-1e-7<=s<=r['chart_s'][1]-compiled['chart_start_m']+1e-7]
        if not matches:raise ValueError('original zero tip absent in written source-owned road')
        for row in matches:
            hi=row['chart_s'][1]-compiled['chart_start_m']
            a,b=states(road,row['lane'],s,abs(s-hi)<=1e-7)
            gap=float(np.linalg.norm(np.subtract(a[:2],b[:2])))
            angle=abs(math.remainder(a[2]-b[2],2*math.pi));dk=abs(a[3]-b[3])
            rows.append(dict(source_lane_id=event['source_lane_id'],source_contact=event['contact'],
                section=row['section'],lane=row['lane'],source_projected_s=projected,written_query_s=s,
                floating_station_snap_m=s-projected,position_gap_m=gap,heading_gap_rad=angle,curvature_gap_per_m=dk,
                status='PASS' if gap<1e-7 and angle<1e-8 and dk<1e-8 else 'FAIL'))
    speeds=[]
    for row in compiled['lane_ledger']:
        side='left' if row['lane']>0 else 'right'
        lane=sections[row['section']].find(f"{side}/lane[@id='{row['lane']}']")
        expected=speed_records(chains[row['source_chain']],*row['chart_s'])
        actual=[(float(v.get('sOffset')),float(v.get('max'))*3.6) for v in lane.findall('speed')]
        passed=len(actual)==len(expected) and bool(np.allclose(actual,expected,rtol=0,atol=1e-8))
        speeds.append(dict(section=row['section'],lane=row['lane'],expected_source_kmh=expected,written_kmh=actual,
                           status='PASS' if passed else 'FAIL'))
    report=dict(status='WRITTEN_EVENTS_PASS_NOT_MAP' if rows and speeds and all(r['status']=='PASS' for r in rows+speeds) else 'FAIL',
        artifact=str(path),sha256=_sha256(path),zero_width_joins=rows,speed_intervals=speeds,
        verified_input_hash_count=len(trial['source_hashes']),all_source_hashes_unchanged=True,
        source_changed=False,production_accepted=False,readback_method='analytic XML edges; not stored solver coefficients',
        evidence_hashes={str(p):_sha256(p) for p in [Path(__file__),folder/'report.json',
            folder/'independent-review/report.json',component/'report.json',component/'shared-state.json']})
    target=folder/'independent-review/written-events.json'
    if target.exists():raise FileExistsError('preserve earlier event evidence')
    dump(target,report);print(report['status'],len(rows),'zero queries',len(speeds),'lane-speed intervals')
    if report['status']=='FAIL':raise ValueError('written source event failed')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');run(p.parse_args().folder)
