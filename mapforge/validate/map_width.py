"""Explicit MAP laneWidth preservation, separate from centerline fidelity.

Only supplied widths are facts; missing widths are not silently turned into
3.5m facts. Constant width over original support is a conservative conversion
contract, NOT a claim that the standard guarantees surveyed constant width.
An explicit, reviewed variable-width interpretation is not implemented yet.
The current geometric audit supports single-Line research charts.
"""
import math
from pathlib import Path
import numpy as np
from mapforge.adapters.v2xmap.xml_reader import parse_map_xml_all
from mapforge.validate.map_source import audit_map_source_manifest


def raw_widths(paths):
    widths={}
    for path in paths:
        for node in parse_map_xml_all(str(path)):
            for link in node.links:
                reg,nid=link.upstream or (None,None)
                for lane in link.lanes:
                    key=f'map:{node.region}:{node.node_id}:from:{reg}:{nid}:{link.name}:lane:{lane.lane_id}'
                    value=None if lane.width_cm is None else lane.width_cm/100.
                    if key in widths and widths[key]!=value:raise ValueError(f'ambiguous raw lane width: {key}')
                    if value is not None and (not math.isfinite(value) or value<0):raise ValueError('invalid raw lane width')
                    widths[key]=value
    return widths


def width_extrema(lane,section_start,a,b):
    """Exact cubic extrema on an absolute station interval, not sparse samples."""
    if not all(math.isfinite(v) for v in (section_start,a,b)) or b<a:
        raise ValueError('invalid width interval')
    records=sorted(((section_start+float(w.get('sOffset')),np.array([float(w.get(k)) for k in 'abcd']))
                    for w in lane.findall('width')), key=lambda row:row[0])
    if any(b[0]<=a[0] for a,b in zip(records,records[1:])):
        raise ValueError('duplicate width station')
    if not records or records[0][0]>a+1e-7:raise ValueError('missing width coverage')
    results=[]
    for i,(s,c) in enumerate(records):
        if not np.isfinite(c).all() or not math.isfinite(s):raise ValueError('nonfinite width polynomial')
        lo=max(s,a);hi=min(records[i+1][0] if i+1<len(records) else b,b)
        if hi<lo:continue
        roots=np.polynomial.polynomial.polyroots([c[1],2*c[2],3*c[3]])
        ds=[lo-s,hi-s]+[float(v.real) for v in roots if abs(v.imag)<1e-9 and lo-s<v.real<hi-s]
        results.extend((float(s+d),float(np.polynomial.polynomial.polyval(d,c))) for d in ds)
    if not results:raise ValueError('width interval has no polynomial')
    return results


def audit(root,manifest,paths,tolerance=1e-6):
    report={'gate_id':'MAP-explicit-width','status':'UNAVAILABLE','tolerance_m':tolerance,
            'policy_id':'map-explicit-constant-width-v1',
            'interpretation':'conservative scalar preservation, not a standard-imposed surveyed-width guarantee',
            'scope':'explicit laneWidth over same-identity original center support; not surveyed boundary truth',
            'lanes':[],'failures':[],'not_supplied':[]}
    admitted=audit_map_source_manifest(manifest,paths)
    report['inputs']=admitted['inputs']
    if admitted['status']!='PASS':
        report['failures'].append({'reason':'raw source admission failed'});return report
    try:
        widths=raw_widths(paths);source={l['source_lane_id']:np.array(l['geometry']['coordinates'],float)
                                       for l in manifest['lanes'] if l.get('geometry')}
        seen=set()
        for road in root.findall('road'):
            if road.get('junction')!='-1':continue
            gs=road.findall('planView/geometry')
            if len(gs)!=1 or gs[0].find('line') is None:raise ValueError('width source support requires a single Line chart')
            g=gs[0];h=float(g.get('hdg'));origin=np.array([float(g.get('x')),float(g.get('y'))]);e=np.array([math.cos(h),math.sin(h)])
            sections=road.findall('lanes/laneSection')
            ends=[float(s.get('s')) for s in sections[1:]]+[float(road.get('length'))]
            for section,end in zip(sections,ends):
                start=float(section.get('s'))
                for side in ('left','right'):
                    for lane in section.findall(side+'/lane'):
                        ud=lane.find("userData[@code='mapforge.source_lane']")
                        if ud is None:continue
                        sid=ud.get('value')
                        if sid not in widths or sid not in source:raise ValueError(f'unresolved raw width owner: {sid}')
                        expected=widths[sid]
                        if expected is None:
                            report['not_supplied'].append(sid);continue
                        stations=(source[sid]-origin)@e;a=max(start,float(stations.min()));b=min(end,float(stations.max()))
                        if b<a:continue
                        seen.add(sid);values=width_extrema(lane,start,a,b)
                        s,value=max(values,key=lambda p:abs(p[1]-expected));error=abs(value-expected)
                        row=dict(source_lane_id=sid,road=road.get('id'),lane=lane.get('id'),section=start,
                                 interval_m=[a,b],expected_m=expected,worst_s_m=s,written_m=value,max_error_m=error)
                        report['lanes'].append(row)
                        if error>tolerance:report['failures'].append(row)
        expected={sid for sid in source if widths.get(sid) is not None}
        for sid in expected-seen:report['failures'].append({'source_lane_id':sid,'reason':'explicit width support not checked'})
        report['status']='FAIL' if report['failures'] else ('PASS' if seen else 'UNAVAILABLE')
        report['max_error_m']=max((r['max_error_m'] for r in report['lanes']),default=None)
    except (ValueError,KeyError,TypeError,IndexError) as exc:
        report['failures'].append({'reason':str(exc)})
    return report
