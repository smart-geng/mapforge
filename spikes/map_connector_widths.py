"""Reapply only the new linked-endpoint width rule to an isolated MAP candidate."""
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.validate.smoothness import _sections,lane_edges_kinematics_at
from mapforge.ops.connector_width import centred_width_records
from mapforge.adapters.opendrive.writer import _f


def width_at(road,lid,contact):
    s=0. if contact=='start' else float(road.get('length'))
    sec=_sections(road)[0 if contact=='start' else -1]
    ids=[x[0] for x in sec[2 if lid>0 else 1]]
    i=ids.index(lid)
    edges=lane_edges_kinematics_at(road,s,'left' if lid>0 else 'right')
    return (1 if lid>0 else -1)*(edges[i+1][0]-edges[i][0])


def patch(root):
    roads={r.get('id'):r for r in root.findall('road')}
    rows=[]
    for r in roads.values():
        if r.get('junction')=='-1' or r.get('name')=='junction_paving':continue
        lanes=r.findall('lanes/laneSection/right/lane')
        if len(lanes)!=1 or lanes[0].get('id')!='-1':raise ValueError('single centered connector only')
        lane=lanes[0];ww=[]
        old=width_at(r,-1,'start')
        for role in ('predecessor','successor'):
            link=r.find('link/'+role);lid=int(lane.find('link/'+role).get('id'))
            ww.append(width_at(roads[link.get('elementId')],lid,link.get('contactPoint')))
        records=centred_width_records(*ww,float(r.get('length')))
        for elem in list(lane.findall('width')):lane.remove(elem)
        at=1 if lane.find('link') is not None else 0
        for i,(s,*co) in enumerate(records):
            lane.insert(at+i,ET.Element('width',{'sOffset':_f(s),**dict(zip('abcd',map(_f,co)))}))
        group=r.find('lanes')
        for elem in list(group.findall('laneOffset')):group.remove(elem)
        for i,(s,*co) in enumerate(records):
            group.insert(i,ET.Element('laneOffset',{'s':_f(s),**dict(zip('abcd',(_f(x/2) for x in co)))}))
        ud=lane.find("userData[@code='mapforge.provenance/v1']")
        if ud is not None:
            prov=json.loads(ud.get('value'))
            prov.update(width_basis='final-linked-lane-endpoints',width_endpoints_m=ww,
                        width_transition='fixed-three-span-C2-zero-end-jets',
                        candidate_only=True,edge_jet_continuity='not-guaranteed-by-width-size-matching')
            ud.set('value',json.dumps(prov,ensure_ascii=False,separators=(',',':')))
        rows.append({'road':r.get('id'),'old_width':old,'endpoint_widths':ww,
                     'width_records':len(records),'min_width_span_m':float(r.get('length'))/len(records)})
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path);a=p.parse_args()
    root=ET.parse(a.source).getroot();rows=patch(root)
    a.target.parent.mkdir(parents=True,exist_ok=True);ET.indent(root)
    ET.ElementTree(root).write(a.target,encoding='utf-8',xml_declaration=True)
    a.target.with_suffix('.width-candidate.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    print(json.dumps(rows))
