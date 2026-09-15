"""Joint ordinary-road/connector experiment; no source or speed relaxation.

This tests a sufficient (not universal) cross-section compatibility condition:
parallel lane edges at the junction mouth. Inferred endpoint slopes may move
only if the existing source-position and world-dynamics constraints allow it.
The first two failed frame-only experiments remain reproducible separately.
"""
import argparse
import copy
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spikes.road_boundary_family import solve_road
from spikes.map_lane_family import ManifestCenters
from spikes.connector_cross_section import endpoint_frame
from mapforge.ops.connector_width import centred_width_records
from mapforge.ops.refline_fit import solve_g2_balanced
from mapforge.validate.g11 import audit_file
from mapforge.validate.junction_edges import audit as edge_audit
from mapforge.validate.smoothness import junction_lane_interfaces
from spikes.connector_shape import turn_branch,heading_values,heading_envelope


def connectors(root, *, curve_solver=None, frozen_curves=None):
    if frozen_curves is not None:
        staged=copy.deepcopy(root)
        reports=_connectors(staged,curve_solver=curve_solver,frozen_curves=frozen_curves)
        root[:]=list(staged)
        return reports
    return _connectors(root,curve_solver=curve_solver)


def _connectors(root, *, curve_solver=None, frozen_curves=None):
    curve_solver = curve_solver or solve_g2_balanced
    roads = {r.get('id'): r for r in root.findall('road')}
    if frozen_curves is not None:
        expected={r.get('id') for r in roads.values() if r.get('junction')!='-1' and r.get('name')!='junction_paving'}
        if set(frozen_curves)!=expected:raise ValueError('frozen connector state must cover the complete dependency set')
    reports = []
    for r in roads.values():
        if r.get('junction') == '-1' or r.get('name') == 'junction_paving':
            continue
        lane = r.find('lanes/laneSection/right/lane')
        if len(r.findall('lanes/laneSection')) != 1 or lane.get('id') != '-1':
            raise ValueError('unsupported connector cross section')
        frames = []
        widths = []
        for role in ('predecessor', 'successor'):
            link = r.find('link/'+role)
            lid = int(lane.find('link/'+role).get('id'))
            contact = link.get('contactPoint')
            f = endpoint_frame(roads[link.get('elementId')], lid, contact,
                               contact == ('end' if role == 'predecessor' else 'start'))
            for e in f['edges'].values():
                if abs((e['heading']-f['pose'][2]+math.pi)%(2*math.pi)-math.pi) > 1e-7:
                    raise ValueError('ordinary endpoint was not jointly solved to parallel edges')
            frames.append(f)
            widths.append(math.hypot(f['edges']['left']['x']-f['edges']['right']['x'],
                                     f['edges']['left']['y']-f['edges']['right']['y']))
        a, b = frames
        if frozen_curves is None:
            cls, tuning = curve_solver(a['pose'], b['pose'], a['k'], b['k'])
        else:
            cls=frozen_curves[r.get('id')]
            if len(cls)!=3:raise ValueError('frozen connector exceeds the three-primitive model')
            for cl in cls:
                if not all(math.isfinite(v) for v in (cl.length,cl.XStart,cl.YStart,cl.ThetaStart,cl.KappaStart,cl.KappaEnd)) or cl.length<=0:
                    raise ValueError('frozen connector has invalid geometry parameters')
            for prev,nxt in zip(cls,cls[1:]):
                if (math.hypot(prev.XEnd-nxt.XStart,prev.YEnd-nxt.YStart)>1e-5
                    or abs((prev.ThetaEnd-nxt.ThetaStart+math.pi)%(2*math.pi)-math.pi)>1e-7
                    or abs(prev.KappaEnd-nxt.KappaStart)>1e-7):
                    raise ValueError('frozen connector has a broken internal G2 seam')
            for cl,frame,end in ((cls[0],a,'Start'),(cls[-1],b,'End')):
                if (math.hypot(getattr(cl,'X'+end)-frame['pose'][0],getattr(cl,'Y'+end)-frame['pose'][1])>1e-5
                    or abs((getattr(cl,'Theta'+end)-frame['pose'][2]+math.pi)%(2*math.pi)-math.pi)>1e-7
                    or abs(getattr(cl,'Kappa'+end)-frame['k'])>1e-7):
                    raise ValueError('frozen connector no longer matches its shared parent state')
            turn=turn_branch(b['pose'][2]-a['pose'][2],sum(.5*(c.KappaStart+c.KappaEnd)*c.length for c in cls))
            k=[cls[0].KappaStart]+[c.KappaEnd for c in cls]
            heading_ratio=float(max(abs(heading_values([c.length for c in cls],k,turn))))
            monotone=math.pi/6<abs(turn)<5*math.pi/6 and max(abs(a['k']),abs(b['k']))<1e-9
            shape=(min(c.length for c in cls)>=5.-1e-7 and min(c.length for c in cls)>=.1*sum(c.length for c in cls)-1e-7
                   and heading_ratio<=1.+1e-7
                   and (not monotone or min(np.sign(turn)*np.array(k))>=-1e-8))
            tuning={'method':'frozen-simultaneous-corridor-connector-state',
                    'shape_constraints_satisfied':bool(shape),'monotone_turn_required':monotone,
                    'heading_policy':heading_envelope(turn)[2],'heading_envelope_ratio':heading_ratio,
                    'source_speed_changed':False,'primitive_count':3,
                    'dynamic_ratio':max(max(abs(v) for v in k)*(15/3.6)**2/2.5,
                                        max(abs(c.dk) for c in cls)*(15/3.6)**3)}
        length = sum(c.length for c in cls)
        # Width varies only within the longest primitive; its jets vanish at
        # both ends, so no width slope crosses a reference sharpness jump.
        span = int(np.argmax([c.length for c in cls]))
        start = sum(c.length for c in cls[:span])
        end = start+cls[span].length
        records = [(0., widths[0], 0., 0., 0.)] if start > 1e-9 else []
        records += [(s+start, *co) for s, *co in centred_width_records(*widths, cls[span].length)]
        if end < length-1e-9:
            records.append((end, widths[1], 0., 0., 0.))
        r.find('planView').clear()
        cursor = 0.
        for c in cls:
            g = ET.SubElement(r.find('planView'), 'geometry', s=str(cursor), x=str(c.XStart),
                              y=str(c.YStart), hdg=str(c.ThetaStart), length=str(c.length))
            ET.SubElement(g, 'spiral', curvStart=str(c.KappaStart), curvEnd=str(c.KappaEnd))
            cursor += c.length
        r.set('length', str(length))
        for element in lane.findall('width')+lane.findall('border'):
            lane.remove(element)
        group = r.find('lanes')
        for element in group.findall('laneOffset'):
            group.remove(element)
        for i, (s, *co) in enumerate(records):
            lane.insert(i+1, ET.Element('width', sOffset=str(s), **dict(zip('abcd', map(str, co)))))
            group.insert(i, ET.Element('laneOffset', s=str(s), **dict(zip('abcd', (str(x/2) for x in co)))))
        ud = lane.find("userData[@code='mapforge.provenance/v1']")
        if ud is not None:
            prov = json.loads(ud.get('value'))
            prov.update(candidate_only=True, curve_fit=tuning,
                        cross_section_fit='joint-parallel-mouth-fixed-C2-width-in-one-primitive')
            ud.set('value', json.dumps(prov, ensure_ascii=False, separators=(',', ':')))
        reports.append({'road': r.get('id'), 'primitive_lengths': [c.length for c in cls],
                        'curve_fit': tuning,
                        'width_records': len(records), 'width_minimum_span_m': min(np.diff(
                            [v[0] for v in records]+[length])), 'endpoint_widths': widths})
    return reports


def run(source, target):
    if source.resolve() == target.resolve():
        raise ValueError('joint candidate must not overwrite its input')
    manifest = json.loads(source.with_suffix('.source-lanes.json').read_text(encoding='utf-8'))
    tree = etree.parse(str(source))
    rows = []
    for road in tree.findall('road'):
        if road.get('junction') != '-1':
            continue
        left = road.findall('lanes/laneSection/left/lane')
        provenance = [l.find("userData[@code='mapforge.provenance/v1']") for l in left]
        mirror = (bool(left) and all(l.find("userData[@code='mapforge.source_lane']") is None for l in left)
                  and all(p is not None and json.loads(p.get('value')).get('support_kind')
                          in ('mirror', 'lane-transition-ribbon') for p in provenance))
        attempts = []
        for phase in (None, 0., 3., 6., 9., 12.):
            result = solve_road(road, ManifestCenters(manifest), np.asarray,
                                source_mode='centers', mirror=mirror, source_tol=.35,
                                junction_endpoint_mode='parallel', knot_phase=phase)
            attempts.append(dict(result, tried_knot_phase=phase))
            if result['status'] == 'CANDIDATE':
                break
        result['attempts'] = attempts
        result['road'] = road.get('id'); rows.append(result)
        print('ORDINARY', road.get('id'), result['status'], result.get('reason'), flush=True)
    target.parent.mkdir(parents=True, exist_ok=True)
    if any(r['status'] != 'CANDIDATE' for r in rows):
        target.with_suffix('.joint.json').write_text(json.dumps({'status':'REJECTED','roads':rows},indent=2),encoding='utf-8')
        return False
    root = ET.fromstring(etree.tostring(tree))
    connection_report = connectors(root)
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    target.with_suffix('.source-lanes.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    g11 = audit_file(target, ROOT/'profiles/validation/g11-opendrive-v1.draft.yaml')
    edges = edge_audit(root)
    report = {'candidate_only':True, 'delivery':'BLOCKED', 'ordinary':rows, 'connectors':connection_report,
              'edges':edges, 'centers':junction_lane_interfaces(root), 'g11':g11}
    target.with_suffix('.joint.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'g11':g11['status'], 'edges':edges['status'], 'edge_maxima':edges['maxima'],
                      'groups':{k:v['status'] for k,v in g11['groups'].items()}}),flush=True)
    return True


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('source',type=Path); p.add_argument('target',type=Path)
    a=p.parse_args(); raise SystemExit(0 if run(a.source,a.target) else 2)
