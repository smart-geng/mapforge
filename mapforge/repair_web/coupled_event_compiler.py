"""Seven-road in-memory compiler and independent XML consumption audit.

No filesystem writes, optimizer, Web mutation or export API. Mechanical
roundtrip is deliberately independent of shape acceptance. Invalid diagnostic
coordinates can be inspected but can never become an applicable candidate.
"""
import copy
import math
from xml.etree import ElementTree as ET

import numpy as np

from mapforge.repair_web.model import parse, digest, intervals, lanes, coefficients
from mapforge.repair_web.coupled_event_state import CoupledEventState, world_state
from mapforge.repair_web.coupled_event_sources import polynomial_absmax, polynomial_shift
from mapforge.repair_web.split_event_admission import START, END, CONNECTORS
from scripts.check_outer_event_written import semantic, boundary_poly
from scripts.internal_edge_jets import states, audit as audit_joints
from mapforge.validate.g11 import _primitive


def replace_records(parent,tag,records):
    old=parent.findall(tag)
    if not old: raise ValueError('Missing existing record class '+tag)
    index=list(parent).index(old[0])
    for e in old: parent.remove(e)
    for i,e in enumerate(records): parent.insert(index+i,e)


def record(tag,station_name,station,co):
    if not math.isfinite(station) or not np.isfinite(co).all(): raise ValueError('Nonfinite compiled record')
    return ET.Element(tag,{station_name:format(float(station),'.17g'),
        **{k:format(float(v),'.17g') for k,v in zip('abcd',co)}})


def protected_semantics(data):
    """Remove exactly mutable fields; all other document content must match."""
    root=parse(data)
    for road in root.findall('road'):
        rid=road.get('id')
        if rid not in ('11',*CONNECTORS): continue
        ll=road.find('lanes')
        for off in ll.findall('laneOffset'):
            if rid!='11' or float(off.get('s'))>=START: ll.remove(off)
        for sec,a,b in intervals(road):
            if rid=='11' and b<=START: continue
            if rid=='11' and a<START: raise ValueError('Parent edit start must be semantic section boundary')
            for lane in lanes(sec).values():
                for w in lane.findall('width'): lane.remove(w)
        if rid!='11':
            road.attrib.pop('length'); road.find('planView').clear()
    return semantic(root)


class SevenRoadCompiler:
    def __init__(self,contract):
        self.contract=contract; self.model=CoupledEventState(contract)
        self.reference_sha=digest(contract.reference); self.protected=protected_semantics(contract.reference)

    def _encode(self,snapshot):
        c=self.contract
        if digest(c.reference)!=self.reference_sha: raise ValueError('Stale reference')
        root=parse(c.reference); roads={r.get('id'):r for r in root.findall('road')}; p=c.parent
        c.frames(snapshot['parent_coefficients'])
        if set(snapshot['turns'])!=set(CONNECTORS): raise ValueError('All six dependent turns required')
        road=roads['11']; ll=road.find('lanes'); x=snapshot['parent_coefficients']
        old=[copy.deepcopy(e) for e in ll.findall('laneOffset') if float(e.get('s'))<START]
        replace_records(ll,'laneOffset',old+[record('laneOffset','s',s,p.power(0,s)@x) for s in p.scope.knots[:-1]])
        layout=c.written_layout()
        for index,(sec,a,b) in enumerate(intervals(road)):
            if b<=START: continue
            for lid,lane in lanes(sec).items():
                if lid>=0: raise ValueError('Reviewed right-hand parent stack required')
                spans=[r for r in layout['records'] if r['section']==index and r['lane']==lid]
                replace_records(lane,'width',[record('width','sOffset',r['start_m']-a,
                    (p.power(-lid-1,r['start_m'])-p.power(-lid,r['start_m']))@x) for r in spans])
        for cid,turn in snapshot['turns'].items():
            road=roads[cid]; refs=turn['refs']; ss=turn['stations']; co=turn['coefficients']
            if len(refs)!=5 or len(ss)!=6 or min(np.diff(ss))<6.-1e-8: raise ValueError('Five long independent intervals required')
            if not np.allclose(np.diff(ss),[r.length for r in refs],atol=1e-9,rtol=0): raise ValueError('Reference/transverse stations differ')
            road.set('length',format(float(ss[-1]),'.17g')); pv=road.find('planView'); pv.clear()
            for s,ref in zip(ss,refs):
                g=ET.SubElement(pv,'geometry',s=format(float(s),'.17g'),x=format(ref.XStart,'.17g'),
                    y=format(ref.YStart,'.17g'),hdg=format(ref.ThetaStart,'.17g'),length=format(ref.length,'.17g'))
                if ref.dk==0 and ref.KappaStart==0: ET.SubElement(g,'line')
                elif ref.dk==0: ET.SubElement(g,'arc',curvature=format(ref.KappaStart,'.17g'))
                else: ET.SubElement(g,'spiral',curvStart=format(ref.KappaStart,'.17g'),curvEnd=format(ref.KappaEnd,'.17g'))
            ll=road.find('lanes'); replace_records(ll,'laneOffset',[record('laneOffset','s',s,v) for s,v in zip(ss,co['left'])])
            lane=ll.find('laneSection/right/lane')
            replace_records(lane,'width',[record('width','sOffset',s,v) for s,v in zip(ss,co['left']-co['right'])])
        return ET.tostring(root,encoding='utf-8',xml_declaration=True)

    def audit_bytes(self,data,snapshot):
        """Independent consumer reconstructs jets from parsed XML records."""
        if type(data) is not bytes or protected_semantics(data)!=self.protected:
            raise ValueError('Unselected geometry, source/ID/TOPO/limit/metadata changed')
        root=parse(data); roads={r.get('id'):r for r in root.findall('road')}; p=self.contract.parent
        if set(snapshot['turns'])!=set(CONNECTORS): raise ValueError('Incomplete shared snapshot')
        x=snapshot['parent_coefficients']; expected_frames=self.contract.frames(x)
        parent=roads['11']; original=self.contract.graph.roads['11']; layout=self.contract.written_layout()
        actual_layout=[]
        for i,(sec,a,b) in enumerate(intervals(parent)):
            if b<=START: continue
            for lid,lane in lanes(sec).items():
                ww=lane.findall('width')
                for j,w in enumerate(ww):
                    start=a+float(w.get('sOffset')); end=a+float(ww[j+1].get('sOffset')) if j+1<len(ww) else b
                    actual_layout.append((i,lid,start,end))
        if actual_layout!=[(r['section'],r['lane'],r['start_m'],r['end_m']) for r in layout['records']]:
            raise ValueError('Unregistered parent record layout')
        expected_offsets=[float(e.get('s')) for e in original.findall('lanes/laneOffset') if float(e.get('s'))<START]+list(p.scope.knots[:-1])
        if [float(e.get('s')) for e in parent.findall('lanes/laneOffset')]!=expected_offsets:
            raise ValueError('Unexpected parent offset layout')
        worst=0.; frozen=0.
        cuts=sorted({0.,START,END,*expected_offsets,*p.scope.knots,
            *[float(s.get('s')) for s in parent.findall('lanes/laneSection')],
            *[a for _,_,a,_ in actual_layout],*[b for _,_,_,b in actual_layout]})
        for a,b in zip(cuts,cuts[1:]):
            for edge in range(5):
                if edge in p.births and a<p.births[edge]: continue
                target=boundary_poly(original,edge,a) if b<=START else p.power(edge,a)@x
                error=polynomial_absmax(boundary_poly(parent,edge,a)-target,b-a); worst=max(worst,error)
                if b<=START: frozen=max(frozen,error)
        if worst>1e-8: raise ValueError('Compiled parent differs from shared model')
        turn_reports={}; max_world=0.; all_contact=[]
        for cid,turn in snapshot['turns'].items():
            road=roads[cid]; ss=turn['stations']; refs=turn['refs']; co=turn['coefficients']
            gg=road.findall('planView/geometry'); oo=road.findall('lanes/laneOffset'); lane=road.find('lanes/laneSection/right/lane'); ww=lane.findall('width')
            if len(gg)!=5 or len(oo)!=5 or len(ww)!=5: raise ValueError('Turn record count drift')
            if abs(float(road.get('length'))-ss[-1])>1e-9: raise ValueError('Turn length drift')
            poly_error=0.
            for i,(g,off,width,ref) in enumerate(zip(gg,oo,ww,refs)):
                if abs(float(g.get('s'))-ss[i])>1e-9 or abs(float(g.get('length'))-ref.length)>1e-9:
                    raise ValueError('Turn geometry stations drift')
                if float(off.get('s'))!=ss[i] or float(width.get('sOffset'))!=ss[i]: raise ValueError('Turn coefficient stations drift')
                primitive=_primitive(g)
                actual_parameters=[primitive[k] for k in ('x','y','hdg','length','k0','k1')]
                expected_parameters=[ref.XStart,ref.YStart,ref.ThetaStart,ref.length,ref.KappaStart,ref.KappaEnd]
                if max(abs(np.asarray(actual_parameters)-expected_parameters))>1e-10:
                    raise ValueError('Whole analytic primitive changed, not just sample values')
                for side,actual in (('left',coefficients(off)),('right',coefficients(off)-coefficients(width))):
                    poly_error=max(poly_error,polynomial_absmax(actual-co[side][i],ref.length))
                # XML reader uses its own primitive advance and stack; both
                # sides of each knot are evaluated, not just drawing points.
                for u in (0.,ref.length*.37,ref.length):
                    actual=states(road,-1,float(ss[i]+u),u==ref.length)
                    for side,observed in zip(('left','right'),actual):
                        expect=world_state(ref,co[side][i],u); diff=np.asarray(observed)-expect
                        diff[2]=math.atan2(math.sin(diff[2]),math.cos(diff[2]))
                        max_world=max(max_world,float(max(abs(diff))))
            if poly_error>1e-8 or max_world>1e-7: raise ValueError('XML world geometry/readback differs from common state')
            contacts=[]
            for endpoint,s in ((0,0.),(1,float(ss[-1]))):
                actual=states(road,-1,s,endpoint==1)
                for side,observed in zip(('left','right'),actual):
                    want=expected_frames[cid][endpoint]['edges'][side]
                    delta=np.asarray(observed)-[want[k] for k in ('x','y','heading','curvature')]
                    delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
                    row=dict(end=endpoint,side=side,position_m=float(np.linalg.norm(delta[:2])),heading_rad=float(abs(delta[2])),curvature_per_m=float(abs(delta[3])))
                    contacts.append(row); all_contact.append(row)
            turn_reports[cid]=dict(geometry_records=5,width_records=5,lane_offset_records=5,
                minimum_independent_span_m=float(min(np.diff(ss))),polynomial_readback_max_m=poly_error,contacts=contacts)
        actual_joints=audit_joints(root)
        relevant=[r for r in actual_joints['rows'] if r['road'] in CONNECTORS or (r['road']=='11' and r['s_m']>=START)]
        geometry_failed=any(r['position_m']>.01 or r['heading_deg']>.1 or r['curvature_per_m']>1e-7 for r in relevant)
        contact_failed=any(r['position_m']>.01 or r['heading_rad']>math.radians(.1) or r['curvature_per_m']>1e-7 for r in all_contact)
        errors=[r for r in actual_joints['errors'] if r['road'] in ('11',*CONNECTORS)]
        from lxml import etree
        from pathlib import Path
        schema=etree.XMLSchema(etree.parse(str(Path(__file__).resolve().parents[2]/'OpenDRIVE_1.5M.xsd')))
        schema_ok=schema.validate(etree.fromstring(data))
        return dict(status='IN_MEMORY_SEVEN_ROAD_READBACK_NOT_EXPORT',xml_sha256=digest(data),
            xsd_1_5M_valid=bool(schema_ok),xsd_errors=[str(e) for e in schema.error_log],
            parent_record_readback_max_m=worst,frozen_upstream_error_m=frozen,
            maximum_world_jet_readback_error=max_world,turns=turn_reports,internal_joints=relevant,
            geometry_joint_or_contact_failure=bool(geometry_failed or contact_failed or errors),joint_errors=errors,
            protected_document_semantics_unchanged=True,other_roads_unchanged=True,source_speed_unchanged=True,
            source_shape_surface_acceptance=False,derived_provenance_regeneration_pending=True,
            written_map_files=0,esmini_checked=False,export_allowed=False,map_accepted=False)

    def diagnose(self,vector):
        snapshot=self.model.evaluate(vector); data=self._encode(snapshot)
        return self.audit_bytes(data,snapshot)

    def export(self,*args,**kwargs):
        raise ValueError('No admitted complete shape/ownership/surface trial or atomic candidate receipt')
