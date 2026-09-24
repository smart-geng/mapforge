"""Source/shape checks on one seven-road snapshot, never per-road fitting.

Parent cubics use interval extrema. Turn distances are explicitly numerical
mesh diagnostics with a chord bound, not a continuous pass certificate. Source
roles, failed reference evidence and unimplemented delivery gates stay visible.
"""
import math

import numpy as np
from pyclothoids import Clothoid
from shapely.geometry import Polygon, LineString

from mapforge.ops.world_curve_fairness import source_turn_evidence, interval_fairness
from mapforge.repair_web.coupled_event_sources import (
    EventSources, polynomial_shift, polynomial_absmax, sample_ribbon, ribbon_intervals,
)
from mapforge.repair_web.coupled_event_state import CoupledEventState
from mapforge.repair_web.outer_event import WEST_SPLIT, cubic_range
from mapforge.repair_web.outer_event_shape import wrong_way_distance
from mapforge.repair_web.split_event_admission import START, END, CONNECTORS
from mapforge.repair_web.unique_edit import METRICS, METRIC_EPS
from mapforge.validate.line_boundary_shape import span_shape, written_boundary_shape
from mapforge.validate.smoothness import _geoms
from scripts.check_outer_event_written import boundary_poly
from spikes.measured_connector_caps import distances, crop_at_projection


def densify(points, step=.1):
    p=np.asarray(points,float); out=[]
    for a,b in zip(p,p[1:]):
        n=max(1,math.ceil(np.linalg.norm(b-a)/step))
        out.extend(a+(b-a)*np.linspace(0,1,n,endpoint=False)[:,None])
    return np.vstack([out,p[-1:]])


def stats(errors):
    e=np.asarray(errors,float)
    if not len(e) or not np.isfinite(e).all(): raise ValueError('No finite source evidence')
    return dict(max_m=float(max(e)), p95_m=float(np.percentile(e,95)), median_m=float(np.median(e)))


def reference_turn(road):
    """Actual XML, not a least-squares projection into the new model."""
    refs=[Clothoid.StandardParams(x,y,h,k0,(k1-k0)/L,L) for _,x,y,h,L,k0,k1 in _geoms(road)]
    if len(refs)!=len(road.findall('planView/geometry')): raise ValueError('Unsupported actual reference primitive')
    sections=road.findall('lanes/laneSection')
    if len(sections)!=1 or float(sections[0].get('s'))!=0: raise ValueError('Reviewed one-section turn required')
    lanes=sections[0].findall('right/lane')
    if len(lanes)!=1 or lanes[0].get('id')!='-1' or sections[0].findall('left/lane'):
        raise ValueError('Reviewed one-lane turn required')
    offset=road.findall('lanes/laneOffset'); width=lanes[0].findall('width')
    def power(items,attr,s):
        e=max((r for r in items if float(r.get(attr))<=s+1e-10),key=lambda r:float(r.get(attr)))
        return polynomial_shift([float(e.get(k)) for k in 'abcd'],s-float(e.get(attr)))
    L=float(road.get('length'))
    cuts=sorted({0.,L,*[float(e.get('s')) for e in offset],*[float(e.get('sOffset')) for e in width]})
    left=np.array([power(offset,'s',s) for s in cuts[:-1]])
    widths=np.array([power(width,'sOffset',s) for s in cuts[:-1]])
    return dict(refs=refs,stations=np.array(cuts),coefficients=dict(left=left,right=left-widths))


def mesh_surface(mesh, source_rows):
    """Physical ribbon diagnostics. No buffers/hulls/paving used to hide holes."""
    left,right=mesh['points']['left'],mesh['points']['right']
    shape=Polygon(np.vstack([left,right[::-1]]))
    result=dict(scope='single_turn_ribbon_not_whole_junction_surface', valid=bool(shape.is_valid),
        left_simple=bool(LineString(left).is_simple), right_simple=bool(LineString(right).is_simple),
        sampled_area_m2=float(shape.area), global_surface_accepted=False)
    raw={}
    for side in ('left','right'):
        fragments=[p for r in source_rows if r['field']==side for p in r['owned_fragments']]
        raw[side]=np.vstack(fragments)
    original=Polygon(np.vstack([raw['left'],raw['right'][::-1]]))
    result['original_owned_band_valid']=bool(original.is_valid)
    if shape.is_valid and original.is_valid:
        result.update(source_band_missing_area_m2=float(original.difference(shape).area),
                      source_band_extra_area_m2=float(shape.difference(original).area))
    return result


class CoupledEventGuards:
    def __init__(self, contract):
        self.contract=contract; self.model=CoupledEventState(contract); self.sources=EventSources(contract)
        self.parent=contract.parent; self.road=self.parent.road
        self.old_cuts=sorted({0.,END,*[float(e.get('s')) for e in self.road.findall('lanes/laneOffset')],
            *[float(s.get('s')) for s in self.road.findall('lanes/laneSection')],
            *[float(s.get('s'))+float(w.get('sOffset')) for s in self.road.findall('lanes/laneSection') for w in s.findall('.//width')]})
        self.caps=written_boundary_shape(contract.reference, WEST_SPLIT.__dict__)['by_edge']
        self.old_reversal=self._reversal(lambda edge,s:boundary_poly(self.road,edge,s), WEST_SPLIT.knots[0],WEST_SPLIT.knots[-1])
        target=[t for t in self.parent.traces if t['key']=='IBD_LANE_BOUNDARY:2023041110502726490' and t['part']==0 and t['record']==0]
        if len(target)!=1: raise ValueError('Original s178 regression source identity changed')
        self.target_t=float(np.interp(178.,target[0]['st'][:,0],target[0]['st'][:,1]))

    def _cells(self, start, end):
        for trace in self.parent.traces:
            floor=max(0.,start,self.parent.births.get(trace['edge'],0.))
            for p,q in zip(trace['st'],trace['st'][1:]):
                if q[0]<=p[0]: raise ValueError('Nonmonotone parent source needs explicit chart')
                lo=max(floor,p[0]); hi=min(end,q[0])
                if hi<=lo: continue
                cuts=sorted({float(lo),float(hi)}|{s for s in [*self.old_cuts,*self.parent.scope.knots] if lo<s<hi})
                slope=(q[1]-p[1])/(q[0]-p[0])
                for a,b in zip(cuts,cuts[1:]):
                    yield trace,a,b,np.array([p[1]+(a-p[0])*slope,slope,0.,0.])

    def _reversal(self, power, start, end):
        values=np.zeros(5)
        for trace,a,b,source in self._cells(start,end):
            values[trace['edge']]+=wrong_way_distance(power(trace['edge'],a),b-a,float(np.sign(source[1])))
        return values

    def parent_checks(self, coefficients=None):
        p=self.parent
        def power(edge,s):
            return boundary_poly(self.road,edge,s) if coefficients is None or s<START else p.power(edge,s)@coefficients
        rows=[]
        for trace,a,b,source in self._cells(0.,END):
            error=polynomial_absmax(power(trace['edge'],a)-source,b-a)
            rows.append(dict(feature=trace['key'],record=trace['record'],part=trace['part'],edge=trace['edge'],
                interval_m=[a,b],maximum_lateral_m=error,local_research_limit_m=.75,
                within_local_research_limit=error<=.75+1e-7,ordinary_final_limit_m=.35,
                within_ordinary_final_limit=error<=.35+1e-7))
        widths=[]
        for edge in range(1,5):
            lo=p.births.get(edge,START)
            cuts=sorted({lo,END}|{s for s in [*p.scope.knots,*self.old_cuts] if lo<s<END})
            for a,b in zip(cuts,cuts[1:]):
                c=power(edge-1,a)-power(edge,a)
                widths.append(dict(lane=-edge,interval_m=[a,b],minimum_m=cubic_range(c,b-a)[0]))
        shape={}; full={}
        for label,lower,upper,target in (('old',WEST_SPLIT.knots[0],WEST_SPLIT.knots[-1],shape),('full',START,END,full)):
            for edge in range(5):
                lo=max(lower,p.births.get(edge,lower))
                cuts=sorted({lo,upper}|{s for s in [*p.scope.knots,*self.old_cuts] if lo<s<upper})
                parts=[span_shape(power(edge,a),b-a) for a,b in zip(cuts,cuts[1:])]
                target[str(edge)]=dict(max_abs_curvature=max(r['max_abs_curvature'] for r in parts),
                    max_abs_curvature_rate=max(r['max_abs_curvature_rate'] for r in parts),
                    curvature_total_variation=sum(r['curvature_variation'] for r in parts)+sum(abs(a['curvature_end']-b['curvature_start']) for a,b in zip(parts,parts[1:])))
        rev=self._reversal(power,WEST_SPLIT.knots[0],WEST_SPLIT.knots[-1]); failures=[]
        for edge in range(5):
            for metric,eps in zip(METRICS,METRIC_EPS):
                if shape[str(edge)][metric]>self.caps[str(edge)][metric]+eps:
                    failures.append(dict(edge=edge,metric=metric,actual=shape[str(edge)][metric],limit=self.caps[str(edge)][metric]))
            if rev[edge]>self.old_reversal[edge]+1e-7:
                failures.append(dict(edge=edge,metric='source_direction_reversal_m',actual=float(rev[edge]),limit=float(self.old_reversal[edge])))
        endpoints=[]
        for r in self.contract.source_binding['receipts']:
            b=r['binding']; debt=r['original_debt']
            if b['binding_kind']=='WRITTEN_INTERVAL_VIA_PROVEN_CONTINUATION' or debt['kind']!='physical_boundary': continue
            s=b['target_station']; xy=np.array([s,power(debt['edge'],s)[0]])
            error=float(np.linalg.norm(np.array(b['source_st'])-xy,axis=1).max())
            endpoints.append(dict(feature=debt['feature'],kind=b['binding_kind'],distance_m=error,limit_m=.35,within_limit=error<=.35+1e-8))
        return dict(source_rows=rows,source_max_m=max(r['maximum_lateral_m'] for r in rows),
            source_local_research_failures=sum(not r['within_local_research_limit'] for r in rows),
            source_ordinary_final_failures=sum(not r['within_ordinary_final_limit'] for r in rows),
            widths=widths,negative_width_intervals=sum(r['minimum_m'] < -1e-7 for r in widths),
            old_guard_domain_m=[WEST_SPLIT.knots[0],WEST_SPLIT.knots[-1]],old_shape_caps=self.caps,
            old_shape_metrics=shape,old_reversal_m=rev.tolist(),old_guard_failures=failures,
            full_event_shape_observation=full,full_event_reversal_observation_m=self._reversal(power,START,END).tolist(),
            target_s178=dict(original_t=self.target_t,actual_t=float(power(3,178.)[0]),error_m=float(power(3,178.)[0]-self.target_t)),
            finite_physical_endpoint_checks=endpoints,independent_parent_movement_paths=19,
            movement_center_equality_applied=False,whole_source_coverage_accepted=False)

    def turn_checks(self,cid,turn,step):
        rows=self.sources.by_turn[cid]; mesh=sample_ribbon(turn,step=step)
        physical=[]; tails=[]; empty=[]; all_support={}; turning={}
        for side in ('left','right'):
            own=[r for r in rows if r['field']==side]
            all_support[side]=np.vstack([r['oriented_full_xy'] for r in own])
            for row in own:
                if not row['owned_fragments']:
                    empty.append(dict(feature=row['feature'],role=row['role'],reason='empty frozen interval; full original retained by ordinary context'))
                    continue
                source=np.vstack([densify(v,step) for v in row['owned_fragments']])
                e=distances(source,mesh['points'][side]); st=stats(e)
                limits=dict(max_m=.35 if row['role']!='via' else 1.5,p95_m=.75,median_m=.35)
                # Against a polyline approximation: only distances exceeding
                # the limit PLUS the chord bound prove a sampled violation.
                violating=[k for k in (limits if row['role']=='via' else ('max_m',)) if st[k]-mesh['chord_error_bound_m']>limits[k]+1e-7]
                physical.append(dict(feature=row['feature'],field=side,role=row['role'],direction='owned_source_to_target',
                    source_intervals_m=row['owned_intervals_m'],statistics=st,limits=limits,
                    proven_sampled_violations=violating,continuous_pass=False))
                if row['role']!='via':
                    # Retain the existing strict tail guard separately. This
                    # projection selects target prefix/suffix only; raw source
                    # is complete and never swapped to a neighbouring line.
                    raw=row['oriented_full_xy']; after=row['role']=='successor'; tip=raw[0 if after else -1]
                    actual=crop_at_projection(mesh['points'][side],tip,after)
                    t=stats(distances(actual,raw))
                    tails.append(dict(feature=row['feature'],field=side,role=row['role'],statistics=t,limit_m=.35,
                        sampled_violation=t['max_m']-mesh['chord_error_bound_m']>.35+1e-7,continuous_pass=False,
                        target_partition_method='legacy known-source tip projection; not full continuous ownership certificate'))
            st=stats(distances(mesh['points'][side],all_support[side]))
            physical.append(dict(field=side,role='complete_original_chain',direction='target_to_source',statistics=st,
                limits=dict(max_m=1.5,p95_m=.75,median_m=.35),
                proven_sampled_violations=[k for k,b in dict(max_m=1.5,p95_m=.75,median_m=.35).items() if st[k]>b+1e-7],continuous_pass=False))
        # Fairing: do not turn a navigation observation into a physical-center
        # equality. Only explicit source boundaries supply reversal evidence.
        evidence=source_turn_evidence(all_support)
        for side in ('left','right'):
            pos=neg=0.; regular=True
            for a,b,ref,offset,co in ribbon_intervals(turn):
                f=interval_fairness(ref.KappaStart+ref.dk*offset,ref.dk,co[side],b-a)
                pos+=f['positive_turn_deg']; neg+=f['negative_turn_deg']; regular=regular and f['regular']
            reverse=neg if evidence[side]['sign']>0 else pos
            extra=reverse-evidence[side]['reverse_turn_deg']
            turning[side]=dict(regular=regular,positive_turn_deg=pos,negative_turn_deg=neg,
                source=evidence[side],additional_reverse_turn_deg=extra,construction_added_turn_limit_deg=.5,
                within_limit=regular and extra<=.5+1e-7,continuous_certificate=False)
        return dict(source_rows=physical,ordinary_tail_rows=tails,empty_owned_parts=empty,
            physical_reverse_turn=turning,center_fairing_acceptance='NOT_EVALUATED_NO_MIDPOINT_SOURCE_ASSUMPTION',
            movement_observations=[dict(source_lane_id=r['source_lane_id'],feature=r['feature'],
                original_vertices=len(r['full_original_xy']),role='independent_movement_path',validation_complete=False)
                for r in rows if r['field']=='movement'],
            sampled_surface=mesh_surface(mesh,rows),chord_error_bound_m=mesh['chord_error_bound_m'],
            evaluation_step_m=step,source_fidelity_continuous_pass=False,output_primitives_added=0)

    def evaluate(self,vector=None,*,reference=False,step=.1):
        if reference:
            if vector is not None: raise ValueError('Choose unchanged XML or common coordinates')
            parent=self.parent_checks(); turns={cid:reference_turn(self.contract.graph.roads[cid]) for cid in CONNECTORS}
            geometry=dict(kind='UNCHANGED_REFERENCE_XML',coordinate_candidate=False)
        else:
            if vector is None: raise ValueError('Explicit complete shared state required')
            snapshot=self.model.evaluate(vector); parent=self.parent_checks(snapshot['parent_coefficients']); turns=snapshot['turns']
            geometry=dict(kind='COMMON_STATE_NOT_SOLVED',coordinate_candidate=False,
                contacts={cid:t['contacts'] for cid,t in turns.items()},joins={cid:t['joins'] for cid,t in turns.items()},
                closure={cid:t['reference_closure'].tolist() for cid,t in turns.items()})
        result={cid:self.turn_checks(cid,t,step) for cid,t in turns.items()}
        return dict(schema='mapforge/coupled-event-guards/v1',status='GUARDS_CONNECTED_ADMISSION_INCOMPLETE',
            source_inventory=self.sources.inventory(),geometry=geometry,parent=parent,turns=result,
            unresolved=['full-domain shape intent and limits outside original research guard domain',
                'continuous source distance and target tail ownership including folds',
                'whole junction source surface with fixed roads and retained islands',
                'actual compiled XML source/geometry/layout readback and atomic transaction'],
            map_accepted=False,trial_admitted=False,export_allowed=False,optimizer_calls=0,new_xodr=False,
            dynamic_acceptance='separate explicit path/speed case required; source limits and old failures unchanged')
