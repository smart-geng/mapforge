"""Original-identity support of the seven-road event; no fitter or writer.

Reuses the admitted structural ownership and complete raw parts. A zero-length
connector tail stays explicitly empty; it must not become a synthetic line.
All LANE_LINK observations remain separate from physical boundary constraints.
"""
from dataclasses import asdict
import copy
import math

import numpy as np
from numpy.polynomial import Polynomial as P

from mapforge.ops.source_domain import clipped_intervals
from mapforge.repair_web.event_source_binding import original_slice
from mapforge.repair_web.split_event_admission import CONNECTORS
from mapforge.validate.shp_boundary_fidelity import _project


class EventSources:
    def __init__(self, contract):
        self.contract = contract
        partition = contract.domain['partition']; projection = partition['projection']
        self.rows = []; self.by_turn = {}
        resolved = {r['connector']: r for r in contract.domain['comparison']['actual']}
        for cid in CONNECTORS:
            proof = resolved[cid]; ports = contract.graph.connections[cid]
            if any(proof[k] != asdict(port) for k,port in zip(('incoming_port','outgoing_port'), ports)):
                raise ValueError('Frozen source path does not match current physical ports')
            ids = proof['source_path']
            if len(ids) != 3 or len(set(ids)) != 3:
                raise ValueError('This admitted event requires its explicit three-source path')
            mappings = [r['source_side_mapping'] for r in contract.port_sources if r['connector']==cid]
            if not mappings or any(m != mappings[0] for m in mappings): raise ValueError('Inconsistent physical side identity')
            sides = mappings[0]; rows = []
            for index, sid in enumerate(ids):
                obs = contract.packet['observations'][sid]
                if obs['geometry_role'] != 'lane_path_observation_not_assumed_boundary_midpoint':
                    raise ValueError('Unreviewed original path role')
                rel = {r['declared_side']:r['boundary_key'] for r in obs['boundary_relations']}
                if len(obs['boundary_relations']) != 2 or set(rel) != {'left','right'}:
                    raise ValueError('Unique declared source boundaries required')
                center_part = partition['features']['lane:'+sid]['parts']
                if len(center_part) != 1: raise ValueError('Multipart source requires explicit support')
                center = _project(np.array(center_part[0]['raw_vertices']), projection['lat_0'], projection['lon_0'])
                for field in ('left','right','movement'):
                    key = 'lane:'+sid if field=='movement' else 'boundary:'+rel[sides[field]]
                    feature = partition['features'][key]
                    if len(feature['parts']) != 1: raise ValueError('Multipart source requires explicit support')
                    part = feature['parts'][0]
                    cuts = [c for c in part['consumer_cuts'] if c['owner']=='connector:'+cid and c['source_lane_id']==sid]
                    if len(cuts) != 1: raise ValueError('Missing or ambiguous original consumer')
                    cut = cuts[0]
                    raw = _project(np.array(part['raw_vertices']), projection['lat_0'], projection['lon_0'])
                    intrinsic, intervals = clipped_intervals(raw, cut['planes'])
                    if not np.allclose(intrinsic,part['source_vertex_s_m'],atol=1e-8,rtol=0): raise ValueError('Original arclength drift')
                    if len(intervals)!=len(cut['intervals_m']) or any(not np.allclose(a,b,atol=1e-8,rtol=0)
                                                                 for a,b in zip(intervals,cut['intervals_m'])):
                        raise ValueError('Original ownership planes no longer reproduce the frozen intervals')
                    if index==1 and (cut['planes'] or intervals != [[0., intrinsic[-1]]]):
                        raise ValueError('Original via must remain complete')
                    reverse = (raw[-1]-raw[0])@(center[-1]-center[0]) < 0
                    oriented = raw[::-1] if reverse else raw
                    # Every original part and vertex is retained. Interval
                    # slicing adds intersection points, not geometry edits.
                    fragments = [original_slice(raw, intrinsic, span) for span in intervals]
                    if reverse: fragments = [p[::-1] for p in fragments[::-1]]
                    row = dict(connector=cid, source_lane_id=sid, feature=key, source_part=0, field=field,
                        role=('predecessor','via','successor')[index], full_original_xy=raw,
                        oriented_full_xy=oriented, original_arclength_m=np.array(intrinsic),
                        owned_intervals_m=copy.deepcopy(intervals), owned_fragments=fragments,
                        original_planes=copy.deepcopy(cut['planes']), travel_reversed=bool(reverse),
                        independent_movement=field=='movement', source_role_changed=False)
                    rows.append(row); self.rows.append(row)
            for field in ('left','right','movement'):
                items = [r for r in rows if r['field']==field]
                if any(np.linalg.norm(a['oriented_full_xy'][-1]-b['oriented_full_xy'][0])>1e-5 for a,b in zip(items,items[1:])):
                    raise ValueError('Original chain gap: no invented bridge or nearest source replacement')
            self.by_turn[cid] = rows

    def inventory(self):
        return dict(connectors=list(self.by_turn), source_consumer_parts=len(self.rows),
            physical_consumers=sum(r['field']!='movement' for r in self.rows),
            independent_movement_consumers=sum(r['field']=='movement' for r in self.rows),
            empty_owned_parts=[dict(connector=r['connector'],feature=r['feature'],field=r['field'],role=r['role'])
                               for r in self.rows if not r['owned_intervals_m']],
            source_binding_sha256=self.contract.source_binding['content_sha256'],
            complete_vias_preserved=True, source_vertices_removed=0, synthetic_joins=0,
            source_geometry_accepted=False, movement_validation_complete=False)


def polynomial_shift(c, shift):
    co = P(c)(P([shift,1.])).coef
    return np.pad(co,(0,4-len(co)))


def polynomial_absmax(c, length):
    poly = P(c); roots = poly.deriv().roots()
    sites = [0.,length]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<length]
    return float(max(abs(poly(np.array(sites)))))


def ribbon_intervals(turn):
    """Re-expression for EVALUATION only; no extra output primitives."""
    refs = turn['refs']; knots = np.asarray(turn['stations'])
    rs = np.r_[0.,np.cumsum([r.length for r in refs])]
    cuts = np.unique(np.r_[rs,knots])
    for a,b in zip(cuts,cuts[1:]):
        if b-a < 1e-10: continue
        i = min(len(refs)-1,np.searchsorted(rs,(a+b)/2,side='right')-1)
        j = min(len(knots)-2,np.searchsorted(knots,(a+b)/2,side='right')-1)
        yield float(a),float(b),refs[i],float(a-rs[i]),{
            field:polynomial_shift(turn['coefficients'][field][j], a-knots[j]) for field in ('left','right')}


def sample_ribbon(turn, *, step=.1, chord_tolerance=.0001):
    """Analytic curve samples with a conservative numerical chord bound.

    |r''| is bounded from its two polynomial moving-frame components. h^2*M/8
    bounds chord interpolation error; not a formal floating-point certificate.
    The evaluation mesh is NEVER serialized as OpenDRIVE geometry.
    """
    if not .02<=step<=.25 or not 0<chord_tolerance<=.001: raise ValueError('Unreviewed evaluation resolution')
    pieces = {k:[] for k in ('left','right','center')}; stations=[]; max_bound=0.
    for a,b,ref,offset,co in ribbon_intervals(turn):
        L=b-a; k=P([ref.KappaStart+ref.dk*offset,ref.dk]); fields=dict(co,center=(co['left']+co['right'])/2)
        bound=0.
        for c in fields.values():
            t=P(c); dt=t.deriv(); A=1-k*t
            tang=-ref.dk*t-2*k*dt; normal=k*A+t.deriv(2)
            bound=max(bound,math.hypot(polynomial_absmax(tang.coef,L),polynomial_absmax(normal.coef,L)))
        h=min(step, math.sqrt(8*chord_tolerance/bound)) if bound>0 else step
        count=max(1,math.ceil(L/h))
        if count>100000: raise ValueError('Diagnostic geometry too extreme for bounded evaluation mesh')
        u=np.linspace(0,L,count+1)
        h=L/count; max_bound=max(max_bound,bound*h*h/8)
        xy=np.array([[ref.X(offset+s),ref.Y(offset+s)] for s in u])
        heading=np.array([ref.Theta(offset+s) for s in u]); normal=np.c_[-np.sin(heading),np.cos(heading)]
        for field,c in fields.items(): pieces[field].append(xy+P(c)(u)[:,None]*normal)
        stations.append(a+u)
    return dict(points={k:np.vstack(v) for k,v in pieces.items()},stations=np.concatenate(stations),
        chord_error_bound_m=max_bound, evaluation_step_max_m=step, output_primitives_added=0,
        bound_method='polynomial second-derivative norm times h^2/8, floating-point not formal certificate')
