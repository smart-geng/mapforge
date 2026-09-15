"""Dynamics bound to the same five-parent/all-turn geometric evaluation.

Does not guess movement speed intervals or mistake original MAX_SPEED for
approved movement design speed. No optimizer, speed writer or geometry fit.
"""
import hashlib
from collections import Counter

import numpy as np

from mapforge.ops.continuous_offset_dynamics import audit_span, parent_spans, turn_spans


def source_speed_binding(provider, observations):
    rows=[]
    for sid in provider.source_ids:
        speed=observations[sid].get('source_max_speed_kmh')
        if isinstance(speed,bool) or not isinstance(speed,(float,int)) or not np.isfinite(speed) or speed<=0:
            raise ValueError('explicit positive unchanged original speed required: '+sid)
        rows.append(dict(source_lane=sid,source_speed_kmh=float(speed)))
    if not rows:raise ValueError('empty original movement speed support')
    uniform=len({r['source_speed_kmh'] for r in rows})==1
    return dict(rows=rows,evaluation_speed_kmh=max(r['source_speed_kmh'] for r in rows),
        policy='uniform-original-limit' if uniform else 'maximum-original-limit-envelope-only',
        source_interval_mapping_complete=uniform,
        approved_movement_design_speed=False,source_speed_changed=False)


def summarize_rows(rows):
    counts=dict(Counter(r['check']['status'] for r in rows))
    return dict(spans=len(rows),counts=counts,
        status='FAIL' if counts.get('FAIL') else 'UNKNOWN' if counts.get('UNKNOWN') or not rows else 'BOUNDED',
        minimum_check_span_m=min((r['end']-r['start'] for r in rows),default=None),
        covered_span_sum_m=sum(r['end']-r['start'] for r in rows),
        all_parameter_intervals_accounted=bool(rows) and all(r['check']['all_parameter_intervals_accounted'] for r in rows),
        observed_worst=[max((dict(r['check']['observed_maxima'][j],start=r['start'],end=r['end'],
                                  identity=r.get('identity')) for r in rows if r['check']['observed_maxima'][j]),
                           key=lambda r:r['value'],default=None) for j in range(2)])


class WholeSourceDynamics:
    def __init__(self,geometry,observations):
        self.geometry=geometry
        # Store exact input speed bindings once; the geometry may move, the
        # source records/limit contract must not mutate during a solve.
        self.speed_bindings={cid:source_speed_binding(p,observations)
                             for cid,p in geometry.source_support.items()}

    def evaluate(self,vector,*,progress=None):
        v=np.asarray(vector,float)
        snapshot=self.geometry.evaluate(v)  # All parents FIRST, then ALL turns.
        parent_rows={}; turn_rows={}
        for rid,state in snapshot['parents'].items():
            rows=[]
            for r in parent_spans(state):
                co=r.pop('coefficients')
                rows.append(dict(r,cubic_coefficients=co.tolist(),check=audit_span(co,r['end']-r['start'],r['curvature'],r['sharpness'],r['speed_kmh'])))
            parent_rows[rid]=rows
            if progress:progress('parent',rid,summarize_rows(rows))
        for cid,state in snapshot['turns'].items():
            binding=self.speed_bindings[cid];rows=[]
            for r in turn_spans(state):
                co=r.pop('coefficients');speed=binding['evaluation_speed_kmh']
                rows.append(dict(r,identity=dict(turn=cid),speed_kmh=speed,cubic_coefficients=co.tolist(),
                    check=audit_span(co,r['end']-r['start'],r['curvature'],r['sharpness'],speed)))
            turn_rows[cid]=rows
            if progress:progress('turn',cid,summarize_rows(rows))
        parents={rid:summarize_rows(r) for rid,r in parent_rows.items()}
        turns={cid:dict(summarize_rows(r),speed_binding=self.speed_bindings[cid]) for cid,r in turn_rows.items()}
        return dict(status='WHOLE_STATE_DYNAMIC_DEMAND_CHECK_NOT_MAP',
            vector_sha256=hashlib.sha256(v.astype('<f8').tobytes()).hexdigest(),
            parents=parents,turns=turns,parent_rows=parent_rows,turn_rows=turn_rows,
            parent_count=len(parents),turn_count=len(turns),limits=dict(ay_mps2=2.5,lateral_rate_mps3=1.),
            parent_and_turns_from_same_evaluation=True,
            all_declared_parameter_intervals_accounted=all(x['all_parameter_intervals_accounted'] for x in [*parents.values(),*turns.values()]),
            source_interval_speed_mapping_complete=all(b['source_interval_mapping_complete'] for b in self.speed_bindings.values()),
            minimum_reference_primitive_m=min(t['minimum_reference_span_m'] for t in snapshot['turns'].values()),
            parent_reference_primitives=len(parents),turn_reference_primitives=sum(t['reference_primitives'] for t in snapshot['turns'].values()),
            initial_geometry_max_scaled_equality=float(np.max(abs(snapshot['equalities']),initial=0.)),
            initial_geometry_min_inequality=float(np.min(snapshot['inequalities'])),
            geometry_segments_added=0,source_speed_changed=False,geometry_optimized=False,
            joins_checked_by_this_layer=False,approved_movement_design_speed=False,
            physical_midpoint_domains_not_complete_written_ownership=True,
            full_original_written_domain_accepted=False,physical_surface_accepted=False,
            full_map_dynamics_accepted=False,formal_source_certificate=False,export_allowed=False)
