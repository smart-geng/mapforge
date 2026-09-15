"""Source-identity residuals and dependency-local finite linearization.

No optimizer/writer. In particular this is NOT the obsolete fixed-parent
C/CZ QP. Parent chart/cut/knot trials rebuild source rows and all incident
turn source partitions; unrelated blocks alone reuse the same snapshot.
Min/max/percentile/contact events are nonsmooth. Stencils report one-sided
domain restrictions and left/right disagreements instead of claiming C1.
"""
from dataclasses import dataclass

import numpy as np
from scipy.sparse import csc_matrix

from mapforge.ops.source_constraint_envelope import SourceConstraintEnvelope, ConstraintStructureError


@dataclass
class ConstraintSnapshot:
    vector: np.ndarray
    full: dict
    blocks: dict
    values: np.ndarray


def _turn_signature(turn):
    r=turn['result']
    return (tuple((k,tuple(fields)) for k,fields in r[3].items()),
            tuple(r[10]), tuple(r[9] or {}), len(turn['equalities']), len(r[5]), len(r[0]))


def turn_inequality_keys(turn):
    r=turn['result']
    keys=['minimum-width-m','minimum-forward-factor']
    keys += [f'source:{field}:{direction}:{metric}' for field,fields in r[3].items()
             for direction in fields for metric in ('max-m','p95-m','median-m')]
    keys += ['ordinary-tail:'+field+':max-m' for field in r[10]]
    if r[9] is not None:
        keys += ['added-reverse-turn:'+field+':scaled-rad' for field in ('left','right','center')]
    # Synthetic dependency fixtures may not implement the production kernel.
    if len(keys)!=len(r[5]):
        raise ConstraintStructureError('movement inequality meanings do not cover every row')
    return tuple(keys)


def squared_distance_slack(slack,limit):
    """(B²-e²)/(2B), equivalent to B-e >=0 for distance e>=0.

    Same feasibility AND same derivative at the active boundary e=B. The
    norm cusp at a zero-distance, strictly inactive constraint disappears.
    Median/nearest-feature switches elsewhere remain explicitly nonsmooth.
    """
    s=np.asarray(slack,float);b=np.asarray(limit,float)
    if (not np.isfinite(s).all() or not np.isfinite(b).all() or np.any(b<=0)
            or np.any(s>b+1e-10)):
        raise ValueError('positive exact fidelity budgets and nonnegative distances required')
    return s*(1-s/(2*b))


class WholeSourceConstraintSystem:
    def __init__(self, geometry):
        self.geometry=geometry
        self.initial=geometry.initial.copy()
        full=geometry.evaluate(self.initial)
        self.parents={rid:SourceConstraintEnvelope(p['model'],p['coefficients'])
                      for rid,p in full['parents'].items()}
        self.turn_signatures={cid:_turn_signature(t) for cid,t in full['turns'].items()}
        self.turn_distance_rows={}
        self.slices={}; self.equality_indices=[]; self.inequality_indices=[]
        self.row_identities=[]; self.block_order=[]
        for rid,layout in self.parents.items():
            self._register(('parent',rid),layout.eq_keys,layout.keys)
        for cid in geometry.turn_slices:
            t=full['turns'][cid]
            keys=turn_inequality_keys(t);budgets=geometry.kernels[cid].get('source_metric_budgets',{})
            indices=[];limits=[]
            for i,key in enumerate(keys):
                metric=key.rsplit(':',1)[-1] if key.startswith('source:') else (
                    'ordinary-tail:max-m' if key.startswith('ordinary-tail:') else None)
                if metric is not None:
                    if metric not in budgets:raise ConstraintStructureError('exact kernel fidelity budget missing')
                    indices.append(i);limits.append(budgets[metric])
            self.turn_distance_rows[cid]=(np.array(indices,int),np.array(limits,float))
            self._register(('turn',cid),range(len(t['equalities'])),keys)
        self.equality_indices=np.array(self.equality_indices,int)
        self.inequality_indices=np.array(self.inequality_indices,int)
        self.bounds=np.tile([-np.inf,np.inf],(len(self.initial),1))
        self.steps=np.full(len(self.initial),1e-5)
        self.fixed_columns=[]; self.column_owner={}
        self.dependents={rid:tuple(cid for cid in geometry.turn_slices
            if any(p.road==rid for p in geometry.ports.graph.connections[cid])) for rid in self.parents}
        for rid,sl in geometry.ports.slices.items():
            parent=geometry.ports.parents[rid]
            self.bounds[sl.start:sl.start+4]=[[-2,2],[-2,2],[-.1,.1],[-.004,.004]]
            self.steps[sl.start+2:sl.start+4]=[1e-7,1e-9]
            for j,cp in enumerate(('start','end')):
                if not any(p.contact==cp for p in parent.port_features):
                    col=sl.start+4+j; self.fixed_columns.append(col); self.bounds[col]=[0,0]
            for col in range(sl.start,sl.stop):self.column_owner[col]=('parent',rid)
        for cid,sl in geometry.turn_slices.items():
            self.bounds[sl]=geometry.kernels[cid]['bounds']
            for col in range(sl.start,sl.stop):self.column_owner[col]=('turn',cid)
        self.free_columns=np.array([i for i in range(len(self.initial)) if i not in self.fixed_columns],int)
        self.base=self._pack(self.initial,full)

    def _register(self,key,eq_keys,iq_keys):
        a=len(self.row_identities)
        self.row_identities.extend((key,'equality',k) for k in eq_keys)
        b=len(self.row_identities)
        self.row_identities.extend((key,'inequality',k) for k in iq_keys)
        c=len(self.row_identities)
        self.equality_indices.extend(range(a,b));self.inequality_indices.extend(range(b,c))
        self.slices[key]=slice(a,c);self.block_order.append(key)

    def _block(self,key,full):
        kind,rid=key
        if kind=='parent':return self.parents[rid].evaluate(full['parents'][rid])
        turn=full['turns'][rid]
        if _turn_signature(turn)!=self.turn_signatures[rid]:
            raise ConstraintStructureError('movement constraint identities changed: '+rid)
        eq=turn['equalities'];original=turn['result'][5];iq=np.array(original,copy=True)
        if not np.isfinite(np.r_[eq,iq]).all():raise ValueError('nonfinite movement constraints: '+rid)
        indices,limits=self.turn_distance_rows[rid]
        iq[indices]=squared_distance_slack(iq[indices],limits)
        return dict(equalities=eq,inequalities=iq,original_inequalities=original,
            fidelity_feasibility_equivalent=True)

    def _pack(self,vector,full):
        blocks={key:self._block(key,full) for key in self.block_order}
        values=np.concatenate([np.r_[b['equalities'],b['inequalities']] for b in blocks.values()])
        if len(values)!=len(self.row_identities):raise ConstraintStructureError('whole residual dimension changed')
        return ConstraintSnapshot(np.array(vector,copy=True),full,blocks,values)

    def _check_vector(self,vector):
        v=np.asarray(vector,float)
        if v.shape!=self.initial.shape or not np.isfinite(v).all():raise ValueError('complete finite whole state required')
        if np.any(v<self.bounds[:,0]) or np.any(v>self.bounds[:,1]):
            raise ValueError('whole state outside registered coordinate/long-primitive bounds')
        return v

    def evaluate(self,vector):
        v=self._check_vector(vector)
        return self._pack(v,self.geometry.evaluate(v))

    def trial_column(self,base,column,delta):
        """Local evaluation exactly equivalent to a whole snapshot at x+he_i."""
        if column not in self.column_owner or column in self.fixed_columns:
            raise ValueError('unregistered or fixed coordinate is not a derivative column')
        v=base.vector.copy();v[column]+=delta
        return self.trial_vector(base,v)

    def affected_blocks(self,columns):
        """Structural dependence, not numerical-zero pruning."""
        affected={self.column_owner[int(j)] for j in columns}
        for kind,rid in tuple(affected):
            if kind=='parent':affected.update(('turn',cid) for cid in self.dependents[rid])
        return tuple(key for key in self.block_order if key in affected)

    def trial_vector(self,base,vector):
        """Atomic joint trial: rebuild ALL changed parents before ANY turn.

        The two ends of a movement always belong to this SAME vector. This
        is not a sequence of individually accepted parent/turn updates.
        Unchanged blocks alone reuse their source-bound snapshot. Errors do
        not modify the supplied base, geometry.initial or other trials.
        """
        v=self._check_vector(vector).copy();g=self.geometry
        columns=np.flatnonzero(v!=base.vector)
        affected=self.affected_blocks(columns)
        full=dict(base.full,parents=dict(base.full['parents']),turns=dict(base.full['turns']))
        for kind,rid in affected:
            if kind=='parent':
                p=g.ports.parents[rid]
                full['parents'][rid]=p.evaluate_from_snapshot(v[g.ports.slices[rid]],base.full['parents'][rid])
        for k,cid in affected:
            if k=='turn':full['turns'][cid]=g.evaluate_turn(cid,v[g.turn_slices[cid]],full['parents'])
        # All unchanged blocks were evaluated at exactly the SAME unchanged
        # coordinates. Do not recompute or average independent parent fits.
        blocks=dict(base.blocks);values=base.values.copy()
        for block in affected:
            b=self._block(block,full);blocks[block]=b
            values[self.slices[block]]=np.r_[b['equalities'],b['inequalities']]
        # A local snapshot must not carry stale whole-state totals alongside
        # its fresh block values. Both views describe exactly the same trial.
        if 'equalities' in full:
            full['equalities']=np.concatenate([p['source_equalities'] for p in full['parents'].values()]+
                [t['equalities'] for t in full['turns'].values()])
            full['inequalities']=np.concatenate([p['source_inequality_slack'] for p in full['parents'].values()]+
                [t['result'][5] for t in full['turns'].values()])
            full['source_objective']=sum(float(t['result'][11]) for t in full['turns'].values())
        return ConstraintSnapshot(v,full,blocks,values)

    def linearize(self,base=None,columns=None,*,step_scale=1.,progress=None,coefficient_stencils=False):
        """Sparse stencil matrix, NOT a smooth-Jacobian or feasibility claim.

        A subset returns exactly that many columns (no fabricated zero cols).
        An invalid side is recorded; both invalid sides abort. Unknown bugs
        are not turned into zero derivatives or swallowed. Caller must handle
        nonsmooth rows/domain-active directions in its constrained algorithm.
        """
        base=self.base if base is None else base
        columns=self.free_columns if columns is None else np.asarray(columns,int)
        if (columns.ndim!=1 or len(set(columns))!=len(columns) or not np.isfinite(step_scale)
                or step_scale<=0 or any(int(j) not in self.free_columns for j in columns)):
            raise ValueError('unique free stencil columns and positive scale required')
        vals=[];rows=[];cols=[];diagnostics=[]
        sided={name:dict(values=[],rows=[],cols=[],valid=[]) for name in ('right','left')}
        fast=None
        if coefficient_stencils:
            from mapforge.ops.source_coefficient_stencil import ParentCoefficientStencils
            fast=ParentCoefficientStencils(self,base)
        for out_col,j in enumerate(columns):
            j=int(j);h=self.steps[j]*step_scale;sides={};errors={};fast_rows={}
            for direction,name in ((1,'plus'),(-1,'minus')):
                try:
                    quick=None if fast is None else fast.evaluate(j,direction*h)
                    if quick is None:sides[name]=self.trial_column(base,j,direction*h).values
                    else:sides[name]=quick['values'];fast_rows[name]=quick
                except ConstraintStructureError:raise
                except (ValueError,ArithmeticError) as exc:errors[name]=str(exc)
            if not sides:raise ValueError(f'no admissible stencil for coordinate {j}: {errors}')
            right=(sides['plus']-base.values)/h if 'plus' in sides else None
            left=(base.values-sides['minus'])/h if 'minus' in sides else None
            for name,slope in (('right',right),('left',left)):
                data=sided[name];data['valid'].append(slope is not None)
                if slope is not None:
                    if not np.isfinite(slope).all():raise ValueError('nonfinite one-sided stencil')
                    nz=np.flatnonzero(slope)
                    data['rows'].extend(nz.tolist());data['cols'].extend([out_col]*len(nz))
                    data['values'].extend(slope[nz].tolist())
            mismatch=[]
            if right is not None and left is not None:
                col=(right+left)/2
                scaled=abs(right-left)/(1+np.maximum(abs(right),abs(left)))
                mismatch=np.flatnonzero(scaled>1e-2).tolist()
                method='two-sided-secant'
            else:
                col=right if right is not None else left;method='one-sided-domain-stencil'
            if not np.isfinite(col).all():raise ValueError('nonfinite derivative stencil')
            nz=np.flatnonzero(col)  # Exact zero only; no magnitude pruning.
            rows.extend(nz.tolist());cols.extend([out_col]*len(nz));vals.extend(col[nz].tolist())
            diag=dict(column=j,owner=self.column_owner[j],step=h,method=method,domain_rejections=errors,
                left_right_disagreement_rows=mismatch,nonsmooth_or_numerically_unresolved=bool(mismatch),
                recomputed_blocks=[self.column_owner[j]]+([('turn',c) for c in self.dependents[self.column_owner[j][1]]]
                    if self.column_owner[j][0]=='parent' else []),
                jacobian_smoothness_certified=False)
            if fast_rows:
                diag['coefficient_evaluation']=next(iter(fast_rows.values()))['method']
                diag['recomputed_blocks']=next(iter(fast_rows.values()))['recomputed_blocks']
                diag['original_certificate_accounting']=next(iter(fast_rows.values()))['source_certificate_rows']
            diagnostics.append(diag)
            if progress is not None:progress(diag)
        matrix=csc_matrix((vals,(rows,cols)),shape=(len(base.values),len(columns)))
        return dict(matrix=matrix,columns=columns.copy(),diagnostics=diagnostics,
            sided_matrices={name:csc_matrix((d['values'],(d['rows'],d['cols'])),shape=matrix.shape)
                            for name,d in sided.items()},
            sided_valid={name:np.array(d['valid'],bool) for name,d in sided.items()},
            base_vector=base.vector.copy(),base_values=base.values.copy(),
            row_identities=tuple(self.row_identities),
            all_free_columns_evaluated=np.array_equal(columns,self.free_columns),
            smooth_jacobian_certified=False,optimization_ran=False,export_allowed=False)

    def summary(self,snapshot=None):
        s=self.base if snapshot is None else snapshot
        return dict(status='COMPLETE_GEOMETRY_ENVELOPE_NOT_SOLVED',variables=len(s.vector),
            free_variables=len(self.free_columns),fixed_columns=self.fixed_columns,
            equality_count=len(self.equality_indices),inequality_count=len(self.inequality_indices),
            max_scaled_equality=float(max(abs(s.values[self.equality_indices]),default=0.)),
            minimum_mixed_slack=float(min(s.values[self.inequality_indices])),
            parent_source_certificates={rid:b['accounting'] for (kind,rid),b in s.blocks.items() if kind=='parent'},
            all_source_rows_retained=True,source_min_envelopes_nonsmooth=True,
            connector_fidelity_form='(B^2-e^2)/(2B); exact same bounds, not changed geometry',
            original_kernel_minimum_slack=float(min(min(t['result'][5]) for t in s.full['turns'].values())),
            domain_dynamics_surface_complete=False,optimization_ran=False,export_allowed=False)
