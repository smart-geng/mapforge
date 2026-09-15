"""Two-sided working models and exact, atomic whole-state step verification.

Min envelopes and order statistics are NOT replaced by central derivatives.
The interval here encloses the sums of the measured coordinate-side slopes;
it is NOT a mathematical bound on finite nonlinear motion, nor a generalized
Jacobian certificate. Every trial is independently evaluated at the full
new state. No optimizer, acceptance tolerance relaxation or XODR writer.
"""
import numpy as np

from mapforge.ops.source_constraint_envelope import ConstraintStructureError


class WholeSourceStepModel:
    def __init__(self,system,base,stencil,*,radius):
        self.system=system;self.base=base;self.stencil=stencil
        if (not np.array_equal(base.vector,stencil['base_vector']) or
                not np.array_equal(base.values,stencil['base_values']) or
                tuple(system.row_identities)!=stencil['row_identities']):
            raise ConstraintStructureError('stencil belongs to a different whole state or source identity')
        self.columns=np.asarray(stencil['columns'],int)
        self.radius=np.asarray(radius,float).copy()
        if (self.radius.shape!=base.vector.shape or not np.isfinite(self.radius).all() or
                np.any(self.radius<0) or np.any(self.radius[system.free_columns]<=0) or
                np.any(self.radius[system.fixed_columns]!=0)):
            raise ValueError('positive registered radii for all free coordinates, zero for fixed coordinates required')
        shape=(len(base.values),len(self.columns))
        for side in ('right','left'):
            matrix=stencil['sided_matrices'][side]
            valid=np.asarray(stencil['sided_valid'][side])
            if matrix.shape!=shape or valid.shape!=(len(self.columns),) or not np.isfinite(matrix.data).all():
                raise ValueError('complete side matrix and explicit valid-column mask required')
        # Missing sides remain missing. A numeric sparse zero is NEVER used
        # as the derivative of a direction rejected by the original domain.
        right=stencil['sided_matrices']['right'].toarray()
        left=stencil['sided_matrices']['left'].toarray()
        rv=stencil['sided_valid']['right'];lv=stencil['sided_valid']['left']
        if np.any(~(rv|lv)):raise ValueError('no original-domain side for a stencil column')
        right[:,~rv]=left[:,~rv];left[:,~lv]=right[:,~lv]
        self.low=np.minimum(right,left);self.high=np.maximum(right,left)

    def predict(self,delta):
        d=np.asarray(delta,float)
        if d.shape!=self.base.vector.shape or not np.isfinite(d).all():
            raise ValueError('complete finite joint step required')
        if np.any(abs(d)>self.radius):raise ValueError('joint step exceeds registered coordinate radius')
        unchecked=np.ones(len(d),bool);unchecked[self.columns]=False
        if np.any(d[unchecked]!=0):raise ValueError('joint step uses a fixed or unchecked stencil column')
        q=d[self.columns]
        for sign,side in ((1,'right'),(-1,'left')):
            if np.any((sign*q>0)&~self.stencil['sided_valid'][side]):
                raise ValueError('joint step uses a rejected source-domain stencil side')
        self.system._check_vector(self.base.vector+d)
        positive=np.maximum(q,0.);negative=np.minimum(q,0.)
        lower=self.base.values+self.low@positive+self.high@negative
        upper=self.base.values+self.high@positive+self.low@negative
        return dict(lower=lower,upper=upper,
            central=self.base.values+self.stencil['matrix']@q,
            complete_free_columns=bool(self.stencil['all_free_columns_evaluated']),
            nonlinear_enclosure_certified=False)

    def verify(self,delta,*,independent_full=False):
        """Check the actual complete geometry; NEVER accept on prediction alone.

        Source inequalities use their existing exact sign, not a caller's
        epsilon. Equality residuals remain raw mixed scales and cannot be
        certified by this diagnostic. Improving a rejected state is not a
        map acceptance, even if every modeled inequality currently passes.
        """
        predicted=self.predict(delta)
        trial=self.system.trial_vector(self.base,self.base.vector+np.asarray(delta,float))
        full_error=None
        if independent_full:
            full=self.system.evaluate(trial.vector)
            full_error=float(np.max(abs(full.values-trial.values),initial=0.))
            if full_error>1e-10:
                raise ValueError('atomic joint trial differs from independent full-state evaluation')
        iq=self.system.inequality_indices;eq=self.system.equality_indices
        before=self.base.values[iq];after=trial.values[iq]
        newly_bad=iq[(before>=0)&(after<0)]
        prediction_missed=iq[(predicted['lower'][iq]>=0)&(after<0)]
        outside=np.maximum(predicted['lower']-trial.values,trial.values-predicted['upper'])
        changes=abs(trial.values-self.base.values)
        scale=1+np.maximum.reduce([abs(predicted['lower']-self.base.values),
                                  abs(predicted['upper']-self.base.values),changes])
        return dict(snapshot=trial,report=dict(
            status='ATOMIC_JOINT_STEP_CHECKED_NOT_ACCEPTED_MAP',
            moved_columns=np.flatnonzero(np.asarray(delta)!=0).tolist(),
            recomputed_blocks=self.system.affected_blocks(np.flatnonzero(np.asarray(delta)!=0)),
            complete_free_columns=bool(self.stencil['all_free_columns_evaluated']),
            independent_full_max_difference=full_error,
            newly_violated_inequality_rows=newly_bad.tolist(),
            falsely_predicted_feasible_rows=prediction_missed.tolist(),
            actual_violated_inequalities=int(np.count_nonzero(after<0)),
            actual_minimum_mixed_slack=float(np.min(after,initial=np.inf)),
            actual_max_scaled_equality=float(np.max(abs(trial.values[eq]),initial=0.)),
            maximum_scaled_model_discrepancy=float(np.max(np.maximum(outside,0.)/scale,initial=0.)),
            all_modeled_inequalities_feasible=bool(np.all(after>=0)),
            original_rows_accounted=all(b['accounting']['all_original_rows_accounted']
                for (kind,_),b in trial.blocks.items() if kind=='parent' and b.get('accounting')),
            equality_feasibility_certified=False,domain_dynamics_surface_complete=False,
            optimization_ran=False,xodr_generated=False,export_allowed=False))
