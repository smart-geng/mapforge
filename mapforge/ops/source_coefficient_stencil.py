"""Exact all-witness coefficient stencils in ONE unchanged source chart.

Only a coefficient column has this affine source structure. This helper is
rebuilt for each new whole snapshot; chart/cut/knot columns MUST use the
full original-source evaluator. It neither freezes charts in the optimizer
nor differentiates only an arbitrary argmin at tied source witnesses.
"""
import numpy as np

from mapforge.ops.source_constraint_envelope import envelope_rows, ConstraintStructureError
from spikes.source_contact_fit import polynomial_bernstein


class SourceCoefficientStencil:
    def __init__(self,layout,state):
        model=state['model'];x=state['coefficients']
        # Validate full original identities/budget before taking a snapshot.
        baseline=layout.evaluate(state)
        groups,self.accounting=envelope_rows(model,x)
        rows=[];values=[];starts=[];cache={}
        for key in layout.keys:
            starts.append(len(values))
            for value,witness in groups[key]:
                if 'row' in witness:
                    row=model.C[witness['row']]
                else:
                    if witness['kind']!='regular-reference-chart':
                        raise ConstraintStructureError('unknown affine witness meaning')
                    feature=witness['features'][0];a,b=witness['span']
                    token=(feature,a,b)
                    if token not in cache:
                        cache[token]=polynomial_bernstein(
                            lambda s,d=0:model.expression(feature,s,d),a,b,3)
                    row=-model.reference_curvature*cache[token][witness['bernstein']]
                rows.append(row);values.append(value)
        self.rows=np.array(rows,float);self.slacks=np.array(values,float)
        self.starts=np.array(starts,int);self.E=model.E.copy();self.eq=baseline['equalities'].copy()
        self.baseline=baseline
        if not np.allclose(self.values(0,0.),np.r_[baseline['equalities'],baseline['inequalities']],rtol=0,atol=1e-12):
            raise ValueError('affine witnesses differ from complete source envelope')

    def values(self,column,delta):
        # Every witness participates in this min, including ties and witnesses
        # that become active during this finite coefficient displacement.
        iq=np.minimum.reduceat(self.slacks+delta*self.rows[:,column],self.starts)
        return np.r_[self.eq+delta*self.E[:,column],iq]


class ParentCoefficientStencils:
    """Snapshot-bound fast coefficient values plus exact port dependencies."""
    def __init__(self,system,base):
        self.system=system;self.base=base;self.parents={}
        for rid,parent in system.geometry.ports.parents.items():
            # Generic numerical fixtures/providers remain on the ordinary
            # evaluator, never silently assumed to have cubic-source structure.
            if not hasattr(parent,'coefficient_slice'):continue
            state=base.full['parents'][rid];model=state['model']
            jet_matrices={port:np.array([model.expression(key,state['stations'][port],j)
                for key in keys for j in range(3)]) for port,keys in parent.port_features.items()}
            self.parents[rid]=(SourceCoefficientStencil(system.parents[rid],state),jet_matrices)

    def evaluate(self,column,delta):
        s=self.system;base=self.base;g=s.geometry
        kind,rid=s.column_owner[column]
        if kind!='parent' or rid not in self.parents:return None
        parent=g.ports.parents[rid];local=column-g.ports.slices[rid].start
        if local<parent.coefficient_slice.start:return None
        j=local-parent.coefficient_slice.start
        v=base.vector.copy();v[column]+=delta;s._check_vector(v)
        affine,jets=self.parents[rid]
        for port,matrix in jets.items():
            changed=base.full['parents'][rid]['jets'][port]+delta*matrix[:,j]
            if changed[0]<changed[3]-1e-8:
                raise ValueError('physical source edges reversed at a shared port')
        # Exact basis zeros, NOT a threshold on observed finite differences.
        affected=tuple(cid for cid in s.dependents[rid]
            if any(port.road==rid and np.any(jets[port][:,j]!=0.)
                   for port in g.ports.graph.connections[cid]))
        values=base.values.copy();values[s.slices[('parent',rid)]]=affine.values(j,delta)
        if affected:
            parents=dict(base.full['parents'])
            parents[rid]=parent.evaluate_from_snapshot(v[g.ports.slices[rid]],base.full['parents'][rid])
            for cid in affected:
                turn=g.evaluate_turn(cid,v[g.turn_slices[cid]],parents)
                block=s._block(('turn',cid),dict(turns={cid:turn}))
                values[s.slices[('turn',cid)]]=np.r_[block['equalities'],block['inequalities']]
        if not np.isfinite(values).all():raise ValueError('nonfinite affine coefficient stencil')
        return dict(values=values,recomputed_blocks=[('parent',rid)]+[('turn',cid) for cid in affected],
            source_certificate_rows=affine.accounting,
            method='all-source-affine-witnesses-and-exact-port-basis-dependencies')
