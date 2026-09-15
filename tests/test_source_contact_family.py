"""Why mutable source endpoints need a derivative-capable fixed model family."""
from unittest.mock import patch
import numpy as np
from mapforge.ops.joint_connector_fit import fit_joint,basis_for,coefficients_to_basis
from mapforge.ops.coupled_source_junction import center_endpoint_residual
from mapforge.ops.source_connector_ribbon import reference_samples
from spikes.measured_connector_caps import seed_chain,ribbon
from tests.test_source_junction_group import two_parent_problem
from mapforge.ops.joint_connector_fit import resolve_contact_caps
import pytest


def test_cap_choice_uses_permitted_parent_slope_not_only_initial_value():
    problem,_=two_parent_problem()
    for parent in problem.parents.values():
        for port in parent.maps:
            assert max(abs(parent.jets(port,np.zeros(parent.nvar))[[1,4]]))<1e-12
            assert parent.contact_slope_can_vary(port)
    a,b=problem.frames('100',problem.contact_jets(problem.initial,'100'))
    assert resolve_contact_caps(a,b)==(False,False)
    assert resolve_contact_caps(a,b,(True,True))==(True,True)
    with pytest.raises(ValueError):resolve_contact_caps(a,b,(True,))


def test_flat_seed_does_not_prove_a_mutable_parent_has_flat_contact_slopes():
    problem,root=two_parent_problem();x=problem.initial.copy()
    # Change permitted source width slope, without modifying the source model.
    sl=problem.parent_slices['10'];x[sl.start+1]=.004;x[sl.start+5]=-.002
    a,b=problem.frames('100',problem.contact_jets(x,'100'));errors=[]
    for capped in (False,True):
        cls=seed_chain(a,b,6. if capped else 0.,6. if capped else 0.)
        i=int(capped);q=np.r_[[c.length for c in cls],20*cls[i].KappaEnd,20*cls[i+1].KappaEnd]
        xy,n=reference_samples(cls,np.linspace(0,sum(c.length for c in cls),100))
        raw={name:xy+t*n for name,t in [('left',1.5),('right',-1.5),('center',0.)]}
        kn,co=ribbon(cls,a,b,raw,True);_,B,_=basis_for(cls)
        with patch('mapforge.ops.joint_connector_fit.needs_cap',return_value=capped):
            kernel=fit_joint(root.find("road[@id='100']"),a,b,raw,raw,q,
                initial_coefficients=coefficients_to_basis(kn,co,B),fair_world=True,_problem_only=True)
        result=kernel['evaluate'](kernel['unpack'](kernel['initial']))
        # Edges are individually exact in BOTH families.
        assert max(abs(result[4][:12]))<1e-7
        errors.append(max(abs(center_endpoint_residual(result,[a,b]))))
    assert errors[0]>1e-5
    assert errors[1]<1e-7
