import numpy as np
import pytest
from tests.test_long_source_seed import truth
from mapforge.ops.long_connector_chain import chain
from mapforge.ops.regular_ribbon_seed import initialize_regular_ribbon
from mapforge.ops.joint_connector_fit import basis_for


def test_regular_initialization_preserves_originals_basis_and_exact_end_jets():
    a,b,q,raw=truth();copies={k:v.copy() for k,v in raw.items()};q[0]+=.5
    refs=chain(q,a,b,(False,False))
    co,report=initialize_regular_ribbon(refs,a,b,raw);kn,B,_=basis_for(refs)
    assert len(co)==2*len(B.c) and report['maximum_end_jet_error']<1e-8
    assert report['actual_ribbon_width_minimum_m']>=.1-1e-8
    assert report['actual_ribbon_forward_minimum']>=.1-1e-8
    assert report['minimum_reference_span_m']>=6 and report['minimum_width_span_m']>=3
    assert report['qp']['original_constraints_all_rechecked']
    assert not report['production_accepted'] and not report['source_fidelity_accepted']
    assert all(np.array_equal(raw[k],v) for k,v in copies.items())
    # End-reference XY is still deliberately open. This initializer must not
    # translate it or pretend to have solved the shared endpoint condition.
    assert abs(refs[-1].XEnd-b['pose'][0])>.1


def test_regular_initializer_rejects_missing_source_or_short_model():
    a,b,q,raw=truth();refs=chain(q,a,b,(False,False))
    with pytest.raises(ValueError,match='three-field'):initialize_regular_ribbon(refs,a,b,{'center':raw['center']})
    q[0]=5.
    with pytest.raises(ValueError,match='fixed long'):initialize_regular_ribbon(chain(q,a,b,(False,False)),a,b,raw)


def test_inverted_source_end_edges_are_not_silently_repaired():
    a,b,q,raw=truth()
    a=dict(a,edges=dict(left=a['edges']['right'],right=a['edges']['left']))
    with pytest.raises(ValueError,match='QP failed|readback failed'):
        initialize_regular_ribbon(chain(q,a,b,(False,False)),a,b,raw)
