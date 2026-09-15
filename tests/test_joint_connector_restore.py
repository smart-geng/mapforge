import copy
import xml.etree.ElementTree as ET
import numpy as np
import pytest

from tests.test_junction_edges import network
from spikes.measured_connector_caps import frames, needs_cap, seed_chain
from mapforge.ops.source_connector_ribbon import reference_samples
from mapforge.ops.joint_connector_fit import fit_joint
from mapforge.ops.joint_connector_restore import restore
from scripts.internal_edge_jets import audit
from scripts.review_measured_ribbon import target_curves, fidelity


def truth():
    root=network(end_width=3.)
    end=root.find("road[@id='11']/planView/geometry")
    end.set('x','55');end.set('y','35');end.set('hdg',str(np.pi/2))
    road=root.find("road[@id='100']");a,b=frames(root,road)
    caps=(needs_cap(a),needs_cap(b))
    cls=seed_chain(a,b,6. if caps[0] else 0.,6. if caps[1] else 0.)
    first=int(caps[0]);q=np.r_[[c.length for c in cls],cls[first].KappaEnd*20,cls[first+1].KappaEnd*20]
    xy,normal=reference_samples(cls,np.linspace(0,sum(c.length for c in cls),400))
    raw={k:xy+t*normal for k,t in (('left',1.5),('right',-1.5),('center',0.))}
    new,report=fit_joint(road,a,b,raw,raw,q)
    return new,a,b,raw,report


def test_projection_repairs_numeric_jets_without_adding_primitives_or_moving_parent():
    road,a,b,raw,report=truth();before=ET.tostring(road);parents=copy.deepcopy((a,b))
    coefficients=np.asarray(report['joint_coefficients'])+np.random.default_rng(9).normal(0,1e-5,len(report['joint_coefficients']))
    saved=coefficients.copy()
    new,result=restore(road,a,b,report['shape_parameters'],coefficients)
    assert ET.tostring(road)==before and (a,b)==parents
    assert np.array_equal(coefficients,saved)
    assert result['projection_residual']<1e-9
    assert not result['geometry_accepted'] and not result['production_accepted']
    assert len(new.findall('planView/geometry'))==len(road.findall('planView/geometry'))
    assert min(result['primitive_lengths_m'])>=6.-1e-8
    assert result['minimum_width_record_span_m']>=3.-1e-8
    root=ET.Element('OpenDRIVE');root.append(new)
    assert audit(root)['status']=='PASS'
    errors=fidelity(raw,target_curves(new,.05))
    assert max(v['max_m'] for field in errors.values() for v in field.values())<.01


@pytest.mark.parametrize('value',[[1.,2.], [float('nan')]*20])
def test_projection_rejects_missing_or_nonfinite_same_basis_coefficients(value):
    road,a,b,raw,report=truth()
    with pytest.raises(ValueError,match='same-basis'):
        restore(road,a,b,report['shape_parameters'],value)
