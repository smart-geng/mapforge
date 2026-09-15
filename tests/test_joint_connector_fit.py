import xml.etree.ElementTree as ET
import numpy as np
import pytest
from tests.test_junction_edges import network
from spikes.measured_connector_caps import frames,needs_cap,seed_chain
from mapforge.ops.source_connector_ribbon import reference_samples
from mapforge.ops.joint_connector_fit import fit_joint
from scripts.internal_edge_jets import audit


def test_joint_solver_keeps_complete_end_states_without_source_mutation():
    root=network(end_width=3.)
    end=root.find("road[@id='11']/planView/geometry")
    end.set('x','55');end.set('y','35');end.set('hdg',str(np.pi/2))
    road=root.find("road[@id='100']");a,b=frames(root,road)
    required=(needs_cap(a),needs_cap(b));cls=seed_chain(a,b,6. if required[0] else 0.,6. if required[1] else 0.)
    first=int(required[0]);q=np.r_[[c.length for c in cls],cls[first].KappaEnd*20,cls[first+1].KappaEnd*20]
    # Analytic valid truth. The old flat-spline varying-width fixture had
    # negative width (-13m); it cannot serve as an expected-success source.
    s=np.linspace(0,sum(c.length for c in cls),400);xy,normal=reference_samples(cls,s)
    raw={k:xy+t*normal for k,t in (('left',1.5),('right',-1.5),('center',0.))}
    before=ET.tostring(root);copy={k:v.copy() for k,v in raw.items()}
    new,report=fit_joint(road,a,b,raw,raw,q)
    assert ET.tostring(root)==before
    assert all(np.array_equal(raw[k],v) for k,v in copy.items())
    assert report['status']=='GEOMETRY_REVIEW_CANDIDATE',report
    assert report['production_accepted'] is False
    assert report['joint_variables']>len(q)
    assert min(report['primitive_lengths_m'])>=6.-1e-7
    assert report['minimum_width_record_span_m']>=3.-1e-7
    assert report['construction_evaluation_step_m']==.1
    assert report['construction_source_margin_m']==.02
    assert report['final_source_thresholds_unchanged']
    assert report['initial_coefficients_supplied'] is False
    assert report['world_fairing_enabled'] is False
    standalone=ET.Element('OpenDRIVE');standalone.append(new)
    assert audit(standalone)['status']=='PASS'


@pytest.mark.parametrize('options',[
    dict(evaluation_step=.5),dict(evaluation_step=0.),dict(evaluation_step=float('nan')),
    dict(source_margin=.001),dict(source_margin=-.1),dict(source_margin=float('inf'))])
def test_construction_cannot_restore_coarse_grid_or_relax_final_gate(options):
    with pytest.raises(ValueError):fit_joint(None,None,None,None,None,None,**options)
