import copy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.interpolate import BSpline, BPoly

from mapforge.ops.port_dependencies import revision
from mapforge.ops.source_contacts import compile_source_contacts
from mapforge.ops.source_roles import resolve_source_roles
from spikes.source_contact_fit import SourceBoundaryBlock, cubic_bernstein, polynomial_bernstein, exact_difference_max
from tests.test_source_contacts import source_fixture, reseal
from tests.test_source_roles import decision_for


def straight_fixture():
    root,source,scope,domain=source_fixture()
    # Synthetic two-source straight road, not changes to any real input.
    del scope['observations']['c'];del scope['boundaries']['B:cl']
    scope['occurrences']=[r for r in scope['occurrences'] if r['source_lane_id']!='c']
    features=domain['partition']['features'];del features['lane:c'];del features['boundary:B:cl']
    for f in features.values():
        f['source_lane_ids']=[s for s in f['source_lane_ids'] if s!='c']
        for p in f['parts']:
            for q in p['raw_vertices']:q[0]*=3
            p['length_m']*=3;p['source_vertex_s_m']=[s*3 for s in p['source_vertex_s_m']]
            for a in p['atoms']:a['source_s_m']=[s*3 for s in a['source_s_m']]
    for o in scope['observations'].values():
        o['source_max_speed_kmh']=60
        for row in o['raw_records']:
            for part in row['parts']:
                for q in part:q[0]*=3
    for o in scope['boundaries'].values():
        for row in o['records']:
            for part in row['parts']:
                for q in part:q[0]*=3
    root.find('road').set('length','60');root.find('road/planView/geometry').set('length','60')
    root.findall('road/lanes/laneSection')[1].set('s','30')
    scope['base_revision']=revision(root);reseal(scope,domain)
    source.rows=source.rows[:1];source.lanes_of=lambda _: [SimpleNamespace(lane_pid=s) for s in ('a','b')]
    contacts=compile_source_contacts(root,source,scope,domain)
    roles=resolve_source_roles(scope,domain,contacts,decision_for(contacts))
    return root,scope,domain,contacts,roles


def test_source_record_cuts_merge_without_adding_curve_variables_and_keep_shared_state():
    args=straight_fixture();before=copy.deepcopy(args[1:]);model=SourceBoundaryBlock(*args)
    assert len(model.owner)==4 and len(model.families)==2
    assert model.owner['boundary:B:al']==model.owner['boundary:B:bl']
    assert model.describe()['minimum_independent_span_m']>=15-1e-7
    # Same vector controls both original records, not independent endpoint stitching.
    x=np.zeros(model.nvar);x[model.families[model.owner['boundary:B:al']].columns]=2
    assert model.expression('boundary:B:al',29)@x==pytest.approx(2)
    assert model.expression('boundary:B:bl',31)@x==pytest.approx(2)
    x,phase=model.solve();assert x is not None and phase['status']=='FEASIBLE'
    audit=model.audit(x)
    assert audit['boundary_same_chart_max_m']<1e-6
    assert audit['exact_width_min_m']==pytest.approx(4,abs=1e-6)
    assert max(c['value'] for c in audit['physical_midpoint_dynamics_worst'])<1e-5
    assert not audit['export_allowed'] and not audit['independent_xodr_validation_ran']
    assert args[1:]==before
    _,necessary=model.necessary_source_preflight()
    assert necessary['status']=='FEASIBLE_NECESSARY_ONLY' and not necessary['all_point_acceptance']


@pytest.mark.parametrize('fault',['stale','short_span','loose_tube','changed_role'])
def test_shared_source_block_fails_closed(fault):
    args=list(straight_fixture());kwargs={}
    if fault=='stale':args[0].find('road').set('length','61')
    elif fault=='short_span':kwargs['min_span']=2
    elif fault=='loose_tube':kwargs['source_tol']=1
    else:args[-1]['feature_roles']['lane:a']['role']='ignored'
    with pytest.raises(ValueError):SourceBoundaryBlock(*args,**kwargs)


def test_exact_bernstein_and_extrema_detect_between_vertex_bulge():
    # cubic with zero endpoint errors, positive interior hump.
    knots=np.array([0,0,0,0,30,30,30,30],float)
    curve=BSpline(knots,[0,1,1,0],3)
    b=cubic_bernstein(lambda s,d=0:curve(s,d),0,30)
    assert b==pytest.approx([0,1,1,0])
    assert exact_difference_max(curve,np.array([[0,0],[30,0]]))==pytest.approx(.75)


def test_quintic_keeps_long_spans_and_exact_c2_not_unrequested_c4():
    args=straight_fixture();model=SourceBoundaryBlock(*args,degree=5)
    assert model.degree==5 and model.describe()['minimum_independent_span_m']>=15
    assert not model.describe()['direct_xodr_width_representation']
    f=model.families[0]
    for knot in np.unique(f.knots)[1:-1]:assert sum(f.knots==knot)==3
    x,phase=model.solve();assert x is not None
    a=model.audit(x)
    assert a['boundary_same_chart_max_m']<1e-6
    assert max(c['value'] for c in a['physical_midpoint_dynamics_worst'])<1e-5
    assert not a['export_allowed']


def test_quintic_bernstein_matches_independent_scipy_and_interior_extrema():
    controls=np.array([0.,0.,1.,1.,0.,0.]);bpoly=BPoly(controls[:,None],[0.,20.])
    spline=BSpline([0.]*6+[20.]*6,controls,5)
    actual=polynomial_bernstein(lambda s,d=0:spline(s,d),0,20,5)
    assert actual==pytest.approx(controls,abs=1e-12)
    ss=np.linspace(0,20,1001)
    assert spline(ss)==pytest.approx(bpoly(ss),abs=1e-12)
    assert exact_difference_max(spline,np.array([[0.,0.],[20.,0.]]))==pytest.approx(.625,abs=1e-10)
    # Certificate subintervals retain the same polynomial, not new degrees of freedom.
    sub=polynomial_bernstein(lambda s,d=0:spline(s,d),3,16,5)
    assert BPoly(sub[:,None],[3.,16.])(ss[(ss>=3)&(ss<=16)])==pytest.approx(spline(ss[(ss>=3)&(ss<=16)]))


def test_basis_degree_cannot_be_increased_without_bound():
    with pytest.raises(ValueError):SourceBoundaryBlock(*straight_fixture(),degree=9)
