"""Long layout admission: immutable reference, exact objective, no silent edit."""
from dataclasses import replace

import numpy as np
import pytest
import yaml

from scripts.check_outer_event_control import inputs, DECISION
from mapforge.repair_web.outer_event_layout import LayoutFairing, proposals, reference_objective, migration_error


@pytest.fixture(scope='module')
def context():
    event, control = inputs()
    roles = {(d['source_lane_id'], d['contact']) for d in
             yaml.safe_load(DECISION.read_text(encoding='utf8'))['decisions']
             if d['physical_authority']=='original_left_right_boundaries'
             and d['lane_path_role']=='movement_path_observation'}
    choices = proposals(control)
    return event, control, roles, choices


@pytest.fixture(scope='module')
def layout(context):
    _, control, roles, choices = context
    return LayoutFairing(control, choices[0][1], approved_roles=roles)


def test_two_source_deterministic_long_layouts_only(context):
    event,control,_,choices=context
    assert [n for n,_ in choices]==['source-plateau-long-support','balanced-long-support']
    assert proposals(control)==choices
    for _,scope in choices:
        assert scope.knots[:7]==event.scope.knots[:7]
        assert scope.knots[-1]==event.end and len(scope.knots)==10
        assert min(np.diff(scope.knots))>=6.
        assert scope.births==event.scope.births
    assert choices[0][1].knots[-2]==control.trace['st'][1,0]


def test_projection_is_not_noop_and_never_silently_applied(layout,context):
    assert layout.migration['max_m']>.09  # genuine representational difference
    assert layout.unchanged_xml() is context[1].reference
    with pytest.raises(ValueError,match='Zero edit'):layout.solve(0.)
    assert not layout.preflight['projected_seed_is_map']
    assert not layout.preflight['map_accepted']


def test_new_basis_has_all_source_ownership_and_port_constraints(layout,context):
    original=context[0];new=layout.event
    assert new.inventory==original.inventory and new.paths==original.paths
    assert new.nvar==original.nvar==54 and new.Z.shape[1]==24
    assert max(abs(new.E@layout.reference-new.e))<1e-8
    def domains(event):
        result={}
        for trace,lo,hi,_ in event.source_cells():
            key=(trace['key'],trace['record'],trace['part'],trace['edge'])
            result[key]=result.get(key,0.)+hi-lo
        return result
    a,b=domains(original),domains(new)
    assert a.keys()==b.keys()
    assert list(a.values())==pytest.approx(list(b.values()),abs=1e-10)
    assert layout.control.reference_t==context[1].reference_t
    assert np.array_equal(layout.control.budgets,context[1].budgets)


def test_objective_keeps_original_xml_not_projected_seed(layout):
    # The true objective contains a nonzero residual at the seed. Recentring
    # at it would silently replace the old shape/minimum-change definition.
    residual=layout.M@layout.reference-layout.objective_reference
    assert np.dot(residual,residual)==pytest.approx(layout.preflight['projection_energy'],abs=1e-10)
    assert np.linalg.norm(residual)>1.
    assert np.max(abs((layout.M@layout.event.Z).T@residual))<1e-9


def test_piecewise_objective_matches_independent_polynomial_integral(layout):
    from numpy.polynomial import Polynomial as P
    from mapforge.repair_web.outer_event_control import _road
    from mapforge.repair_web.outer_event_layout import xml_cuts
    from scripts.check_outer_event_written import boundary_poly
    event=layout.event;road=_road(layout.control.reference,event);energy=0.
    for edge,bs in event.splines.items():
        cuts=sorted(set(xml_cuts(road,min(bs.t),max(bs.t)))|set(bs.t))
        for lo,hi in zip(cuts,cuts[1:]):
            delta=P(event.power(edge,lo)@layout.reference-boundary_poly(road,edge,lo))
            integral=(delta*delta+100*delta.deriv()*delta.deriv()).integ()
            energy+=integral(hi-lo)-integral(0.)
    assert energy==pytest.approx(layout.preflight['projection_energy'],abs=1e-10)


def test_original_basis_exact_recovery_control(context):
    event,control,_,_=context
    M,y=reference_objective(event,control.reference)
    assert np.linalg.norm(M@control.current-y)<1e-9
    assert migration_error(event,control.reference,control.current)['max_m']<1e-9


@pytest.mark.parametrize('change', ['birth','chart','tolerance','short','count','first','end','nan'])
def test_scope_widening_or_silent_semantic_edit_rejected(context,change):
    event,control,roles,choices=context
    scope=choices[0][1];knots=list(scope.knots)
    if change=='birth':scope=replace(scope,births=((3,131.0543),(4,152.)))
    elif change=='chart':scope=replace(scope,road='12')
    elif change=='tolerance':scope=replace(scope,source_tolerance_m=.76)
    elif change=='short':knots[7]=knots[6]+1;scope=replace(scope,knots=tuple(knots))
    elif change=='count':scope=replace(scope,knots=scope.knots+(210.,))
    elif change=='first':knots[1]+=1;scope=replace(scope,knots=tuple(knots))
    elif change=='end':knots[-1]+=1;scope=replace(scope,knots=tuple(knots))
    else:knots[7]=float('nan');scope=replace(scope,knots=tuple(knots))
    with pytest.raises(ValueError):LayoutFairing(control,scope,approved_roles=roles)


def test_missing_existing_role_approval_rejected(context):
    _,control,_,choices=context
    with pytest.raises(ValueError,match='approval'):LayoutFairing(control,choices[0][1],approved_roles=set())


def test_time_budget_no_candidate(layout):
    state,report=layout.solve(-.125,max_seconds=1e-12)
    assert state is None and report['status']=='TIME_BUDGET_EXHAUSTED'
    assert report['original_xml_objective'] and not report['map_accepted']
