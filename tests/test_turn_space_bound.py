import math
import pytest
from spikes.turn_space_bound import audit


def test_tight_turn_cannot_be_solved_by_more_primitives():
    r=audit((0.,0.,0.),(8.,8.,math.pi/2))
    assert r['status']=='VIOLATES_NECESSARY_BOUND'
    assert r['quadrature_error_estimate_m']<1e-6
    assert r['global_infeasibility_claimed'] is False


def test_open_space_does_not_prove_feasibility():
    r=audit((0.,0.,0.),(30.,30.,math.pi/2))
    assert r['status']=='NOT_EXCLUDED'


def test_rotation_and_turn_direction_do_not_change_bound():
    a=audit((0.,0.,0.),(8.,8.,math.pi/2))
    b=audit((0.,0.,0.),(8.,-8.,-math.pi/2))
    c=audit((3.,5.,math.pi/2),(-5.,13.,math.pi))
    for r in (b,c):
        assert r['deficit_m']==pytest.approx(a['deficit_m'],abs=1e-7)


def test_other_endpoint_models_are_not_falsely_rejected():
    assert audit((0.,0.,0.),(10.,0.,0.))['status']=='NOT_APPLICABLE'
    assert audit((0.,0.,0.),(8.,8.,math.pi/2),k0=.01)['status']=='NOT_APPLICABLE'
