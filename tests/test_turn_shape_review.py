import numpy as np
import json
from scripts.review_turn_shape import reverse_turn


def test_single_turn_wave_is_not_accepted_because_end_headings_match():
    source=np.linspace(0,-np.pi/2,50)
    target=np.r_[np.linspace(0,-.5,20),np.linspace(-.5,-.2,10),np.linspace(-.2,-np.pi/2,20)]
    result=reverse_turn(source,target)
    assert result['status']=='EXTRA_REVERSE_TURN_DETECTED'
    assert result['extra_reverse_turn_deg']>17.
    json.dumps(result,allow_nan=False)


def test_legitimate_source_s_bend_is_not_automatically_forbidden():
    source=np.r_[np.linspace(0,-.8,20),np.linspace(-.8,-.1,20)]
    assert reverse_turn(source,source)['status']=='REQUIRES_GENERAL_SHAPE_REVIEW'


def test_monotone_turn_across_angle_wrap_has_no_false_wave():
    h=np.linspace(3.,4.6,40)
    r=reverse_turn((h+np.pi)%(2*np.pi)-np.pi,h)
    assert r['status']=='NO_EXTRA_REVERSE_TURN_OBSERVED'
    assert abs(r['extra_reverse_turn_deg'])<1e-8
