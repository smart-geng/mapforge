import math
import numpy as np
import pytest
import xml.etree.ElementTree as ET
from spikes.one_arc_source_ribbon import fit,turn_chart
from scripts.review_measured_ribbon import target_curves


def test_single_arc_chart_exact_circular_truth_not_general_fallback():
    radius=20.;theta=np.linspace(0,math.pi/2,101)
    def curve(r):return np.c_[r*np.sin(theta),radius-r*np.cos(theta)]
    raw={'left':curve(18.),'center':curve(20.),'right':curve(22.)}
    def frame(i,h):
        return dict(pose=(*raw['center'][i],h),k=1/radius,dk=0.,edges={side:
            dict(x=raw[side][i,0],y=raw[side][i,1],heading=h,curvature=1/r)
            for side,r in [('left',18.),('right',22.)]})
    a,b=frame(0,0.),frame(-1,math.pi/2)
    road=ET.fromstring('''<road id="100" length="30"><planView/>
        <lanes><laneSection s="0"><center><lane id="0" type="none"/></center><right><lane id="-1" type="driving">
        <link/><width sOffset="0" a="4" b="0" c="0" d="0"/><speed sOffset="0" max="16.6666666667" unit="m/s"/>
        </lane></right></laneSection></lanes></road>''')
    before=ET.tostring(road);new,report=fit(road,a,b,raw,tube=.1)
    assert new is not None and ET.tostring(road)==before
    assert report['reference_primitives']==1 and report['minimum_width_span_m']>=3
    assert not report['production_accepted']
    assert len(new.findall('planView/geometry/arc'))==1
    target=target_curves(new,.1)
    for side,r in [('left',18.),('center',20.),('right',22.)]:
        assert np.linalg.norm(target[side]-[0.,20.],axis=1)==pytest.approx(r,abs=1e-7)
    with pytest.raises(ValueError,match='nonparallel'):turn_chart(a,a,raw)
