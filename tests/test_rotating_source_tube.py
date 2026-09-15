import copy
import numpy as np
import pytest
from tests.test_source_boundary_joint import fixture
from tests.test_map_joint_source_certificate import centers
from spikes.road_boundary_family import solve_road
from spikes.map_lane_family import ManifestCenters
from spikes.rotating_source_tube import RotatingSourceTube


def setup(tmp_path,source=None):
    _,road=fixture(tmp_path);source=ManifestCenters(source or centers())
    model=solve_road(road,source,np.asarray,source_mode='centers',source_error_budget='absolute',
        all_source_vertices=True,source_certificate=True,model_only=True)
    x=np.zeros(model.nvar)
    for i,f in enumerate(model.families):x[f.columns]=-3.5*i
    return RotatingSourceTube(model,source),x


def test_angle_reprojects_raw_source_instead_of_reusing_old_tube(tmp_path):
    tube,x=setup(tmp_path);before=copy.deepcopy(tube.sources)
    assert max(tube.evaluate(x,0.))<-.99
    assert max(tube.evaluate(x,.02))>1.
    assert all(np.array_equal(tube.sources[k]['xy'],v['xy']) for k,v in before.items())
    assert len(tube.model.original.findall('planView/geometry'))==1


def test_active_envelope_jacobian_matches_independent_state_perturbation(tmp_path):
    tube,x=setup(tmp_path);x+=np.random.default_rng(52).normal(0,.01,len(x))
    angle=.00173;v,J,a=tube.evaluate(x,angle,True)
    direction=np.random.default_rng(31).normal(0,.1,len(x));dh=.004;eps=1e-6
    diff=(tube.evaluate(x+eps*direction,angle+eps*dh)-tube.evaluate(x-eps*direction,angle-eps*dh))/(2*eps)
    assert np.max(abs(diff-(J@direction+a*dh)))<1e-4


def test_spike_between_old_sampling_stations_stays_constrained(tmp_path):
    tube,x=setup(tmp_path,centers(spike=True))
    assert max(tube.evaluate(x,0.))>1.


def test_far_source_endpoint_cannot_be_silently_clipped(tmp_path):
    raw=centers();raw['lanes'][0]['geometry']['coordinates'][0][0]=-2.
    tube,x=setup(tmp_path,raw)
    assert max(tube.evaluate(x,0.))>4.


def test_all_outside_vertices_remain_checked_not_only_extreme_endpoints(tmp_path):
    raw=centers();raw['lanes'][0]['geometry']['coordinates']=[[-.2,-1.75],[-.1,-4.],[0.,-1.75],[120.,-1.75]]
    tube,x=setup(tmp_path,raw)
    assert max(tube.evaluate(x,0.))>5.


def test_axes_and_restoration_cannot_run_as_independent_local_repair():
    from scripts.rebuild_map_joint import geometry_stage
    with pytest.raises(ValueError,match='coupled'):geometry_stage(None,None,rotating_axes=True)


def test_nonfinite_or_reversed_chart_is_rejected(tmp_path):
    tube,x=setup(tmp_path)
    with pytest.raises(ValueError,match='finite'):tube.evaluate(x,np.nan)
    with pytest.raises(ValueError,match='folds'):tube.evaluate(x,np.pi/2)


def test_curve_bulge_between_raw_vertices_is_part_of_whole_segment_envelope(tmp_path):
    tube,x=setup(tmp_path)
    f=tube.model.families[1];x[f.columns.start+4]+=3.
    assert max(tube.evaluate(x,0.))>0.


def test_rotating_axis_cannot_silently_move_unhandled_road_objects():
    import xml.etree.ElementTree as ET
    from tests.test_joint_corridor_connectors import setup_network
    from spikes.joint_corridor_connectors import solve
    root,manifest=setup_network();road=root.find("road[@id='10']")
    ET.SubElement(ET.SubElement(road,'objects'),'object',id='test',s='10',t='5')
    before=ET.tostring(root)
    with pytest.raises(ValueError,match='objects'):solve(root,manifest,rotating_axes=True)
    assert ET.tostring(root)==before


def test_whole_tube_bounds_independent_dense_world_reconstruction(tmp_path):
    tube,x=setup(tmp_path);rng=np.random.default_rng(777);x+=rng.normal(0,.035,len(x));angle=.0012
    bound=(max(tube.evaluate(x,angle))+1)*tube.tolerance
    e=tube.tangent(angle);normal=np.array([-e[1],e[0]])
    worst=0.
    for record in tube.sources.values():
        for a,b in zip(record['xy'][:-1],record['xy'][1:]):
            for q in np.linspace(a,b,803):
                station=tube.length+(q-tube.anchor)@e
                span=min(record['spans'],key=lambda v:max(v[0]-station,0.,station-v[1]))
                s=float(np.clip(station,span[0],span[1]))
                lateral=sum(tube.model.families[k].basis(s)@x[tube.model.families[k].columns] for k in span[2:])/2
                world=tube.anchor+(s-tube.length)*e+lateral*normal
                worst=max(worst,float(np.linalg.norm(world-q)))
    assert worst<=bound+1e-9


def test_reversed_source_retains_original_vertex_and_segment_indices(tmp_path):
    raw=centers();raw['lanes'][0]['geometry']['coordinates'].reverse()
    tube,x=setup(tmp_path,raw)
    assert tube.sources['1']['original_vertex_indices']==[1,0]
    assert max(tube.evaluate(x,0.))<-.99


def test_tied_witnesses_keep_consistent_coefficient_and_angle_derivatives(tmp_path):
    # Infinitesimal coordinate rotation plus t(s) += (L-s)*angle leaves a
    # straight physical line unchanged. Differentiating max(error) separately
    # for angle/coefficients used to violate this by >160 in normalized units.
    tube,x=setup(tmp_path);v,J,a=tube.evaluate(x,0.,True);direction=np.zeros_like(x)
    for f in tube.model.families:
        greville=np.array([np.mean(f.knots[i+1:i+4]) for i in range(f.columns.stop-f.columns.start)])
        direction[f.columns]=tube.length-greville
    selected=np.array([l['kind']=='rotating-source' for l in tube.labels])
    assert np.max(abs((J@direction+a)[selected]))<1e-8
    assert selected.sum()>2  # All tied witnesses, not only one max per segment.
