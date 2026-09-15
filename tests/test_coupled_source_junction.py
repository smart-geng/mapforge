import copy
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from mapforge.ops.arc_source_chart import ArcChart
from mapforge.ops.coupled_source_junction import SourceParentState, CoupledSourceJunction
from mapforge.ops.port_dependencies import PortDependencies, LanePort
from spikes.connector_cross_section import endpoint_frame
from tests.test_junction_edges import network


def parent_fixture(curvature=.002, varying=True):
    root = network(end_width=3.)
    road = root.find("road[@id='10']")
    geometry = road.find('planView/geometry')
    geometry.remove(geometry.find('line')); ET.SubElement(geometry, 'arc', curvature=str(curvature))
    x = np.array([1.5,.005,.0001,0.,-1.5,.002,.0001,0.]) if varying else np.array([1.5,0.,0.,0.,-1.5,0.,0.,0.])
    for key, value in zip('abcd', x[:4]): road.find('lanes/laneOffset').set(key, str(value))
    for key, value in zip('abcd', x[:4]-x[4:]): road.find('lanes/laneSection/right/lane/width').set(key, str(value))
    def expression(key, station, derivative=0):
        row = np.zeros(8); shift = 0 if key == 'left' else 4
        for i in range(4):
            basis = np.eye(4)[i]
            row[shift+i] = np.polynomial.polynomial.polyval(station, np.polynomial.polynomial.polyder(basis, derivative))
        return row
    model = SimpleNamespace(nvar=8, E=np.empty((0,8)), C=np.r_[np.eye(8),-np.eye(8)],
        lower=np.full(16,-10.), road='10', expression=expression,
        axis=ArcChart((0.,0.),0.,curvature), reference_curvature=curvature)
    compiled = dict(chart_start_m=0.,chart_end_m=20.,lane_ledger=[dict(section=0,lane=-1,
        left='left',right='right',chart_s=[0.,20.])])
    return root, SourceParentState(model,x,compiled)


@pytest.mark.parametrize('forward',[False,True])
@pytest.mark.parametrize('contact',['start','end'])
def test_parent_coefficient_map_matches_actual_xml_all_edge_and_center_jets(forward,contact):
    root,parent = parent_fixture()
    port=LanePort('10',contact,-1)
    actual=endpoint_frame(root.find("road[@id='10']"),-1,contact,forward)
    direct=parent.frame(port,parent.jets(port,np.zeros(parent.nvar)),forward)
    np.testing.assert_allclose(direct['pose'],actual['pose'],atol=1e-12)
    for field in ('left','right','center'):
        a=actual['center'] if field=='center' else actual['edges'][field]
        b=direct['center'] if field=='center' else direct['edges'][field]
        for key in ('x','y','heading','curvature'): assert a[key]==pytest.approx(b[key],abs=1e-12)


def double_graph(root):
    road=copy.deepcopy(root.find("road[@id='100']"));road.set('id','101');root.append(road)
    j=root.find('junction');conn=copy.deepcopy(j.find('connection'))
    conn.set('id','1');conn.set('connectingRoad','101');j.append(conn)
    graph=PortDependencies(root);graph.validate_junction_table();return graph


def test_incomplete_dependency_set_rejected_before_optimization():
    root,parent=parent_fixture();graph=double_graph(root)
    with pytest.raises(ValueError,match='every dependent turn'):
        CoupledSourceJunction(graph,{'10':parent},{'100':{}})


def test_one_parent_change_updates_both_incident_ports_without_source_mutation():
    root,parent=parent_fixture();graph=double_graph(root)
    kernels={rid:dict(initial=np.zeros(1),bounds=np.array([[-1.,1.]])) for rid in graph.connections}
    problem=CoupledSourceJunction(graph,{'10':parent},kernels)
    before=problem.initial.copy();after=before.copy();after[0]=.01
    for rid in ('100','101'):
        old=problem.frames(rid,problem.contact_jets(before,rid))
        new=problem.frames(rid,problem.contact_jets(after,rid))
        assert abs(old[0]['edges']['left']['y']-new[0]['edges']['left']['y'])>.009
        assert old[1]==new[1]
    assert np.array_equal(problem.initial,before)
    assert parent.x0[0]==1.5


def real_kernel_fixture():
    from spikes.measured_connector_caps import frames,seed_chain,needs_cap,ribbon
    from mapforge.ops.joint_connector_fit import fit_joint,basis_for,coefficients_to_basis
    from mapforge.ops.source_connector_ribbon import reference_samples
    root,parent=parent_fixture(curvature=0.,varying=False)
    g=root.find("road[@id='11']/planView/geometry")
    g.set('x','55');g.set('y','35');g.set('hdg',str(np.pi/2))
    road=root.find("road[@id='100']");a,b=frames(root,road);caps=(needs_cap(a),needs_cap(b))
    cls=seed_chain(a,b,6. if caps[0] else 0.,6. if caps[1] else 0.);i=int(caps[0])
    q=np.r_[[c.length for c in cls],20*cls[i].KappaEnd,20*cls[i+1].KappaEnd]
    xy,n=reference_samples(cls,np.linspace(0,sum(c.length for c in cls),100))
    raw={k:xy+t*n for k,t in [('left',1.5),('right',-1.5),('center',0.)]}
    kn,co=ribbon(cls,a,b,raw,True);_,B,_=basis_for(cls)
    kernel=fit_joint(road,a,b,raw,raw,q,initial_coefficients=coefficients_to_basis(kn,co,B),
        fair_world=True,_problem_only=True)
    return root,parent,kernel


def test_real_kernel_block_jacobian_matches_shared_variable_perturbation():
    root,parent,kernel=real_kernel_fixture()
    graph=PortDependencies(root)
    problem=CoupledSourceJunction(graph,{'10':parent},{'100':kernel})
    x=problem.initial.copy();x[0]+=.002
    before=problem.residual(x);J=problem.jacobian(x)
    # Constant and linear coefficient modes affect the real geometric residuals,
    # not a mocked connector objective. Direction respects finite-difference scale.
    step=1e-7;direction=np.zeros_like(x);direction[0]=.3;direction[1]=.01
    numeric=(problem.residual(x+step*direction)-before)/step
    np.testing.assert_allclose(J@direction,numeric,atol=2e-3,rtol=3e-3)
    assert J.shape==(len(before),len(x)) and np.linalg.norm(J@direction)>0
    assert problem.summary(x)['simultaneous_parent_connector_variables']


def test_fixed_original_endpoint_obstruction_is_detected_before_solve():
    root,parent,kernel=real_kernel_fixture()
    graph=PortDependencies(root);problem=CoupledSourceJunction(graph,{'10':parent},{'100':kernel})
    with pytest.raises(ValueError,match='original fixed-port source'):
        problem.fixed_source_obstructions()
    frame=problem.fixed['100',1]
    tangent=np.array([np.cos(frame['pose'][2]),np.sin(frame['pose'][2])])
    curves={}
    for side in ('left','right'):
        edge=frame['edges'][side];p=np.array([edge['x'],edge['y']])
        curves[side]=np.array([p,p+20*tangent])
    kernel['source_tails']=[dict(role='successor',source_lane_id='synthetic-exit',curves=curves)]
    assert problem.fixed_source_obstructions()==[]
    normal=np.array([-tangent[1],tangent[0]])
    curves['left']=curves['left']+.6*normal
    conflicts=problem.fixed_source_obstructions()
    assert len(conflicts)==1 and conflicts[0]['parent']=='11'
    assert conflicts[0]['fixed_endpoint_source_distance_m']==pytest.approx(.6)
    assert not conflicts[0]['global_impossibility_proven']
