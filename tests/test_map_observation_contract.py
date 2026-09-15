import numpy as np
import pytest
from scripts.map_observation_contract import normal_cut,project_point,paths_between


def test_normal_width_is_not_chart_transverse_width():
    t=np.array([1.,1.])/np.sqrt(2);n=np.array([-t[1],t[0]])
    edges=[np.array([-10*t+d*n,10*t+d*n]) for d in (-1.75,1.75)]
    cut=normal_cut(np.zeros(2),t,edges)
    assert cut['normal_width_m']==pytest.approx(3.5)
    # Re-cut the SAME boundaries normal to a different reference chart.
    # Their vertical (chart t) separation is 3.5*sqrt(2), NOT 3.5.
    chart_cut=normal_cut(np.zeros(2),np.array([1.,0.]),edges)
    assert chart_cut['normal_width_m']==pytest.approx(3.5*np.sqrt(2))
    assert chart_cut['normal_width_m']>cut['normal_width_m']
    assert cut['center_outside_m']==0


def test_outside_path_and_missing_boundary_are_not_repaired():
    edges=[np.array([[0.,d],[10.,d]]) for d in (2.,5.5)]
    cut=normal_cut(np.array([5.,0.]),np.array([1.,0.]),edges)
    assert cut['normal_width_m']==pytest.approx(3.5) and cut['center_outside_m']==2
    assert normal_cut(np.zeros(2),np.array([1.,0.]),edges[:1])['status']=='UNAVAILABLE'
    assert normal_cut(np.array([-1.,0.]),np.array([1.,0.]),edges)['status']=='UNAVAILABLE'


def test_projection_keeps_original_segment_index_and_topology_all_paths():
    result=project_point(np.array([7.,2.]),np.array([[0.,0.],[0.,0.],[10.,0.]]))
    assert result['source_segment']==1 and result['fraction']==pytest.approx(.7)
    graph={'a':['b','c'],'b':['d'],'c':['d']}
    result=paths_between({'a'},{'d'},graph,lambda _:True)
    assert result['paths']==[['a','b','d'],['a','c','d']]
    assert result['identity_confirmed'] is False
    assert paths_between({'a'},{'d'},graph,lambda _:True,limit=1)['status']=='SEARCH_LIMIT'
    long_graph={str(i):[str(i+1)] for i in range(25)}
    assert paths_between({'0'},{'25'},long_graph,lambda _:True)['status']=='SEARCH_LIMIT'


def test_normal_collinear_with_boundary_is_ambiguous():
    edges=[np.array([[5.,1.],[5.,2.],[6.,3.]]),np.array([[0.,-2.],[10.,-2.]])]
    assert normal_cut(np.array([5.,0.]),np.array([1.,0.]),edges)['status']=='UNAVAILABLE'
