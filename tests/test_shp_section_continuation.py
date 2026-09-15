"""Source forks: path centers do not identify the physical continuing lane."""
from types import SimpleNamespace

import numpy as np
import pytest

from mapforge.adapters.shp.ibd_reader import LaneRec, _known_width
from mapforge.ops.shp_to_xodr import _match_side_sections, _source_endpoint_widths


def row(sid, width, center):
    return {"l":SimpleNamespace(lane_pid=sid), "w0":width,"w1":width,
            "v0":center,"v1":center}


def section(lanes):
    return {"span":object(), "lanes":lanes}


def test_explicit_zero_is_not_replaced_with_nominal_width():
    lane = LaneRec('a','r',1,3500,1,60,np.array([[0.,0.],[20.,0.]]),
                   s_width_mm=0,e_width_mm=3500,s_width_known=True,e_width_known=True)
    assert _source_endpoint_widths(lane)==(0,3500)
    lane.s_width_known=False
    assert _source_endpoint_widths(lane)==(3500,3500)
    assert all(_known_width(v) for v in (0,'0','0.0',3500))
    assert not any(_known_width(v) for v in (None,'','bad',-1,float('nan')))


@pytest.mark.parametrize('reverse',[False,True])
def test_split_and_merge_keep_wide_continuation_not_nearer_zero_width_path(reverse):
    # Real node4: the zero-width branch center coincides with the parent's
    # path, while the full-width continuing lane's center is farther away.
    sa=section([row('parent',4.35,0.),row('other',3.45,-3.5)])
    sb=section([row('continue',4.35,1.5),row('branch',0.,0.),row('other2',3.45,-3.5)])
    src=SimpleNamespace(topo_out={'parent':['continue','branch'],'other':['other2']},topo_in={})
    specs=[sb,sa] if reverse else [sa,sb]
    got=_match_side_sections(specs,src,{})[0]
    assert got==({0:0,2:1} if reverse else {0:0,1:2})


def test_unused_declared_branch_is_not_attached_to_an_unrelated_near_lane():
    a=section([row('parent',3.5,0.),row('unrelated',3.5,-.2)])
    b=section([row('continue',3.5,0.),row('branch',0.,-.2)])
    src=SimpleNamespace(topo_out={'parent':['continue','branch']},topo_in={})
    assert _match_side_sections([a,b],src,{})==[{0:0}]


def test_missing_topology_keeps_explicit_inference_accounting():
    stats={}
    src=SimpleNamespace(topo_out={},topo_in={})
    assert _match_side_sections([section([row('a',3.5,0)]),section([row('b',3.5,.1)])],src,stats)==[{0:0}]
    assert stats['inferred_section_lane_matches']==1


def test_real_node4_source_width_and_declared_fork():
    from scripts.gen_all import shp_source
    source=shp_source()
    mother=source.lane('2023081117282865661')
    cont=source.lane('2023081117251325687')
    branch=source.lane('2023081117251327605')
    assert _source_endpoint_widths(mother)[1]==4350
    assert _source_endpoint_widths(cont)[0]==4350
    assert branch.s_width_known and _source_endpoint_widths(branch)[0]==0
    assert {cont.lane_pid,branch.lane_pid}<=set(source.topo_out[mother.lane_pid])
