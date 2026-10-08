"""Actual XML coverage of concavity, holes, short events and sweep topology."""
from copy import deepcopy
from itertools import count
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from shapely.affinity import affine_transform
from shapely.geometry import MultiPolygon, Point, Polygon, box

from mapforge.adapters.opendrive import writer as W
from mapforge.workbench import source_surface_cells as C


def write_actual(tmp_path, geometry, axis=(1,0)):
    doc=W.XodrDoc("source cells")
    proof=C.append_polygon(doc,geometry,1,count(50),preferred_axis=axis,
                           provenance={"source_refs":[{"layer":"original","record_index":7}]})
    path=tmp_path/"cells.xodr"
    doc.write(path)
    root=ET.parse(path).getroot()
    actual=C.written_geometry(root,proof["road_ids"])
    audit=C.audit_written(root,proof)
    assert audit["status"]=="PASS",audit
    assert audit["passed"] and audit["duplicate_station_count"]==0
    assert audit["minimum_written_width_m"] is None or audit["minimum_written_width_m"]>=0
    assert proof["complexity"]["sampling_grid_events"]==0
    assert proof["complexity"]["endpoint_padding_m"]==0
    for road in root.findall("road"):
        assert len(road.findall("planView/geometry"))==1
        assert road.find("planView/geometry/line") is not None
        assert len(road.findall("lanes/laneSection"))==1
        assert road.find("link") is None
        assert not road.findall("lanes/laneSection/*/lane/link")
        assert road.find("lanes/laneSection/right/lane").get("type")=="restricted"
    return root,proof,actual


def test_rectangle_is_one_long_road_not_vertex_or_sample_fragments(tmp_path):
    shape=Polygon([(0,0),(1,0),(2,0),(30,0),(30,8),(2,8),(1,8),(0,8)])
    root,proof,actual=write_actual(tmp_path,shape)
    assert proof["complexity"]["line_roads"]==1
    assert proof["complexity"]["width_records"]==3
    assert actual.symmetric_difference(shape).area==0
    assert float(root.find("road").get("length"))==30


def test_concave_c_keeps_separated_transverse_intervals(tmp_path):
    shape=Polygon([(0,0),(10,0),(10,2),(2,2),(2,8),(10,8),(10,10),(0,10)])
    _,proof,actual=write_actual(tmp_path,shape)
    assert proof["complexity"]["line_roads"]==3
    assert not actual.covers(Point(6,5))
    assert actual.intersection(box(2.1,2.1,9.9,7.9)).is_empty
    assert actual.symmetric_difference(shape).area==0
    assert any(t["split_intervals"]==1 for t in proof["natural_topology_events"])


def test_hole_is_not_reduced_to_exterior_or_filled(tmp_path):
    hole=box(3,3,8,7)
    shape=Polygon(box(0,0,12,10).exterior.coords,[hole.exterior.coords])
    _,proof,actual=write_actual(tmp_path,shape)
    assert proof["source_holes"]==1
    assert len(actual.interiors)==1
    assert actual.intersection(hole).area==0
    assert actual.symmetric_difference(shape).area==0
    assert proof["complexity"]["line_roads"]==4


def test_multiple_holes_split_and_merge_at_same_station(tmp_path):
    holes=[box(3,2,8,4),box(3,6,8,8)]
    shape=Polygon(box(0,0,12,10).exterior.coords,[h.exterior.coords for h in holes])
    _,proof,actual=write_actual(tmp_path,shape)
    assert len(actual.interiors)==2
    assert proof["complexity"]["line_roads"]==5
    assert not actual.covers(Point(5,3)) and not actual.covers(Point(5,7))
    assert actual.symmetric_difference(shape).area==0
    split=next(t for t in proof["natural_topology_events"] if t["split_intervals"])
    assert split["before_intervals"]==1 and split["after_intervals"]==3


def test_disconnected_and_zero_width_contact_components_are_not_joined(tmp_path):
    shape=MultiPolygon([box(0,0,5,2),box(5,2,8,5),box(0,8,8,10)])
    assert shape.is_valid
    _,proof,actual=write_actual(tmp_path,shape)
    assert proof["source_components"]==3
    assert proof["complexity"]["line_roads"]==3
    assert not actual.covers(Point(3,5))
    assert actual.symmetric_difference(shape).area==0


def test_vertical_step_starts_new_cell_instead_of_diagonal_fill(tmp_path):
    shape=Polygon([(0,0),(3,0),(3,-2),(8,-2),(8,4),(0,4)])
    _,proof,actual=write_actual(tmp_path,shape)
    assert proof["complexity"]["line_roads"]==2
    assert any(t["vertical_boundary_steps"]==1 for t in proof["natural_topology_events"])
    assert not actual.covers(Point(2.999,-1))
    assert actual.covers(Point(3.001,-1))
    assert actual.symmetric_difference(shape).area==0


@pytest.mark.parametrize("axis",[(1,0),(-1,0),(0,1),(0,-1),(3,7)])
def test_oblique_fixed_axes_use_actual_serialized_heading(tmp_path,axis):
    shape=Polygon([(1.1,2.2),(12.8,1.7),(15.6,7.4),(9.2,9.1),(8.7,5.2),(3.2,6.8)])
    _,proof,actual=write_actual(tmp_path,shape,axis)
    assert proof["numeric_serialization_band_m"]<1e-6
    assert actual.symmetric_difference(shape).area<1e-5
    assert shape.buffer(1e-6).covers(actual) and actual.buffer(1e-6).covers(shape)


def test_rotated_parallel_vertex_pairs_normalize_only_binary_roundoff(tmp_path):
    angle=.73456789;c,s=math.cos(angle),math.sin(angle)
    source=Polygon([(0,0),(4,0),(10,0),(10,3),(4,3),(0,3)])
    shape=affine_transform(source,[c,-s,s,c,20.2,-10.7])
    _,proof,actual=write_actual(tmp_path,shape,(c,s))
    assert proof["complexity"]["line_roads"]==1
    assert proof["complexity"]["width_records"]==2
    assert proof["complexity"]["minimum_width_event_span_m"]>3.9
    for adjustment in proof["numerical_station_normalizations"]:
        assert adjustment["max_station_change_m"]<1e-12
        lo,hi=adjustment["common_roundoff_interval_m"]
        assert lo<=adjustment["normalized_station_m"]<=hi
    assert actual.symmetric_difference(shape).area<1e-5


def test_real_short_event_is_kept_and_reported_for_existing_gates(tmp_path):
    shape=Polygon([(0,0),(5,0),(5,1),(5.00001,1),(5.00001,0),(10,0),(10,4),(0,4)])
    _,proof,actual=write_actual(tmp_path,shape)
    assert proof["complexity"]["line_roads"]==3
    assert proof["complexity"]["minimum_road_length_m"]==pytest.approx(1e-5)
    assert not actual.covers(Point(5.000005,.5))
    assert actual.symmetric_difference(shape).area<1e-10
    # No minimum-length gate is waived: this report explicitly retains a
    # real short line that the normal G11 evaluator may reject.
    assert proof["production_qualification"]=="NOT_EVALUATED"


def test_triangle_zero_width_tips_remain_nonnegative_after_xml_rounding(tmp_path):
    shape=Polygon([(0,0),(10/3,0),(11/7,13/11)])
    _,proof,actual=write_actual(tmp_path,shape,(3,2))
    assert proof["complexity"]["line_roads"]==1
    assert actual.symmetric_difference(shape).area<1e-7


def test_writer_precision_collapse_rejects_instead_of_removing_real_vertex(tmp_path):
    # Separate natural events on one continuous cell, resolvable in binary64
    # but both rounded to station 10 by the unchanged %.10g writer.
    shape=Polygon([(0,0),(10,0),(10.000000001,0.0001),(20,0),(20,5),(0,5)])
    doc=W.XodrDoc("precision")
    with pytest.raises(C.SurfaceCellError,match="source-events-collapse-at-writer-precision"):
        C.append_polygon(doc,shape,1,count(50),preferred_axis=(1,0))
    assert doc.roads==[]


@pytest.mark.parametrize("malformation",["negative-width","duplicate-station","routing-link","missing-road"])
def test_actual_xml_corruption_is_rejected(tmp_path,malformation):
    shape=Polygon([(0,0),(3,0),(10,1),(10,4),(3,4),(0,4)])
    root,proof,_=write_actual(tmp_path,shape)
    road=root.find("road")
    if malformation=="negative-width":
        road.find("lanes/laneSection/right/lane/width").set("a","-1")
    elif malformation=="duplicate-station":
        road.findall("lanes/laneSection/right/lane/width")[1].set("sOffset","0")
    elif malformation=="routing-link":
        junction=ET.SubElement(root,"junction",id="1")
        ET.SubElement(junction,"connection",incomingRoad=road.get("id"),connectingRoad="100")
    else:
        root.remove(road)
    audit=C.audit_written(root,proof)
    assert audit["status"]=="FAIL" and not audit["passed"]
    assert "error_code" in audit


def test_shifted_xml_cannot_expand_its_own_numeric_tolerance(tmp_path):
    root,proof,_=write_actual(tmp_path,box(0,0,10,3))
    root.find("road/planView/geometry").set("y","0.1")
    audit=C.audit_written(root,proof)
    assert not audit["passed"]
    assert not audit["per_cell"][0]["expected_xml_unchanged"]
    assert audit["symmetric_difference_m2"]>1


@pytest.mark.parametrize("bad_axis",[(0,0),(float("nan"),1),(float("inf"),1),(1,2,3)])
def test_axis_must_be_explicit_and_finite(bad_axis):
    doc=W.XodrDoc("t")
    with pytest.raises(C.SurfaceCellError,match="axis-required"):
        C.append_polygon(doc,box(0,0,1,1),1,count(50),preferred_axis=bad_axis)
    assert doc.roads==[]


def test_invalid_source_and_id_exhaustion_leave_existing_document_intact():
    doc=W.XodrDoc("t");original=W.Road(10);doc.add_road(original)
    with pytest.raises(C.SurfaceCellError,match="invalid-source-polygon"):
        C.append_polygon(doc,Polygon([(0,0),(3,3),(0,3),(3,0)]),1,count(50),preferred_axis=(1,0))
    shape=Polygon(box(0,0,10,10).exterior.coords,[box(3,3,7,7).exterior.coords])
    with pytest.raises(C.SurfaceCellError,match="insufficient-road-ids"):
        C.append_polygon(doc,shape,1,iter([50]),preferred_axis=(1,0))
    with pytest.raises(C.SurfaceCellError,match="colliding-road-id"):
        C.append_polygon(doc,box(0,0,10,3),1,iter([10]),preferred_axis=(1,0))
    assert doc.roads==[original]


def test_empty_source_is_explicit_and_does_not_create_geometry(tmp_path):
    _,proof,actual=write_actual(tmp_path,Polygon())
    assert actual.is_empty and proof["road_ids"]==[]
    assert proof["source_area_m2"]==0
