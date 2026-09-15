"""Source-domain accounting must follow curved charts without editing sources."""
import copy
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mapforge.ops.arc_source_chart import ArcChart
from mapforge.ops.source_domain import (
    exact_parent_chart, linear_chart, clipped_intervals, _span_planes,
    _outside_plane, _require_chart_support, prepare_source_domain, require_domain_replay,
)
from mapforge.ops.reconstruction_scope import prepare_scope, whole_junction_scope
from tests.test_source_domain import domain_fixture


def road(k=0.01, length=50., heading=.3):
    root = ET.Element('road', id='test', length=str(length))
    g = ET.SubElement(ET.SubElement(root, 'planView'), 'geometry',
                      s='0', x='17', y='-9', hdg=str(heading), length=str(length))
    ET.SubElement(g, 'arc', curvature=str(k))
    return root


@pytest.mark.parametrize('k', [.01, -.01, 1e-12, 0.])
def test_arc_normal_planes_match_projected_source_segments(k):
    r = road(k); before = ET.tostring(r); chart = exact_parent_chart(r)
    axis = ArcChart((17., -9.), .3, k)
    # One ORIGINAL straight segment, not a resampled curved approximation.
    xy = axis.world(np.array([[-5., -3.], [55., 4.]]))
    _require_chart_support(chart, xy)
    stations, intervals = clipped_intervals(xy, _span_planes(chart, 10., 35.))
    assert len(intervals) == 1
    length = stations[-1]
    actual_tips = xy[0] + np.array(intervals[0])[:, None]/length*(xy[1]-xy[0])
    assert axis.project(actual_tips)[:, 0] == pytest.approx([10., 35.], abs=1e-8)
    dense = xy[0]+np.linspace(0, 1, 701)[:, None]*(xy[1]-xy[0])
    source_s = axis.project(dense)[:, 0]
    accepted = np.ones(len(dense), dtype=bool)
    for a, b, offset in _span_planes(chart, 10., 35.):
        accepted &= dense @ [a, b] >= offset
    assert np.array_equal(accepted, (source_s >= 10.) & (source_s <= 35.))
    assert ET.tostring(r) == before


@pytest.mark.parametrize('k', [.01, -.01])
def test_sections_and_both_tails_cover_the_same_original_part_exactly(k):
    chart = exact_parent_chart(road(k)); axis = ArcChart((17., -9.), .3, k)
    xy = axis.world(np.array([[-7., -3.], [57., -2.]]))
    source_length = np.linalg.norm(xy[1]-xy[0])
    planes = [_outside_plane(chart, 'start'), _span_planes(chart, 0., 20.),
              _span_planes(chart, 20., 50.), _outside_plane(chart, 'end')]
    spans = sorted(pair for p in planes for pair in clipped_intervals(xy, p)[1])
    assert len(spans) == 4
    assert spans[0][0] == pytest.approx(0.)
    assert spans[-1][1] == pytest.approx(source_length)
    assert [a[1] for a in spans[:-1]] == pytest.approx([a[0] for a in spans[1:]])
    # The old tangent-at-start cut is measurably not the curved endpoint cut.
    line = dict(chart, curvature=0.)
    old = clipped_intervals(xy, _span_planes(line, 0., 50.))[1]
    new = clipped_intervals(xy, _span_planes(chart, 0., 50.))[1]
    assert abs(old[0][1]-new[0][1]) > .5


def test_reject_unsupported_wrapped_or_inconsistent_charts_without_line_fallback():
    with pytest.raises(ValueError, match='exact Line'):
        linear_chart(road())
    with pytest.raises(ValueError, match='forward branch'):
        exact_parent_chart(road(.04))
    r = road(); g = r.find('planView/geometry'); g.set('length', '49')
    with pytest.raises(ValueError, match='mismatch'): exact_parent_chart(r)
    r = road(); g = r.find('planView/geometry'); g[0].tag = 'spiral'
    with pytest.raises(ValueError, match='unsupported'): exact_parent_chart(r)
    r = road(); r.find('planView').append(copy.deepcopy(r.find('planView/geometry')))
    with pytest.raises(ValueError, match='one exact'): exact_parent_chart(r)
    chart = exact_parent_chart(road(0.01, heading=0.))
    with pytest.raises(ValueError, match='forward arc chart'):
        _require_chart_support(chart, [[17., -9.], [17., 92.]])


def test_line_mode_stays_explicit_and_v2_replay_rejects_modified_chart():
    root, source, _ = domain_fixture()
    header = ET.SubElement(root, 'header')
    ET.SubElement(header, 'geoReference').text = '+proj=eqc +lat_0=0 +lon_0=0 +lat_ts=0 +R=6378137 +units=m'
    scope = prepare_scope(root, source, ['10'])
    jid = root.find('junction').get('id')
    old = prepare_source_domain(root, source, scope, 'original-j', jid)
    new = prepare_source_domain(root, source, scope, 'original-j', jid, 'exact-line-arc-v1')
    assert old['schema'] == 'mapforge.source-domain/v1'
    assert new['schema'] == 'mapforge.source-domain/v2'
    assert old['partition']['features'] == new['partition']['features']
    assert require_domain_replay(new, root, source, scope)
    from mapforge.ops.reconstruction_scope import digest
    new['partition']['exact_parent_charts']['10']['curvature'] = .01
    new['content_sha256'] = digest({k: v for k, v in new.items() if k != 'content_sha256'})
    with pytest.raises(ValueError, match='fresh original inputs'):
        require_domain_replay(new, root, source, scope)


def test_whole_junction_collects_unconnected_attached_arm_and_preserves_scope():
    root, _, _ = domain_fixture(); jid = root.find('junction').get('id')
    extra = copy.deepcopy(root.find("road[@id='10']")); extra.set('id', 'other-arm')
    if extra.find('link') is not None:
        extra.remove(extra.find('link'))
    ET.SubElement(ET.SubElement(extra, 'link'), 'successor', elementType='junction', elementId=jid)
    root.append(extra); before = ET.tostring(root)
    scope = whole_junction_scope(root, jid)
    assert set(scope['ordinary_roads']) == {'10', '11', 'other-arm'}
    assert len(scope['connectors']) == 2 and not scope['geometry_validated']
    assert ET.tostring(root) == before
    extra.find('link/successor').set('elementId', 'other-junction')
    ET.SubElement(extra.find('link'), 'predecessor', elementType='junction', elementId=jid)
    with pytest.raises(ValueError, match='outside-boundary'): whole_junction_scope(root, jid)


def test_complete_selection_does_not_certify_original_movement_completeness():
    root, source, _ = domain_fixture(); jid = root.find('junction').get('id')
    root.remove(root.find("road[@id='second']"))
    root.find('junction').remove(root.find("junction/connection[@id='second']"))
    whole = whole_junction_scope(root, jid)
    packet = prepare_scope(root, source, whole['ordinary_roads'],
                           {'source_junction_id': 'original-j', 'xodr_junction_id': jid})
    assert len(whole['connectors']) == 1
    assert packet['source_admission'] == 'REJECTED' and not packet['export_allowed']
