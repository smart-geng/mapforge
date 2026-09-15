import copy
import pytest
from scripts.gen_all import _sha256
from scripts.seal_parent_rebuild_diagnostic import require_same_artifact, require_inventory, interface_maxima


def test_actual_xml_not_only_reported_hash_is_bound(tmp_path):
    first = tmp_path / 'a.xodr'
    second = tmp_path / 'b.xodr'
    first.write_text('<OpenDRIVE/>', encoding='utf8')
    second.write_bytes(first.read_bytes())
    row = dict(artifact=str(first), sha256=_sha256(first))
    assert require_same_artifact(first, [row]) == _sha256(first)
    with pytest.raises(ValueError, match='same actual XML'):
        require_same_artifact(first, [dict(row, artifact=str(second))])
    first.write_text('<OpenDRIVE changed="true"/>', encoding='utf8')
    with pytest.raises(ValueError, match='same actual XML'):
        require_same_artifact(first, [row])


def test_no_partial_or_falsified_independent_turn_inventory():
    trial = dict(connectors=[dict(road='100'), dict(road='101')])
    geometry = dict(rows=[dict(road='100', geometry_status='FAIL'), dict(road='101', geometry_status='PASS')],
                    geometry_failed_roads=['100'])
    shape = dict(records=copy.deepcopy(trial['connectors']))
    require_inventory(trial, geometry, shape)
    with pytest.raises(ValueError, match='omitted'):
        require_inventory(trial, geometry, dict(records=shape['records'][:1]))
    with pytest.raises(ValueError, match='failure summary'):
        require_inventory(trial, dict(geometry, geometry_failed_roads=[]), shape)
    with pytest.raises(ValueError, match='duplicate'):
        require_inventory(trial, geometry, dict(records=[shape['records'][0]] * 2))


def test_interface_maxima_use_actual_measurements_from_all_steps():
    measurements = dict(rows=[dict(samples=[dict(position_m=1., heading_deg=2., curvature_per_m=.1),
        dict(position_m=2., heading_deg=1., curvature_per_m=.2)])])
    assert interface_maxima(measurements) == dict(position_m=2., heading_deg=2., curvature_per_m=.2)
    with pytest.raises(ValueError, match='finite consumer'):
        interface_maxima(dict(rows=[]))
    measurements['rows'][0]['samples'][1]['position_m'] = float('nan')
    with pytest.raises(ValueError, match='finite consumer'):
        interface_maxima(measurements)
