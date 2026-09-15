"""Speed limits are input facts; design speeds are independent test conditions.

No geometry-dependent speed adjustment and no writes to the map. A proposed
movement contract may be measured but cannot authorize delivery.
"""
from math import isfinite

import numpy as np

from mapforge.ops.port_dependencies import revision


def lane_limits(lane):
    records = []
    units = {'m/s': 3.6, 'km/h': 1., 'mph': 1.609344}
    for item in lane.findall('speed'):
        unit = item.get('unit', 'm/s')
        if unit not in units:
            raise ValueError(f'unsupported lane speed unit: {unit}')
        s, value = float(item.get('sOffset')), float(item.get('max'))
        if not isfinite(s) or s < 0 or not isfinite(value) or value < 0:
            raise ValueError('lane speed offset/value must be finite and nonnegative')
        if records and s <= records[-1]['s_offset_m']:
            raise ValueError('lane speed records must be strictly ascending')
        records.append({'s_offset_m': s, 'max_kmh': value * units[unit],
                        'raw_max': item.get('max'), 'raw_unit': unit})
    return records


def limit_samples(records, stations, fallback_kmh):
    """Apply sOffset intervals, including a genuine zero speed (not missing)."""
    stations = np.asarray(stations, float)
    values = np.full(stations.shape, float(fallback_kmh))
    missing = np.ones(stations.shape, bool)
    for row in records:
        mask = stations >= row['s_offset_m']
        values[mask] = row['max_kmh']; missing[mask] = False
    return values, missing


def movement_bindings(root, contract):
    if contract is None:
        return {}
    if (contract.get('schema') != 'mapforge/movement-design-speed/v1'
            or contract.get('artifact_revision') != revision(root)):
        raise ValueError('movement design speed contract is absent or stale')
    if contract.get('approval_state') not in ('proposed', 'approved'):
        raise ValueError('explicit design speed approval state required')
    expected = set()
    for road in root.findall('road'):
        if road.get('junction', '-1') == '-1' or road.get('name') == 'junction_paving':
            continue
        for section in road.findall('lanes/laneSection'):
            for lane in section.findall('left/lane') + section.findall('right/lane'):
                if lane.get('type') == 'driving':
                    expected.add((road.get('id'), float(section.get('s')), lane.get('id')))
    bindings = {}
    for row in contract.get('bindings', []):
        key = (row['road_id'], float(row['section_s_m']), row['lane_id'])
        speed = row['target_speed_kmh']
        if key in bindings or key not in expected:
            raise ValueError('duplicate, ordinary, or unknown movement target')
        if isinstance(speed, bool) or not isfinite(float(speed)) or float(speed) <= 0:
            raise ValueError('positive finite movement design speed required')
        if not row.get('movement_id') or not row.get('basis'):
            raise ValueError('movement identity and independent speed basis required')
        bindings[key] = dict(row, target_speed_kmh=float(speed))
    if set(bindings) != expected:
        raise ValueError('design speed contract must cover every driving movement')
    return bindings
