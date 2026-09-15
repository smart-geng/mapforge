"""Fixed source identities over variable-size, COMPLETE certificate partitions.

For a feature F, g_F(x) = min(all its current certificate slacks). Thus
g_F >= 0 iff EVERY original row in that group passes. No averaging, dropping
outliers or adding geometric pieces. Witness switches are nonsmooth: this is
an envelope oracle, not a claim of a continuously differentiable constraint.
"""
from collections import Counter
import json

import numpy as np

from spikes.source_contact_fit import polynomial_bernstein


class ConstraintStructureError(ValueError):
    """A discrete source/model identity changed inside a continuous trial."""


def _identity(value):
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'))


def source_keys(label):
    kind = label['kind']
    if kind == 'source':
        return (_identity(['source', label['feature']]),)
    if kind == 'source-endpoint-cap':
        # A raw center can change from a supported interval to an end cap;
        # it remains the SAME feature with its SAME complete source budget.
        return (_identity(['source', 'lane:'+label['source_lane']]),)
    if kind == 'width':
        return (_identity([kind, label['source_lane']]),)
    if kind == 'transition-width':
        # A zero-length cross-record transition can open under an Arc chart.
        # Attach every transition witness to ALL its existing source lanes,
        # rather than creating/removing a new residual at that event. The
        # duplicated witness changes neither the intersection nor the budget.
        if not label['source_lanes']:
            raise ConstraintStructureError('transition width has no original lane identity')
        return tuple(_identity(['width',s]) for s in sorted(set(label['source_lanes'])))
    if kind == 'regular-reference-chart':
        return ()  # Replaced by the equivalent, finite k=0 form below.
    raise ConstraintStructureError('unregistered source constraint kind: '+kind)


def equality_keys(labels):
    counts = Counter(); result = []
    for row in labels:
        key = _identity([row['features'], row['derivative'], row['physical_relation']])
        result.append(_identity([key, counts[key]])); counts[key] += 1
    return tuple(result)


def envelope_rows(model, x):
    """Return every witness; regularity slacks alone are dimensionless."""
    x = np.asarray(x, float)
    if x.shape != (model.nvar,) or not np.isfinite(x).all():
        raise ValueError('finite complete source coefficients required')
    if len(model.labels) != len(model.C) or len(model.C) != len(model.lower):
        raise ConstraintStructureError('source rows and identities are incomplete')
    values = model.C@x-model.lower
    groups = {}; original_rows = 0; replaced = 0; expanded = 0
    for i, label in enumerate(model.labels):
        keys = source_keys(label)
        if not keys:
            replaced += 1; continue
        for key in keys:
            groups.setdefault(key, []).append((float(values[i]), dict(row=i, **label)))
        original_rows += 1
        expanded += len(keys)
    regular_rows = 0
    for family in model.families:
        key = _identity(['regular-reference-chart', list(family.features)])
        # Old form .8/abs(k)-sign(k)*t has a discontinuity at k=0.
        # .8-k*t >=0 is identical for nonzero k, finite on BOTH sides of 0,
        # and includes the zero-curvature chart rather than dropping rows.
        for a,b in zip(np.unique(family.knots)[:-1], np.unique(family.knots)[1:]):
            rows = polynomial_bernstein(lambda s,d=0:model.expression(family.features[0],s,d),a,b,3)
            for j,row in enumerate(rows):
                slack = float(.8-model.reference_curvature*(row@x))
                groups.setdefault(key, []).append((slack, dict(kind='regular-reference-chart',
                    features=list(family.features), span=[float(a),float(b)], bernstein=j,
                    unit='dimensionless', normalized_equivalent=True)))
                regular_rows += 1
    if not all(np.isfinite(v) for rows in groups.values() for v,_ in rows):
        raise ValueError('nonfinite complete source certificate')
    return groups, dict(original_rows=original_rows, expanded_witness_rows=expanded, replaced_regular_rows=replaced,
        normalized_regular_rows=regular_rows, original_total=len(values),
        all_original_rows_accounted=(original_rows+replaced == len(values)))


class SourceConstraintEnvelope:
    def __init__(self, model, coefficients):
        groups,_ = envelope_rows(model,coefficients)
        self.keys = tuple(sorted(groups))
        self.eq_keys = equality_keys(model.equality_labels)
        self.road = model.road
        self.families = tuple(tuple(f.features) for f in model.families)
        self.source_tolerance = model.source_tol

    def evaluate(self, state):
        model = state['model']; x = state['coefficients']
        if (model.road != self.road or model.source_tol != self.source_tolerance
                or tuple(tuple(f.features) for f in model.families) != self.families):
            raise ConstraintStructureError('source owner, budget or family structure changed')
        groups,accounting = envelope_rows(model,x)
        if tuple(sorted(groups)) != self.keys or equality_keys(model.equality_labels) != self.eq_keys:
            raise ConstraintStructureError('source constraint identity changed; re-register outside the trial')
        slacks = []; witnesses = []; ties = []
        for key in self.keys:
            rows = groups[key]; values = np.array([v for v,_ in rows]); i = int(np.argmin(values))
            active = np.flatnonzero(abs(values-values[i]) <= 1e-10)
            slacks.append(values[i]); witnesses.append(dict(identity=key, value=float(values[i]),
                witness=rows[i][1], witness_count=len(rows), active_witnesses=len(active)))
            if len(active)>1: ties.append(key)
        eq = np.asarray(model.E@x,float)
        if len(eq) != len(self.eq_keys) or not np.isfinite(eq).all():
            raise ConstraintStructureError('incomplete or nonfinite source contact equalities')
        return dict(equalities=eq, inequalities=np.array(slacks), witnesses=witnesses,
            accounting=accounting, tied_envelopes=ties, nonsmooth_envelope=True,
            complete_certificate_feasibility_equivalent=True, export_allowed=False)
