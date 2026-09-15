"""A fixed discrete cubic layout with continuous, shared knot coordinates.

This is the coordinate layer of the existing source-boundary model, not a
new fitter. Source vertices stay immutable. Crossing a discrete event/order
or minimum-span boundary rejects the trial instead of changing its dimension.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SourceKnotLayout:
    features: tuple
    anchors: tuple
    positions: tuple
    family_nodes: tuple
    minimum_span: float

    @classmethod
    def capture(cls, model):
        """Freeze the current *discrete* layout once, outside a line search.

        A node is either an original source contact or a fraction of the
        complete support between contacts. Fractions can move continuously;
        no original contact is converted to an independent movable point.
        """
        if model.export_structural_stations:
            raise ValueError('explicit external station anchors need their own source binding')
        endpoints = []
        for group in model.endpoint_groups:
            if len(group) > 2:
                key, station = group[0]
                index = int(np.argmin(abs(model.raw[key][:, 0]-station)))
                endpoints.append((float(station), (key, index)))
        # Retain the original extreme vertex identities, not moving min/max
        # switches. A candidate crossing their order is a structural change.
        lo_key = min(model.owner, key=lambda k: (model.raw[k][0, 0], k))
        hi_key = max(model.owner, key=lambda k: (model.raw[k][-1, 0], k))
        endpoints += [(float(model.raw[lo_key][0, 0]), (lo_key, 0)),
                      (float(model.raw[hi_key][-1, 0]), (hi_key, len(model.raw[hi_key])-1))]
        endpoints.sort(key=lambda item: item[0])
        unique = []
        for item in endpoints:
            if not unique or abs(item[0]-unique[-1][0]) > 1e-7:
                unique.append(item)
        values = np.array([item[0] for item in unique])
        positions = []
        for knot in model.global_breaks:
            i = int(np.searchsorted(values, knot, side='right')-1)
            i = min(max(i, 0), len(values)-2)
            fraction = float((knot-values[i])/(values[i+1]-values[i]))
            positions.append((i, fraction))
        family_nodes = []
        for family in model.families:
            indices = []
            for knot in np.unique(family.knots)[1:-1]:
                i = int(np.argmin(abs(model.global_breaks-knot)))
                if abs(model.global_breaks[i]-knot) > 1e-7:
                    raise ValueError('family has an independent nonshared interior knot')
                indices.append(i)
            family_nodes.append(tuple(indices))
        return cls(tuple(tuple(f.features) for f in model.families),
                   tuple(item[1] for item in unique), tuple(positions),
                   tuple(family_nodes), float(model.min_span))

    @property
    def free_nodes(self):
        return tuple(i for i, (_, fraction) in enumerate(self.positions)
                     if 1e-10 < fraction < 1-1e-10)

    def rebuild(self, model, displacements=None):
        """Return the same families/columns on the newly projected originals.

        ``displacements`` are shared interior-knot movements in metres,
        measured from their source-anchored fractional coordinates.
        """
        if tuple(tuple(f.features) for f in model.families) != self.features:
            raise ValueError('physical source-family identity changed')
        if abs(model.min_span-self.minimum_span) > 1e-10:
            raise ValueError('fixed layout cannot change its span budget')
        delta = np.zeros(len(self.free_nodes)) if displacements is None else np.asarray(displacements, float)
        if delta.shape != (len(self.free_nodes),) or not np.isfinite(delta).all():
            raise ValueError('finite shared interior-knot vector required')
        values = np.array([model.raw[key][index, 0] for key, index in self.anchors])
        if np.any(np.diff(values) <= 1e-7):
            raise ValueError('source event order changed; choose a different discrete layout')
        knots = np.array([values[i]+f*(values[i+1]-values[i]) for i, f in self.positions])
        knots[list(self.free_nodes)] += delta
        if np.any(np.diff(knots) < self.minimum_span-1e-7):
            raise ValueError('shared knot order/span budget violated; no added or removed knots')
        cuts = []
        for family, indices in zip(model.families, self.family_nodes):
            lo = min(model.raw[k][0, 0] for k in family.features)
            hi = max(model.raw[k][-1, 0] for k in family.features)
            row = np.r_[lo, knots[list(indices)], hi]
            if np.any(np.diff(row) < self.minimum_span-1e-7):
                raise ValueError('family support/order/span changed; no short fallback')
            # A physical branch contact must remain an actual knot if this
            # family owns that contact. No arbitrary sample becomes a knot.
            for group in model.endpoint_groups:
                if len(group) > 2 and any(k in family.features for k, _ in group):
                    if np.min(abs(row-group[0][1])) > 1e-7:
                        raise ValueError('source branch event left its bound layout')
            cuts.append(row)
        return knots, cuts
