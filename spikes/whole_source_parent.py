"""Whole-junction source coordinates; no fitter or XODR promotion.

The starting tangent is merely an invertible Cartesian coordinate frame for
ORIGINAL vertices. It never replaces a baseline Arc by a Line in a map or in
source-domain accounting. Curve variables are installed by the shared Arc
block afterwards. Default historical SourceBoundaryBlock remains Line-only.
"""
from mapforge.ops.source_domain import exact_parent_chart
from spikes.source_contact_fit import SourceBoundaryBlock
import numpy as np


class OriginalParentBoundaryBlock(SourceBoundaryBlock):
    def __init__(self, *args, speed_node_fields=None, **kwargs):
        self.source_speed_boundary_policy = 'original-lane-node'
        self.source_speed_node_fields = speed_node_fields or {}
        super().__init__(*args, **kwargs)

    def _source_chart(self, road):
        chart = exact_parent_chart(road)
        self.original_parent_chart = chart
        self.source_coordinate_policy = 'original-points-in-parent-start-tangent-frame/v1'
        return {key: chart[key] for key in ('origin', 'tangent', 'length')}


def original_greville_seed(model):
    """Fixed-size quasi-interpolant initializer, NOT fitting or acceptance.

    Source count cannot add knots. Evaluate the original physical family at
    existing Greville sites; C2/width/source violations remain joint residuals.
    """
    result = np.zeros(model.nvar)
    for family in model.families:
        sites = [np.mean(family.knots[i+1:i+4]) for i in range(family.columns.stop-family.columns.start)]
        values = []
        for s in sites:
            matches = [model.traces[key] for key in family.features
                       if model.raw[key][0,0]-1e-8 <= s <= model.raw[key][-1,0]+1e-8]
            if not matches:
                raise ValueError('Greville initialization would extrapolate an original source family')
            ys = []
            for trace in matches:
                st = trace.st
                at = min(max(s, st[0,0]), st[-1,0])
                i = min(max(np.searchsorted(st[:,0], at, side='right')-1, 0), len(st)-2)
                ys.append(float(trace.values(i, at)))
            if np.ptp(ys) > 1e-7:
                raise ValueError('original family has conflicting values at a shared source contact')
            values.append(ys[0])
        result[family.columns] = values
    return result
