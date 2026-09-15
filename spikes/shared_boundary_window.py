"""One physical boundary controls BOTH neighbouring lane widths/port states.

Research scope: straight parent, end contact, no boundary identity changes
inside the selected long window. No source sorting, short repair knots, speed
changes or topology inference. The outside shape is preserved analytically.
"""
import copy

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import linprog

from spikes.mouth_anchor_review import project_curve
from spikes.road_boundary_family import ordered_source_boundary, world_kinematics
from mapforge.validate.smoothness import _sections, _offsets


def poly_jets(entries, station, select=None):
    pick = station if select is None else select
    e = next((e for e in reversed(entries) if e[0] <= pick), entries[0])
    u = station-e[0]; _, a, b, c, d = e
    return np.array([a+u*(b+u*(c+u*d)), b+u*(2*c+3*u*d), 2*c+6*u*d, 6*d])


class SharedBoundaryWindow:
    def __init__(self, road, side, index, span=30., min_span=15.):
        self.road = copy.deepcopy(road); self.side = side; self.index = index
        self.length = float(road.get('length')); self.lo = self.length-span; self.span = span
        self.sections = _sections(road); self.offsets = _offsets(road)
        geoms = road.findall('planView/geometry')
        if len(geoms) != 1 or geoms[0].find('line') is None:
            raise ValueError('shared-window prototype requires straight parent frame')
        if side not in ('left', 'right') or index < 1:
            raise ValueError('prototype edits an interior side boundary, not laneOffset')
        if not np.isfinite([span,min_span,self.length]).all() or not 0 < span < self.length or min_span <= 0 or span < 2*min_span:
            raise ValueError('at least two long control intervals required')
        sides = 1 if side == 'right' else 2
        self.lane_ids = [v[0] for v in self.sections[-1][sides]]
        if index >= len(self.lane_ids):
            raise ValueError('two adjacent lanes required')
        self.affected_lanes = self.lane_ids[index-1:index+1]
        for sec in self.sections:
            if sec[0] < self.lo and sec is not self.active_section(self.lo):
                continue
            ids = [v[0] for v in sec[sides]]
            if ids[:index+1] != self.lane_ids[:index+1]:
                raise ValueError('window crosses a birth/death/rank change; expand physical graph first')
        count = int(np.floor(span/min_span+1e-10))
        breaks = np.linspace(0, 1, count+1)
        self.knots = self.lo+span*breaks
        kv = np.r_[np.zeros(4), breaks[1:-1], np.ones(4)]
        self.basis = BSpline(kv, np.eye(len(kv)-4), 3)
        anchor = self.old_jets(self.lo)[index]
        matrix = np.array([self.basis(0, d) for d in range(3)])
        self.base = np.linalg.lstsq(matrix, anchor[:3]*np.array([1, span, span**2]), rcond=None)[0]
        self.Z = null_space(matrix)
        self.nvar = self.Z.shape[1]
        self._source_A = self._source_y = None
        self.source_labels = []

    def active_section(self, s):
        return next((v for v in reversed(self.sections) if v[0] <= s), self.sections[0])

    def old_jets(self, s, select=None):
        select = s if select is None else select
        sec = self.active_section(select)
        edge = poly_jets(self.offsets, s, select); out = [edge.copy()]
        for lid, kind, entries in sec[1 if self.side == 'right' else 2]:
            v = poly_jets(entries, s-sec[0], select-sec[0])
            edge = v if kind == 'border' else edge+(-1 if self.side == 'right' else 1)*v
            out.append(edge.copy())
        return np.array(out)

    def expression(self, s, derivative=0):
        b = self.basis(np.clip((np.asarray(s)-self.lo)/self.span, 0, 1), derivative)/self.span**derivative
        return b@self.base, b@self.Z

    def jets(self, s, variables, select=None):
        result = self.old_jets(s, select)
        if s >= self.lo:
            result[self.index] = [base+row@variables for base, row in
                                  (self.expression(s, d) for d in range(4))]
        return result

    def gather_source(self, src, project):
        rows, targets, labels = [], [], []
        for si, sec in enumerate(self.road.findall('lanes/laneSection')):
            lo = max(self.lo, float(sec.get('s')))
            hi = min(self.length, self.sections[si+1][0] if si+1 < len(self.sections) else self.length)
            if hi <= lo: continue
            for lid in self.affected_lanes:
                lane = next(v for v in sec.findall(self.side+'/lane') if int(v.get('id')) == lid)
                ud = next((e for e in lane.findall('userData') if e.get('code') == 'mapforge.source_lane'), None)
                if ud is None: raise ValueError('affected parent lane lacks original source identity')
                sid = ud.get('value'); record = src.lane(sid)
                if record is None: raise ValueError('missing original parent lane')
                candidates = [project_curve(g, self.road, project) for g in src.lane_boundary_geometries(sid)]
                lower = ordered_source_boundary(candidates, select_high=False)
                upper = ordered_source_boundary(candidates, select_high=True)
                rank = self.lane_ids.index(lid)+1
                curves = [('inner', upper if self.side == 'right' else lower, rank-1),
                          ('outer', lower if self.side == 'right' else upper, rank),
                          ('center', project_curve(record.geometry, self.road, project), None)]
                for field, curve, edge_index in curves:
                    a, b = max(lo, curve[0, 0]), min(hi, curve[-1, 0])
                    if b <= a: raise ValueError('original curve does not support edited occurrence')
                    # Full raw vertices IN THE WINDOW plus interpolated cut
                    # intersections. Outside remains unchanged, not discarded.
                    ss = np.unique(np.r_[a, b, np.arange(a, b, .5),
                                          curve[(curve[:, 0] >= a)&(curve[:, 0] <= b), 0]])
                    for s in ss:
                        old = self.old_jets(s, select=min(s, hi-1e-8))
                        base, row = self.expression(s)
                        if field == 'center':
                            fixed = old[rank if rank-1 == self.index else rank-1, 0]
                            base, row = (base+fixed)/2, row/2
                        elif edge_index != self.index:
                            base, row = old[edge_index, 0], row*0
                        value = float(np.interp(s, curve[:, 0], curve[:, 1]))
                        rows.append(row); targets.append(value-base)
                        labels.append({'source_lane': sid, 'lane': lid, 'field': field, 's': float(s),
                                       'mutable': bool(np.linalg.norm(row) > 1e-12)})
        self._source_A, self._source_y, self.source_labels = np.array(rows), np.array(targets), labels
        if not rows: raise ValueError('empty source support')

    def source_preflight(self, tolerance=.35):
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('source tolerance must be positive and finite')
        A, y = self._source_A, self._source_y
        if A is None: raise ValueError('load source first')
        # Phase I with a single normalized slack; labels explain conflicts.
        M = np.vstack([A, -A]); upper = np.r_[y+tolerance, -y+tolerance]
        result = linprog(np.r_[np.zeros(self.nvar), 1.], A_ub=np.c_[M, -np.ones(len(M))], b_ub=upper,
                         bounds=[(None, None)]*self.nvar+[(0, None)], method='highs')
        if not result.success: raise ValueError('linear source preflight failed numerically')
        dual = -result.ineqlin.marginals
        conflicts = [dict(self.source_labels[i%len(A)], dual_weight=float(dual[i])) for i in np.argsort(-dual) if dual[i] > 1e-7]
        return result.x[:-1], {'source_feasible': bool(result.fun < 1e-8),
                               'minimum_additional_source_slack_m': float(result.fun),
                               'conflicts': conflicts, 'scope': 'chosen boundary/window linear model only'}

    def source_errors(self, variables):
        return self._source_A@variables-self._source_y

    def sample_limits(self, variables, step=.5, speed=60/3.6):
        # Samples include original polynomial breaks with both one-sided jets.
        stations = np.unique(np.r_[np.arange(self.lo, self.length, step), self.knots,
                                     self._source_stations(), self.breaks()])
        widths, dynamics = [], []
        for s in stations:
            for select in (max(self.lo, s-1e-8), min(self.length, s+1e-8)):
                if s < self.lo or s > self.length: continue
                edges = self.jets(s, variables, select)
                for lid in self.affected_lanes:
                    i = self.lane_ids.index(lid)
                    widths.append((edges[i, 0]-edges[i+1, 0])*(-1 if self.side == 'left' else 1))
                    center = (edges[i]+edges[i+1])/2
                    dyn = world_kinematics(center, 0., 0.)
                    dynamics.append(abs(dyn)*np.array([speed**2/2.5, speed**3]))
        return np.array(widths), np.array(dynamics)

    def _source_stations(self):
        return [r['s'] for r in self.source_labels]

    def breaks(self):
        cuts = {self.lo, self.length, *self.knots, *(v[0] for v in self.offsets)}
        for sec in self.sections:
            cuts.add(sec[0])
            for _, _, entries in sec[1 if self.side == 'right' else 2]:
                cuts.update(sec[0]+v[0] for v in entries)
        return sorted(s for s in cuts if self.lo <= s <= self.length)

    def write(self, variables):
        variables = np.asarray(variables, float)
        if variables.shape != (self.nvar,) or not np.isfinite(variables).all():
            raise ValueError('invalid shared-boundary state')
        result = copy.deepcopy(self.road)
        global_cuts = self.breaks()
        sections = result.findall('lanes/laneSection')
        for si, sec in enumerate(sections):
            a = float(sec.get('s')); b = self.sections[si+1][0] if si+1 < len(sections) else self.length
            if b <= self.lo: continue
            cuts = sorted({a, b, *(s for s in global_cuts if a < s < b)})
            for lane in sec.findall(self.side+'/lane'):
                lid = int(lane.get('id'))
                if lid not in self.affected_lanes: continue
                i = self.lane_ids.index(lid)
                for e in lane.findall('width')+lane.findall('border'): lane.remove(e)
                pos = 1 if lane.find('link') is not None else 0
                for lo, hi in zip(cuts[:-1], cuts[1:]):
                    e = self.jets(lo, variables, (lo+hi)/2)
                    coeff = (e[i]-e[i+1])*(-1 if self.side == 'left' else 1)/np.array([1., 1., 2., 6.])
                    lane.insert(pos, lane.makeelement('width', attrib=dict(sOffset=str(lo-a), **dict(zip('abcd', map(str, coeff))))))
                    pos += 1
        return result
