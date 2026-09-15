"""Extracted whole-road variable block for dependency-closed reconstruction.

No autonomous write/accept path: shared parent ports must be consumed by all
incident connectors before a caller can accept a complete transaction.
"""
import copy
from dataclasses import dataclass

import numpy as np
from lxml import etree
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import linprog

from mapforge.ops.lane_family_border import _materialize_widths


@dataclass
class RoadConstraintModel:
    original: object
    families: list
    owner: dict
    ids: dict
    starts: np.ndarray
    ends: np.ndarray
    A: np.ndarray
    target: np.ndarray
    E: np.ndarray
    er: np.ndarray
    C: np.ndarray
    lower: np.ndarray
    labels: list
    source_rows: list
    dynamics_rows: np.ndarray
    reference: np.ndarray
    dynamics_limits: np.ndarray
    dynamics_labels: list
    events: list
    equality_labels: list

    @property
    def nvar(self):
        return self.A.shape[1]

    def preflight(self):
        return self._phase(self.E, self.er)

    def _phase(self, E, er):
        base = np.linalg.lstsq(E, er, rcond=None)[0]
        if len(E) and np.max(abs(E@base-er)) > 1e-7:
            return None, {'status':'INFEASIBLE', 'reason':'inconsistent exact equalities'}
        Z = null_space(E)
        D = self.C@Z
        lower = self.lower-self.C@base
        phase = linprog(np.r_[np.zeros(Z.shape[1]), 1.], A_ub=np.c_[-D, -np.ones(len(D))],
                        b_ub=-lower, bounds=[(None, None)]*Z.shape[1]+[(0, None)], method='highs')
        if not phase.success:
            return None, {'status': 'ERROR', 'message': str(phase.message)}
        weights = -phase.ineqlin.marginals
        conflicts = [dict(self.labels[i], dual_weight=float(weights[i]))
                     for i in np.argsort(-weights) if weights[i] > 1e-7]
        x = base+Z@phase.x[:-1]
        return x, {'status': 'FEASIBLE' if phase.fun <= 1e-7 else 'INFEASIBLE',
                   'normalized_phase_slack': float(phase.fun), 'dual_support': conflicts,
                   'variables': self.nvar, 'free_variables': Z.shape[1],
                   'scope': 'linear source/width/end-structure model; dynamics and incident connectors not yet solved'}

    def diagnose_branch_ties(self, family=None):
        """Counterfactual only; never returns relaxed parameters for writing.

        Compare C0/C1/C2 branch ties with identical basis, source tubes and
        positive-width bounds. Feasible C1 is NOT an accepted G2 solution.
        """
        rows = []
        if family is not None and not any(l['kind']=='branch-tie' and l.get('family')==family for l in self.equality_labels):
            raise ValueError('unknown branch family')
        for order in range(3):
            keep = np.array([l['kind'] != 'branch-tie' or
                             (family is not None and l.get('family') != family) or l['derivative'] <= order
                             for l in self.equality_labels], dtype=bool)
            _, report = self._phase(self.E[keep], self.er[keep])
            rows.append(dict(report, branch_derivative_order=order,
                             family=family,diagnostic_only=True, removed_equalities=int((~keep).sum())))
        return rows

    def dynamics(self, x):
        from spikes.road_boundary_family import world_kinematics
        if not len(self.dynamics_rows):
            return np.empty((0, 2))
        return world_kinematics(self.dynamics_rows@x, self.reference[:, 0], self.reference[:, 1])/self.dynamics_limits

    def materialize(self, x):
        x = np.asarray(x, float)
        if x.shape != (self.nvar,) or not np.isfinite(x).all():
            raise ValueError('invalid whole-road state vector')
        out = copy.deepcopy(self.original)
        group = out.find('lanes'); sections = out.findall('lanes/laneSection')
        for e in group.findall('laneOffset'): group.remove(e)
        def coefficients(fi, s):
            f = self.families[fi]; curve = BSpline(f.knots, x[f.columns], 3)
            return {k: format(float(curve(s, nu=d))/[1,1,2,6][d], '.16g') for d, k in enumerate('abcd')}
        for i, s in enumerate(np.unique(self.families[0].knots)[:-1]):
            group.insert(i, etree.Element('laneOffset', s=str(s), **coefficients(0, s)))
        for (side, si, lid), fi in self.owner.items():
            lane = next(l for l in sections[si].findall(side+'/lane') if l.get('id') == lid)
            for e in lane.findall('width')+lane.findall('border'): lane.remove(e)
            a, b = self.starts[si], self.ends[si]
            cuts = np.unique(np.r_[a, [s for s in self.families[fi].knots if a < s < b]])
            index = 1 if lane.find('link') is not None else 0
            for s in cuts:
                lane.insert(index, etree.Element('border', sOffset=str(s-a), **coefficients(fi, s)))
                index += 1
        if _materialize_widths(out, sections, self.starts, self.ends) is None:
            raise ValueError('whole-road widths cannot be materialized')
        return out
