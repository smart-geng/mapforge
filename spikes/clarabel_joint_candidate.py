"""Independent QP backend experiment; identical geometry constraints and residual gates."""
import argparse
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.linalg import null_space
from scipy.sparse import csc_matrix, triu

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spikes import road_boundary_family as family
from spikes.joint_mouth_candidate import run


def interior_qp(H, g, E, er, C, lo, initial):
    import clarabel
    base = np.linalg.lstsq(E, er, rcond=None)[0] if len(E) else np.zeros(len(g))
    Z = null_space(E) if len(E) else np.eye(len(g))
    if not Z.shape[1]:
        violation = max(float(np.max(lo-C@base, initial=0.)), float(np.max(abs(E@base-er), initial=0.)))
        return (base if violation <= 1e-6 else None), {'solver': 'clarabel', 'status': 'equality-fixed',
                                                       'original_residual': violation}
    reduced = Z.T@H@Z
    Z = Z/np.sqrt(np.maximum(np.diag(reduced), 1e-10))
    P = Z.T@H@Z; q = Z.T@(H@base-g)
    A = -C@Z; b = C@base-lo
    magnitude = np.linalg.norm(A, axis=1)
    active = magnitude > 1e-10
    if np.any(b[~active] < -1e-6):
        return None, {'solver': 'clarabel', 'status': 'equality-inequality conflict'}
    A = A[active]/magnitude[active, None]; b = b[active]/magnitude[active]
    norm = max(float(np.linalg.norm(P, 2)), 1.)
    settings = clarabel.DefaultSettings()
    settings.verbose = False; settings.max_iter = 250; settings.time_limit = 45.
    settings.tol_gap_abs = 1e-10; settings.tol_gap_rel = 1e-10; settings.tol_feas = 1e-10
    qp = clarabel.DefaultSolver(triu(csc_matrix((P+P.T)/(2*norm))).tocsc(), q/norm,
                                csc_matrix(A), b, [clarabel.NonnegativeConeT(len(b))], settings)
    result = qp.solve()
    report = {'solver': 'clarabel', 'status': str(result.status), 'iterations': result.iterations,
              'primal_residual': result.r_prim, 'dual_residual': result.r_dual}
    if str(result.status) not in ('Solved', 'AlmostSolved'):
        return None, report
    x = base+Z@np.asarray(result.x)
    report['original_inequality_residual'] = float(max(0., np.max(lo-C@x, initial=0.)))
    report['original_equality_residual'] = float(np.max(abs(E@x-er), initial=0.))
    if report['original_inequality_residual']>1e-6:
        index=int(np.argmax(lo-C@x))
        report['worst_original_row']=index
        report['worst_original_row_norm']=float(np.linalg.norm(C[index]))
        report['state_max_abs']=float(max(abs(x)))
    # A feasible candidate need not prove a globally optimal smoothing cost.
    # Keep the actual optimizer status visible; acceptance is by independently
    # recomputed ORIGINAL constraints, then the unchanged geometry/dynamics gates.
    # Never accept an infeasibility/numerical-error result as a candidate.
    if (not np.isfinite(x).all() or max(report['original_inequality_residual'],
                                      report['original_equality_residual']) > 1e-6
            or result.r_dual > 1e-6):
        return None, report
    report['accepted_as_feasible_candidate'] = True
    report['global_optimum_claimed'] = False
    return x, report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('source', type=Path)
    parser.add_argument('target', type=Path); args = parser.parse_args()
    with patch.object(family, '_convex_qp', interior_qp):
        ok = run(args.source, args.target)
    raise SystemExit(0 if ok else 2)
