"""Read-only necessary-condition audit of saved R2; no solve or new state."""
import json
from pathlib import Path
import sys

import numpy as np
from numpy.polynomial import Polynomial as P

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle
from mapforge.repair_web.local_refinement import prepare_local_refinement, LO, HI, EDGE
from mapforge.repair_web.model import atomic, digest, json_bytes
from mapforge.repair_web.refinement_target import RefinementProblem
from scripts.check_outer_event_control import inputs, POINT


def reversal_lower_support(cells, particular, direction, saved_scalar):
    """Sum fixed signed t(b)-t(a): a global affine lower bound of reversal.

    The cut/sign selection is from the SAVED state only. For a flat source,
    +/- displacement <= total variation; for a directed source, the chosen
    opposite displacement <= wrong-way variation. This is valid at every z,
    not a linearization assumed locally accurate.
    """
    intercept, slope, terms = 0., 0., []
    for cp in cells:
        if cp['edge'] != EDGE: continue
        b = P(cp['c']+cp['D']@particular); d = P(cp['D']@direction)
        c = b+d*saved_scalar; L = cp['hi']-cp['lo']
        roots = c.deriv().roots()
        cuts = [0.]+sorted(float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < L)+[L]
        source_direction = float(np.sign(cp['source'][1]))
        for lo, hi in zip(cuts, cuts[1:]):
            delta = c(hi)-c(lo)
            sign = float(np.sign(delta)) if source_direction == 0 else (-source_direction if source_direction*delta < 0 else 0.)
            a, g = float(sign*(b(hi)-b(lo))), float(sign*(d(hi)-d(lo)))
            intercept += a; slope += g
            terms.append(dict(key=cp['key'], record=cp['record'], part=cp['part'],
                interval_m=[cp['lo']+lo, cp['lo']+hi], source_direction=source_direction,
                sign=sign, intercept_m=a, slope_m=g))
    return intercept, slope, terms


def minimum_abs_curvature(velocity, acceleration, lo, hi):
    """All stationary/zero/end candidates for |a(z)/(1+v(z)^2)^1.5|."""
    v, a = P(velocity), P(acceleration); q = 1+v*v
    stationary = a.deriv()*q-3*a*v*v.deriv()
    points = [lo, hi]
    for polynomial in (a, stationary):
        points.extend(float(r.real) for r in polynomial.roots() if abs(r.imag) < 1e-9 and lo < r.real < hi)
    observations = [dict(scalar=float(z), abs_curvature=float(abs(a(z))/q(z)**1.5)) for z in points]
    return min(observations, key=lambda r: r['abs_curvature']), observations


def main():
    trial = ROOT/'out/node4-refinement-target-r2-20260917'; dest = trial/'necessary-condition-audit'
    if dest.exists(): raise ValueError('Read existing condition audit; do not overwrite')
    binding = json.loads((trial/'inspection/binding.json').read_bytes())
    binding[str(trial/'inspection/inspection.json')] = digest((trial/'inspection/inspection.json').read_bytes())
    binding[str(Path(__file__).resolve())] = digest(Path(__file__).read_bytes())
    binding[str(ROOT/'tests/test_refinement_conflict.py')] = digest((ROOT/'tests/test_refinement_conflict.py').read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Evidence drift: '+path)
    verify()
    report = json.loads((trial/'evaluation.json').read_bytes())
    inspection = json.loads((trial/'inspection/inspection.json').read_bytes())
    event, control = inputs()
    model = prepare_local_refinement(event, control.reference,
        EditScopeRequest(control.reference_sha256, '11', (EDGE,), (LO, HI), (ScopeHandle(POINT),)))
    problem = RefinementProblem(event, control, model)
    p, n, z = np.asarray(report['particular']), np.asarray(report['direction']), report['scalar']
    intercept, slope, terms = reversal_lower_support(problem.sources, p, n, z)
    old = report['guard_evaluation']['reversal_by_edge_m'][EDGE]
    equality_error = abs(intercept+slope*z-old)
    if equality_error > 1e-9: raise ValueError('Support does not reproduce saved reversal')
    if abs(slope) < 1e-12: raise ValueError('No informative scalar bound; no conflict claimed')
    cap = problem.reversal_caps[str(EDGE)]+1e-7
    bound = (cap-intercept)/slope
    lo, hi = report['domain']['lower'], report['domain']['upper']
    if slope > 0: hi = min(hi, bound)
    else: lo = max(lo, bound)
    if lo > hi: raise ValueError('Linear conflict; this audit expects curvature witness')
    # Use the saved maximum-curvature witness, not a new search for a better shape.
    s = next(w['s'] for w in inspection['witnesses'] if w['metric'] == 'curvature')
    b = model.power(EDGE, s, p)
    d = np.column_stack([model.delta_power(s, v) for v in ([1., 0.], [0., 1.])])@n
    velocity, acceleration = [b[1], d[1]], [2*b[2], 2*d[2]]
    minimum, checked = minimum_abs_curvature(velocity, acceleration, lo, hi)
    kcap = problem.caps['by_edge'][str(EDGE)]['max_abs_curvature']+1e-9
    gap = minimum['abs_curvature']-kcap
    result = dict(status='FIXED_R2_CONDITIONS_CONFLICT' if gap > 1e-8 else 'NO_CONFLICT_ESTABLISHED',
        scope='exact source target + frozen R2 basis/scope + existing research guards ONLY',
        reversal_support=dict(intercept_m=intercept, slope_m=slope, bound_with_epsilon_m=cap,
            scalar_bound=bound, sign='<=' if slope > 0 else '>=', saved_state_equality_error_m=equality_error,
            terms=terms), necessary_scalar_interval=[lo, hi],
        curvature_witness=dict(station_m=s, velocity_affine=velocity, acceleration_affine=acceleration,
            minimum_over_necessary_interval=minimum, algebraic_extrema=checked,
            cap_with_epsilon=kcap, positive_gap=gap),
        numerical_method='float64 algebraic roots; not interval arithmetic or a formal certificate',
        optimizer_calls=0, new_target_states=0, new_xodr=False, guard_changes=False,
        general_map_infeasibility=False, map_accepted=False, input_bindings=len(binding), input_drift=0)
    verify(); dest.mkdir(exist_ok=False)
    atomic(dest/'binding.json', json_bytes(binding)); atomic(dest/'conditions.json', json_bytes(result))
    print(json.dumps({k:v for k,v in result.items() if k != 'reversal_support'}))


if __name__ == '__main__': main()
