"""Research kernel for coupled lane ribbons; not a production converter.

At a G2 reference joint let A=1-k*t and q=t'/A. If every boundary
shares q, imposing [t'']=-q*t*[k'] makes world curvature continuous
for BOTH edges and every affine cross-section fraction between them.
Unlike forcing t'=0, q remains a shape variable. No knots are inserted
by optimization. Regularity, source fidelity and dynamics remain gates.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SectionSpline:
    knots: np.ndarray
    coefficients: np.ndarray  # (boundary, interval, a/b/c/d), physical metres
    rank: int
    condition: float
    residual: float


def cubic_bernstein(coefficients, lengths):
    """Power coefficients on [0,L] -> Bernstein coefficients on [0,1].

    Bounding all four values bounds the entire cubic, a sufficient (not
    necessary) condition. This basis change adds no exported records/DOFs.
    """
    c = np.asarray(coefficients, dtype=float)
    lengths = np.asarray(lengths, dtype=float)
    if c.shape[-1:] != (4,) or not np.isfinite(c).all():
        raise ValueError('finite cubic coefficients required')
    if not np.isfinite(lengths).all() or np.any(lengths <= 0):
        raise ValueError('positive finite lengths required')
    a, b, cc, d = np.moveaxis(c, -1, 0)
    b, cc, d = b*lengths, cc*lengths**2, d*lengths**3
    return np.stack((a, a+b/3, a+2*b/3+cc/3, a+b+cc+d), axis=-1)


def _basis(span):
    return np.array([[1., span, span**2, span**3],
                     [0., 1., 2*span, 3*span**2],
                     [0., 0., 2., 6*span]])


def solve_shared_section(lengths, curvature_nodes, start_jets, end_jets,
                         shears=None):
    """Solve all boundaries together for fixed primitives and joint shears.

    start/end_jets: (boundary, t/t'/t''). curvature_nodes has n+1 values
    for n line/arc/spiral primitives. n-1 dimensionless shears are shared
    by ALL boundaries. Geometry itself is integrated/closed by the caller.

    n+2 width spans: reference knots plus one midpoint at each end. For a
    fixed shear there are 4(n+2) equations and unknowns per boundary;
    optimization has n-1 shared transverse variables, not per-point edits.
    This layout matches the existing flat-join control experiment; it is
    NOT a claim that its shortest width intervals meet every export policy.
    """
    lengths = np.asarray(lengths, dtype=float)
    curvature_nodes = np.asarray(curvature_nodes, dtype=float)
    start, end = np.asarray(start_jets, float), np.asarray(end_jets, float)
    if lengths.ndim != 1 or len(lengths) < 2 or not np.isfinite(lengths).all() or np.any(lengths <= 0):
        raise ValueError('at least two positive finite primitives required')
    nref = len(lengths)
    if curvature_nodes.shape != (nref+1,) or not np.isfinite(curvature_nodes).all():
        raise ValueError('n+1 finite curvature nodes required')
    if start.ndim != 2 or start.shape[1] != 3 or len(start) < 2 or end.shape != start.shape:
        raise ValueError('matching endpoint jets for at least two boundaries required')
    if not np.isfinite(start).all() or not np.isfinite(end).all():
        raise ValueError('finite endpoint jets required')
    q = np.zeros(nref-1) if shears is None else np.asarray(shears, float)
    if q.shape != (nref-1,) or not np.isfinite(q).all():
        raise ValueError('one finite shear per internal reference joint required')
    length = float(sum(lengths))
    ref = np.r_[0., np.cumsum(lengths)]
    knots = np.sort(np.r_[ref, lengths[0]/2, length-lengths[-1]/2])
    spans = np.diff(knots)/length
    n = len(spans)
    sharpness = np.diff(curvature_nodes)/lengths
    joint = {int(np.argmin(abs(knots-s))): j for j, s in enumerate(ref[1:-1])}
    matrix, rhs = [], []
    def row(terms, value):
        v = np.zeros(4*n)
        for i, b in terms:
            v[4*i:4*i+4] += b
        matrix.append(v)
        rhs.append(np.broadcast_to(value, (len(start),)))
    for derivative in range(3):
        row([(0, _basis(0)[derivative])], start[:, derivative]*length**derivative)
        row([(n-1, _basis(spans[-1])[derivative])], end[:, derivative]*length**derivative)
    for i in range(1, n):
        left, right = _basis(spans[i-1]), _basis(0)
        for derivative in (0, 1):
            row([(i, right[derivative]), (i-1, -left[derivative])], 0.)
        second = right[2].copy()
        if i in joint:
            j = joint[i]
            second += length**2*q[j]*(sharpness[j+1]-sharpness[j])*right[0]
        row([(i, second), (i-1, -left[2])], 0.)
    for i, j in joint.items():
        row([(i, _basis(0)[1]+length*q[j]*curvature_nodes[j+1]*_basis(0)[0])], length*q[j])
    mat, target = np.asarray(matrix), np.asarray(rhs)
    # Row equilibration preserves equations and exposes genuinely singular
    # layouts, rather than accepting least-squares end-state drift.
    norms = np.linalg.norm(mat, axis=1)
    equilibrated, scaled = mat/norms[:, None], target/norms[:, None]
    rank = int(np.linalg.matrix_rank(equilibrated))
    if rank != len(mat):
        raise ValueError('rank-deficient shared-section system')
    values = np.linalg.solve(equilibrated, scaled)
    residual = float(np.max(abs(equilibrated@values-scaled)))
    if not np.isfinite(values).all() or residual > 1e-7:
        raise ValueError('shared-section equality solve failed')
    coeff = values.T.reshape(len(start), n, 4)/np.array([1., length, length**2, length**3])
    return SectionSpline(knots, coeff, rank, float(np.linalg.cond(equilibrated)), residual)


def width_control_bounds(section):
    """Adjacent ordered boundaries, LEFT to RIGHT; no abs/clip/sorting repair."""
    return cubic_bernstein(section.coefficients[:-1]-section.coefficients[1:],
                           np.diff(section.knots))
