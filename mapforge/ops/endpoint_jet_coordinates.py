"""Eliminate clamped cubic endpoint jets without introducing spline knots.

The first/last three coefficients are dependent coordinates. Only existing
interior coefficients are optimized. No null-space orientation can flip when
reference lengths change, and port constraints are satisfied algebraically.
"""
import numpy as np


def complete_endpoint_coefficients(basis, length, interior, target):
    n=len(basis.c);middle=np.asarray(interior,float);target=np.asarray(target,float)
    if (n<7 or middle.shape!=(n-6,) or target.shape!=(6,) or not np.isfinite(middle).all()
        or not np.isfinite(target).all() or not np.isfinite(length) or length<=0):
        raise ValueError('finite existing cubic basis and six endpoint jets required')
    E=np.array([basis(t,d)/length**d for t in (0.,1.) for d in range(3)])
    norms=np.linalg.norm(E,axis=1);E=E/norms[:,None];rhs=target/norms
    dependent=np.r_[0:3,n-3:n];free=np.arange(3,n-3)
    c=np.zeros(n);c[free]=middle
    c[dependent]=np.linalg.solve(E[:,dependent],rhs-E[:,free]@middle)
    if np.max(abs(E@c-rhs))>1e-10:raise ValueError('endpoint elimination lost precision')
    return c
