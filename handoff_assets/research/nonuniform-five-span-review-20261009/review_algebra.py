"""Independent algebra and dense numerical cross-checks of the nonuniform five-span exclusion.

Reviewer-authored; reads the reviewed research's derivation.json only to compare numbers and never imports its
code. Needs sympy (run with a separate review environment; the project's locked environment has none).
Checks:
  1. dkappa/dl of an offset curve, derived symbolically for a general reference k(s) and offset t(s), against the
     reviewed closed form (which assumes k''=0); road 10 is one spiral on the whole domain, so k''=0 there.
  2. the truncated-power coefficient identity w/c0 and the rank-3 end-condition matrix (dimension 2).
  3. baseline rate sign and monotonicity of right-1/right-2 centres over the whole domain (dense, numeric).
  4. the two linear activity coefficients at the witness station over a dense grid of the knot simplex, versus the
     reviewed uniform lower bound; and the full nonlinear excess on a grid of (u, v) in the endpoint box.
Grids are evidence, not proofs; the reviewed interval proof is what covers the continuum.
"""
from __future__ import annotations

from fractions import Fraction as F
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import sympy as sp

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
REVIEWED = OUT.parent


def symbolic_rate():
    s = sp.symbols("s")
    k, t = sp.Function("k")(s), sp.Function("t")(s)
    th = sp.integrate(k, s)  # heading of the reference (symbolic antiderivative is enough for derivatives)
    T = sp.Matrix([sp.cos(th), sp.sin(th)])
    n = sp.Matrix([-sp.sin(th), sp.cos(th)])
    # derivative of the offset point r + t n with respect to s, using r' = T
    d1 = T + sp.diff(t, s) * n + t * sp.diff(n, s)
    d2 = sp.diff(d1, s)
    cross = sp.simplify(d1[0] * d2[1] - d1[1] * d2[0])
    speed2 = sp.simplify(d1[0] ** 2 + d1[1] ** 2)
    kappa = cross / speed2 ** sp.Rational(3, 2)
    rate = sp.diff(kappa, s) / sp.sqrt(speed2)            # d kappa / d (offset arclength)
    sym = dict(zip(["k", "k1", "k2", "t", "t1", "t2", "t3"], sp.symbols("k k1 k2 t t1 t2 t3")))
    subs = {sp.diff(k, s, 2): sym["k2"], sp.diff(k, s): sym["k1"], k: sym["k"],
            sp.diff(t, s, 3): sym["t3"], sp.diff(t, s, 2): sym["t2"], sp.diff(t, s): sym["t1"], t: sym["t"]}
    general = sp.simplify(rate.subs(subs))
    K, K1, K2, Tt, T1, T2, T3 = (sym[x] for x in ["k", "k1", "k2", "t", "t1", "t2", "t3"])
    A = 1 - K * Tt
    Ap = -K1 * Tt - K * T1
    D = A ** 2 + T1 ** 2
    N = K * A ** 2 + A * T2 + K1 * Tt * T1 + 2 * K * T1 ** 2
    Np = K1 * A ** 2 + 2 * K * A * Ap + A * T3 + 3 * K1 * T1 ** 2 + 3 * K * T1 * T2   # reviewed N' (k''=0)
    J = Np * D - 3 * N * (A * Ap + T1 * T2)
    reviewed = J / D ** 3
    diff_general = sp.simplify(general * D ** 3 - J)
    diff_spiral = sp.simplify(diff_general.subs(K2, 0))
    lam = sp.lambdify((K, K1, K2, Tt, T1, T2, T3), general, "math")
    return {"rate_minus_reviewed_times_D3": str(sp.factor(diff_general)),
            "difference_when_k2_zero": str(diff_spiral),
            "reviewed_form_exact_for_spiral_reference": diff_spiral == 0}, lam


def truncated_power_identity():
    h = sp.symbols("h0:4", positive=True)
    t = [0, h[0], h[0] + h[1], h[0] + h[1] + h[2], sum(h)]
    # normalized cubic B-spline N(s) = (t4-t0) * sum_j (s - t_j)_+^3 / prod_{k!=j}(t_j - t_k)  (s in its support)
    coef = [sp.simplify((t[4] - t[0]) / sp.prod([t[j] - t[m] for m in range(5) if m != j])) for j in range(5)]
    c0, w = coef[0], -coef[1]
    claimed = (1 + h[0] / (h[1] + h[2] + h[3])) * (1 + h[0] / h[1]) * (1 + h[0] / (h[1] + h[2]))
    x = sp.symbols("x0:5", positive=True)
    M = sp.Matrix([[xi ** 3 for xi in x], [3 * xi ** 2 for xi in x], [6 * xi for xi in x]])
    minors = [sp.factor(M[:, list(cols)].det()) for cols in itertools.combinations(range(5), 3)]
    return {"c0_matches_claim": sp.simplify(c0 - 1 / (h[0] * (h[0] + h[1]) * (h[0] + h[1] + h[2]))) == 0,
            "w_over_c0_matches_claim": sp.simplify(w / c0 - claimed) == 0,
            "end_matrix_3x3_minor_example": str(minors[0]),
            "every_3x3_minor_nonzero_for_distinct_positive_x": all(
                sp.simplify(m.subs({x[0]: 1, x[1]: 2, x[2]: 3, x[3]: 5, x[4]: 7})) != 0 for m in minors)}


def baseline():
    road = ET.parse(ROOT / "out/workbench/wb11-source-tracks-20261009-v2/candidate.xodr").getroot().find("road[@id='10']")
    contract = json.loads((ROOT / "profiles/repair/0621-road10-local-shape-comparison-v1.json").read_text(), parse_float=str)  # exact decimals
    a, b = (F(x) for x in contract["domain"])
    geo = next(g for g in road.findall("planView/geometry") if F(g.get("s")) <= a < F(g.get("s")) + F(g.get("length")))
    sp_ = geo.find("spiral")
    assert sp_ is not None and F(geo.get("s")) + F(geo.get("length")) >= b
    k0 = F(sp_.get("curvStart")); sigma = (F(sp_.get("curvEnd")) - k0) / F(geo.get("length")); g0 = F(geo.get("s"))

    def affine(records, key, origin):
        e = max((r for r in records if origin + F(r.get(key)) <= a), key=lambda r: origin + F(r.get(key)))
        assert not any(a < origin + F(r.get(key)) < b for r in records) and F(e.get("c")) == F(e.get("d")) == 0
        s0 = origin + F(e.get(key)); return F(e.get("a")), F(e.get("b")), s0
    off = affine(road.findall("lanes/laneOffset"), "s", F(0))
    sec = road.find("lanes/laneSection[@s='20']")
    w1 = affine(sec.findall("right/lane[@id='-1']/width"), "sOffset", F(20))
    w2 = affine(sec.findall("right/lane[@id='-2']/width"), "sOffset", F(20))
    val = lambda p, s: p[0] + p[1] * (s - p[2])
    tracks = {"right-1-center": (lambda s: val(off, s) - val(w1, s) / 2, off[1] - w1[1] / 2),
              "right-2-center": (lambda s: val(off, s) - val(w1, s) - val(w2, s) / 2, off[1] - w1[1] - w2[1] / 2)}
    kappa = lambda s: k0 + sigma * (s - g0)
    return a, b, F(contract["comparison_stations_m"][1]), kappa, sigma, tracks


def main():
    rate_check, R = symbolic_rate()
    identity = truncated_power_identity()
    a, b, c, kappa, sigma, tracks = baseline()
    rold = lambda name, s, dq=(0.0, 0.0, 0.0, 0.0): R(float(kappa(s)), float(sigma), 0.0,
                                                    float(tracks[name][0](s)) + dq[0], float(tracks[name][1]) + dq[1], dq[2], dq[3])
    # 3. baseline sign and monotonicity on a dense grid over [a, b]
    stations = [a + (b - a) * F(i, 4000) for i in range(4001)]
    mono = {}
    for name in tracks:
        values = [rold(name, s) for s in stations]
        steps = [y - x for x, y in zip(values, values[1:])]
        mono[name] = {"rate_min": min(values), "rate_max": max(values),
                      "all_positive": min(values) > 0,
                      "increasing": all(d > 0 for d in steps), "decreasing": all(d < 0 for d in steps)}
    # 4. knot simplex grid: jets of -B0 and B1 at c by Cox-de Boor in floats
    L, H = float(b - a), float(b - a) - 12
    fa, fb, fc = float(a), float(b), float(c)

    def jets(knots, x):
        def N(i, p, x, d):
            if d > p: return 0.0
            if p == 0: return 1.0 if knots[i] <= x < knots[i + 1] else 0.0
            if d == 0:
                l = (x - knots[i]) / (knots[i + p] - knots[i]) * N(i, p - 1, x, 0)
                r = (knots[i + p + 1] - x) / (knots[i + p + 1] - knots[i + 1]) * N(i + 1, p - 1, x, 0)
                return l + r
            return p * (N(i, p - 1, x, d - 1) / (knots[i + p] - knots[i])
                        - N(i + 1, p - 1, x, d - 1) / (knots[i + p + 1] - knots[i + 1]))
        return [N(0, 3, x, d) for d in range(4)]

    reviewed = json.loads((REVIEWED / "derivation.json").read_text())
    ell = float(reviewed["linear_coefficient_lower_bound"]); cap = float(reviewed["sum_u_v_cap_m"])
    name = "right-2-center"; base = rold(name, c)
    eps = 1e-7
    steps = 15
    grid = [3 + (H - 3) * i / steps for i in range(steps + 1)]
    lin_min = {"u": math.inf, "v": math.inf}; worst_knots = {}; span2 = True; count = 0
    nonlinear_min = math.inf; nonlinear_count = 0
    for h in itertools.product(grid, repeat=4):
        h4 = L - sum(h)
        if not 3 - 1e-12 <= h4 <= H + 1e-12:
            continue
        t = [fa]
        for x in (*h, h4): t.append(t[-1] + x)
        t[-1] = fb
        span2 &= t[1] < fc < t[2]
        q_u = [-x for x in jets(t[0:5] + [t[5]], fc)]       # -B0 on knots t0..t4
        q_v = jets(t[1:6] + [t[5]], fc)                     # B1 on knots t1..t5
        count += 1
        for var, dq in (("u", q_u), ("v", q_v)):
            slope = (rold(name, c, [eps * x for x in dq]) - base) / eps   # d Rnew / d(var) at 0
            if slope < lin_min[var]:
                lin_min[var] = slope; worst_knots[var] = [round(x, 6) for x in (*h, h4)]
        if count % 97 == 0:  # sample the full nonlinear excess on a (u, v) grid inside the endpoint box
            for i, j in itertools.product(range(7), repeat=2):
                u, v = cap * i / 6, cap * j / 6
                if (u == 0 and v == 0) or u + v > cap: continue
                dq = [u * x + v * y for x, y in zip(q_u, q_v)]
                excess = rold(name, c, dq) - base
                nonlinear_min = min(nonlinear_min, excess / (u + v)); nonlinear_count += 1
    out = {"schema": "mapforge/review/nonuniform-five-span-algebra/v1",
           "symbolic_rate": rate_check, "basis_algebra": identity,
           "baseline_monotonicity_dense_4001": mono,
           "witness_station_in_span2_for_all_grid_knots": span2, "grid_knot_count": count, "grid_steps_per_span": steps,
           "linear_coefficient_min_on_grid_per_m_of_rate": lin_min,
           "linear_coefficient_argmin_spans_m": worst_knots,
           "note_units": "grid slope is dRnew/du (1/m^2 per m); the reviewed ell bounds the numerator P = (Rnew-Rold) D^3",
           "reviewed_ell": ell, "reviewed_cap_m": cap,
           "nonlinear_excess_over_r_min_on_grid": nonlinear_min, "nonlinear_samples": nonlinear_count,
           "baseline_rate_at_witness": base}
    (OUT / "algebra.json").write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf8")
    print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
