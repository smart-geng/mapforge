"""Exact rational linear activity coefficients of P=(Rnew-Rold)*Dnew^3 at the witness station, at chosen knots.

Reviewer-authored. Reuses only the reviewer's own exact Cox-de Boor basis (falsify.py) and the offset-curve rate
form confirmed symbolically in review_algebra.py (k''=0 on the road 10 spiral). Compares with the reviewed ell.
"""
from fractions import Fraction as F
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0, str(OUT))
from falsify import bspline, peval, pderiv  # noqa: E402

road = ET.parse(ROOT / "out/workbench/wb11-source-tracks-20261009-v2/candidate.xodr").getroot().find("road[@id='10']")
contract = json.loads((ROOT / "profiles/repair/0621-road10-local-shape-comparison-v1.json").read_text(), parse_float=str)
a, b = (F(x) for x in contract["domain"]); c = F(contract["comparison_stations_m"][1])
g = next(g for g in road.findall("planView/geometry") if g.get("s") == contract["domain"][0])
k0 = F(g.find("spiral").get("curvStart")); sigma = (F(g.find("spiral").get("curvEnd")) - k0) / F(g.get("length"))
def rec(records, key, origin):
    e = max((r for r in records if origin + F(r.get(key)) <= a), key=lambda r: origin + F(r.get(key)))
    return F(e.get("a")), F(e.get("b")), origin + F(e.get(key))
off = rec(road.findall("lanes/laneOffset"), "s", F(0)); sec = road.find("lanes/laneSection[@s='20']")
w1 = rec(sec.findall("right/lane[@id='-1']/width"), "sOffset", F(20)); w2 = rec(sec.findall("right/lane[@id='-2']/width"), "sOffset", F(20))
val = lambda p, s: p[0] + p[1] * (s - p[2])
t0 = val(off, c) - val(w1, c) - val(w2, c) / 2; t1 = off[1] - w1[1] - w2[1] / 2
k = k0 + sigma * (c - a)

def J_D(t, tp, tpp, tppp):
    A = 1 - k * t; Ap = -sigma * t - k * tp; D = A * A + tp * tp
    N = k * A * A + A * tpp + sigma * t * tp + 2 * k * tp * tp
    Np = sigma * A * A + 2 * k * A * Ap + A * tppp + 3 * sigma * tp * tp + 3 * k * tp * tpp
    return Np * D - 3 * N * (A * Ap + tp * tpp), D

J0, D0 = J_D(t0, t1, F(0), F(0)); Rold = J0 / D0 ** 3

def coefficient(jets):
    """Exact d/dε of P(ε) = J(base+ε·jets) - Rold·D(base+ε·jets)^3 at ε=0 (P is polynomial in ε)."""
    eps = F(1, 10 ** 30)
    def P(e):
        J, D = J_D(t0 + e * jets[0], t1 + e * jets[1], e * jets[2], e * jets[3]); return J - Rold * D ** 3
    # exact derivative of a polynomial: use symmetric difference on two tiny steps and Richardson (exact rationals)
    d1 = (P(eps) - P(-eps)) / (2 * eps); d2 = (P(2 * eps) - P(-2 * eps)) / (4 * eps)
    return (4 * d1 - d2) / 3

reviewed = json.loads((OUT.parent / "derivation.json").read_text())
ell = F(reviewed["linear_coefficient_lower_bound_exact"])
L = b - a; H = L - 12
results = []
for name, h in [("h1=H vertex", [F(3), H, F(3), F(3), F(3)]), ("h0=H vertex", [H, F(3), F(3), F(3), F(3)]),
                ("h2=H vertex", [F(3), F(3), H, F(3), F(3)]), ("h3=H vertex", [F(3), F(3), F(3), H, F(3)]),
                ("h4=H vertex", [F(3), F(3), F(3), F(3), H]), ("equal", [L / 5] * 5)]:
    t = [a]
    for x in h: t.append(t[-1] + x)
    assert t[-1] == b and t[1] < c < t[2]
    B0, B1 = bspline(t[0:5]), {kk + 1: p for kk, p in bspline(t[1:6]).items()}
    jet = lambda poly: [peval(poly, c), peval(pderiv(poly), c), peval(pderiv(pderiv(poly)), c), peval(pderiv(pderiv(pderiv(poly))), c)]
    ju = [-x for x in jet(B0[1])]; jv = jet(B1[1])
    cu, cv = coefficient(ju), coefficient(jv)
    results.append({"knots": name, "coef_u": float(cu), "coef_v": float(cv),
                    "coef_u_minus_ell": float(cu - ell), "coef_v_minus_ell": float(cv - ell),
                    "below_reviewed_ell": bool(min(cu, cv) < ell)})
print(json.dumps({"D0": float(D0), "Rold": float(Rold), "reviewed_ell": float(ell), "results": results}, indent=1))
