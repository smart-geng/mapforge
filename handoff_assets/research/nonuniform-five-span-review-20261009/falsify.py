"""Independent end-to-end falsification attempt for the nonuniform five-span exclusion.

Reviewer-authored; imports nothing from the reviewed research.py. Builds actual candidate XML for sampled
knot placements and nonzero (u, v) and asks the production local-shape checker (mapforge.workbench.local_shape)
whether any of them shows NO_REGRESSION_FOUND. Basis functions come from an exact Cox-de Boor recursion, not
from the truncated-power formulas used in the reviewed derivation. Candidates stay in memory; nothing is
accepted or written as a map. A failed search is evidence, not a proof.
"""
from __future__ import annotations

from collections import Counter
from fractions import Fraction as F
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import time
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0, str(ROOT))
from mapforge.workbench import local_shape  # noqa: E402

BASE = ROOT / "out/workbench/wb11-source-tracks-20261009-v2/candidate.xodr"
CONTRACT = ROOT / "profiles/repair/0621-road10-local-shape-comparison-v1.json"
fl = lambda x: F.from_float(float(x))


# ---- exact polynomials (coefficient lists in absolute s) -------------------------------------------------
def padd(p, q):
    n = max(len(p), len(q))
    return [(p[i] if i < len(p) else 0) + (q[i] if i < len(q) else 0) for i in range(n)]


def pscale(p, c):
    return [c * v for v in p]


def pmul(p, q):
    out = [F(0)] * (len(p) + len(q) - 1)
    for i, x in enumerate(p):
        for j, y in enumerate(q):
            out[i + j] += x * y
    return out


def pderiv(p):
    return [i * p[i] for i in range(1, len(p))] or [F(0)]


def peval(p, s):
    return sum((c * s ** i for i, c in enumerate(p)), F(0))


def recentre(p, x0):
    """Coefficients in ds = s - x0 (Taylor at x0)."""
    out, d, fact = [], p, 1
    for k in range(4):
        out.append(peval(d, x0) / fact)
        d = pderiv(d)
        fact *= k + 1
    return out


def bspline(knots):
    """Exact cubic B-spline on knots[0..4] as {span index: polynomial}, by Cox-de Boor recursion."""
    def basis(i, p):
        if p == 0:
            return {i: [F(1)]}
        out = {}
        left, right = knots[i + p] - knots[i], knots[i + p + 1] - knots[i + 1]
        for span, poly in basis(i, p - 1).items():
            term = pmul([-knots[i] / left, 1 / left], poly)
            out[span] = padd(out.get(span, [F(0)]), term)
        for span, poly in basis(i + 1, p - 1).items():
            term = pmul([knots[i + p + 1] / right, -1 / right], poly)
            out[span] = padd(out.get(span, [F(0)]), term)
        return out
    return basis(0, 3)


def q_pieces(knots, u, v):
    """q = -u B0 + v B1 per span j (knots[j], knots[j+1]), j = 0..4."""
    b0, b1 = bspline(knots[0:5]), {k + 1: p for k, p in bspline(knots[1:6]).items()}
    return {j: padd(pscale(b0.get(j, [F(0)]), -u), pscale(b1.get(j, [F(0)]), v)) for j in range(5)}


def check_basis(knots):
    """Independent checks of the basis claims for one knot placement (exact)."""
    b0, b1 = bspline(knots[0:5]), {k + 1: p for k, p in bspline(knots[1:6]).items()}
    a, b = knots[0], knots[5]
    out = {}
    for name, fn in (("B0", b0), ("B1", b1)):
        jets = lambda j, s: [peval(d, s) for d in (fn.get(j, [F(0)]), pderiv(fn.get(j, [F(0)])), pderiv(pderiv(fn.get(j, [F(0)]))))]
        out[name + "_zero_c2_at_a"] = jets(0, a) == [0, 0, 0]
        out[name + "_zero_c2_at_b"] = jets(4, b) == [0, 0, 0]
        out[name + "_c2_at_inner_knots"] = all(jets(j - 1, knots[j]) == jets(j, knots[j]) for j in range(1, 5))
    out["B0_third_at_a_plus"] = float(peval(pderiv(pderiv(pderiv(b0[0]))), a))
    out["B1_third_at_b_minus"] = float(peval(pderiv(pderiv(pderiv(b1[4]))), b))
    return out


# ---- candidate XML ----------------------------------------------------------------------------------------
def fmt(x):
    return repr(float(x))


def records(parent, tag, key, origin):
    rows = []
    for e in parent.findall(tag):
        start = origin + fl(e.get(key))
        rows.append((start, [fl(e.get(c)) for c in "abcd"]))
    return rows


def absolute(start, local):
    """Local cubic in ds=s-start -> absolute-s polynomial."""
    p = [F(0)]
    shift = [-start, F(1)]
    power = [F(1)]
    for c in local:
        p = padd(p, pscale(power, c))
        power = pmul(power, shift)
    return p


def rewrite(parent, tag, key, origin, cuts, delta, a, b):
    """Split the records of one polynomial lane property at ``cuts`` and add ``delta`` on [a, b)."""
    rows = records(parent, tag, key, origin)
    starts = sorted({s for s, _ in rows} | set(cuts))
    template = parent.findall(tag)
    insert_at = list(parent).index(template[0])
    for e in template:
        parent.remove(e)
    for i, x in enumerate(starts):
        start, local = max((r for r in rows if r[0] <= x), key=lambda r: r[0])
        poly = absolute(start, local)
        if a <= x < b:
            poly = padd(poly, delta(x))
        coeff = recentre(poly, x)
        e = ET.Element(tag, {key: fmt(x - origin), **{c: fmt(v) for c, v in zip("abcd", coeff)}})
        e.tail = template[0].tail
        parent.insert(insert_at + i, e)


def candidate(base_root_bytes, knots, u, v):
    root = ET.fromstring(base_root_bytes)
    road = root.find("road[@id='10']")
    a, b = knots[0], knots[5]
    pieces = q_pieces(knots, u, v)

    def q_at(x):
        j = max(i for i in range(5) if knots[i] <= x)
        return pieces[j]
    cuts = list(knots)
    rewrite(road.find("lanes"), "laneOffset", "s", F(0), cuts, q_at, a, b)
    sec = road.find("lanes/laneSection[@s='20']")
    median = sec.find("left/lane[@id='1']")
    rewrite(median, "width", "sOffset", fl(sec.get("s")), cuts, lambda x: pscale(q_at(x), -1), a, b)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def knot_sets(rng, count):
    contract = json.loads(CONTRACT.read_text())
    a, b = map(fl, contract["domain"])
    L = b - a
    H = L - 12
    sets = [("equal", [L / 5] * 5)]
    for i in range(5):  # every simplex vertex: one span at its maximum
        sets.append((f"vertex{i}", [H if j == i else F(3) for j in range(5)]))
    while len(sets) < count:
        x = [rng.random() for _ in range(5)]
        h = [F(3) + (L - 15) * F(xi).limit_denominator(10 ** 6) / F(sum(x)).limit_denominator(10 ** 6) for xi in x]
        h[4] = L - sum(h[:4])
        if all(F(3) <= hi <= H for hi in h):
            sets.append((f"random{len(sets)}", h))
    out = []
    for name, h in sets:
        t = [a]
        for hi in h[:4]:
            t.append(fl(t[-1] + hi))  # stations exactly representable in the XML
        t.append(b)
        out.append((name, t))
    return out


ANGLES = [2 * math.pi * k / 16 for k in range(16)]
MAGNITUDES = [F(1, 10000), F(1, 1000), F(1, 100), F(3, 100)]


def sweep(job):
    name, knots = job
    base = BASE.read_bytes()
    contract = json.loads(CONTRACT.read_text())
    basis = check_basis(knots)
    rows = []
    for angle in ANGLES:
        for m in MAGNITUDES:
            u = fl(m * math.cos(angle)) if abs(math.cos(angle)) > 1e-12 else F(0)
            v = fl(m * math.sin(angle)) if abs(math.sin(angle)) > 1e-12 else F(0)
            report = local_shape.compare(base, candidate(base, knots, u, v), contract)
            issues = report["issues"]
            rows.append({"knots": name, "spans_m": [float(knots[i + 1] - knots[i]) for i in range(5)],
                         "u_m": float(u), "v_m": float(v), "status": report["status"],
                         "issue_codes": sorted({i.get("code") for i in issues}),
                         "rate_regressions": sorted({(i.get("lane_id"), i.get("curve"), tuple(i.get("domain", ())))
                                                     for i in issues if i.get("code") == "local-rate-regression"}),
                         "unavailable_reason": issues[0].get("reason") if report["status"] == "UNAVAILABLE" and issues else None,
                         "basis_checks": basis})
    return name, rows


def main():
    from multiprocessing import Pool
    base = BASE.read_bytes()
    assert hashlib.sha256(base).hexdigest() == "9aed2e47be1fc04deee6220d7f4bccdcc02004f65e31c99c1d7e3888aa96c224"
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    jobs = knot_sets(random.Random(20261009), count)
    rows, statuses = [], Counter()
    started = time.monotonic()
    with Pool(4) as pool:
        for name, part in pool.imap_unordered(sweep, jobs):
            rows += part
            statuses.update(r["status"] for r in part)
            print(json.dumps({"knots": name, "done": len(rows), "statuses": dict(statuses),
                              "elapsed_s": round(time.monotonic() - started, 1)}), flush=True)
    found = [r for r in rows if r["status"] == "NO_REGRESSION_FOUND"]
    summary = {"schema": "mapforge/review/nonuniform-five-span-falsification/v1", "samples": len(rows),
               "knot_sets": [{"name": n, "stations_m": [float(x) for x in k]} for n, k in jobs],
               "angles": 16, "magnitudes_m": [float(m) for m in MAGNITUDES],
               "statuses": dict(statuses), "no_regression_counterexamples": found,
               "checker": "mapforge.workbench.local_shape.compare (production, unmodified)",
               "reviewed_research_imported": False,
               "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "rows": rows}
    (OUT / "falsification.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=list), encoding="utf8")
    print(json.dumps({k: summary[k] for k in ("samples", "statuses")}), "counterexamples", len(found))


if __name__ == "__main__":
    main()
