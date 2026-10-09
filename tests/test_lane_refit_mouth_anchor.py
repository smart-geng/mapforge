"""Lane refit, mouth anchor (2026-10-09): a corner dropped from a junction-mouth zone keeps the run before it on
the source. Profile shaped like 0621 road 10's median edge: flat, a 0.11 m rise over 16 m, a corner 7.8 m before
the mouth and 0.26 m back into it."""
import numpy as np
import pytest

from mapforge.ops import lane_refit as LR

P = {"mode": "g2", "kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05, "c2_ends": True, "lsq_refine": True,
     "short_kappa": True}
B = 94.25
S = np.arange(0.0, B + 0.01, 0.5)
T = np.interp(S, [0.0, 70.0, 86.5, B], [3.43, 3.43, 3.544, 3.28])


def _fit(**kw):
    info = {}
    pieces = LR.fit_boundary(S, T, 0.0, B, end=(None, "mouth"), info=info, **P, **kw)
    return pieces, info


def _dev(pieces, lo=0.0, hi=B):
    sel = (S >= lo) & (S <= hi)
    return max(abs(LR._at(pieces, x) - y) for x, y in zip(S[sel], T[sel]))


def test_without_the_anchor_the_dropped_corner_pulls_the_run_before_the_zone_off():
    pieces, info = _fit()
    assert _dev(pieces, 70.0, B - LR.MOUTH_ZONE_M) > 0.15
    assert not any(B - LR.MOUTH_ZONE_M < v < B for v in info["vs"])


def test_with_the_anchor_the_run_before_the_zone_follows_the_source():
    pieces, info = _fit(mouth_anchor=True)
    zone_edge = B - LR.MOUTH_ZONE_M
    assert _dev(pieces, 0.0, zone_edge) < 0.03
    assert _dev(pieces) < 0.06
    # still no corner inside the zone: one vertex at its edge, then one run into the mouth
    assert any(abs(v - zone_edge) <= LR.GRID for v in info["vs"])
    assert not any(zone_edge + LR.GRID < v < B for v in info["vs"])
    # the mouth slope stays capped and the boundary stays C2
    assert abs(LR._at(pieces, B, 1)) <= LR.MOUTH_SLOPE_CAP + 1e-9
    gaps = [abs(LR._eval(p, p[1], o) - LR._eval(q, q[0], o)) for p, q in zip(pieces[:-1], pieces[1:]) for o in (0, 1, 2)]
    assert max(gaps) < 1e-9


def test_a_mouth_zone_without_a_source_corner_is_unchanged():
    straight = 3.43 + 0.002 * S
    plain = LR.fit_boundary(S, straight, 0.0, B, end=(None, "mouth"), **P)
    anchored = LR.fit_boundary(S, straight, 0.0, B, end=(None, "mouth"), mouth_anchor=True, **P)
    assert plain == anchored


def test_a_flare_deep_in_the_zone_is_still_left_out():
    """shp-node17 road 10: the source turns out 0.5-4.5 m before the mouth (a curb-return flare). No straight run
    from the zone edge into the mouth, so no anchor: the result equals the fit without the option."""
    flare = np.interp(S, [0.0, B - 4.5, B], [-4.80, -4.80, -5.15])
    plain = LR.fit_boundary(S, flare, 0.0, B, end=(None, "mouth"), **P)
    anchored = LR.fit_boundary(S, flare, 0.0, B, end=(None, "mouth"), mouth_anchor=True, **P)
    assert plain == anchored


def test_start_mouths_are_anchored_the_same_way():
    flipped = T[::-1]
    pieces = LR.fit_boundary(S, flipped, 0.0, B, start=(None, "mouth"), mouth_anchor=True, **P)
    sel = S >= LR.MOUTH_ZONE_M
    assert max(abs(LR._at(pieces, x) - y) for x, y in zip(S[sel], flipped[sel])) < 0.03


def test_the_default_variant_anchors_mouths():
    assert LR.VARIANTS["g2-k04-c2"]["mouth_anchor"] is True
    assert "mouth_anchor" not in LR.VARIANTS["g2-k04"]      # earlier registered variants keep their meaning
    import inspect
    assert inspect.signature(LR.fit_boundary).parameters["mouth_anchor"].default is False
