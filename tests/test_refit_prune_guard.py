"""Guard against a vertex pruning that throws a boundary far off its source (2026-10-13). Width profile shaped like
generalization 0711 road 11, right lane -2/-3: opening over the first 5 m of a free road start, 2.85 m wide up to s 60,
closing over 4.5 m into an advanced death at s 64.53 (protected, zero width), zero up to the section end."""
import numpy as np
import pytest

from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_prune_guard as G

DEATH, END = 64.53, 72.53
S = np.arange(0.25, END, 0.5)
P = {"mode": "g2", "kappa_cap": 0.04, "seg_min": 6.0, "rdp_tol": 0.05, "c2_ends": True, "short_kappa": True,
     "lsq_refine": True, "free_end_runs": True}


def _width(s):
    return np.interp(s, [0.0, 5.0, 60.0, DEATH, END], [0.1, -2.85, -2.85, 0.0, 0.0])


def _fit(**kw):
    info = {}
    pieces = LR.fit_boundary(S, _width(S), 0.0, END, end=(0.0, None), protect=[DEATH], protect_values={DEATH: 0.0},
                             info=info, **P, **kw)
    grid = np.arange(0.0, END, 0.1)
    return pieces, info, float(np.max(np.abs(LR._eval_many(pieces, grid) - _width(grid))))


def test_the_plain_pruning_turns_the_lane_into_one_ramp():
    _, info, dev = _fit()
    assert list(info["vs"]) == [0.0, DEATH, END]      # the free start's corner and the taper start both gone
    assert dev > 3.5


def test_the_guard_keeps_the_flat_width_and_both_tapers():
    LR.PRUNE_GUARDS.clear()
    pieces, info, dev = _fit(prune_guard=True)
    assert dev < 0.75                                  # the taper start moved out to SEG_MIN before the death
    flat = np.arange(12.0, 55.0, 0.5)
    assert np.max(np.abs(LR._eval_many(pieces, flat) + 2.85)) < 0.06
    assert min(np.diff(info["vs"])) >= P["seg_min"] - 1e-9       # runs still at least SEG_MIN
    assert DEATH in [float(x) for x in info["vs"]]               # the protected vertex stays where it was
    assert info["h"][1] == pytest.approx(info["vs"][1])          # the moved free-start run rounds its corner over all
    (record,) = LR.PRUNE_GUARDS
    assert record["used"] and record["guarded_error_m"] < record["pruned_error_m"]
    assert record["pruned_error_m"] > G.GUARD_M


def test_a_short_run_at_a_fixed_vertex_moves_its_free_vertex_to_seg_min():
    grid = np.arange(0.0, 20.0 + 1e-9, 0.5)
    vals = np.interp(grid, [0.0, 10.0, 14.0, 20.0], [0.0, 0.0, 2.0, 2.0])
    vs, vt = G.prune([0.0, 10.0, 14.0, 20.0], [0.0, 0.0, 2.0, 2.0], grid, vals, 6.0, protected={14.0})
    assert list(vs) == [0.0, 8.0, 14.0, 20.0] and list(vt) == [0.0, 0.0, 2.0, 2.0]


def test_a_pruning_close_to_its_vertices_is_left_alone():
    ref_s, ref_t = np.array([0.0, 10.0, 12.0, 30.0]), np.array([0.0, 0.0, 0.1, 0.1])
    vs, vt = np.array([0.0, 12.0, 30.0]), np.array([0.0, 0.1, 0.1])
    info = {}
    out = G.guard(ref_s, ref_t, vs, vt, ref_s, ref_t, 6.0, set(), False, False, False, None, info)
    assert out[0] is vs and out[1] is vt and info == {}


def test_a_pruning_that_does_not_settle_is_not_used(monkeypatch):
    monkeypatch.setattr(G, "_options", lambda vs, vt, *args: [(list(vs), list(vt))])   # a move that changes nothing
    ref_s, ref_t = [0.0, 10.0, 12.0, 30.0], [0.0, 0.0, 3.0, 3.0]
    assert G.prune(ref_s, ref_t, np.array(ref_s), np.array(ref_t), 6.0) is None
    vs, vt = np.array([0.0, 30.0]), np.array([0.0, 3.0])
    info = {}
    out = G.guard(ref_s, ref_t, vs, vt, np.array(ref_s), np.array(ref_t), 6.0, set(), False, False, False, None, info)
    assert out[0] is vs and out[1] is vt
    assert info == {"pruned_error_m": 1.8, "guarded_error_m": None, "used": False}


def test_error_is_the_largest_gap_between_the_two_polylines():
    assert G.error(np.array([0.0, 5.0, 10.0]), np.array([0.0, 1.0, 0.0]),
                   np.array([0.0, 10.0]), np.array([0.0, 0.0])) == pytest.approx(1.0)


def test_without_the_flag_the_fit_is_unchanged():
    a, _, _ = _fit()
    b, _, _ = _fit(prune_guard=False)
    assert a == b
