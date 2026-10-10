"""Lane refit, advanced birth/death station (2026-10-10): where the source lane runs on past the section boundary the
converter closed it at, still open and narrowing, the event is where its taper reaches zero. Profile shaped like
0621 road 11 lane 4: 3.2 m wide, a 1:4 taper from s = 51.6 reaching zero at 64.3, section boundary at 63.38."""
import numpy as np

from mapforge.ops import lane_refit as LR

X, D = 63.3837, 6.71
S = np.arange(0.0, 64.01, 0.5)      # the source runs to 64.0, 0.6 m past the boundary
W = np.interp(S, [0.0, 51.6, 64.3], [3.2, 3.2, 0.0])


def test_a_lane_still_open_at_its_section_boundary_dies_where_its_source_taper_reaches_zero():
    s, w = np.append(S, 64.3), np.append(W, 0.0)
    assert abs(LR._source_event("death", X, D, s, w) - 64.3) < 1e-6


def test_the_event_is_never_extrapolated_past_the_source_lane_tip():
    """shp-node17 road 12 lane 4: the source lane starts 0.13 m wide; its zero would lie past its tip."""
    assert LR._source_event("death", X, D, S, W) == S[-1]


def test_the_event_stays_inside_the_zero_width_extension():
    assert LR._source_event("death", X, 3.1, S, W) == X + 3.1 - LR.H_MIN


def test_a_lane_already_closed_at_its_section_boundary_keeps_it():
    closed = np.interp(S, [0.0, 51.0, 63.4], [3.2, 3.2, 0.0])
    assert LR._source_event("death", X, D, S, closed) == X


def test_no_source_past_the_boundary_keeps_it():
    sel = S <= X
    assert LR._source_event("death", X, D, S[sel], W[sel]) == X


def test_a_width_that_is_not_narrowing_keeps_it():
    flat = np.where(S > 60.0, 0.5, W)
    assert LR._source_event("death", X, D, S, flat) == X


def test_free_end_events_without_an_extension_keep_their_station():
    assert LR._source_event("death", X, None, S, W) == X


def test_births_mirror_deaths():
    s = -np.append(S, 64.3)[::-1]   # the same taper seen from the other side: birth at -63.38, zero at -64.3
    w = np.append(W, 0.0)[::-1]
    assert abs(LR._source_event("birth", -X, D, s, w) + 64.3) < 1e-6
