import numpy as np

from scripts.map_raw_geometry_review import isolated_detours


def test_extreme_detour_is_review_only_and_does_not_mutate_source():
    points = np.array([[0., 0.], [1400., -800.], [0., 50.]])
    before = points.copy()
    rows = isolated_detours(points)
    assert len(rows) == 1 and rows[0]['point_index_0based'] == 1
    assert rows[0]['status'] == 'REVIEW_REQUIRED'
    assert rows[0]['source_action'] == 'NONE; original point retained'
    np.testing.assert_array_equal(points, before)


def test_review_does_not_call_an_ordinary_bend_an_isolated_kilometre_outlier():
    assert isolated_detours([[0., 0.], [3., 25.], [0., 50.]]) == []
    assert isolated_detours([[0., 0.], [200., 0.], [200., 200.]]) == []
    assert isolated_detours([]) == []
