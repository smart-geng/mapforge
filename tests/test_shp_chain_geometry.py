# -*- coding: utf-8 -*-
"""SHP ROADLINK 拼链几何单测：只在真实连接端判断，不跨街角拼成长 leg。"""
import math

import numpy as np
import pytest

from mapforge.ops.shp_to_xodr import _corridor_turn, _join_angle


def _identity(x):
    return np.asarray(x, float)


def test_join_angle_uses_enter_join_endpoint_tangents():
    chain = [("current", np.array([[10.0, 0.0], [20.0, 0.0]]))]
    straight = np.array([[0.0, 0.0], [10.0, 0.0]])
    corner = np.array([[10.0, 10.0], [10.0, 0.0]])
    assert _join_angle(_identity, chain, straight, is_enter=True) == 0.0
    assert _join_angle(_identity, chain, corner, is_enter=True) == pytest.approx(
        math.radians(90.0), abs=1e-12)


def test_join_angle_uses_leave_join_endpoint_tangents():
    chain = [("current", np.array([[0.0, 0.0], [10.0, 0.0]]))]
    straight = np.array([[10.0, 0.0], [20.0, 0.0]])
    corner = np.array([[10.0, 0.0], [10.0, 10.0]])
    assert _join_angle(_identity, chain, straight, is_enter=False) == 0.0
    assert _join_angle(_identity, chain, corner, is_enter=False) == pytest.approx(
        math.radians(90.0), abs=1e-12)


def test_corridor_turn_rejects_link_that_itself_rounds_corner():
    straight = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
    corner = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    assert _corridor_turn(straight) == 0.0
    assert _corridor_turn(corner) == pytest.approx(math.radians(90.0), abs=1e-12)
