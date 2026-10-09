"""Authenticate a historical diagnostic replay across native math libraries.

This compares reports only. It neither evaluates gates nor changes stored
metrics. Discrete decisions, policies, identities and every other field must
match exactly; only named metre measurements permit 1e-12 m roundoff.
"""
from __future__ import annotations

import math

_METRES = {
    ("coverage", "target_length_m"),
    ("source_to_target", "median_m"), ("source_to_target", "p95_m"),
    ("target_to_source", "median_m"), ("target_to_source", "p95_m"),
    ("endpoint", "travel_end_m"),
}
MAX_REPLAY_ROUNDOFF_M = 1e-12


def matches_fidelity_replay(saved, replay):
    def compare(a, b, path=()):
        if type(a) is not type(b):
            return False
        if isinstance(a, dict):
            return (a.keys() == b.keys()
                    and all(compare(a[k], b[k], (*path, k)) for k in a))
        if isinstance(a, list):
            return (len(a) == len(b)
                    and all(compare(x, y, (*path, i)) for i, (x, y) in enumerate(zip(a, b))))
        if isinstance(a, float):
            if not math.isfinite(a) or not math.isfinite(b):
                return False
            if a == b:
                return True
            return (len(path) == 4 and path[0] == "per_lane"
                    and type(path[1]) is int and path[2:] in _METRES
                    and abs(a - b) <= MAX_REPLAY_ROUNDOFF_M)
        return a == b
    return compare(saved, replay)
