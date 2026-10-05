"""Tolerance-aware comparison for recomputed evidence and scoreboard baselines.

Bit-exact digests of recomputed floats are machine-bound: after the 2026-10
migration (AVX2-only CPU, same package versions) frozen node4 replays differed
by at most 5.1e-15 relative while inputs and code were byte-identical. Use
``equivalent`` when a recomputation is compared with a stored result; keep exact
SHA checks for stored bytes (sources, code, written files).
"""
import math


def equivalent(a, b, *, rel_tol=1e-9, abs_tol=1e-9, ignore=('content_sha256',)):
    """Structure, keys, strings, ints and bools exact; floats within tolerance."""
    if isinstance(a, dict) and isinstance(b, dict):
        keys = set(a) - set(ignore)
        return keys == set(b) - set(ignore) and all(
            equivalent(a[k], b[k], rel_tol=rel_tol, abs_tol=abs_tol, ignore=ignore) for k in keys)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(
            equivalent(x, y, rel_tol=rel_tol, abs_tol=abs_tol, ignore=ignore) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, float) or isinstance(b, float):
        return (isinstance(a, (int, float)) and isinstance(b, (int, float))
                and math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol))
    return a == b
