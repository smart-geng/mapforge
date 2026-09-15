"""Explicit research-code migration; raw/config/approved role revisions freeze."""
from pathlib import Path

from scripts.gen_all import _sha256

ROOT = Path(__file__).resolve().parents[1]


def bind_current_code(previous):
    hashes = {}; changes = []
    for name, expected in previous.items():
        path = Path(name).resolve(); current = _sha256(path)
        if current != expected:
            allowed = path.suffix == '.py' and any(path.is_relative_to(ROOT/d) for d in ('mapforge','spikes','scripts'))
            if not allowed:
                raise ValueError('immutable research input changed: '+name)
            changes.append(dict(file=name,previous_sha256=expected,current_sha256=current))
        hashes[name] = current
    return hashes, changes
