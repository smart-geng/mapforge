"""Which code files are bound by evidence: their current bytes' SHA-256 appears in out/ (or docs, experiments,
profiles, ledger). Changing such a file breaks the replay of that evidence (CLAUDE.md "证据绑定"), so new behaviour
goes into new modules (2026-10-08).

The scan only finds hashes written as 64 hex characters; CLAUDE.md lists further files bound in other ways
(cli.py, decision.py, tests/conftest.py, ...): treat the union as bound.

    .venv\\Scripts\\python scripts\\bound_files.py [path ...]      # no paths: list every bound .py file
"""
from __future__ import annotations

import hashlib
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEX = re.compile(rb"[0-9a-f]{64}")
TEXT = (".json", ".jsonl", ".md", ".txt", ".yaml", ".csv")


def recorded_hashes():
    seen = set()
    for dirpath, _dirs, files in os.walk(ROOT / "out"):
        for name in files:
            if name.endswith(TEXT):
                p = Path(dirpath) / name
                try:
                    if p.stat().st_size <= 50_000_000:
                        seen.update(HEX.findall(p.read_bytes()))
                except OSError:
                    pass
    for extra in ("experiments", "docs", "profiles", "ledger"):
        for p in (ROOT / extra).rglob("*"):
            if p.is_file() and p.suffix in (".json", ".jsonl", ".md", ".yaml"):
                seen.update(HEX.findall(p.read_bytes()))
    return seen


def main(argv):
    seen = recorded_hashes()
    paths = [Path(x) for x in argv] or [p for d in ("mapforge", "scripts", "spikes", "tests")
                                        for p in sorted((ROOT / d).rglob("*.py")) if "__pycache__" not in p.parts]
    for p in paths:
        p = p if p.is_absolute() else ROOT / p
        bound = hashlib.sha256(p.read_bytes()).hexdigest().encode() in seen
        if argv or bound:
            print(("BOUND  " if bound else "free   ") + p.relative_to(ROOT).as_posix())


if __name__ == "__main__":
    main(sys.argv[1:])
