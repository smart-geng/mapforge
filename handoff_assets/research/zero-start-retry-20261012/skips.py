"""Every lane_refit 'skipped' entry in conversion transform.json files (anywhere in the JSON), with the path to it.
argv: glob root dir, glob pattern."""
import json, sys
from pathlib import Path


def walk(x, path):
    if isinstance(x, dict):
        for k, v in x.items():
            if k == "skipped" and isinstance(v, list) and v and isinstance(v[0], dict) and "road" in v[0]:
                yield path + [k], v
            yield from walk(v, path + [k])
    elif isinstance(x, list):
        for i, v in enumerate(x):
            yield from walk(v, path + [i])


for f in sorted(Path(sys.argv[1]).glob(sys.argv[2])):
    data = json.loads(f.read_text(encoding="utf-8"))
    for path, entries in walk(data, []):
        bad = [e for e in entries if "reason" in e and e["reason"] not in ("already refitted",)]
        if bad:
            print(f.name, "/".join(map(str, path)), json.dumps(bad, ensure_ascii=False)[:600])
