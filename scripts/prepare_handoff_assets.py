"""Build immutable, SHA-256-bound Git parts from explicitly selected local roots."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import zipfile

try:
    from .restore_handoff_assets import SCHEMA, member, safe_file, sha_file, validate
except ImportError:
    from restore_handoff_assets import SCHEMA, member, safe_file, sha_file, validate

PART_BYTES = 32 * 1024**2
OMIT = {".venv", "__pycache__", ".pytest_cache", ".transfers"}


def excluded(path):
    return (any(p in OMIT for p in path.parts) or path.suffix == ".pyc"
            or (path.name.startswith("server") and path.suffix == ".log")
            or path.name.startswith("session-") or path.name in {"session.json", "server.json"})


def build(workspace, output, name, roots, include=None):
    files, omitted = [], []
    for root in roots:
        member(root)
        directory = safe_file(workspace, root)
        if not directory.is_dir():
            raise ValueError(f"missing selected root: {root}")
        for path in sorted(directory.rglob("*")):
            if include is not None and path.relative_to(workspace).as_posix() not in include:
                continue
            relative = path.relative_to(workspace)
            if excluded(relative):
                if path.is_file():
                    omitted.append(relative.as_posix())
                continue
            safe_file(workspace, relative.as_posix())
            if path.is_file():
                # Session credentials must never enter the public asset bundle.
                if path.suffix.lower() in {".json", ".txt", ".md", ".html", ".xml", ".log"}:
                    data = path.read_bytes()
                    if b"#token=" in data or b'"token":' in data or b"Bearer " in data:
                        omitted.append(relative.as_posix())
                        continue
                files.append({"path": relative.as_posix(), "size": path.stat().st_size,
                              "sha256": sha_file(path)})
    bundle = {"id": name, "roots": roots, "files": files, "excluded_files": omitted}
    validate(bundle)
    with tempfile.TemporaryDirectory(prefix="handoff-build-") as temporary:
        archive = Path(temporary) / "assets.zip"
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
            for item in files:
                data_path = workspace / item["path"]
                info = zipfile.ZipInfo(item["path"], (2026, 10, 9, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                with data_path.open("rb") as source, z.open(info, "w", force_zip64=True) as sink:
                    h = hashlib.sha256()
                    while block := source.read(1024**2):
                        h.update(block)
                        sink.write(block)
                if h.hexdigest() != item["sha256"]:
                    raise ValueError(f"source changed while packaging: {item['path']}")
        bundle.update(size=archive.stat().st_size, sha256=sha_file(archive), parts=[])
        with archive.open("rb") as source:
            index = 0
            while block := source.read(PART_BYTES):
                index += 1
                relative = f"parts/{name}.{index:03}.part"
                destination = output / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open("xb") as sink:
                    sink.write(block)
                bundle["parts"].append({"path": relative, "size": len(block),
                                         "sha256": hashlib.sha256(block).hexdigest()})
    return bundle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", required=True, type=Path)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("immutable output already exists; choose a new directory")
    args.output.mkdir(parents=True)
    selection = json.loads(args.selection.read_text(encoding="utf-8"))
    bundles = []
    for entry in selection["bundles"]:
        bundle = build(args.workspace.resolve(), args.output, entry["id"], entry["roots"], entry.get("include"))
        bundles.append(bundle)
        print(json.dumps({"bundle": entry["id"], "files": len(bundle["files"]),
                          "archive_bytes": bundle["size"], "parts": len(bundle["parts"])}, ensure_ascii=False), flush=True)
    manifest = {"schema": SCHEMA, "purpose": "DEVELOPMENT_CONTINUATION_NOT_FORMAL_DELIVERY",
                "source_revision": selection["source_revision"], "bundles": bundles}
    (args.output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
