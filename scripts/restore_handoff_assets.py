"""Restore verified Git handoff parts with Python's standard library only.

Existing data is never overwritten. A matching root is an idempotent no-op;
any differing or incomplete root blocks the entire selected bundle.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
import zipfile

SCHEMA = "mapforge/git-handoff/v1"
MAX_EXPANDED = 8 * 1024**3
MAX_PART = 48 * 1024**2
SHA = re.compile(r"[0-9a-f]{64}")


def sha_file(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            h.update(block)
    return h.hexdigest()


def member(name):
    if not isinstance(name, str) or not name or "\\" in name or ":" in name:
        raise ValueError("unsafe asset path")
    parts = name.split("/")
    reserved = {"CON", "PRN", "AUX", "NUL", *[f"COM{i}" for i in range(1, 10)],
                *[f"LPT{i}" for i in range(1, 10)]}
    if any(p in {"", ".", ".."} or p.rstrip(" .") != p or len(p) > 255
           or p.split(".")[0].upper() in reserved
           or any(ord(c) < 32 or c in '<>"|?*' for c in p) for p in parts):
        raise ValueError("unsafe asset path")
    return parts


def safe_file(root, name):
    parts = member(name)
    current = root
    for part in parts:
        current = current / part
        if current.is_symlink() or getattr(current, "is_junction", lambda: False)():
            raise ValueError("asset path traverses a link")
        if current.exists():
            attrs = getattr(current.lstat(), "st_file_attributes", 0)
            if attrs & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400):
                raise ValueError("asset path traverses a reparse point")
    return current


def validate(bundle):
    roots = bundle["roots"]
    if not roots or len(set(roots)) != len(roots):
        raise ValueError("invalid asset roots")
    for root in roots:
        member(root)
    folded_roots = [r.casefold() for r in roots]
    if any(a == b or a.startswith(b + "/") or b.startswith(a + "/")
           for i, a in enumerate(folded_roots) for b in folded_roots[i + 1:]):
        raise ValueError("overlapping asset roots")
    files, folded, total, spellings = {}, set(), 0, {}
    for item in bundle["files"]:
        name = item["path"]
        parts = member(name)
        for i in range(1, len(parts) + 1):
            prefix = "/".join(parts[:i])
            if spellings.setdefault(prefix.casefold(), prefix) != prefix:
                raise ValueError("case alias in asset paths")
        if not any(name.startswith(r + "/") for r in roots):
            raise ValueError("file outside asset roots")
        if name.casefold() in folded:
            raise ValueError("duplicate asset path")
        if type(item["size"]) is not int or item["size"] < 0 or not SHA.fullmatch(item["sha256"]):
            raise ValueError("invalid asset binding")
        folded.add(name.casefold())
        files[name] = item
        total += item["size"]
    if not files or total > MAX_EXPANDED:
        raise ValueError("asset expanded budget exceeded")
    if any(parent.casefold() in folded for name in files
           for parent in ["/".join(name.split("/")[:i]) for i in range(1, len(name.split("/")))]):
        raise ValueError("file also used as directory")
    return files


def restore_bundle(asset_dir, bundle, target):
    asset_dir, target = Path(asset_dir).resolve(), Path(target).resolve()
    files = validate(bundle)
    if not SHA.fullmatch(bundle["sha256"]):
        raise ValueError("invalid archive binding")
    target.mkdir(parents=True, exist_ok=True)
    # Check every destination before extracting or publishing anything.
    missing = []
    for root in bundle["roots"]:
        destination = safe_file(target, root)
        if destination.exists():
            if not destination.is_dir():
                raise ValueError(f"existing root is not a directory: {root}")
            for name, item in files.items():
                if name.startswith(root + "/"):
                    p = safe_file(target, name)
                    if not p.is_file() or p.stat().st_size != item["size"] or sha_file(p) != item["sha256"]:
                        raise ValueError(f"existing data differs; no overwrite: {name}")
        else:
            missing.append(root)
    with tempfile.TemporaryDirectory(prefix=".handoff-", dir=target) as temporary:
        stage = Path(temporary)
        archive = stage / "bundle.zip"
        whole = hashlib.sha256()
        total = 0
        with archive.open("xb") as sink:
            seen_parts = set()
            for item in bundle["parts"]:
                name = item["path"]
                part = safe_file(asset_dir, name)
                if name in seen_parts or type(item["size"]) is not int or not 0 < item["size"] <= MAX_PART:
                    raise ValueError("invalid asset part")
                seen_parts.add(name)
                if not SHA.fullmatch(item["sha256"]) or part.stat().st_size != item["size"] or sha_file(part) != item["sha256"]:
                    raise ValueError(f"asset part checksum failed: {name}")
                with part.open("rb") as source:
                    for block in iter(lambda: source.read(1024**2), b""):
                        whole.update(block)
                        sink.write(block)
                        total += len(block)
        if total != bundle["size"] or whole.hexdigest() != bundle["sha256"]:
            raise ValueError("archive checksum failed")
        expanded = stage / "expanded"
        with zipfile.ZipFile(archive) as z:
            infos = z.infolist()
            if len(infos) != len(files) or {i.filename for i in infos} != set(files):
                raise ValueError("archive members differ from manifest")
            for info in infos:
                item = files[info.filename]
                mode = info.external_attr >> 16
                if info.is_dir() or stat.S_ISLNK(mode) or info.flag_bits & 1 or info.file_size != item["size"]:
                    raise ValueError("unsafe archive member")
                output = safe_file(expanded, info.filename)
                output.parent.mkdir(parents=True, exist_ok=True)
                h = hashlib.sha256()
                count = 0
                with z.open(info) as source, output.open("xb") as sink:
                    for block in iter(lambda: source.read(1024**2), b""):
                        count += len(block)
                        if count > item["size"]:
                            raise ValueError("expanded file exceeds binding")
                        h.update(block)
                        sink.write(block)
                if count != item["size"] or h.hexdigest() != item["sha256"]:
                    raise ValueError(f"expanded file checksum failed: {info.filename}")
        published = []
        try:
            for root in missing:
                destination = safe_file(target, root)
                destination.parent.mkdir(parents=True, exist_ok=True)
                if destination.exists():
                    raise ValueError("destination appeared during restoration")
                os.rename(expanded / root, destination)
                published.append(root)
        except BaseException:
            # Only roll back directories published by this invocation.
            for root in reversed(published):
                os.rename(safe_file(target, root), expanded / root)
            raise
    return {"bundle": bundle["id"], "restored_roots": missing,
            "unchanged_roots": [r for r in bundle["roots"] if r not in missing],
            "files": len(files), "sha256": bundle["sha256"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--target", type=Path, default=Path.cwd())
    parser.add_argument("--bundle", action="append", help="repeat to select; default is all")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if manifest["schema"] != SCHEMA:
        parser.error("unsupported handoff manifest")
    bundles = manifest["bundles"]
    selected = set(args.bundle or [b["id"] for b in bundles])
    if selected - {b["id"] for b in bundles}:
        parser.error("unknown bundle")
    for bundle in bundles:
        if bundle["id"] in selected:
            print(json.dumps(restore_bundle(args.manifest.parent, bundle, args.target), ensure_ascii=False))


if __name__ == "__main__":
    main()
