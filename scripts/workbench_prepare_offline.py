"""Prepare and verify exact Windows CPython 3.11 wheels from the existing lock.

Examples (run with the locked development Python):
  python scripts/workbench_prepare_offline.py plan --out out/offline-media-v1
  python scripts/workbench_prepare_offline.py download --out out/offline-media-v1 --proxy socks5h://172.18.38.39:10086
  python scripts/workbench_prepare_offline.py verify --out out/offline-media-v1

``verify`` never downloads or repairs anything. Download uses curl's documented
--proxy/--noproxy/--proto/--max-filesize options and exact HTTPS URLs already in
uv.lock; it does not query an index, change versions, build sdists or install.
This is wheelhouse preparation, not a complete installer or release approval.
"""
from __future__ import annotations

import argparse
from email.parser import BytesParser
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tomllib
from urllib.parse import unquote, urlsplit
from uuid import uuid4
import zipfile

from packaging.specifiers import SpecifierSet
from packaging.tags import compatible_tags, cpython_tags, parse_tag
from packaging.utils import canonicalize_name, parse_wheel_filename
from packaging.version import Version

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "mapforge/offline-wheelhouse/v1"
TARGET = {"implementation": "cpython", "python": "3.11.16", "abi": "cp311", "platform": "win_amd64"}
MANIFEST = "wheelhouse-manifest.json"
REQUIREMENTS = "requirements.lock.txt"


class PreparationError(ValueError):
    pass


def _json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _file_sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _link(path):
    if path.is_symlink():
        return True
    try:
        return bool(getattr(path.lstat(), "st_file_attributes", 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT)
    except FileNotFoundError:
        return False


def _root(directory, *, create=False):
    path = Path(directory).absolute()
    if any(_link(p) for p in (path, *path.parents)):
        raise PreparationError("介质目录及父目录不得使用符号链接或 junction")
    if create:
        path.mkdir(parents=True, exist_ok=True)
    if not path.is_dir():
        raise PreparationError("介质目录不存在")
    return path.resolve()


def _path(root, name):
    target = root / name
    if any(_link(p) for p in (target, *target.parents) if p.is_relative_to(root)):
        raise PreparationError("介质文件不得使用链接或 junction")
    if not target.resolve().is_relative_to(root):
        raise PreparationError("介质文件路径越界")
    return target


def _write_new_or_identical(path, data):
    if path.exists():
        if path.read_bytes() != data:
            raise PreparationError(f"已有登记文件内容不同，保留原件：{path.name}")
        return
    temporary = path.with_name("." + path.name + "." + uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        # Windows rename refuses to overwrite an existing destination. A
        # completed file is immutable; retries only reuse identical bytes.
        if path.exists():
            raise PreparationError("介质文件在写入期间出现，请重新核对")
        os.rename(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def target_tags():
    """Explicit target, independent of the preparing host's Python/platform."""
    return tuple(dict.fromkeys([
        *cpython_tags(python_version=(3, 11), abis=["cp311"], platforms=["win_amd64"]),
        *compatible_tags(python_version=(3, 11), interpreter="cp311", platforms=["win_amd64"]),
    ]))


def _wheel_url(url):
    if not isinstance(url, str):
        raise PreparationError("wheel URL 必须来自锁文件")
    parsed = urlsplit(url)
    if (parsed.scheme != "https" or parsed.hostname != "files.pythonhosted.org"
            or parsed.netloc != "files.pythonhosted.org" or parsed.query or parsed.fragment
            or not parsed.path.startswith("/packages/")):
        raise PreparationError("仅接受锁定的 files.pythonhosted.org HTTPS wheel URL")
    filename = unquote(parsed.path.rsplit("/", 1)[-1])
    if not re.fullmatch(r"[A-Za-z0-9_.+!-]+\.whl", filename):
        raise PreparationError("wheel 文件名无效或含路径字符")
    return filename


def build_plan(lock_path=ROOT / "uv.lock"):
    raw = Path(lock_path).read_bytes()
    lock = tomllib.loads(raw.decode("utf-8-sig"))
    if lock.get("version") != 1:
        raise PreparationError("只支持 uv.lock version=1；不能猜测新锁格式")
    try:
        python_range = SpecifierSet(lock["requires-python"])
    except (KeyError, ValueError) as exc:
        raise PreparationError("锁文件缺少有效 Python 版本约束") from exc
    if Version(TARGET["python"]) not in python_range:
        raise PreparationError("锁文件不支持指定的 Python 3.11.16")
    packages = lock.get("package")
    if not isinstance(packages, list) or not packages:
        raise PreparationError("锁文件没有 package 清单")
    ranks = {tag: i for i, tag in enumerate(target_tags())}
    chosen, seen = [], set()
    for package in packages:
        name = canonicalize_name(package["name"], validate=True)
        version = Version(package["version"])
        source = package.get("source", {})
        if name == "mapforge" and source == {"virtual": "."}:
            continue
        if name in seen:
            raise PreparationError("同名多版本锁暂不支持，必须明确审查目标解析分支")
        seen.add(name)
        if source != {"registry": "https://pypi.org/simple"}:
            raise PreparationError(f"不支持的依赖来源：{name}；不回落到源代码构建")
        candidates = []
        for wheel in package.get("wheels", []):
            filename = _wheel_url(wheel.get("url"))
            wheel_name, wheel_version, _, tags = parse_wheel_filename(filename)
            if wheel_name != name or wheel_version != version:
                raise PreparationError(f"wheel 身份与锁定包不一致：{filename}")
            accepted = tags.intersection(ranks)
            if not accepted:
                continue
            digest = wheel.get("hash", "")
            size = wheel.get("size")
            if not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
                raise PreparationError(f"wheel 缺少有效 SHA256：{filename}")
            if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
                raise PreparationError(f"wheel 缺少有效长度：{filename}")
            item = {"name": str(name), "version": str(version), "filename": filename,
                    "url": wheel["url"], "sha256": digest.split(":", 1)[1], "size_bytes": size,
                    "wheel_tags": sorted(str(tag) for tag in tags),
                    "selected_target_tag": str(min(accepted, key=ranks.get))}
            candidates.append((min(ranks[tag] for tag in accepted), filename, item))
        if not candidates:
            raise PreparationError(f"锁文件没有 CPython3.11/Win x64 兼容 wheel：{name}=={version}；禁止回落 sdist")
        chosen.append(min(candidates, key=lambda item: (item[0], item[1]))[2])
    if not chosen:
        raise PreparationError("锁文件没有第三方 wheel")
    return {"schema": SCHEMA, "target": TARGET.copy(), "lock_sha256": _sha(raw),
            "scope": "all-non-virtual-packages-in-lock-including-all-groups",
            "wheels": sorted(chosen, key=lambda item: item["name"]),
            "package_count": len(chosen), "download_size_bytes": sum(w["size_bytes"] for w in chosen),
            "install_acceptance": False, "redistribution_approval": False}


def _manifest(plan):
    return _json({"schema": SCHEMA, "state": "PLAN_ONLY", "plan_sha256": _sha(_json(plan)), "plan": plan}) + b"\n"


def _requirements(plan):
    text = "# CPython 3.11.16 / Windows x64; exact selected wheel hashes from uv.lock.\n"
    text += "# Use --offline --no-index --find-links wheelhouse --require-hashes --only-binary :all:.\n"
    return (text + "".join(f"{w['name']}=={w['version']} --hash=sha256:{w['sha256']}\n" for w in plan["wheels"])).encode()


def prepare_plan(lock_path, directory):
    plan = build_plan(lock_path)
    root = _root(directory, create=True)
    wheelhouse = _path(root, "wheelhouse")
    if wheelhouse.exists() and not wheelhouse.is_dir():
        raise PreparationError("wheelhouse 必须是目录")
    _write_new_or_identical(_path(root, MANIFEST), _manifest(plan))
    _write_new_or_identical(_path(root, REQUIREMENTS), _requirements(plan))
    wheelhouse.mkdir(exist_ok=True)
    return plan


def verify_wheel(path, item):
    """Read archive bytes and metadata without extracting or importing code."""
    path = Path(path)
    if _link(path) or not path.is_file():
        raise PreparationError("wheel 缺失或使用了链接")
    if path.stat().st_size == 0:
        raise PreparationError("wheel 是零字节占位文件")
    if path.stat().st_size != item["size_bytes"]:
        raise PreparationError("wheel 长度与锁文件不一致")
    if _file_sha(path) != item["sha256"]:
        raise PreparationError("wheel SHA256 与锁文件不一致")
    try:
        _, _, _, filename_tags = parse_wheel_filename(item["filename"])
        with zipfile.ZipFile(path) as archive:
            members = archive.infolist()
            names = [m.filename for m in members]
            if len(set(names)) != len(names):
                raise PreparationError("wheel 含重复成员")
            for name in names:
                if (name.startswith("/") or "\\" in name or ":" in name
                        or any(p in {".", ".."} for p in name.split("/"))):
                    raise PreparationError("wheel 含越界或非规范成员路径")
            metadata_names = [n for n in names if n.endswith(".dist-info/METADATA") and n.count("/") == 1]
            if len(metadata_names) != 1:
                raise PreparationError("wheel 缺少唯一顶层 METADATA")
            meta_name = metadata_names[0]
            dist_info = meta_name.rsplit("/", 1)[0].removesuffix(".dist-info")
            try:
                dist_name, dist_version = dist_info.rsplit("-", 1)
                if canonicalize_name(dist_name) != item["name"] or Version(dist_version) != Version(item["version"]):
                    raise PreparationError("wheel dist-info 目录身份不匹配")
            except ValueError as exc:
                raise PreparationError("wheel dist-info 目录身份无效") from exc
            wheel_name = meta_name.rsplit("/", 1)[0] + "/WHEEL"
            if archive.getinfo(meta_name).file_size > 1024 * 1024 or archive.getinfo(wheel_name).file_size > 65536:
                raise PreparationError("wheel 元数据超出限定大小")
            metadata = BytesParser().parsebytes(archive.read(meta_name))
            wheel = BytesParser().parsebytes(archive.read(wheel_name))
            if (canonicalize_name(metadata.get("Name", "")) != item["name"]
                    or Version(metadata.get("Version", "")) != Version(item["version"])):
                raise PreparationError("wheel METADATA 身份与锁文件不一致")
            requires_python = metadata.get("Requires-Python")
            if requires_python and Version(TARGET["python"]) not in SpecifierSet(requires_python):
                raise PreparationError("wheel METADATA 不支持 Python3.11.16")
            if not wheel.get("Wheel-Version", "").startswith("1."):
                raise PreparationError("不支持的 Wheel-Version")
            declared_tags = {tag for value in wheel.get_all("Tag", []) for tag in parse_tag(value)}
            if declared_tags != filename_tags or not declared_tags.intersection(target_tags()):
                raise PreparationError("wheel 内部标签与文件名或目标平台不匹配")
            licenses = [n for n in names if ".dist-info/" in n
                        and any(s in n.lower() for s in ("license", "copying", "notice"))]
    except (zipfile.BadZipFile, KeyError, ValueError) as exc:
        if isinstance(exc, PreparationError):
            raise
        raise PreparationError("wheel ZIP/元数据不可读：" + str(exc)) from exc
    return {"name": item["name"], "filename": item["filename"], "sha256": item["sha256"],
            "size_bytes": item["size_bytes"], "license_members": licenses}


def verify_bundle(lock_path, directory):
    """Pure file verification. There is deliberately no downloader argument."""
    plan = build_plan(lock_path)
    root = _root(directory)
    issues, checked = [], []
    for partial in root.glob(".download-*.part"):
        issues.append({"file": partial.name, "code": "unfinished-download", "message": "存在中断下载的临时文件，未完成介质整理"})
    for name, expected in ((MANIFEST, _manifest(plan)), (REQUIREMENTS, _requirements(plan))):
        try:
            path = _path(root, name)
            if not path.is_file() or path.read_bytes() != expected:
                issues.append({"file": name, "code": "plan-binding-mismatch", "message": "清单/requirements 缺失或与当前锁文件不一致"})
        except PreparationError as exc:
            issues.append({"file": name, "code": "unsafe-file", "message": str(exc)})
    try:
        wheelhouse = _path(root, "wheelhouse")
        if not wheelhouse.is_dir():
            raise PreparationError("wheelhouse 目录不存在")
        expected_names = {item["filename"] for item in plan["wheels"]}
        for actual in sorted(wheelhouse.iterdir()):
            if actual.name not in expected_names:
                issues.append({"file": actual.name, "code": "unexpected-wheelhouse-entry", "message": "不允许混入额外、错平台或未锁定的文件"})
        for item in plan["wheels"]:
            try:
                checked.append(verify_wheel(_path(root, "wheelhouse/" + item["filename"]), item))
            except (PreparationError, OSError) as exc:
                issues.append({"file": item["filename"], "code": "invalid-wheel", "message": str(exc)})
    except (PreparationError, OSError) as exc:
        issues.append({"file": "wheelhouse", "code": "invalid-wheelhouse", "message": str(exc)})
    return {"schema": SCHEMA, "status": "WHEELHOUSE_VERIFIED_NOT_INSTALL_ACCEPTANCE" if not issues else "INCOMPLETE",
            "complete": not issues, "plan_sha256": _sha(_json(plan)), "lock_sha256": plan["lock_sha256"],
            "target": plan["target"], "expected_count": plan["package_count"], "verified_count": len(checked),
            "verified_wheels": checked, "issues": issues, "network_used": False,
            "install_acceptance": False, "redistribution_approval": False}


def _proxy(value):
    parsed = urlsplit(value)
    try:
        host = ipaddress.ip_address(parsed.hostname or "")
        valid_host = host.is_private or host.is_loopback
    except ValueError:
        valid_host = parsed.hostname == "localhost"
    if (parsed.scheme not in {"socks5h", "http"} or parsed.port not in {10086, 10087}
            or not valid_host or parsed.username or parsed.password or parsed.path not in {"", "/"}
            or parsed.query or parsed.fragment):
        raise PreparationError("下载必须显式使用本地/私网 10086 或 10087 代理，不接受凭据或直连回落")
    return value


def curl_download(item, destination, *, proxy, curl="curl.exe"):
    proxy = _proxy(proxy)
    _wheel_url(item["url"])
    executable = shutil.which(curl)
    if executable is None:
        raise PreparationError("找不到 curl；不尝试其他下载器或直连")
    args = [executable, "--disable", "--silent", "--show-error", "--fail",
            "--proxy", proxy, "--noproxy", "", "--proto", "=https",
            "--connect-timeout", "20", "--max-time", "180", "--max-filesize", str(item["size_bytes"]),
            "--output", str(destination), "--write-out", "%{http_code}", "--url", item["url"]]
    # No --location: even a redirect cannot escape the reviewed origin.
    result = subprocess.run(args, capture_output=True, text=True, timeout=190, check=False)
    if result.returncode != 0 or result.stdout.strip() != "200":
        raise PreparationError(f"代理下载失败 {item['filename']}：curl={result.returncode}, HTTP={result.stdout.strip()}；{result.stderr[:400]}")


def download_bundle(lock_path, directory, *, proxy, curl="curl.exe", progress=None):
    _proxy(proxy)
    plan = prepare_plan(lock_path, directory)
    root = _root(directory)
    wheelhouse = _path(root, "wheelhouse")
    expected_names = {item["filename"] for item in plan["wheels"]}
    if any(p.name not in expected_names for p in wheelhouse.iterdir()):
        raise PreparationError("wheelhouse 有未登记文件；保留现状并停止下载")
    downloaded, reused = 0, 0
    for item in plan["wheels"]:
        path = _path(root, "wheelhouse/" + item["filename"])
        if path.exists():
            verify_wheel(path, item)  # Never overwrite an invalid existing wheel.
            reused += 1
        else:
            temporary = _path(root, ".download-" + uuid4().hex + ".part")
            try:
                curl_download(item, temporary, proxy=proxy, curl=curl)
                verify_wheel(temporary, item)
                with temporary.open("rb+") as stream:
                    os.fsync(stream.fileno())
                if path.exists():
                    raise PreparationError("下载期间目标文件已出现，未覆盖")
                os.rename(temporary, path)
                downloaded += 1
            finally:
                temporary.unlink(missing_ok=True)
        if progress:
            progress({"name": item["name"], "completed": downloaded + reused, "total": plan["package_count"]})
    result = verify_bundle(lock_path, root)
    result.update(network_used=downloaded > 0, downloaded_count=downloaded, reused_count=reused)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("plan", "download", "verify"))
    parser.add_argument("--lock", type=Path, default=ROOT / "uv.lock")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--proxy", help="download 模式必需；如 socks5h://172.18.38.39:10086")
    parser.add_argument("--curl", default="curl.exe")
    args = parser.parse_args(argv)
    if args.mode == "download" and not args.proxy:
        parser.error("download 必须指定 --proxy；不会尝试直连")
    if args.mode != "download" and args.proxy:
        parser.error("plan/verify 为零网络模式，不接受 --proxy")
    try:
        if args.mode == "plan":
            result = {"status": "PLAN_ONLY", "plan": prepare_plan(args.lock, args.out), "network_used": False}
        elif args.mode == "verify":
            result = verify_bundle(args.lock, args.out)
        else:
            result = download_bundle(args.lock, args.out, proxy=args.proxy, curl=args.curl,
                                     progress=lambda p: print(json.dumps(p, ensure_ascii=False), file=sys.stderr, flush=True))
    except (PreparationError, OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "FAILED", "error": str(exc), "install_acceptance": False}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if args.mode == "plan" or result["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
