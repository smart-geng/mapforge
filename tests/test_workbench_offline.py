"""Exact wheel selection, immutable downloads and genuinely offline verification."""
import hashlib
import io
import json
from pathlib import Path
import socket
from types import SimpleNamespace
import zipfile

import pytest

from scripts import workbench_prepare_offline as offline


def wheel_bytes(name="sample", version="1.0", tag="py3-none-any", *, python=">=3.8", metadata_name=None,
                internal_tag=None, extra=None, dist_info_name=None):
    stream = io.BytesIO()
    directory = dist_info_name or f"{name.replace('-', '_')}-{version}.dist-info"
    with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(directory + "/METADATA", f"Metadata-Version: 2.1\nName: {metadata_name or name}\nVersion: {version}\nRequires-Python: {python}\n")
        archive.writestr(directory + "/WHEEL", f"Wheel-Version: 1.0\nGenerator: unit-fixture\nRoot-Is-Purelib: true\nTag: {internal_tag or tag}\n")
        archive.writestr(directory + "/licenses/LICENSE", "test fixture only")
        archive.writestr(name.replace("-", "_") + "/__init__.py", "# never imported by verifier\n")
        if extra:
            archive.writestr(*extra)
    return stream.getvalue()


def wheel(name="sample", version="1.0", tag="py3-none-any", **kwargs):
    data = wheel_bytes(name, version, tag, **kwargs)
    filename = f"{name.replace('-', '_')}-{version}-{tag}.whl"
    return {"url": "https://files.pythonhosted.org/packages/aa/bb/" + filename,
            "hash": "sha256:" + hashlib.sha256(data).hexdigest(), "size": len(data)}, data


def write_lock(path, entries, *, requires_python="==3.11.*"):
    text = f'version = 1\nrevision = 3\nrequires-python = "{requires_python}"\n'
    text += '\n[[package]]\nname = "mapforge"\nversion = "0.0.1.dev0"\nsource = { virtual = "." }\n'
    for name, version, wheels in entries:
        text += f'\n[[package]]\nname = "{name}"\nversion = "{version}"\nsource = {{ registry = "https://pypi.org/simple" }}\n'
        text += "wheels = [\n" + "\n".join("{ " + ", ".join(f"{k} = {json.dumps(v)}" for k, v in item.items()) + " }," for item in wheels) + "\n]\n"
    path.write_text(text, encoding="utf8")
    return path


@pytest.fixture
def bundle(tmp_path):
    item, data = wheel()
    lock = write_lock(tmp_path / "uv.lock", [("sample", "1.0", [item])])
    directory = tmp_path / "offline"
    plan = offline.prepare_plan(lock, directory)
    return SimpleNamespace(lock=lock, directory=directory, item=plan["wheels"][0], data=data, plan=plan,
                           wheel_path=directory / "wheelhouse" / plan["wheels"][0]["filename"])


def test_chooses_native_cp311_over_abi3_and_pure_independent_of_host(tmp_path):
    wheels = [wheel(tag=tag)[0] for tag in ("cp312-cp312-win_amd64", "cp311-cp311-win32",
                                         "cp311-cp311-manylinux_2_17_x86_64", "py3-none-any",
                                         "cp39-abi3-win_amd64", "cp311-cp311-win_amd64")]
    lock = write_lock(tmp_path / "uv.lock", [("sample", "1.0", wheels)])
    plan = offline.build_plan(lock)
    assert plan["wheels"][0]["filename"] == "sample-1.0-cp311-cp311-win_amd64.whl"
    assert plan["target"] == offline.TARGET
    assert plan["package_count"] == 1
    assert plan["install_acceptance"] is False


@pytest.mark.parametrize("tag", ["cp39-abi3-win_amd64", "py2.py3-none-any", "py3-none-any"])
def test_compatible_older_abi3_and_universal_wheels_are_accepted(tmp_path, tag):
    item, _ = wheel(tag=tag)
    lock = write_lock(tmp_path / "uv.lock", [("sample", "1.0", [item])])
    assert offline.build_plan(lock)["package_count"] == 1


@pytest.mark.parametrize("tag", ["cp312-cp312-win_amd64", "cp312-abi3-win_amd64", "cp311-cp311-win32",
                                   "cp311-cp311-win_arm64", "cp311-cp311-manylinux_2_17_x86_64", "py2-none-any"])
def test_no_matching_platform_or_python_never_falls_back_to_sdist(tmp_path, tag):
    item, _ = wheel(tag=tag)
    lock = write_lock(tmp_path / "uv.lock", [("sample", "1.0", [item])])
    with pytest.raises(offline.PreparationError, match="禁止回落 sdist"):
        offline.build_plan(lock)


def test_real_lock_selects_55_exact_artifacts_without_network(monkeypatch):
    monkeypatch.setattr(socket, "socket", lambda *a, **k: pytest.fail("network used"))
    plan = offline.build_plan()
    assert plan["package_count"] == 55
    assert plan["download_size_bytes"] == 106615270
    assert all("linux" not in w["filename"] and "win32" not in w["filename"] and "arm64" not in w["filename"] for w in plan["wheels"])
    assert any(w["selected_target_tag"].startswith("cp39-abi3") for w in plan["wheels"])


@pytest.mark.parametrize("change", ["hash", "size", "url", "identity", "requires_python", "duplicate"])
def test_malformed_or_unreviewed_lock_is_rejected(tmp_path, change):
    item, _ = wheel()
    if change == "hash": item["hash"] = "md5:" + "a" * 32
    if change == "size": item["size"] = 0
    if change == "url": item["url"] = item["url"].replace("files.pythonhosted.org", "unreviewed.example")
    if change == "identity": item["url"] = item["url"].replace("sample-1.0", "other-1.0")
    entries = [("sample", "1.0", [item])]
    if change == "duplicate": entries *= 2
    lock = write_lock(tmp_path / "uv.lock", entries, requires_python="==3.12.*" if change == "requires_python" else "==3.11.*")
    with pytest.raises(offline.PreparationError):
        offline.build_plan(lock)


def test_plan_and_verify_never_launch_downloader_or_socket_even_when_missing(bundle, monkeypatch):
    monkeypatch.setattr(socket, "socket", lambda *a, **k: pytest.fail("network used"))
    monkeypatch.setattr(offline.subprocess, "run", lambda *a, **k: pytest.fail("subprocess used"))
    monkeypatch.setattr(offline, "curl_download", lambda *a, **k: pytest.fail("download used"))
    before = {p: p.read_bytes() for p in bundle.directory.rglob("*") if p.is_file()}
    report = offline.verify_bundle(bundle.lock, bundle.directory)
    assert report["status"] == "INCOMPLETE"
    assert report["verified_count"] == 0 and not report["network_used"]
    assert {p: p.read_bytes() for p in bundle.directory.rglob("*") if p.is_file()} == before


def test_valid_wheelhouse_is_read_back_as_media_not_install_acceptance(bundle):
    bundle.wheel_path.write_bytes(bundle.data)
    report = offline.verify_bundle(bundle.lock, bundle.directory)
    assert report["complete"] and report["verified_count"] == 1
    assert report["status"] == "WHEELHOUSE_VERIFIED_NOT_INSTALL_ACCEPTANCE"
    assert not report["install_acceptance"] and not report["redistribution_approval"]
    assert report["verified_wheels"][0]["license_members"] == ["sample-1.0.dist-info/licenses/LICENSE"]
    requirements = (bundle.directory / offline.REQUIREMENTS).read_text()
    assert "sample==1.0 --hash=sha256:" + bundle.item["sha256"] in requirements


@pytest.mark.parametrize("mode", ["zero", "wrong-size", "wrong-hash", "extra-platform", "partial", "manifest", "requirements"])
def test_incomplete_or_tampered_media_cannot_pass(bundle, mode):
    bundle.wheel_path.write_bytes(bundle.data)
    if mode == "zero": bundle.wheel_path.write_bytes(b"")
    if mode == "wrong-size": bundle.wheel_path.write_bytes(bundle.data[:-1])
    if mode == "wrong-hash": bundle.wheel_path.write_bytes(b"X" + bundle.data[1:])
    if mode == "extra-platform": (bundle.wheel_path.parent / "sample-1.0-cp311-cp311-win32.whl").write_bytes(bundle.data)
    if mode == "partial": (bundle.directory / ".download-abcd.part").write_bytes(b"unfinished")
    if mode == "manifest": (bundle.directory / offline.MANIFEST).write_text("{}")
    if mode == "requirements": (bundle.directory / offline.REQUIREMENTS).write_text("sample==2.0\n")
    assert not offline.verify_bundle(bundle.lock, bundle.directory)["complete"]


@pytest.mark.parametrize("kwargs", [{"metadata_name": "other"}, {"python": ">=3.12"},
                                      {"internal_tag": "cp311-cp311-win32"},
                                      {"extra": ("../escape.py", "bad")},
                                      {"dist_info_name": "other-1.0.dist-info"}])
def test_even_hash_matching_archive_must_have_valid_target_metadata(tmp_path, kwargs):
    item, data = wheel(**kwargs)
    lock = write_lock(tmp_path / "uv.lock", [("sample", "1.0", [item])])
    directory = tmp_path / "media"
    plan = offline.prepare_plan(lock, directory)
    (directory / "wheelhouse" / plan["wheels"][0]["filename"]).write_bytes(data)
    report = offline.verify_bundle(lock, directory)
    assert not report["complete"]


@pytest.mark.parametrize("proxy", ["", "http://127.0.0.1:9999", "socks5://127.0.0.1:10086",
                                    "http://public.example:10086", "http://user:pass@127.0.0.1:10086"])
def test_download_refuses_unapproved_or_missing_proxy_before_network(bundle, monkeypatch, proxy):
    monkeypatch.setattr(offline.subprocess, "run", lambda *a, **k: pytest.fail("network attempted"))
    with pytest.raises(offline.PreparationError):
        offline.download_bundle(bundle.lock, bundle.directory, proxy=proxy)


def test_curl_request_forces_proxy_and_https_without_implicit_config_or_redirects(bundle, monkeypatch):
    captured = []
    monkeypatch.setattr(offline.shutil, "which", lambda value: "curl.exe")
    def run(args, **kwargs):
        captured.append((args, kwargs))
        return SimpleNamespace(returncode=0, stdout="200", stderr="")
    monkeypatch.setattr(offline.subprocess, "run", run)
    offline.curl_download(bundle.item, bundle.directory / "temp", proxy="socks5h://172.18.38.39:10086")
    args, kwargs = captured[0]
    assert args[1] == "--disable"
    assert args[args.index("--proxy") + 1] == "socks5h://172.18.38.39:10086"
    assert args[args.index("--noproxy") + 1] == ""
    assert args[args.index("--proto") + 1] == "=https"
    assert "--location" not in args and "--insecure" not in args
    assert args[args.index("--url") + 1] == bundle.item["url"]
    assert not kwargs.get("shell", False)


@pytest.mark.parametrize("status,code", [("302", 0), ("404", 22), ("000", 7)])
def test_curl_failure_does_not_fall_back_to_direct_connection(bundle, monkeypatch, status, code):
    calls = []
    monkeypatch.setattr(offline.shutil, "which", lambda value: "curl.exe")
    monkeypatch.setattr(offline.subprocess, "run", lambda *a, **k: calls.append(a) or SimpleNamespace(returncode=code, stdout=status, stderr="failure"))
    with pytest.raises(offline.PreparationError, match="代理下载失败"):
        offline.curl_download(bundle.item, bundle.directory / "temp", proxy="http://127.0.0.1:10087")
    assert len(calls) == 1


def test_download_checks_temporary_bytes_then_reuses_valid_complete_wheel(bundle, monkeypatch):
    calls = []
    def download(item, path, **kwargs):
        calls.append(item["filename"])
        assert not bundle.wheel_path.exists()
        path.write_bytes(bundle.data)
    monkeypatch.setattr(offline, "curl_download", download)
    report = offline.download_bundle(bundle.lock, bundle.directory, proxy="socks5h://172.18.38.39:10086")
    assert report["complete"] and report["downloaded_count"] == 1
    assert bundle.wheel_path.read_bytes() == bundle.data
    assert not list(bundle.directory.glob(".download-*.part"))
    again = offline.download_bundle(bundle.lock, bundle.directory, proxy="socks5h://172.18.38.39:10086")
    assert again["complete"] and again["reused_count"] == 1 and not again["network_used"]
    assert len(calls) == 1


def test_download_failure_keeps_no_publishable_partial_wheel(bundle, monkeypatch):
    monkeypatch.setattr(offline, "curl_download", lambda item, path, **kwargs: path.write_bytes(bundle.data[:-1]))
    with pytest.raises(offline.PreparationError, match="长度"):
        offline.download_bundle(bundle.lock, bundle.directory, proxy="socks5h://172.18.38.39:10086")
    assert not bundle.wheel_path.exists()
    assert not list(bundle.directory.glob(".download-*.part"))
    assert not offline.verify_bundle(bundle.lock, bundle.directory)["complete"]


def test_existing_invalid_wheel_and_changed_plan_are_not_overwritten(bundle, monkeypatch):
    monkeypatch.setattr(offline, "curl_download", lambda *a, **k: pytest.fail("must not overwrite"))
    bundle.wheel_path.write_bytes(b"existing-bad-data")
    with pytest.raises(offline.PreparationError):
        offline.download_bundle(bundle.lock, bundle.directory, proxy="http://127.0.0.1:10087")
    assert bundle.wheel_path.read_bytes() == b"existing-bad-data"
    before = (bundle.directory / offline.MANIFEST).read_bytes()
    bundle.lock.write_text(bundle.lock.read_text() + "\n# changed lock\n")
    with pytest.raises(offline.PreparationError, match="保留原件"):
        offline.prepare_plan(bundle.lock, bundle.directory)
    assert (bundle.directory / offline.MANIFEST).read_bytes() == before


def test_cli_verify_is_nonzero_for_missing_wheels_and_cannot_take_proxy(bundle, capsys):
    assert offline.main(["verify", "--lock", str(bundle.lock), "--out", str(bundle.directory)]) == 2
    assert json.loads(capsys.readouterr().out)["network_used"] is False
    with pytest.raises(SystemExit):
        offline.main(["verify", "--out", str(bundle.directory), "--proxy", "http://127.0.0.1:10087"])
