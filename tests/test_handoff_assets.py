"""Restoration must verify all bytes before publishing and never overwrite data."""
import hashlib
import json
from pathlib import Path
import zipfile

import pytest

from scripts.restore_handoff_assets import restore_bundle, sha_file, validate


def fixture(tmp_path, entries=None):
    assets=tmp_path/"assets"
    assets.mkdir()
    entries=entries or {"raw/中文/road.dbf":b"real source", "out/proof/report.json":b"{}"}
    archive=assets/"data.part"
    with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED) as z:
        for name,data in entries.items():
            z.writestr(name,data)
    bundle={"id":"test","roots":["raw","out/proof"],
            "files":[{"path":name,"size":len(data),"sha256":hashlib.sha256(data).hexdigest()}
                     for name,data in entries.items()],
            "size":archive.stat().st_size,"sha256":sha_file(archive),
            "parts":[{"path":"data.part","size":archive.stat().st_size,"sha256":sha_file(archive)}]}
    return assets,bundle,tmp_path/"target"


def test_restore_and_idempotent_no_overwrite(tmp_path):
    assets,bundle,target=fixture(tmp_path)
    result=restore_bundle(assets,bundle,target)
    assert result["restored_roots"]==bundle["roots"]
    assert (target/"raw/中文/road.dbf").read_bytes()==b"real source"
    (target/"raw/extra.txt").write_text("keep")
    assert restore_bundle(assets,bundle,target)["unchanged_roots"]==bundle["roots"]
    assert (target/"raw/extra.txt").read_text()=="keep"


def test_existing_conflict_blocks_all_publication(tmp_path):
    assets,bundle,target=fixture(tmp_path)
    (target/"out/proof").mkdir(parents=True)
    (target/"out/proof/report.json").write_text("different")
    with pytest.raises(ValueError,match="no overwrite"):
        restore_bundle(assets,bundle,target)
    assert not (target/"raw").exists()
    assert (target/"out/proof/report.json").read_text()=="different"


@pytest.mark.parametrize("fault",["part_hash","part_size","archive_hash","member_hash","member_size"])
def test_corruption_publishes_nothing(tmp_path,fault):
    assets,bundle,target=fixture(tmp_path)
    if fault=="part_hash":bundle["parts"][0]["sha256"]="0"*64
    if fault=="part_size":bundle["parts"][0]["size"]+=1
    if fault=="archive_hash":bundle["sha256"]="0"*64
    if fault=="member_hash":bundle["files"][0]["sha256"]="0"*64
    if fault=="member_size":bundle["files"][0]["size"]+=1
    with pytest.raises(ValueError):restore_bundle(assets,bundle,target)
    assert not (target/"raw").exists()
    assert not (target/"out/proof").exists()


@pytest.mark.parametrize("name",["../outside","/absolute","C:/file","raw\\evil","raw/NUL.txt","raw/a:stream","raw/x/../file"])
def test_unsafe_paths_rejected(tmp_path,name):
    assets,bundle,target=fixture(tmp_path)
    bundle["files"][0]["path"]=name
    with pytest.raises(ValueError):restore_bundle(assets,bundle,target)
    assert not target.exists()


def test_archive_extra_member_rejected(tmp_path):
    assets,bundle,target=fixture(tmp_path)
    archive=assets/"data.part"
    with zipfile.ZipFile(archive,"a") as z:z.writestr("raw/extra","hidden")
    bundle["size"]=bundle["parts"][0]["size"]=archive.stat().st_size
    bundle["sha256"]=bundle["parts"][0]["sha256"]=sha_file(archive)
    with pytest.raises(ValueError,match="members differ"):restore_bundle(assets,bundle,target)
    assert not (target/"raw").exists()


def test_case_duplicate_rejected(tmp_path):
    assets,bundle,target=fixture(tmp_path)
    bundle["files"].append(dict(bundle["files"][0],path="RAW/中文/road.dbf"))
    with pytest.raises(ValueError):restore_bundle(assets,bundle,target)


def test_link_target_not_followed(tmp_path):
    assets,bundle,target=fixture(tmp_path)
    target.mkdir()
    outside=tmp_path/"outside"
    outside.mkdir()
    try:(target/"raw").symlink_to(outside,target_is_directory=True)
    except OSError:pytest.skip("symlink permission unavailable")
    with pytest.raises(ValueError,match="link"):restore_bundle(assets,bundle,target)
    assert not list(outside.iterdir())


def test_builder_preserves_bound_worker_logs_but_omits_sessions(tmp_path):
    from scripts.prepare_handoff_assets import build
    workspace = tmp_path / "workspace"
    source = workspace / "out/proof"
    source.mkdir(parents=True)
    (source / "worker.log").write_bytes(b"frozen research worker")
    (source / "server.log").write_bytes(b"http://127.0.0.1/#token=private")
    (source / "session-8795.json").write_bytes(b'{"url":"private"}')
    (source / "secret.json").write_bytes(b'{"token":"private"}')
    output = tmp_path / "parts"
    output.mkdir()
    bundle = build(workspace, output, "test", ["out/proof"])
    assert [f["path"] for f in bundle["files"]] == ["out/proof/worker.log"]
    assert len(bundle["excluded_files"]) == 3
    target = tmp_path / "restored"
    restore_bundle(output, bundle, target)
    assert (target / "out/proof/worker.log").read_bytes() == b"frozen research worker"
