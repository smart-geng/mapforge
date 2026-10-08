"""Installed runtime versions must agree with both pinned dependency inputs."""
from importlib import metadata

import pytest

from mapforge.workbench import surface_compiler as compiler


@pytest.fixture
def runtime_files(tmp_path, monkeypatch):
    (tmp_path / "pyproject.toml").write_text(
        '[project]\ndependencies = ["numpy==2.2.6", "PyYAML==6.0.3"]\n', encoding="utf-8")
    (tmp_path / "uv.lock").write_text(
        '[[package]]\nname = "numpy"\nversion = "2.2.6"\n'
        '[[package]]\nname = "pyyaml"\nversion = "6.0.3"\n', encoding="utf-8")
    monkeypatch.setattr(compiler, "ROOT", tmp_path)
    versions = {"numpy": "2.2.6", "PyYAML": "6.0.3"}
    monkeypatch.setattr(compiler.metadata, "version", lambda name: versions[name])
    return tmp_path, versions


def test_declared_locked_and_installed_versions_are_all_bound(runtime_files):
    actual = compiler._runtime_versions()
    assert actual["core_distributions"] == {"numpy": "2.2.6", "pyyaml": "6.0.3"}
    assert actual["installed_package_bytes_verified"] is False


def test_different_installed_version_cannot_keep_the_same_lock_fingerprint(runtime_files):
    _, versions = runtime_files
    versions["numpy"] = "2.3.0"
    with pytest.raises(compiler.SurfaceCompilerRejected) as error:
        compiler._runtime_versions()
    assert error.value.code == "runtime-version-mismatch"


def test_missing_installed_package_rejects(runtime_files, monkeypatch):
    def missing(name):
        raise metadata.PackageNotFoundError(name)
    monkeypatch.setattr(compiler.metadata, "version", missing)
    with pytest.raises(compiler.SurfaceCompilerRejected) as error:
        compiler._runtime_versions()
    assert error.value.code == "runtime-package-missing"


@pytest.mark.parametrize("change", ["wrong-lock", "ambiguous-lock", "unpinned", "duplicate", "empty"])
def test_pin_inconsistency_rejects_before_generation(runtime_files, change):
    root, _ = runtime_files
    if change == "wrong-lock":
        (root / "uv.lock").write_text('[[package]]\nname="numpy"\nversion="2.3.0"\n', encoding="utf-8")
    elif change == "ambiguous-lock":
        with (root / "uv.lock").open("a", encoding="utf-8") as stream:
            stream.write('[[package]]\nname="numpy"\nversion="2.3.0"\n')
    else:
        dependencies = {"unpinned": '["numpy>=2.2"]', "duplicate": '["numpy==2.2.6", "NumPy==2.2.6"]',
                        "empty": '[]'}[change]
        (root / "pyproject.toml").write_text('[project]\ndependencies=' + dependencies + '\n', encoding="utf-8")
    with pytest.raises(compiler.SurfaceCompilerRejected):
        compiler._runtime_versions()
