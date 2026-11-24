import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "field_deps", Path(__file__).resolve().parents[2] / "tools/ros2/field_deps.py"
)
deps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deps)


def test_publish_and_repin_without_mutating_previous_release(tmp_path):
    install = tmp_path / "install"
    install.mkdir()
    library = install / "lib.so.1"
    library.write_bytes(b"first build")
    (install / "lib.so").symlink_to("lib.so.1")
    for name in deps.OVERLAYS:
        (tmp_path / name).mkdir()
    store = tmp_path / "store"
    first, second = tmp_path / "first", tmp_path / "second"
    version = deps.pin(install, tmp_path, first, store)
    assert version == deps.pin(install, tmp_path, second, store)
    library.write_bytes(b"next build")
    updated = deps.pin(install, tmp_path, second, store)
    assert updated != version and first.resolve() == version
    assert (first / "install/lib.so").read_bytes() == b"first build"
    assert (second / "install/lib.so").read_bytes() == b"next build"
    (updated / "install/lib.so.1").chmod(0o700)
    with pytest.raises(ValueError, match="Damaged"):
        deps.pin(install, tmp_path, first, store)
    assert first.resolve() == version


def test_refuse_legacy_directory_and_external_symlink(tmp_path):
    install = tmp_path / "install"
    install.mkdir()
    for name in deps.OVERLAYS:
        (tmp_path / name).mkdir()
    legacy = tmp_path / "old"
    legacy.mkdir()
    with pytest.raises(ValueError, match="migration"):
        deps.pin(install, tmp_path, legacy, tmp_path / "store")
    (install / "external").symlink_to("/tmp")
    with pytest.raises(ValueError, match="escapes"):
        deps.pin(install, tmp_path, legacy, tmp_path / "store")
    assert legacy.is_dir() and not legacy.is_symlink()


def test_inherit_checks_installed_version_without_staging(tmp_path, monkeypatch):
    install = tmp_path / "install"
    install.mkdir()
    (install / "lib.so").write_bytes(b"commissioned")
    for name in deps.OVERLAYS:
        (tmp_path / name).mkdir()
    store = tmp_path / "store"
    active, next_link = tmp_path / "active", tmp_path / "next"
    version = deps.pin(install, tmp_path, active, store)
    monkeypatch.setattr(
        deps.subprocess, "run", lambda *_a, **_k: pytest.fail("copied on inheritance")
    )
    install.rename(tmp_path / "offline")
    assert deps.inherit(active, next_link, store) == version
    assert next_link.readlink() == version
    (version / "install/lib.so").write_bytes(b"bad")
    with pytest.raises(ValueError, match="damaged"):
        deps.inherit(active, tmp_path / "rejected", store)
    assert not (tmp_path / "rejected").is_symlink()
    active.unlink()
    active.symlink_to(tmp_path)
    with pytest.raises(ValueError, match="Invalid"):
        deps.inherit(active, next_link, store)
