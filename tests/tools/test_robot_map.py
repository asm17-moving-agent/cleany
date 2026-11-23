import fcntl
import hashlib
import shutil
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

spec = spec_from_file_location("robot_map", Path(__file__).resolve().parents[2] / "tools/ros2/robot_map.py")
maps = module_from_spec(spec)
spec.loader.exec_module(maps)


def probe(path):
    return {"grid_revision": hashlib.sha256((path / "map.pgm").read_bytes()).hexdigest()}


@pytest.fixture
def local(tmp_path):
    (tmp_path / "survey.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    source = tmp_path / "survey.yaml"
    source.write_text("image: survey.pgm\n")
    return source, tmp_path / "robot"


def test_expand_and_reinstall(local):
    source, robot = local
    first = maps.install(source, robot, "cafe", 1, probe=probe)
    assert maps.install(source, robot, "cafe", 1, probe=probe) == first
    assert (robot / "map/source.yaml").read_text() == "image: map.pgm\n"
    (source.parent / "survey.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    with pytest.raises(ValueError, match="new revision"):
        maps.install(source, robot, "cafe", 1, probe=probe)
    second = maps.install(source, robot, "cafe", 2, probe=probe)
    assert second["map_revision"] == 2 and second["nav2"] != first["nav2"]
    assert list(robot.iterdir()) == [robot / "map"]
    assert set(p.name for p in (robot / "map").iterdir()) == maps.FILES


def test_failed_active_reload_rolls_back(local):
    source, robot = local
    previous = maps.install(source, robot, "cafe", 1, probe=probe)
    def fail_after_exchange(path):
        if path.name == "map":
            raise ValueError("active reload failed")
        return probe(path)
    with pytest.raises(ValueError, match="active reload"):
        maps.install(source, robot, "cafe", 2, probe=fail_after_exchange)
    assert maps.verify_live(robot / "map", probe) == previous
    assert not (robot / "map.next").exists()


def test_registered_digest_and_shared_lock(local):
    source, robot = local
    previous = maps.install(source, robot, "cafe", 1, probe=probe)
    with maps.directory_fd(robot) as fd:
        fcntl.flock(fd, fcntl.LOCK_SH)
        with pytest.raises(BlockingIOError):
            maps.install(source, robot, "cafe", 2, probe=probe)
    bundle = source.parent / "issued"
    shutil.copytree(robot / "map", bundle)
    (bundle / "map.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    with pytest.raises(ValueError, match="checksum"):
        maps.install(bundle, robot, probe=probe)
    assert maps.read_bundle(robot / "map") == previous


def test_interrupted_staging_preserves_unrelated_files(local):
    source, robot = local
    previous = maps.install(source, robot, "cafe", 1, probe=probe)
    staged = robot / "map.next"
    staged.mkdir()
    (staged / "source.yaml").write_text("partial")
    assert maps.install(source, robot, "cafe", 1, probe=probe) == previous
    staged.mkdir()
    (staged / "notes").write_text("operator")
    with pytest.raises(ValueError, match="Unexpected map file"):
        maps.install(source, robot, "cafe", 2, probe=probe)
    assert (staged / "notes").is_file()


def test_targets_use_manifest_identity_and_grid(local, monkeypatch):
    from types import SimpleNamespace
    from cleany_mission_manager.adapters import config

    source, robot = local
    meta = maps.install(source, robot, "cafe", 1, probe=probe)
    field_spec = spec_from_file_location("field_service", Path(maps.__file__).with_name("field_service.py"))
    field = module_from_spec(field_spec)
    field_spec.loader.exec_module(field)
    monkeypatch.setattr(field, "MAP", robot / "map/source.yaml")
    target = SimpleNamespace(map_id="cafe", frame_id="map", seats={"seat-01": []},
                             binding={"source_sha256": meta["source_sha256"]["map.pgm"]})
    monkeypatch.setattr(config, "load_targets", lambda path: target)
    monkeypatch.setenv("CLEANY_MISSION_GRID_REVISION", meta["nav2"]["grid_revision"])
    assert field.bound_targets() is target
    monkeypatch.setenv("CLEANY_MISSION_GRID_REVISION", "outdated")
    with pytest.raises(ValueError, match="grid revision"):
        field.bound_targets()
