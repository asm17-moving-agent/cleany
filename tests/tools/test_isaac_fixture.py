"""Catch incompatible Mission targets before starting a GPU simulation."""
import importlib.util
import json
import os
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
import pytest
import yaml

pytest.importorskip("xacro")
ROOT = Path(__file__).resolve().parents[2]


def test_generated_collada_uses_vertex_indices_not_normal_or_uv_indices(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "tools/validation/isaac"))
    from mesh_geometry import read_geometry
    mesh = tmp_path / "indexed.dae"
    mesh.write_text('''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
      <asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
      <library_geometries><geometry><mesh>
        <source id="positions"><float_array>0 0 0 2 0 0 0 3 0</float_array>
          <technique_common><accessor stride="3" count="3"/></technique_common></source>
        <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles count="1"><input semantic="NORMAL" source="#normals" offset="0"/>
          <input semantic="VERTEX" source="#vertices" offset="1"/>
          <input semantic="TEXCOORD" source="#uv" offset="2"/>
          <p>5 2 7 5 0 8 5 1 9</p></triangles>
      </mesh></geometry></library_geometries>
      <library_visual_scenes><visual_scene><node><instance_geometry url="#shape"/></node></visual_scene></library_visual_scenes>
    </COLLADA>''')
    assert read_geometry(mesh) == ([[0., 0., 0.], [2., 0., 0.], [0., 3., 0.]], [3], [2, 0, 1])
    mesh.write_text(mesh.read_text().replace('<node>', '<node><translate>1 0 0</translate>'))
    with pytest.raises(AssertionError):
        read_geometry(mesh)


@pytest.mark.skipif(not os.getenv("FACILITY_FIXTURE"), reason="External facility map and Fuel mesh required")
def test_original_facility_world_and_selected_targets(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "tools/validation/isaac"))
    from prepare import prepare
    import json
    fixture = Path(os.environ["FACILITY_FIXTURE"])
    (tmp_path / "assets").mkdir()
    for suffix in ("pgm", "yaml"):
        shutil.copyfile(fixture / f"facility_integration.{suffix}", tmp_path / f"assets/facility_integration.{suffix}")
    shutil.copyfile(fixture / "fuel-assets/officechairgrey/OfficeChairGrey.obj", tmp_path / "assets/OfficeChairGrey.obj")
    prepare(ROOT, tmp_path, "facility18", "seat-12")
    evidence = json.loads((tmp_path / "provenance/facility18.json").read_text())
    assert evidence["collisions"] == 2117
    assert evidence["counts"]["office_chair"] == 48
    assert evidence["counts"]["facility_chair"] == 34
    assert evidence["map_check"]["targets_clear"] == ["home", "seat-12"]
    assert evidence["parking"]["within_existing_stop_footprint"]
    assert evidence["parking"]["excluded_links"] == []
    entries = json.loads((tmp_path / "config/world_geometry.json").read_text())
    for name in ("office_chair_13", "office_chair_43", "facility_ceiling"):
        assert any(e["model"] == name and e["kind"] == "collision" for e in entries)


def test_exported_targets_load_and_clear_the_navigation_footprint(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "tools/validation/isaac"))
    monkeypatch.setenv("AMENT_PREFIX_PATH", os.environ.get("AMENT_PREFIX_PATH", ""))
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/cleany_mission_manager"))
    from cleany_mission_manager.adapters.config import load_targets
    spec = importlib.util.spec_from_file_location("isaac_prepare", ROOT / "tools/validation/isaac/prepare.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.prepare(ROOT, tmp_path)
    targets = load_targets(str(tmp_path / "config/targets.yaml"))
    assert "seat-1" in targets.seats
    metadata = yaml.safe_load((tmp_path / "assets/isaac_room.yaml").read_text())
    grid = np.asarray(Image.open(tmp_path / "assets/isaac_room.pgm"))
    # Check the occupied map against the existing safety envelope, including its
    # interior; just checking the robot center missed the first invalid fixture.
    for target in (targets.home, targets.seats["seat-1"]):
        for dx in np.linspace(-.48, .48, 25):
            for dy in np.linspace(-.40, .40, 21):
                column = int((target.x+dx-metadata["origin"][0])/metadata["resolution"])
                row = grid.shape[0]-1-int((target.y+dy-metadata["origin"][1])/metadata["resolution"])
                assert 0 <= row < grid.shape[0] and 0 <= column < grid.shape[1]
                assert grid[row, column] == 254, (target, dx, dy)


@pytest.mark.parametrize("world", ["room", "facility18"])
def test_launch_keeps_tuned_mppi_and_complete_safety_profile(tmp_path, monkeypatch, world):
    pytest.importorskip("launch")
    from launch import LaunchContext
    monkeypatch.setenv("ROS_DISTRO", "humble")
    for package in ("cleany_navigation", "cleany_mission_manager"):
        monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src" / package))
    monkeypatch.syspath_prepend(str(ROOT / "tools/validation/isaac"))
    from prepare import prepare
    prepare(ROOT, tmp_path)
    scene_file = tmp_path / "config/room.json"
    scene = json.loads(scene_file.read_text())
    scene["world"] = world
    scene_file.write_text(json.dumps(scene))
    spec = importlib.util.spec_from_file_location("isaac_launch", ROOT / "tools/validation/isaac/mission.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(output=str(tmp_path), source=str(ROOT), scenario="success")
    module.start(context)  # Construct launch actions; do not start ROS processes.
    parameters = yaml.safe_load((tmp_path / "config/navigation.yaml").read_text())
    control = parameters["controller_server"]["ros__parameters"]
    assert control["FollowPath"]["plugin"] == "cleany_navigation::MPPIController"
    assert control["FollowPath"]["time_steps"] == 80
    assert control["FollowPath"]["PathFollowCritic"]["cost_weight"] == 15.
    assert control["FollowPath"]["GoalCritic"]["cost_weight"] == 15.
    assert control["FollowPath"]["validate_output"]
    assert control["FollowPath"]["path_seed_count"] == 128
    assert control["FollowPath"]["path_progress_timeout"] == 12.
    assert control["FollowPath"]["vx_max"] == (.25 if world == "facility18" else .15)
    assert control["FollowPath"]["wz_max"] == (.5 if world == "facility18" else .3)
    assert control["goal_checker"]["xy_goal_tolerance"] == .03
    planner = parameters["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "cleany_navigation::SmacLatticePlanner"
    assert Path(planner["lattice_filepath"]).name == "omni-r10cm.json"
    local = parameters["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["inflation_layer"]["inflation_radius"] == .8
    assert "depth_layer" not in local["plugins"]
    monitor = parameters["collision_monitor"]["ros__parameters"]
    assert monitor["observation_sources"] == ["scan"]
    assert monitor["SlowZone"]["slowdown_ratio"] == .35
