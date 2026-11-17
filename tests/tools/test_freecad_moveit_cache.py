"""Verify lower-body containment without changing articulated robot state."""

import json
from copy import deepcopy
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def module(monkeypatch):
    pytest.importorskip("trimesh")
    monkeypatch.syspath_prepend(str(ROOT / "tools"))
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/cleany_moveit_config"))
    import prepare_freecad_moveit_cache

    return prepare_freecad_moveit_cache


def test_serialized_box_encloses_points_with_very_different_scales(module):
    points = np.array([[0.123456789, -1e-9, 2000], [0.123456799, 1e-8, 2000.00000001]])
    collision, result = module.enclosing_primitive("thin", points)
    center = np.fromstring(collision.find("origin").get("xyz"), sep=" ")
    half = np.fromstring(collision.find("geometry/box").get("size"), sep=" ") / 2
    assert (np.abs(points - center) <= half).all()
    assert result["minimum_source_containment_slack_m"] >= 0


def test_cylinder_preserves_wheel_axis_and_contains_rotated_source(module):
    axis = np.array([0.000003, 1.0, 0.00000001])
    axis /= np.linalg.norm(axis)
    angles = np.linspace(0, 2 * np.pi, 40)
    points = np.array(
        [
            [0.061 * np.cos(t), y, 0.061 * np.sin(t)]
            for t in angles
            for y in [-0.025, 0.025]
        ]
    )
    collision, result = module.enclosing_primitive("wheel", points, axis)
    origin = collision.find("origin")
    matrix = Rotation.from_euler(
        "xyz", np.fromstring(origin.get("rpy"), sep=" ")
    ).as_matrix()
    np.testing.assert_allclose(matrix[:, 2], axis, atol=1e-15)
    for angle in [0.0, 0.4, 1.2, np.pi]:
        rotated = Rotation.from_rotvec(axis * angle).apply(points)
        local = (rotated - result["center"]) @ matrix
        assert np.linalg.norm(local[:, :2], axis=1).max() <= result["radius"] + 1e-15
        assert np.abs(local[:, 2]).max() <= result["length"] / 2 + 1e-15


@pytest.fixture
def package(module, tmp_path):
    import trimesh
    from prepare_freecad_collision_cache import build_cache

    package = tmp_path / "description"
    (package / "meshes/freecad_final").mkdir(parents=True)
    (package / "urdf").mkdir()
    (package / "config/geometry").mkdir(parents=True)
    parts = []
    root = ET.Element("robot", name="sample")
    base = ET.SubElement(root, "link", name="base_link")
    wheel = ET.SubElement(root, "link", name="wheel")
    arm = ET.SubElement(root, "link", name="arm")
    for name, owner, center, group in [
        ("panel", base, [1, 0, 0], "lower"),
        ("bolt", base, [1.01, 0.1, 0], "lower"),
        ("wheel", wheel, [0, 0, 0], "drive"),
        ("arm", arm, [0, 0, 0.5], "arm"),
    ]:
        relative = f"meshes/freecad_final/{name}.stl"
        mesh = trimesh.creation.box(extents=[0.02, 0.03, 0.04])
        mesh.apply_translation(center)
        mesh.export(package / relative)
        parts.append(
            {
                "name": name,
                "groups": [group],
                "meshes": [
                    {
                        "mesh": relative,
                        "sha256": sha256((package / relative).read_bytes()).hexdigest(),
                    }
                ],
            }
        )
        collision = ET.SubElement(owner, "collision", name=name)
        ET.SubElement(collision, "origin", xyz="0.01 0.02 0.03", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(
            geometry, "mesh", filename="package://cleany_description/" + relative
        )
        owner.append(deepcopy(collision))
        owner[-1].tag = "visual"
    for name in ("wheel", "arm"):
        joint = ET.SubElement(root, "joint", name=f"joint_{name}", type="continuous")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(joint, "axis", xyz="0 1 0")
    for variant in ("freecad_final", "freecad_kinematic"):
        (package / "urdf" / f"cleany_{variant}.urdf").write_bytes(ET.tostring(root))
    (package / "config/geometry/freecad_final.json").write_text(
        json.dumps({"parts": parts})
    )
    cache = tmp_path / "indexed"
    build_cache(package, cache)
    config = tmp_path / "config.json"
    config.write_text(
        json.dumps(
            {
                "schema": "cleany.freecad_moveit_primitives/v1",
                "box_each_groups": ["drive"],
                "box_together_groups": ["lower"],
                "box_sets": [],
                "wheel_parts": ["wheel"],
                "require_groups": ["lower", "drive"],
            }
        )
    )
    return package, cache, config


def test_cache_keeps_frames_visuals_and_arm_geometry(module, package, tmp_path):
    from cleany_moveit_config.freecad_collision import inspection_urdf

    source, cache, config = package
    output = tmp_path / "primitives"
    result = module.build_moveit_cache(source, cache, output, config)
    assert result["meshes"] == 1
    assert result["collisions"] == 3
    assert len(result["primitives"]) == 2
    assert all(
        record["minimum_source_containment_slack_m"] >= 0
        for record in result["primitives"]
    )
    original = ET.parse(source / "urdf/cleany_freecad_kinematic.urdf").getroot()
    loaded = ET.fromstring(inspection_urdf(source, "freecad_kinematic", output))
    assert [ET.tostring(j) for j in original.findall("joint")] == [
        ET.tostring(j) for j in loaded.findall("joint")
    ]
    assert [ET.tostring(v) for v in original.findall("link/visual")] == [
        ET.tostring(v) for v in loaded.findall("link/visual")
    ]
    assert loaded.find("link[@name='wheel']/collision/geometry/cylinder") is not None
    assert loaded.find("link[@name='arm']/collision/geometry/mesh") is not None
    with pytest.raises(ValueError, match="does not provide"):
        inspection_urdf(source, "freecad_final", output)
    with pytest.raises(ValueError, match="new output"):
        module.build_moveit_cache(source, cache, output, config)


def test_cannot_merge_across_joint_links(module, package, tmp_path):
    source, cache, config = package
    settings = json.loads(config.read_text())
    settings.update(
        box_each_groups=[],
        box_together_groups=[],
        box_sets=[{"name": "bad", "parts": ["panel", "bolt", "wheel"]}],
    )
    config.write_text(json.dumps(settings))
    with pytest.raises(ValueError, match="independently moving"):
        module.build_moveit_cache(source, cache, tmp_path / "bad", config)


def test_source_tampering_is_rejected(module, package, tmp_path):
    source, cache, config = package
    (source / "meshes/freecad_final/panel.stl").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="differs from its FreeCAD manifest"):
        module.build_moveit_cache(source, cache, tmp_path / "bad", config)


def _ring_top(module):
    from optimize_arm_collision_meshes import Mesh

    vertices = np.array(
        [
            [-2.0, -2.0, 1.0],
            [2.0, -2.0, 1.0],
            [2.0, 2.0, 1.0],
            [-2.0, 2.0, 1.0],
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ]
    )
    faces = []
    for index in range(4):
        following = (index + 1) % 4
        faces.extend(
            [[index, following, following + 4], [index, following + 4, index + 4]]
        )
    return Mesh(vertices, np.asarray(faces))


def test_body_enclosure_keeps_top_access_and_cad_floor(module):
    import trimesh
    from freecad_body_enclosure import body_boxes, largest_top_opening
    from optimize_arm_collision_meshes import Mesh

    ring = _ring_top(module)
    np.testing.assert_array_equal(largest_top_opening(ring), [[-1, -1], [1, 1]])
    floor = trimesh.creation.box(extents=[4, 4, 0.5])
    floor.apply_translation([0, 0, -0.75])
    floor = Mesh(floor.vertices, floor.faces)
    boxes, report = body_boxes([ring, floor], [ring], [floor])
    assert len(boxes) == 5
    assert report["pocket_bounds_m"] == [[-1.0, -1.0, -0.5], [1.0, 1.0, 1.0]]
    for z in [-0.49, 0, 0.99, 1.01]:
        assert not any(
            np.all([0, 0, z] >= bounds[0]) and np.all([0, 0, z] <= bounds[1])
            for bounds in boxes
        )
    for point in np.vstack([ring.vertices, floor.vertices]):
        assert any(
            np.all(point >= bounds[0]) and np.all(point <= bounds[1])
            for bounds in boxes
        )


def test_body_rejects_a_crossing_triangle_even_when_all_vertices_are_outside(module):
    import trimesh
    from freecad_body_enclosure import body_boxes
    from optimize_arm_collision_meshes import Mesh

    ring = _ring_top(module)
    floor = trimesh.creation.box(extents=[4, 4, 0.5])
    floor.apply_translation([0, 0, -0.75])
    floor = Mesh(floor.vertices, floor.faces)
    for z in [0.5, 1.0]:
        obstacle = Mesh(
            np.array([[-1.8, 0, z], [1.8, 0.8, z], [1.8, -0.8, z]]),
            np.array([[0, 1, 2]]),
        )
        with pytest.raises(ValueError, match="crosses"):
            body_boxes([ring, floor, obstacle], [ring], [floor])


def test_surface_profile_rejects_unknown_groups(module, package, tmp_path):
    source, cache, config = package
    settings = json.loads(config.read_text())
    settings["surface_error_by_group_m"] = {"missing": 0.0005}
    config.write_text(json.dumps(settings))
    with pytest.raises(ValueError, match="Unknown surface reduction group"):
        module.build_moveit_cache(source, cache, tmp_path / "bad", config)


def test_enclosures_keep_joint_ownership_and_visuals(module, package, tmp_path):
    from cleany_moveit_config.freecad_collision import inspection_urdf

    source, cache, config = package
    settings = json.loads(config.read_text())
    settings["component_enclosures"] = {
        "groups": ["arm"],
        "max_hull_expansion_m": 0.001,
        "enclosure_margin_m": 1e-6,
        "max_iterations": 1024,
        "planes_per_iteration": 1,
        "parts": {"arm": {"slabs": {"axis": 2, "count": 2}}},
    }
    config.write_text(json.dumps(settings))
    output = tmp_path / "enclosures"
    result = module.build_moveit_cache(source, cache, output, config)
    assert result["collision_detector"] == "cleany/CADInspectionFCL"
    loaded = ET.fromstring(inspection_urdf(source, "freecad_kinematic", output))
    collisions = loaded.findall("link[@name='arm']/collision")
    assert len(collisions) == 2
    assert all(c.get("name").startswith("cad_enclosure__") for c in collisions)
    assert all(c.find("origin").get("xyz") == "0.01 0.02 0.03" for c in collisions)
    original = ET.parse(source / "urdf/cleany_freecad_kinematic.urdf").getroot()
    assert [ET.tostring(v) for v in original.findall("link/visual")] == [
        ET.tostring(v) for v in loaded.findall("link/visual")
    ]


def test_enclosure_partition_keeps_disconnected_fork_gap(module, tmp_path):
    import trimesh
    from freecad_component_enclosures import write_enclosures
    from scipy.spatial import ConvexHull

    parts = []
    for center in ([0, -0.03, 0], [0, 0.03, 0]):
        part = trimesh.creation.box(extents=[0.04, 0.01, 0.02])
        part.apply_translation(center)
        parts.append(part)
    source = tmp_path / "fork.stl"
    trimesh.util.concatenate(parts).export(source)
    settings = {
        "max_hull_expansion_m": 0.001,
        "enclosure_margin_m": 1e-6,
        "max_iterations": 1024,
        "planes_per_iteration": 1,
    }
    output = tmp_path / "pieces"
    reports = write_enclosures(source, output, settings, {})
    assert len(reports) == 2
    for report in reports:
        restored = trimesh.load(output / report["file"], force="mesh", process=False)
        assert restored.is_watertight
        assert report["maximum_source_plane_violation_m"] <= 0
        assert report["max_expansion_from_component_hull_m"] <= 0.001
        hull = ConvexHull(restored.vertices)
        assert (hull.equations[:, 3] > 0).any()  # The origin remains in the fork gap.

    combined_dir = tmp_path / "combined"
    combined = write_enclosures(
        source, combined_dir, settings, {"combine_components": True}
    )
    assert len(combined) == 1
    assert combined[0]["source_component_count"] == 2
    restored = trimesh.load(
        combined_dir / combined[0]["file"], force="mesh", process=False
    )
    assert (ConvexHull(restored.vertices).equations[:, 3] < 0).all()
    with pytest.raises(ValueError, match="unpartitioned rigid part"):
        write_enclosures(
            source,
            tmp_path / "invalid",
            settings,
            {"combine_components": True, "planes": [{"axis": 0, "offset_m": 0}]},
        )


def test_enclosure_profile_rejects_unknown_part(module, package, tmp_path):
    source, cache, config = package
    settings = json.loads(config.read_text())
    settings["component_enclosures"] = {"parts": {"missing": {}}}
    config.write_text(json.dumps(settings))
    with pytest.raises(ValueError, match="Unknown enclosure"):
        module.build_moveit_cache(source, cache, tmp_path / "bad", config)


@pytest.mark.parametrize("protected", ["panel", "arm"])
def test_protected_contact_parts_cannot_be_enclosed(
    module, package, tmp_path, protected
):
    source, cache, config = package
    settings = json.loads(config.read_text())
    settings["preserve_collision_parts"] = [protected]
    settings["component_enclosures"] = {"parts": {"arm": {}}}
    config.write_text(json.dumps(settings))
    with pytest.raises(ValueError, match="Protected collision parts"):
        module.build_moveit_cache(source, cache, tmp_path / "bad", config)


@pytest.mark.parametrize("members", [["panel", "bolt"], ["panel", "arm"]])
def test_rigid_enclosure_sets_require_common_joint_ownership(
    module, package, tmp_path, members
):
    source, cache, config = package
    settings = json.loads(config.read_text())
    settings["box_together_groups"] = []
    settings["require_groups"] = ["drive"]
    settings["component_enclosures"] = {
        "sets": [{"name": "rigid_case", "parts": members}],
        "max_hull_expansion_m": 0.001,
        "enclosure_margin_m": 1e-6,
        "max_iterations": 1024,
        "planes_per_iteration": 1,
    }
    config.write_text(json.dumps(settings))
    if "arm" in members:
        with pytest.raises(ValueError, match="independently moving"):
            module.build_moveit_cache(source, cache, tmp_path / "invalid", config)
    else:
        result = module.build_moveit_cache(source, cache, tmp_path / "combined", config)
        record = result["component_enclosures"][0]
        assert record["source_parts"] == members
        assert len(record["source_meshes"]) == 2
        assert len(record["pieces"]) == 1
