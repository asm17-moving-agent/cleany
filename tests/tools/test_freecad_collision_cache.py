"""Verify collision indexing and rejection of stale or damaged caches."""

import json
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


def test_collision_cache_preserves_triangles_and_model(monkeypatch, tmp_path):
    trimesh = pytest.importorskip("trimesh")
    monkeypatch.syspath_prepend(str(ROOT / "tools"))
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/cleany_moveit_config"))
    from cleany_moveit_config.freecad_collision import inspection_urdf
    from optimize_arm_collision_meshes import read_stl
    from prepare_freecad_collision_cache import build_cache

    package = tmp_path / "description"
    relative = "meshes/freecad_final/surface.stl"
    stl = package / relative
    stl.parent.mkdir(parents=True)
    trimesh.Trimesh(
        vertices=[[0, 0, 0], [0.125, 0, 0], [0, 0.125, 0], [0.125, 0.125, 0]],
        faces=[[0, 1, 2], [2, 1, 3]],
        process=False,
    ).export(stl)
    geometry = package / "config/geometry/freecad_final.json"
    geometry.parent.mkdir(parents=True)
    geometry.write_text(
        json.dumps(
            {
                "parts": [
                    {
                        "meshes": [
                            {
                                "mesh": relative,
                                "sha256": sha256(stl.read_bytes()).hexdigest(),
                            }
                        ]
                    }
                ]
            }
        )
    )
    (package / "urdf").mkdir()
    source = (
        '<robot name="sample"><link name="base_link"><collision><geometry>'
        f'<mesh filename="package://cleany_description/{relative}"/>'
        "</geometry></collision></link></robot>"
    )
    for variant in ("freecad_final", "freecad_kinematic"):
        (package / "urdf" / f"cleany_{variant}.urdf").write_text(source)

    cache = tmp_path / "cache"
    record = build_cache(package, cache)
    assert record["meshes"] == 1
    assert record["triangles"] == 2
    mesh_path = cache / next(iter(record["assets"]))
    original = read_stl(stl)
    indexed = trimesh.load_mesh(mesh_path, process=False)
    np.testing.assert_array_equal(
        original.vertices[original.faces], indexed.vertices[indexed.faces]
    )
    loaded = ET.fromstring(inspection_urdf(package, "freecad_final", cache))
    assert loaded.find(".//mesh").get("filename") == mesh_path.as_uri()
    preserved = build_cache(
        package,
        tmp_path / "preserved",
        max_surface_error_m=0.0001,
        preserve_links=("base_link",),
    )
    assert preserved["preserve_links"] == ["base_link"]
    assert not any(
        check.get("reduced") for check in preserved["geometry_checks"].values()
    )
    with pytest.raises(ValueError, match="Unknown static CAD link"):
        build_cache(package, tmp_path / "invalid-link", preserve_links=("missing",))
    contents = mesh_path.read_bytes()
    mesh_path.write_bytes(contents[:-1])
    with pytest.raises(ValueError, match="mesh was modified"):
        inspection_urdf(package, "freecad_final", cache)
    mesh_path.write_bytes(contents)
    geometry.write_text(geometry.read_text() + "\n")
    with pytest.raises(ValueError, match="different FreeCAD meshes"):
        inspection_urdf(package, "freecad_final", cache)


def test_curobo_world_cache_preserves_link_triangles_and_rejects_damage(
    monkeypatch, tmp_path
):
    trimesh = pytest.importorskip("trimesh")
    monkeypatch.syspath_prepend(str(ROOT / "tools/validation"))
    from check_freecad_curobo import load_snapshot, load_world_cache, write_world_cache

    package = tmp_path / "description"
    meshes = package / "meshes/freecad_final"
    meshes.mkdir(parents=True)
    source_mesh = trimesh.creation.box(extents=[0.1, 0.2, 0.3])
    source_mesh.export(meshes / "RPLidarA1M8.stl")
    (package / "urdf").mkdir()
    urdf = package / "urdf/cleany_freecad_final.urdf"
    urdf.write_text(
        '<robot name="cleany_freecad_final_static"><link name="base_link"/>'
        '<link name="lidar"><collision><geometry><mesh '
        'filename="package://cleany_description/meshes/freecad_final/RPLidarA1M8.stl"/>'
        '</geometry></collision></link><joint name="lidar_mount" type="fixed">'
        '<parent link="base_link"/><child link="lidar"/></joint></robot>'
    )
    (package / "config/geometry").mkdir(parents=True)
    (package / "config/geometry/freecad_final.json").write_text("{}")
    cache = tmp_path / "world"
    write_world_cache(package, cache)
    original, original_probe = load_snapshot(package)
    loaded, loaded_probe = load_world_cache(package, cache)
    np.testing.assert_array_equal(
        original["lidar"].triangles, loaded["lidar"].triangles
    )
    np.testing.assert_array_equal(original_probe, loaded_probe)
    path = next((cache / "meshes").glob("*.glb"))
    data = path.read_bytes()
    path.write_bytes(data[:-1])
    with pytest.raises(ValueError, match="asset was modified"):
        load_world_cache(package, cache)
    path.write_bytes(data)
    urdf.write_text(urdf.read_text() + "\n")
    with pytest.raises(ValueError, match="different CAD model"):
        load_world_cache(package, cache)


def test_curobo_primitives_keep_solids_separate_and_bake_joint_origins(
    monkeypatch, tmp_path
):
    trimesh = pytest.importorskip("trimesh")
    monkeypatch.syspath_prepend(str(ROOT / "tools/validation"))
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/cleany_moveit_config"))
    from check_freecad_curobo import (
        load_snapshot,
        load_world_cache,
        load_world_shapes,
        write_world_cache,
    )
    from cleany_moveit_config import freecad_collision

    package = tmp_path / "description"
    (package / "urdf").mkdir(parents=True)
    (package / "config/geometry").mkdir(parents=True)
    (package / "meshes").mkdir()
    trimesh.creation.box(extents=[0.1, 0.1, 0.1]).export(package / "meshes/lidar.stl")
    trimesh.creation.box(extents=[0.1, 0.1, 0.1]).export(
        package / "meshes/enclosure.glb"
    )
    (package / "config/geometry/freecad_final.json").write_text(
        json.dumps(
            {
                "parts": [
                    {"name": "RPLidarA1M8", "meshes": [{"mesh": "meshes/lidar.stl"}]}
                ]
            }
        )
    )
    urdf = """<robot name="sample"><link name="base_link">
    <collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision>
    <collision><origin xyz="0.1 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision>
    </link><link name="wheel"><collision><origin xyz="0.1 0 0"/>
    <geometry><cylinder radius="0.06" length="0.04"/></geometry></collision></link>
    <joint name="wheel_joint" type="continuous"><parent link="base_link"/><child link="wheel"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 1"/></joint></robot>"""
    root = ET.fromstring(urdf)
    rotated_box = ET.SubElement(root.find("link[@name='wheel']"), "collision")
    ET.SubElement(rotated_box, "origin", xyz="0.2 0 0")
    geometry = ET.SubElement(rotated_box, "geometry")
    ET.SubElement(geometry, "box", size="0.1 0.2 0.3")
    for name in ("cad_enclosure__one", "cad_enclosure__two", "unchanged_surface"):
        collision = ET.SubElement(root.find("link"), "collision", name=name)
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(
            geometry, "mesh", filename=(package / "meshes/enclosure.glb").as_uri()
        )
    urdf = ET.tostring(root, encoding="unicode")
    for variant in ("final", "kinematic"):
        (package / f"urdf/cleany_freecad_{variant}.urdf").write_text(urdf)
    cache = tmp_path / "collision"
    cache.mkdir()
    (cache / "manifest.json").write_text(
        json.dumps({"mode": "moveit_primitives", "retained_surface_error_m": 0.0005})
    )
    monkeypatch.setattr(freecad_collision, "inspection_urdf", lambda *_: urdf)
    meshes, _ = load_snapshot(package, cache)
    assert len(meshes) == 7
    assert {"base_link_solid_2", "base_link_solid_3", "base_link"} <= meshes.keys()
    assert all(
        mesh.is_watertight and mesh.is_winding_consistent for mesh in meshes.values()
    )
    wheel = meshes["wheel_solid_0"]
    np.testing.assert_allclose(wheel.bounds.mean(0), [1, 2.1, 3], atol=2e-7)
    assert wheel.extents[0] >= 0.12
    world = tmp_path / "world"
    write_world_cache(package, world, cache)
    native_meshes, native_boxes, _ = load_world_shapes(package, world)
    assert len(native_meshes) == 4 and len(native_boxes) == 3
    np.testing.assert_allclose(native_boxes["wheel_solid_1"]["pose"][:3], [1, 2.2, 3])
    np.testing.assert_allclose(
        native_boxes["wheel_solid_1"]["pose"][3:], [2**-0.5, 0, 0, 2**-0.5]
    )
    restored, _ = load_world_cache(package, world)
    for name in meshes:
        np.testing.assert_array_equal(meshes[name].triangles, restored[name].triangles)
    manifest_path = world / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["cuboids"]["wheel_solid_1"]["dims"][0] *= 2
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="cuboid data was modified"):
        load_world_shapes(package, world)
    manifest["cuboids"]["wheel_solid_1"]["dims"][0] /= 2
    manifest_path.write_text(json.dumps(manifest))
    model = package / "urdf/cleany_freecad_kinematic.urdf"
    model.write_text(urdf + "\n")
    with pytest.raises(ValueError, match="different CAD joints"):
        load_world_cache(package, world)
