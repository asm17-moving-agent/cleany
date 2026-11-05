"""Validate the saved CAD reference without asserting unmeasured kinematics."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
MANIFEST = json.loads((PACKAGE / "config/geometry/freecad_final.json").read_text())
URDF = ET.parse(PACKAGE / "urdf/cleany_freecad_final.urdf").getroot()


def triangles(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + count * 50
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data, dtype=dtype, offset=84)["v"]


def test_every_source_solid_has_matching_visual_collision_and_integrity() -> None:
    assert len(MANIFEST["parts"]) == 308
    visuals = {v.attrib["name"]: v for v in URDF.findall("link/visual")}
    collisions = {c.attrib["name"]: c for c in URDF.findall("link/collision")}
    for part in MANIFEST["parts"]:
        meshes = []
        for mesh in part["meshes"]:
            path = PACKAGE / mesh["mesh"]
            assert hashlib.sha256(path.read_bytes()).hexdigest() == mesh["sha256"]
            vertices = triangles(path)
            assert len(vertices) == mesh["triangles"]
            meshes.append(vertices.reshape(-1, 3))
            visual = visuals.pop(mesh["name"] + "_visual")
            collision = collisions.pop(mesh["name"] + "_collision")
            assert (
                visual.find("geometry/mesh").attrib
                == collision.find("geometry/mesh").attrib
            )
            expected = "package://cleany_description/" + mesh["mesh"]
            assert visual.find("geometry/mesh").attrib["filename"] == expected
        vertices = np.concatenate(meshes)
        assert len(vertices) == 3 * part["triangles"]
        assert np.isfinite(vertices).all()
        assert [vertices.min(axis=0), vertices.max(axis=0)] == pytest.approx(
            np.asarray(part["bounds_base_link_m"]),
            abs=2e-7,
        )
    assert not collisions
    assert set(visuals) == {"DefaultExpressionScreen"}


def test_saved_pose_cannot_be_confused_with_a_calibrated_joint_model() -> None:
    assert MANIFEST["model_kind"] == "saved_static_assembly"
    assert all(value is None for value in MANIFEST["unresolved"].values())
    assert MANIFEST["capabilities"] == {
        "static_geometry": True,
        "joint_motion": False,
        "actuator_commands": False,
        "dynamic_simulation": False,
        "calibrated_sensor_frames": False,
    }
    links = {link.attrib["name"] for link in URDF.findall("link")}
    children = set()
    for joint in URDF.findall("joint"):
        assert joint.attrib["type"] == "fixed"
        assert joint.find("parent").attrib["link"] == "base_link"
        child = joint.find("child").attrib["link"]
        assert child in links and child.startswith("cad_snapshot_")
        assert child not in children
        children.add(child)
    assert links - children == {"base_link"}
    for tag in (
        "inertial",
        "axis",
        "limit",
        "mimic",
        "transmission",
        "ros2_control",
        "sensor",
    ):
        assert URDF.find(".//" + tag) is None
    assert not any("optical" in link or "grasp_tcp" in link for link in links)


def test_source_units_ground_alignment_and_shorter_frame() -> None:
    parts = {p["name"]: p for p in MANIFEST["parts"]}
    bound = np.asarray(parts["InstalledProfile00"]["bounds_base_link_m"])
    assert bound[1] - bound[0] == pytest.approx([0.02, 0.02, 0.56], abs=2e-6)
    plate = np.asarray(parts["InstalledArmMountPanel"]["bounds_base_link_m"])
    assert plate[0, 2] == pytest.approx(0.3155, abs=2e-6)
    assert plate[1] - plate[0] == pytest.approx([0.16, 0.5, 0.003], abs=2e-6)
    head_bracket = np.asarray(parts["HeadFixedBracket"]["bounds_base_link_m"])
    assert head_bracket[:, 2] == pytest.approx(
        [0.6260115234624802, 0.6857115235624798], abs=2e-6
    )
    for name in [n for n in parts if n.startswith("Drive_") and n.endswith("_wheel")]:
        bound = np.asarray(parts[name]["bounds_base_link_m"])
        center = bound.mean(axis=0)
        assert abs(center[0]) == pytest.approx(0.175, abs=0.00035)
        assert abs(center[1]) == pytest.approx(0.3019, abs=0.00035)
        assert center[2] == pytest.approx(-0.3165, abs=0.00035)
    right = np.asarray(parts["BasePrint"]["bounds_base_link_m"])
    left = np.asarray(parts["BasePrint001"]["bounds_base_link_m"])
    assert right[:, 1].max() < 0 < left[:, 1].min()


def test_wrist_camera_depth_uses_confirmed_caliper_datums() -> None:
    recipe = json.loads(
        (PACKAGE / "config/geometry/freecad_wrist_camera_depth.json").read_text()
    )
    assert MANIFEST["wrist_camera_depth"] == recipe
    parts = {part["name"]: part for part in MANIFEST["parts"]}
    for side, direction in recipe["optical_forward_base_link"].items():
        axis = np.array(direction)

        def depths(role: str) -> np.ndarray:
            part = parts[f"{side}Camera_{role}"]
            return np.concatenate(
                [triangles(PACKAGE / mesh["mesh"]).reshape(-1, 3) for mesh in part["meshes"]]
            ) @ axis

        front = depths("Glass").max()
        board = depths("PCB")
        lens = depths("Lens")
        assert lens.min() == pytest.approx(board.max(), abs=1e-7)
        assert lens.max() == pytest.approx(depths("Glass").min(), abs=1e-7)
        assert (front - board.min()) * 1000 == pytest.approx(
            recipe["lens_front_to_pcb_back_mm"], abs=0.001
        )
        assert (front - depths("FixedMount").min()) * 1000 == pytest.approx(
            recipe["lens_front_to_mount_back_mm"], abs=0.001
        )


def test_screen_texture_is_packaged_without_a_fake_solid() -> None:
    (item,) = MANIFEST["image_planes"]
    image = PACKAGE / "meshes/freecad_final/idle.png"
    assert hashlib.sha256(image.read_bytes()).hexdigest() == item["image_sha256"]
    assert item["collision"] is False
    dae = ET.parse(PACKAGE / "meshes/freecad_final/DefaultExpressionScreen.dae")
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    assert dae.find(".//c:library_images/c:image/c:init_from", ns).text == "idle.png"
    assert dae.find(".//c:triangles", ns).attrib["count"] == "2"


def test_mujoco_loads_the_same_static_assembly_without_actuation() -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(PACKAGE / "mjcf/cleany_freecad_final.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nq == model.nv == model.nu == 0
    assert np.all(model.body_mass == 0)
    assert model.ngeom == sum(len(p["meshes"]) for p in MANIFEST["parts"]) + 1
    assert np.all(model.geom_contype == 0)
    assert np.all(model.geom_conaffinity == 0)
    for name in (
        "InstalledProfile00",
        "InstalledArmMountPanel",
        "BasePrint",
        "HeadCamera",
    ):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = (
            model.mesh_vert[start : start + count]
            @ data.geom_xmat[geom].reshape(3, 3).T
            + data.geom_xpos[geom]
        )
        expected = next(p for p in MANIFEST["parts"] if p["name"] == name)
        bounds = np.asarray(expected["bounds_base_link_m"]) + [0, 0, 0.38]
        assert np.asarray(
            [vertices.min(axis=0), vertices.max(axis=0)]
        ) == pytest.approx(bounds, abs=2e-6)


def test_mujoco_stl_chunks_preserve_every_source_triangle() -> None:
    for part in MANIFEST["parts"]:
        meshes = part["meshes"]
        assert all(mesh["triangles"] <= 200_000 for mesh in meshes)
        if len(meshes) == 1:
            continue
        pieces = [(PACKAGE / mesh["mesh"]).read_bytes() for mesh in meshes]
        header = pieces[0][:80] + struct.pack("<I", part["triangles"])
        records = b"".join(piece[84:] for piece in pieces)
        assert hashlib.sha256(header + records).hexdigest() == part[
            "unpartitioned_stl_sha256"
        ]


def test_launch_rejects_old_calibration_on_new_assembly() -> None:
    spec = importlib.util.spec_from_file_location(
        "description_launch", PACKAGE / "launch/description.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.select_model("freecad_final", "") == "cleany_freecad_final.urdf"
    assert (
        module.select_model("freecad_kinematic", "") == "cleany_freecad_kinematic.urdf"
    )
    assert (
        module.select_model("legacy_runtime", "jetson_20261009") == "cleany.urdf.xacro"
    )
    with pytest.raises(ValueError, match="no validated calibration"):
        module.select_model("freecad_final", "jetson_20261009")
    with pytest.raises(ValueError, match="no validated calibration"):
        module.select_model("freecad_kinematic", "jetson_20261009")
    with pytest.raises(ValueError, match="Unknown model"):
        module.select_model("unknown", "")
