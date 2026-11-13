"""Regression checks for coordinate conversion and safe CAD reading."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "export_freecad_model", ROOT / "tools/export_freecad_model.py"
)
exporter = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = exporter
spec.loader.exec_module(exporter)


def test_coordinate_convention_is_right_handed_and_preserves_millimetres() -> None:
    matrix = exporter.CAD_TO_BASE
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1e-9)
    points = np.array([[0, 0, -95.5], [0, -100, -95.5], [100, 0, -95.5]])
    assert exporter.transformed(points, matrix) == pytest.approx(
        np.array([[0, 0, -0.38], [0.1, 0, -0.38], [0, 0.1, -0.38]]),
    )


def test_placement_rejects_zero_quaternion() -> None:
    element = ET.fromstring(
        '<Property><PropertyPlacement Q0="0" Q1="0" Q2="0" Q3="0" Px="0" Py="0" Pz="0"/></Property>'
    )
    with pytest.raises(ValueError, match="quaternion"):
        exporter.placement(element)


def test_parent_transform_is_applied_once_and_object_placement_is_not_repeated(
    tmp_path, monkeypatch
) -> None:
    document = """<Document ProgramVersion="test"><ObjectData>
      <Object name="CleanyAssembly"><Properties>
        <Property name="Group"><LinkList><Link value="part"/></LinkList></Property>
        <Property name="Placement"><PropertyPlacement Q0="0" Q1="0" Q2="0" Q3="1" Px="10" Py="20" Pz="30"/></Property>
      </Properties></Object>
      <Object name="part"><Properties>
        <Property name="Shape"><Part file="part.brp"/></Property>
        <Property name="Label"><String value="test part"/></Property>
        <Property name="Placement"><PropertyPlacement Q0="0" Q1="0" Q2="0" Q3="1" Px="1000" Py="2000" Pz="3000"/></Property>
      </Properties></Object>
    </ObjectData></Document>"""
    gui = '<Document><ViewProviderData><ViewProvider name="part"><Properties><Property name="ShapeAppearance"><MaterialList file="material" version="3"/></Property></Properties></ViewProvider></ViewProviderData></Document>'
    path = tmp_path / "sample.FCStd"
    with ZipFile(path, "w") as archive:
        archive.writestr("Document.xml", document)
        archive.writestr("GuiDocument.xml", gui)
        archive.writestr("part.brp", b"already placed BREP")
        archive.writestr("material", struct.pack("<I", 1) + bytes([255] * 12))
    vertices = np.array(
        [[1000.0, 2000.0, 3000.0], [1010.0, 2000.0, 3000.0], [1000.0, 2010.0, 3000.0]]
    )
    monkeypatch.setattr(
        exporter, "solid_mesh", lambda *_: (vertices, np.array([[0, 1, 2]]))
    )
    with ZipFile(path) as archive:
        parts, _ = exporter.read_assembly(archive, 0.35)
    expected = vertices + [10, 20, 30]
    assert parts[0].vertices == pytest.approx(
        exporter.transformed(expected, exporter.CAD_TO_BASE)
    )


@pytest.mark.parametrize("deflection", [0, -1, float("nan"), float("inf")])
def test_invalid_tessellation_does_not_create_assets(tmp_path, deflection) -> None:
    destination = tmp_path / "package"
    with pytest.raises(ValueError, match="Deflection"):
        exporter.export(tmp_path / "unused.FCStd", destination, deflection)
    assert not destination.exists()


def test_binary_stl_preserves_geometry_and_winding() -> None:
    part = exporter.Part(
        "p",
        "p",
        (),
        np.array([[0.0, 0, 0], [0.1, 0, 0], [0, 0.2, 0]]),
        np.array([[0, 1, 2]]),
        (1, 1, 1, 1),
        "source",
    )
    data = exporter.stl_bytes(part)
    assert len(data) == 134
    record = np.frombuffer(data, dtype=exporter.STL_TRIANGLE, offset=84)[0]
    assert record["vertices"] == pytest.approx(part.vertices)
    assert record["normal"] == pytest.approx([0, 0, 1])


def test_mesher_origin_artifact_is_rejected_but_unused_nodes_are_ignored() -> None:
    vertices = np.array([[0.0, 0, 931.5], [1, 0, 931.5], [0, 1, 931.5], [0, 0, 0]])
    bounds = np.array([[0.0, 0, 931.49], [1, 1, 931.51]])
    assert exporter.within_bounds(vertices, np.array([[0, 1, 2]]), bounds, 0.35)
    assert not exporter.within_bounds(vertices, np.array([[0, 1, 3]]), bounds, 0.35)
    assert not exporter.within_bounds(vertices, np.array([[0, 1, 8]]), bounds, 0.35)


@pytest.fixture
def export_input(tmp_path, monkeypatch):
    part = exporter.Part(
        "p",
        "p",
        (),
        np.array([[0.0, 0, 0], [0.1, 0, 0], [0, 0.2, 0], [0, 0, 0.3]]),
        np.array([[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]]),
        (1, 1, 1, 1),
        "source",
    )
    source = tmp_path / "sample.FCStd"
    with ZipFile(source, "w"):
        pass
    monkeypatch.setattr(exporter, "MJCF_MAX_STL_TRIANGLES", 2)
    monkeypatch.setattr(
        exporter,
        "read_assembly",
        lambda *_: (
            [part],
            {"program": "test", "design_status": "draft", "images": []},
        ),
    )
    return source, tmp_path / "package", part


@pytest.mark.parametrize("triangle_count", [1, 2, 3, 4])
def test_export_shares_complete_triangle_records_across_model_formats(
    export_input, triangle_count
) -> None:
    source, package, part = export_input
    part.faces.resize((triangle_count, 3), refcheck=False)
    manifest = exporter.export(source, package)
    meshes = manifest["parts"][0]["meshes"]
    assert sum(mesh["triangles"] for mesh in meshes) == triangle_count
    assert all(mesh["triangles"] <= 2 for mesh in meshes)
    files = {package / mesh["mesh"] for mesh in meshes}
    assert set((package / "meshes").rglob("*.stl")) == files
    pieces = [(package / mesh["mesh"]).read_bytes() for mesh in meshes]
    reconstructed = pieces[0][:80] + struct.pack("<I", triangle_count)
    reconstructed += b"".join(piece[84:] for piece in pieces)
    assert reconstructed == exporter.stl_bytes(part)
    urdf = ET.parse(package / "urdf/cleany_freecad_final.urdf")
    mjcf = ET.parse(package / "mjcf/cleany_freecad_final.xml")
    for kind in ("visual", "collision"):
        assert {
            m.get("filename") for m in urdf.findall(f".//{kind}/geometry/mesh")
        } == {"package://cleany_description/" + mesh["mesh"] for mesh in meshes}
    assert {m.get("file") for m in mjcf.findall("asset/mesh")} == {
        p.name for p in files
    }


@pytest.mark.parametrize("modified", [False, True])
def test_legacy_export_cleanup_preserves_user_files_and_retries(
    export_input, monkeypatch, modified
):
    source, package, part = export_input
    manifest_path = package / "config/geometry/freecad_final.json"
    manifest_path.parent.mkdir(parents=True)
    old_mesh = package / "meshes/freecad_final/p.stl"
    old_chunk = old_mesh.parent / "mujoco/p__chunk_00.stl"
    old_chunk.parent.mkdir(parents=True)
    data = exporter.stl_bytes(part)
    old_mesh.write_bytes(data)
    old_chunk.write_bytes(data)
    outside = package / "preserve.stl"
    untracked = old_mesh.parent / "notes.stl"
    outside.write_bytes(data)
    untracked.write_bytes(data)
    legacy = {
        "schema_version": 1,
        "parts": [
            {
                "mesh": str(old_mesh.relative_to(package)),
                "sha256": hashlib.sha256(data).hexdigest(),
                "mujoco_meshes": [
                    {
                        "mesh": str(p.relative_to(package)),
                        "sha256": hashlib.sha256(data).hexdigest(),
                    }
                    for p in (old_chunk, outside)
                ],
            }
        ],
    }
    manifest_path.write_text(json.dumps(legacy))
    assert (
        exporter.load_static_manifest(package)["parts"][0]["meshes"]
        == legacy["parts"][0]["mujoco_meshes"]
    )
    if modified:
        old_mesh.write_bytes(b"user edit")
    with monkeypatch.context() as failure:

        def fail_save(*_):
            raise OSError("injected write failure")

        failure.setattr(exporter, "save_xml", fail_save)
        with pytest.raises(OSError, match="injected"):
            exporter.export(source, package)
    assert old_mesh.exists() and old_chunk.exists()
    assert json.loads(manifest_path.read_text()) == legacy
    if modified:
        with pytest.warns(UserWarning, match="Keeping modified obsolete mesh"):
            exporter.export(source, package)
        assert old_mesh.read_bytes() == b"user edit"
    else:
        exporter.export(source, package)
        assert not old_mesh.exists()
    assert not old_chunk.parent.exists()
    assert outside.read_bytes() == untracked.read_bytes() == data
    generated = {p: p.read_bytes() for p in package.rglob("*") if p.is_file()}
    exporter.export(source, package)
    assert {p: p.read_bytes() for p in package.rglob("*") if p.is_file()} == generated


def test_unknown_manifest_schema_is_rejected_before_export(export_input):
    source, package, _ = export_input
    path = package / "config/geometry/freecad_final.json"
    path.parent.mkdir(parents=True)
    path.write_text('{"schema_version": 99}')
    with pytest.raises(ValueError, match="Unsupported static manifest schema"):
        exporter.export(source, package)
    assert not (package / "meshes").exists()
