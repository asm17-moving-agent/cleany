"""Regression checks for coordinate conversion and safe CAD reading."""

from __future__ import annotations

import importlib.util
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
