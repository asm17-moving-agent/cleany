#!/usr/bin/env python3
"""macro 실행이나 CAD 변경 없이 저장된 FreeCAD BREP 형상을 검사합니다.

cadquery-ocp가 필요하며 좌표는 저장된 BREP의 밀리미터 단위입니다.
importer가 상위 조립체 배치를 두 번 적용하지 않도록 배치를 명시적으로 보고합니다.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.TopoDS import TopoDS_Shape


def read_shape(archive: zipfile.ZipFile, member: str) -> TopoDS_Shape:
    shape = TopoDS_Shape()
    BRepTools.Read_s(shape, io.BytesIO(archive.read(member)), BRep_Builder())
    if shape.IsNull():
        raise ValueError(f"Empty CAD shape: {member}")
    return shape


def bounds(shape: TopoDS_Shape) -> list[float]:
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return list(box.Get())


def inventory(source: Path) -> dict:
    with zipfile.ZipFile(source) as archive:
        document = ET.fromstring(archive.read("Document.xml"))
        objects = {o.get("name"): o for o in document.findall("./ObjectData/Object")}
        records = []
        assemblies = {}

        def visit(name: str, parents: tuple[str, ...]) -> None:
            obj = objects[name]
            label = obj.find("./Properties/Property[@name='Label']/String")
            placement = obj.find("./Properties/Property[@name='Placement']/PropertyPlacement")
            shape = obj.find("./Properties/Property[@name='Shape']/Part")
            if shape is not None and shape.get("file"):
                member = shape.get("file")
                records.append({"name": name, "label": label.get("value") if label is not None else name,
                    "parents": list(parents), "placement": dict(placement.attrib) if placement is not None else {},
                    "brep": member, "sha256": hashlib.sha256(archive.read(member)).hexdigest(),
                    "bounds_mm": bounds(read_shape(archive, member))})
            else:
                assemblies[name] = dict(placement.attrib) if placement is not None else {}
                for child in obj.findall("./Properties/Property[@name='Group']/LinkList/Link"):
                    visit(child.get("value"), parents + (name,))

        visit("CleanyAssembly", ())
        return {"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "units": "mm", "assembly_placements": assemblies, "objects": records}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = inventory(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(f"Inspected {len(result['objects'])} CAD components")


if __name__ == "__main__":
    main()
