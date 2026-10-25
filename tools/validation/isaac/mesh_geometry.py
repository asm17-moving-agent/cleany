"""Read the OBJ and identity-node COLLADA meshes emitted by Cleany's generators."""
from pathlib import Path
import xml.etree.ElementTree as ET


def read_geometry(path: Path) -> tuple[list, list, list]:
    points, counts, indices = [], [], []
    if path.suffix.lower() == ".obj":
        for line in path.read_text().splitlines():
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "v":
                points.append(list(map(float, fields[1:4])))
            elif fields[0] == "f":
                face = [int(value.split("/")[0]) for value in fields[1:]]
                indices.extend(index-1 if index > 0 else len(points)+index for index in face)
                counts.append(len(face))
    elif path.suffix.lower() == ".dae":
        root = ET.parse(path).getroot()
        for element in root.iter():
            element.tag = element.tag.rsplit("}", 1)[-1]
        assert root.findtext("asset/up_axis") == "Z_UP"
        assert float(root.find("asset/unit").get("meter")) == 1.
        assert all(child.tag == "instance_geometry" for node in root.findall(".//node") for child in node)
        meshes = root.findall("library_geometries/geometry/mesh")
        assert len(meshes) == 1, path
        mesh = meshes[0]
        vertices = mesh.find("vertices")
        source_id = vertices.find("input[@semantic='POSITION']").get("source")[1:]
        source = mesh.find(f"source[@id='{source_id}']")
        accessor = source.find("technique_common/accessor")
        assert int(accessor.get("stride")) == 3
        values = list(map(float, source.findtext("float_array").split()))
        points = [values[i:i+3] for i in range(0, len(values), 3)]
        assert len(points) == int(accessor.get("count"))
        assert not any(mesh.findall(tag) for tag in ("polylist", "polygons", "tristrips", "trifans"))
        for triangles in mesh.findall("triangles"):
            inputs = triangles.findall("input")
            vertex = triangles.find("input[@semantic='VERTEX']")
            assert vertex.get("source") == "#"+vertices.get("id")
            stride = max(int(item.get("offset")) for item in inputs)+1
            values = list(map(int, triangles.findtext("p").split()))
            face_indices = values[int(vertex.get("offset"))::stride]
            assert len(face_indices) == int(triangles.get("count"))*3
            indices.extend(face_indices)
            counts.extend([3]*(len(face_indices)//3))
    else:
        raise ValueError(f"Unsupported mesh format: {path}")
    assert points and counts and min(indices) >= 0 and max(indices) < len(points), path
    return points, counts, indices
