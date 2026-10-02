"""Uniform matte felt visual; contact parameters remain independently defined."""

from pathlib import Path
from xml.etree import ElementTree as ET


def apply_felt_floor(
    visual: ET.Element,
    width: float,
    depth: float,
    package: Path,
    output: Path,
    tile_m: float,
) -> None:
    # Explicit UVs keep fibre scale constant as facility dimensions change.
    x, y = width / 2, depth / 2
    u, v = width / tile_m, depth / tile_m
    mesh_path = output.with_suffix('.floor.dae')
    mesh_path.write_text(
        f"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="floor"><mesh>
<source id="positions"><float_array id="positions-array" count="12">{-x} {-y} 0 {x} {-y} 0 {x} {y} 0 {-x} {y} 0</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="8">0 0 {u} 0 {u} {v} 0 {v}</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="normals-array" count="3">0 0 1</float_array><technique_common><accessor source="#normals-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="2"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#normals" offset="2"/><p>0 0 0 1 1 0 2 2 0 0 0 0 2 2 0 3 3 0</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="floor-node"><instance_geometry url="#floor"/></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
""",
        encoding='utf-8',
    )
    for child in list(visual):
        if child.tag in ('geometry', 'material'):
            visual.remove(child)
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = mesh_path.resolve().as_uri()
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '1 1 1 1'
    ET.SubElement(material, 'diffuse').text = '1 1 1 1'
    ET.SubElement(material, 'specular').text = '0 0 0 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = (
        (package / 'materials/felt/albedo.png').resolve().as_uri()
    )
    ET.SubElement(metal, 'roughness').text = '1'
    ET.SubElement(metal, 'metalness').text = '0'
