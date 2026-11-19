"""Facility table, framed glazing, and a ceiling visible only from below."""

from math import atan2, cos, hypot, radians, sin
from pathlib import Path
from xml.etree import ElementTree as ET

from cleany_gazebo_sim.world.generator import _add_box_part


def static_body(world: ET.Element, name: str, pose: tuple) -> ET.Element:
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(map(str, pose))
    return ET.SubElement(model, "link", name="body")


def add_glazing(
    world: ET.Element,
    name: str,
    start: tuple,
    end: tuple,
    height: float,
    config: dict,
    angle_deg: float = 0,
) -> None:
    """A static leaf rotated about its first endpoint; zero angle is closed."""
    length = hypot(end[0] - start[0], end[1] - start[1])
    yaw = atan2(end[1] - start[1], end[0] - start[0]) + radians(angle_deg)
    link = static_body(
        world,
        name,
        (
            start[0] + length / 2 * cos(yaw),
            start[1] + length / 2 * sin(yaw),
            height / 2,
            0,
            0,
            yaw,
        ),
    )
    frame = config["frame_width_m"]
    tint = " ".join(map(str, config["color"]))
    metal = " ".join(map(str, config["frame_color"]))
    _add_box_part(
        link, "glass", (length, config["thickness_m"], height), (0, 0, 0, 0, 0, 0), tint
    )
    visual = link.find("visual[@name='glass_visual']")
    # Keep collision solid. This is a display approximation of glass, not
    # wavelength-dependent transmission/reflection for physical LiDAR.
    ET.SubElement(visual, "transparency").text = str(config["transparency"])
    ET.SubElement(visual, "cast_shadows").text = "false"
    ET.SubElement(visual.find("material"), "specular").text = "0.8 0.8 0.8 1"
    for side in (-1, 1):
        _add_box_part(
            link,
            f"edge_{side}",
            (frame, frame, height),
            (side * (length - frame) / 2, 0, 0, 0, 0, 0),
            metal,
        )
        _add_box_part(
            link,
            f"rail_{side}",
            (length, frame, frame),
            (0, 0, side * (height - frame) / 2, 0, 0, 0),
            metal,
        )


def add_central_table(world: ET.Element, layout) -> None:
    config = layout.raw["central_table"]
    a, b = layout.world(config["bounds"][:2]), layout.world(config["bounds"][2:])
    width, depth = abs(b[0] - a[0]), abs(b[1] - a[1])
    height = config["height_m"]
    link = static_body(
        world,
        "facility_ground_table",
        ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0, 0, 0, 0),
    )
    _add_box_part(
        link,
        "solid_body",
        (width, depth, height),
        (0, 0, height / 2, 0, 0, 0),
        " ".join(map(str, config["color"])),
    )


def add_ceiling(world: ET.Element, layout, target: Path) -> None:
    config = layout.raw["ceiling"]
    a = layout.world(layout.raw["floor_bounds"][:2])
    b = layout.world(layout.raw["floor_bounds"][2:])
    width, depth = abs(b[0] - a[0]), abs(b[1] - a[1])
    x, y = width / 2, depth / 2
    mesh = target.with_suffix(".ceiling.dae")
    # Clockwise triangles have downward normals. With ordinary backface
    # culling, the ceiling disappears when the camera is above it.
    mesh.write_text(
        f"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="ceiling"><mesh>
<source id="positions"><float_array id="positions-array" count="12">{-x} {-y} 0 {x} {-y} 0 {x} {y} 0 {-x} {y} 0</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="normals-array" count="3">0 0 -1</float_array><technique_common><accessor source="#normals-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="2"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><p>0 0 2 0 1 0 0 0 3 0 2 0</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="ceiling"><instance_geometry url="#ceiling"/></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
""",
        encoding="utf-8",
    )
    link = static_body(
        world,
        "facility_ceiling",
        ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, config["height_m"], 0, 0, 0),
    )
    collision = ET.SubElement(link, "collision", name="ceiling_collision")
    ET.SubElement(collision, "pose").text = f"0 0 {config['thickness_m'] / 2} 0 0 0"
    box = ET.SubElement(ET.SubElement(collision, "geometry"), "box")
    ET.SubElement(box, "size").text = f"{width} {depth} {config['thickness_m']}"
    visual = ET.SubElement(link, "visual", name="ceiling_underside")
    ET.SubElement(visual, "cast_shadows").text = "false"
    shape = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
    ET.SubElement(shape, "uri").text = mesh.resolve().as_uri()
    material = ET.SubElement(visual, "material")
    for tag in ("ambient", "diffuse"):
        ET.SubElement(material, tag).text = " ".join(map(str, config["color"]))
