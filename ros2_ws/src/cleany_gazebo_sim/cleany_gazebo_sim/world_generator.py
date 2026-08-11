from __future__ import annotations

from copy import deepcopy
from math import asin, atan2, cos, pi, sin, sqrt
from pathlib import Path
from tempfile import gettempdir
from xml.etree import ElementTree


_WHEEL_HANDEDNESS = {
    'rear_left': -1.0,
    'rear_right': 1.0,
    'front_left': 1.0,
    'front_right': -1.0,
}
_ROLLER_RADIUS = 0.008
_ROLLER_LENGTH = 0.03
_ROLLER_CENTER_RADIUS = 0.0555
_OFFICE_SPAWN_POSE = (5.49526, -8.97241, 0.38, 0.0, 0.0, 2.7409)
_STUDY_CAFE_SPAWN_POSE = (-1.865, -4.705, 0.38, 0.0, 0.0, 1.5708)
_ROBOT_VISIBILITY_FLAGS = '0x02'
_FUEL_VISUALS = {
    'adj_table': (
        'https://fuel.gazebosim.org/1.0/openrobotics/models/'
        'adjtable/3/files/meshes/AdjTable.obj'
    ),
    'wooden_chair': (
        'https://fuel.gazebosim.org/1.0/openrobotics/models/'
        'woodenchair/1/files/meshes/WoodenChair.obj'
    ),
    'square_shelf': (
        'https://fuel.gazebosim.org/1.0/openrobotics/models/'
        'squareshelf/2/files/meshes/SquareShelf.obj'
    ),
    'office_chair_grey': (
        'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
        'OfficeChairGrey/1/files/meshes/OfficeChairGrey.obj'
    ),
}
_HARMONIC_SYSTEM_PLUGINS = {
    'ignition-gazebo-contact-system': 'gz-sim-contact-system',
    'ignition-gazebo-imu-system': 'gz-sim-imu-system',
    'ignition-gazebo-physics-system': 'gz-sim-physics-system',
    'ignition-gazebo-scene-broadcaster-system': (
        'gz-sim-scene-broadcaster-system'
    ),
    'ignition-gazebo-sensors-system': 'gz-sim-sensors-system',
    'ignition-gazebo-user-commands-system': (
        'gz-sim-user-commands-system'
    ),
}


def fixed_roller_visual_sdf(prefix: str, handedness: float) -> str:
    """Generate one wheel's fixed, non-controllable roller visuals."""
    fragments: list[str] = []
    for index in range(12):
        angle = index * pi / 6.0
        x = _ROLLER_CENTER_RADIUS * cos(angle)
        y = _ROLLER_CENTER_RADIUS * sin(angle)
        axis_x = -sin(angle) / sqrt(2.0)
        axis_y = cos(angle) / sqrt(2.0)
        axis_z = handedness / sqrt(2.0)
        roll = -asin(axis_y)
        pitch = atan2(axis_x, axis_z)
        fragments.append(
            f'''<visual name="{prefix}_roller_{index:02d}_visual">
  <pose>{x:.6f} {y:.6f} 0 {roll:.6f} {pitch:.6f} 0</pose>
  <geometry><capsule><radius>{_ROLLER_RADIUS}</radius><length>{_ROLLER_LENGTH}</length></capsule></geometry>
  <material><diffuse>0.06 0.06 0.07 1</diffuse></material>
</visual>'''
        )
    return '\n'.join(fragments)


def materialize_mecanum_wheel_world(template_path: Path) -> Path:
    """Materialize compact mecanum visuals without exposing roller joints."""
    template = template_path.read_text(encoding='utf-8')
    world = template
    for prefix, handedness in _WHEEL_HANDEDNESS.items():
        marker = f'<!-- CLEANY_{prefix.upper()}_ROLLER_VISUALS -->'
        if world.count(marker) != 1:
            raise ValueError(
                f'world template must contain one {prefix} roller marker'
            )
        world = world.replace(
            marker,
            fixed_roller_visual_sdf(prefix, handedness),
        )

    root = ElementTree.fromstring(world)
    robot = root.find("./world/model[@name='cleany_mecanum']")
    if robot is None:
        raise ValueError('world template is missing cleany_mecanum')
    for visual in robot.findall('.//visual'):
        flags = visual.find('visibility_flags')
        if flags is None:
            flags = ElementTree.Element('visibility_flags')
            geometry = visual.find('geometry')
            insert_at = (
                list(visual).index(geometry) if geometry is not None else 0
            )
            visual.insert(insert_at, flags)
        flags.text = _ROBOT_VISIBILITY_FLAGS

    target = Path(gettempdir()) / 'cleany_mecanum_fixed_roller_visuals.sdf'
    ElementTree.register_namespace(
        'gz', 'http://gazebosim.org/schema'
    )
    ElementTree.ElementTree(root).write(
        target, encoding='unicode', xml_declaration=True
    )
    return target


def materialize_husarion_office_world(
    office_template_path: Path,
    robot_template_path: Path,
    target_path: Path | None = None,
    simulator: str = 'harmonic',
) -> Path:
    """Replace Husarion's demo robots with Cleany in the office world."""
    if simulator != 'harmonic':
        raise ValueError('the Cleany office profile supports harmonic only')
    robot_world_path = materialize_mecanum_wheel_world(robot_template_path)
    office_root = ElementTree.parse(office_template_path).getroot()
    robot_root = ElementTree.parse(robot_world_path).getroot()
    office = office_root.find('world')
    robot_world = robot_root.find('world')
    if office is None or robot_world is None:
        raise ValueError('both SDF documents must contain a world')
    if office.find("model[@name='cleany_mecanum']") is not None:
        raise ValueError('office world already contains cleany_mecanum')

    demo_robots = [
        model
        for model in office.findall('model')
        if model.get('name', '').startswith('OpenRobotics/_Rosbot')
    ]
    if not demo_robots:
        raise ValueError('Husarion office world has no demo robots to replace')
    for demo_robot in demo_robots:
        office.remove(demo_robot)

    for plugin in office.findall('plugin'):
        filename = plugin.get('filename', '')
        replacement = _HARMONIC_SYSTEM_PLUGINS.get(filename)
        if replacement is not None:
            plugin.set('filename', replacement)

    cleany = robot_world.find("model[@name='cleany_mecanum']")
    if cleany is None:
        raise ValueError('robot template is missing cleany_mecanum')
    cleany = deepcopy(cleany)
    pose = cleany.find('pose')
    if pose is None:
        raise ValueError('cleany_mecanum is missing its world pose')
    pose.text = ' '.join(str(value) for value in _OFFICE_SPAWN_POSE)
    office.append(cleany)
    office.set('name', 'cleany_husarion_office')

    target = target_path or (
        Path(gettempdir()) / 'cleany_husarion_office.sdf'
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.register_namespace(
        'ignition', 'http://ignitionrobotics.org/schema'
    )
    ElementTree.ElementTree(office_root).write(
        target, encoding='unicode', xml_declaration=True
    )
    return target


def _add_box_model(
    world: ElementTree.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
    size: tuple[float, float, float],
    color: str,
    roughness: float | None = None,
) -> None:
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})
    for element_name in ('collision', 'visual'):
        element = ElementTree.SubElement(
            link, element_name, {'name': element_name}
        )
        geometry = ElementTree.SubElement(element, 'geometry')
        box = ElementTree.SubElement(geometry, 'box')
        ElementTree.SubElement(box, 'size').text = ' '.join(map(str, size))
        if element_name == 'visual':
            material = ElementTree.SubElement(element, 'material')
            if roughness is not None:
                ElementTree.SubElement(material, 'ambient').text = color
            ElementTree.SubElement(material, 'diffuse').text = color
            if roughness is not None:
                ElementTree.SubElement(material, 'specular').text = (
                    '0.03 0.03 0.03 1'
                )
                pbr = ElementTree.SubElement(material, 'pbr')
                metal = ElementTree.SubElement(pbr, 'metal')
                ElementTree.SubElement(metal, 'roughness').text = str(
                    roughness
                )
                ElementTree.SubElement(metal, 'metalness').text = '0.0'


def _add_fuel_furniture(
    world: ElementTree.Element,
    name: str,
    model_key: str,
    pose: tuple[float, float, float, float, float, float],
    collision_size: tuple[float, float, float],
    collision_z: float,
    visual_scale: tuple[float, float, float] | None = None,
) -> None:
    """Use a Fuel mesh only as a visual and a cheap box for collision."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})
    visual = ElementTree.SubElement(link, 'visual', {'name': 'fuel_visual'})
    geometry = ElementTree.SubElement(visual, 'geometry')
    mesh = ElementTree.SubElement(geometry, 'mesh')
    ElementTree.SubElement(mesh, 'uri').text = _FUEL_VISUALS[model_key]
    if visual_scale is not None:
        ElementTree.SubElement(mesh, 'scale').text = ' '.join(
            map(str, visual_scale)
        )

    if model_key == 'adj_table':
        _add_table_collisions(link, 1.6, 0.82, 0.72, 0.68, 0.32)
    elif model_key == 'wooden_chair':
        _add_chair_collisions(link)
    else:
        _add_box_collision(
            link, 'simple_collision', collision_size,
            (0.0, 0.0, collision_z)
        )


def _add_box_collision(
    link: ElementTree.Element,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
) -> None:
    collision = ElementTree.SubElement(link, 'collision', {'name': name})
    ElementTree.SubElement(collision, 'pose').text = (
        f'{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0'
    )
    geometry = ElementTree.SubElement(collision, 'geometry')
    box = ElementTree.SubElement(geometry, 'box')
    ElementTree.SubElement(box, 'size').text = ' '.join(map(str, size))


def _add_cylinder_collision(
    link: ElementTree.Element,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
) -> None:
    collision = ElementTree.SubElement(link, 'collision', {'name': name})
    ElementTree.SubElement(collision, 'pose').text = (
        f'{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0'
    )
    geometry = ElementTree.SubElement(collision, 'geometry')
    cylinder = ElementTree.SubElement(geometry, 'cylinder')
    ElementTree.SubElement(cylinder, 'radius').text = str(radius)
    ElementTree.SubElement(cylinder, 'length').text = str(length)


def _add_table_collisions(
    link: ElementTree.Element,
    width: float,
    depth: float,
    height: float,
    leg_x: float,
    leg_y: float,
) -> None:
    top_thickness = 0.04
    leg_length = height - top_thickness
    _add_box_collision(
        link,
        'tabletop_collision',
        (width, depth, top_thickness),
        (0.0, 0.0, height - top_thickness / 2.0),
    )
    for leg_name, x, y in (
        ('front_left', leg_x, leg_y),
        ('front_right', leg_x, -leg_y),
        ('back_left', -leg_x, leg_y),
        ('back_right', -leg_x, -leg_y),
    ):
        _add_cylinder_collision(
            link,
            f'{leg_name}_leg_collision',
            0.035,
            leg_length,
            (x, y, leg_length / 2.0),
        )


def _add_chair_collisions(link: ElementTree.Element) -> None:
    _add_box_collision(
        link, 'seat_collision', (0.36, 0.34, 0.04),
        (-0.01, 0.0, 0.42)
    )
    _add_box_collision(
        link, 'backrest_collision', (0.05, 0.36, 0.36),
        (-0.19, 0.0, 0.61)
    )
    for leg_name, x, y in (
        ('front_left', 0.14, 0.14),
        ('front_right', 0.14, -0.14),
        ('back_left', -0.14, 0.14),
        ('back_right', -0.14, -0.14),
    ):
        _add_cylinder_collision(
            link,
            f'{leg_name}_leg_collision',
            0.025,
            0.40,
            (x, y, 0.20),
        )


def _add_standard_table(
    world: ElementTree.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
) -> None:
    """Add a 72 cm variant of OpenRobotics' primitive Fuel Table."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})

    _add_table_collisions(link, 1.5, 0.8, 0.72, 0.68, 0.34)

    top = ElementTree.SubElement(link, 'visual', {'name': 'wood_top'})
    ElementTree.SubElement(top, 'pose').text = '0 0 0.70 0 0 0'
    geometry = ElementTree.SubElement(top, 'geometry')
    box = ElementTree.SubElement(geometry, 'box')
    ElementTree.SubElement(box, 'size').text = '1.5 0.8 0.04'
    material = ElementTree.SubElement(top, 'material')
    ElementTree.SubElement(material, 'diffuse').text = '0.58 0.34 0.16 1'
    pbr = ElementTree.SubElement(material, 'pbr')
    metal = ElementTree.SubElement(pbr, 'metal')
    ElementTree.SubElement(metal, 'albedo_map').text = (
        'https://fuel.gazebosim.org/1.0/openrobotics/models/'
        'table/3/files/Table_Diffuse.jpg'
    )

    for leg_name, x, y in (
        ('front_left', 0.68, 0.34),
        ('front_right', 0.68, -0.34),
        ('back_left', -0.68, 0.34),
        ('back_right', -0.68, -0.34),
    ):
        leg = ElementTree.SubElement(
            link, 'visual', {'name': f'{leg_name}_leg'}
        )
        ElementTree.SubElement(leg, 'pose').text = (
            f'{x} {y} 0.34 0 0 0'
        )
        geometry = ElementTree.SubElement(leg, 'geometry')
        cylinder = ElementTree.SubElement(geometry, 'cylinder')
        ElementTree.SubElement(cylinder, 'radius').text = '0.025'
        ElementTree.SubElement(cylinder, 'length').text = '0.68'
        material = ElementTree.SubElement(leg, 'material')
        ElementTree.SubElement(material, 'diffuse').text = (
            '0.20 0.20 0.22 1'
        )


def _add_box_part(
    link: ElementTree.Element,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
    color: str,
) -> None:
    """Add matching primitive collision and visual elements."""
    pose_text = ' '.join(map(str, pose))
    size_text = ' '.join(map(str, size))
    for element_name in ('collision', 'visual'):
        element = ElementTree.SubElement(
            link, element_name, {'name': f'{name}_{element_name}'}
        )
        ElementTree.SubElement(element, 'pose').text = pose_text
        geometry = ElementTree.SubElement(element, 'geometry')
        box = ElementTree.SubElement(geometry, 'box')
        ElementTree.SubElement(box, 'size').text = size_text
        if element_name == 'visual':
            material = ElementTree.SubElement(element, 'material')
            ElementTree.SubElement(material, 'ambient').text = color
            ElementTree.SubElement(material, 'diffuse').text = color


def _add_cylinder_part(
    link: ElementTree.Element,
    name: str,
    radius: float,
    length: float,
    pose: tuple[float, float, float, float, float, float],
    color: str,
) -> None:
    """Add matching cylindrical collision and visual elements."""
    pose_text = ' '.join(map(str, pose))
    for element_name in ('collision', 'visual'):
        element = ElementTree.SubElement(
            link, element_name, {'name': f'{name}_{element_name}'}
        )
        ElementTree.SubElement(element, 'pose').text = pose_text
        geometry = ElementTree.SubElement(element, 'geometry')
        cylinder = ElementTree.SubElement(geometry, 'cylinder')
        ElementTree.SubElement(cylinder, 'radius').text = str(radius)
        ElementTree.SubElement(cylinder, 'length').text = str(length)
        if element_name == 'visual':
            material = ElementTree.SubElement(element, 'material')
            ElementTree.SubElement(material, 'ambient').text = color
            ElementTree.SubElement(material, 'diffuse').text = color


def _add_demo_desk(
    world: ElementTree.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
    front_sign: float,
) -> None:
    """Add the 1.2 m white desk with two compact A-frame supports."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})
    white = '0.92 0.93 0.94 1'

    corner_radius = 0.06
    tabletop_half_depth = 0.385
    # Keep the partition-side corners square and round only the two corners
    # on the chair-facing edge. The union of two boxes and two cylinders
    # gives visual and collision geometry the same footprint.
    _add_box_part(
        link, 'tabletop_back', (1.2, 0.77 - corner_radius, 0.04),
        (
            0.0, -front_sign * corner_radius / 2.0, 0.70,
            0.0, 0.0, 0.0,
        ),
        white,
    )
    _add_box_part(
        link,
        'tabletop_front_center',
        (1.2 - 2.0 * corner_radius, corner_radius, 0.04),
        (
            0.0,
            front_sign * (tabletop_half_depth - corner_radius / 2.0),
            0.70,
            0.0, 0.0, 0.0,
        ),
        white,
    )
    for side_name, x in (
        ('left', -0.60 + corner_radius),
        ('right', 0.60 - corner_radius),
    ):
        _add_cylinder_part(
            link,
            f'tabletop_front_{side_name}_corner',
            corner_radius,
            0.04,
            (
                x,
                front_sign * (tabletop_half_depth - corner_radius),
                0.70,
                0.0,
                0.0,
                0.0,
            ),
            white,
        )
    leg_bottom_z = 0.02
    leg_top_z = 0.67
    # Supports sit 8 cm inboard from every tabletop edge.
    for support_name, x in (('left', -0.52), ('right', 0.52)):
        for side_name, bottom_y in (
            ('front', tabletop_half_depth - 0.08),
            ('back', -tabletop_half_depth + 0.08),
        ):
            top_y = 0.035 if bottom_y > 0 else -0.035
            delta_y = top_y - bottom_y
            delta_z = leg_top_z - leg_bottom_z
            leg_length = sqrt(delta_y ** 2 + delta_z ** 2)
            roll = atan2(-delta_y, delta_z)
            _add_box_part(
                link,
                f'{support_name}_{side_name}_leg',
                (0.045, 0.045, leg_length),
                (
                    x,
                    (bottom_y + top_y) / 2.0,
                    (leg_bottom_z + leg_top_z) / 2.0,
                    roll,
                    0.0,
                    0.0,
                ),
                white,
            )
    _add_box_part(
        link, 'upper_crossbar', (0.82, 0.045, 0.045),
        (0.0, 0.0, 0.62, 0.0, 0.0, 0.0), white
    )


def _add_rounded_partition(
    world: ElementTree.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
) -> None:
    """Add a thin divider with four rounded corners in the X-Z plane."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})
    color = '0.78 0.80 0.82 1'
    width = 1.2
    thickness = 0.025
    height = 0.72
    radius = 0.05

    _add_box_part(
        link, 'partition_center', (width - 2.0 * radius, thickness, height),
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), color
    )
    _add_box_part(
        link, 'partition_middle', (width, thickness, height - 2.0 * radius),
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), color
    )
    for horizontal_name, x in (
        ('left', -width / 2.0 + radius),
        ('right', width / 2.0 - radius),
    ):
        for vertical_name, z in (
            ('bottom', -height / 2.0 + radius),
            ('top', height / 2.0 - radius),
        ):
            _add_cylinder_part(
                link,
                f'partition_{vertical_name}_{horizontal_name}_corner',
                radius,
                thickness,
                (x, 0.0, z, pi / 2.0, 0.0, 0.0),
                color,
            )


def _add_desk_monitor(
    world: ElementTree.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
    front_sign: float,
) -> None:
    """Add one black 27-inch 16:9 monitor facing its chair."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})
    bezel_color = '0.025 0.025 0.03 1'
    screen_color = '0.008 0.010 0.014 1'

    _add_box_part(
        link, 'monitor_panel', (0.62, 0.035, 0.36),
        (0.0, 0.0, 1.00, 0.0, 0.0, 0.0), bezel_color
    )
    _add_box_part(
        link, 'monitor_stem', (0.035, 0.035, 0.12),
        (0.0, 0.0, 0.79, 0.0, 0.0, 0.0), bezel_color
    )
    _add_box_part(
        link, 'monitor_base', (0.24, 0.16, 0.02),
        (0.0, front_sign * 0.04, 0.73, 0.0, 0.0, 0.0),
        bezel_color,
    )

    screen = ElementTree.SubElement(
        link, 'visual', {'name': 'monitor_screen_visual'}
    )
    ElementTree.SubElement(screen, 'pose').text = (
        f'0.0 {front_sign * 0.0185} 1.0 0.0 0.0 0.0'
    )
    geometry = ElementTree.SubElement(screen, 'geometry')
    box = ElementTree.SubElement(geometry, 'box')
    # 0.598 x 0.336 m is a 27-inch diagonal at 16:9.
    ElementTree.SubElement(box, 'size').text = '0.598 0.002 0.336'
    material = ElementTree.SubElement(screen, 'material')
    ElementTree.SubElement(material, 'ambient').text = screen_color
    ElementTree.SubElement(material, 'diffuse').text = screen_color


def _add_office_chair(
    world: ElementTree.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
) -> None:
    """Add a Fuel office-chair visual with lightweight collisions."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})

    visual = ElementTree.SubElement(
        link, 'visual', {'name': 'office_chair_visual'}
    )
    # Fuel's chair faces -Y. Rotate it so the model's +X is the front.
    ElementTree.SubElement(visual, 'pose').text = (
        '0 0 0 0 0 1.5707963267948966'
    )
    geometry = ElementTree.SubElement(visual, 'geometry')
    mesh = ElementTree.SubElement(geometry, 'mesh')
    ElementTree.SubElement(mesh, 'uri').text = _FUEL_VISUALS[
        'office_chair_grey'
    ]
    ElementTree.SubElement(mesh, 'scale').text = '0.9 0.9 0.9'

    _add_cylinder_collision(
        link, 'caster_base_collision', 0.32, 0.06,
        (0.0, 0.0, 0.05)
    )
    _add_cylinder_collision(
        link, 'center_column_collision', 0.045, 0.34,
        (-0.02, 0.0, 0.22)
    )
    _add_box_collision(
        link, 'seat_collision', (0.52, 0.55, 0.08),
        (-0.03, 0.0, 0.42)
    )
    _add_box_collision(
        link, 'backrest_collision', (0.10, 0.48, 0.50),
        (-0.35, 0.0, 0.73)
    )


def _chair_pose_toward(
    chair_xy: tuple[float, float],
    table_xy: tuple[float, float],
) -> tuple[float, float, float, float, float, float]:
    """Point WoodenChair's local +X front toward its assigned table."""
    chair_x, chair_y = chair_xy
    table_x, table_y = table_xy
    yaw = atan2(table_y - chair_y, table_x - chair_x)
    return (chair_x, chair_y, 0.0, 0.0, 0.0, yaw)


def _add_planter(
    world: ElementTree.Element,
    name: str,
    xy: tuple[float, float],
) -> None:
    """Add a lightweight visual landmark with primitive collision."""
    model = ElementTree.SubElement(world, 'model', {'name': name})
    ElementTree.SubElement(model, 'static').text = 'true'
    ElementTree.SubElement(model, 'pose').text = (
        f'{xy[0]} {xy[1]} 0 0 0 0'
    )
    link = ElementTree.SubElement(model, 'link', {'name': 'body'})
    for element_name in ('collision', 'visual'):
        pot = ElementTree.SubElement(
            link, element_name, {'name': f'pot_{element_name}'}
        )
        ElementTree.SubElement(pot, 'pose').text = '0 0 0.25 0 0 0'
        geometry = ElementTree.SubElement(pot, 'geometry')
        cylinder = ElementTree.SubElement(geometry, 'cylinder')
        ElementTree.SubElement(cylinder, 'radius').text = '0.24'
        ElementTree.SubElement(cylinder, 'length').text = '0.5'
        if element_name == 'visual':
            material = ElementTree.SubElement(pot, 'material')
            ElementTree.SubElement(material, 'diffuse').text = (
                '0.42 0.20 0.10 1'
            )
    foliage = ElementTree.SubElement(
        link, 'visual', {'name': 'foliage_visual'}
    )
    ElementTree.SubElement(foliage, 'pose').text = '0 0 0.72 0 0 0'
    geometry = ElementTree.SubElement(foliage, 'geometry')
    sphere = ElementTree.SubElement(geometry, 'sphere')
    ElementTree.SubElement(sphere, 'radius').text = '0.38'
    material = ElementTree.SubElement(foliage, 'material')
    ElementTree.SubElement(material, 'diffuse').text = '0.12 0.42 0.16 1'


def materialize_study_cafe_world(
    robot_template_path: Path,
    target_path: Path | None = None,
    simulator: str = 'harmonic',
) -> Path:
    """Build a spacious, lightweight study-cafe evaluation world."""
    if simulator != 'harmonic':
        raise ValueError('the study cafe profile supports harmonic only')
    generated_robot_world = materialize_mecanum_wheel_world(
        robot_template_path
    )
    root = ElementTree.parse(generated_robot_world).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError('robot template must contain a world')
    world.set('name', 'cleany_study_cafe')

    scene = world.find('scene')
    if scene is None:
        scene = ElementTree.SubElement(world, 'scene')
    ambient = scene.find('ambient')
    if ambient is None:
        ambient = ElementTree.SubElement(scene, 'ambient')
    ambient.text = '0.65 0.65 0.65 1'
    background = scene.find('background')
    if background is None:
        background = ElementTree.SubElement(scene, 'background')
    background.text = '0.82 0.83 0.84 1'

    robot = world.find("model[@name='cleany_mecanum']")
    if robot is None:
        raise ValueError('robot template is missing cleany_mecanum')
    pose = robot.find('pose')
    if pose is None:
        raise ValueError('cleany_mecanum is missing its world pose')
    pose.text = ' '.join(map(str, _STUDY_CAFE_SPAWN_POSE))

    # Eight desk columns form 3-2-3 blocks. The measured clear horizontal
    # gaps are 1.33 m. The outer desk sides
    # touch the east and west wall faces.
    desk_x_positions = (
        -5.53, -4.33, -3.13,
        -0.60, 0.60,
        3.13, 4.33, 5.53,
    )
    # Each pair of rows is back-to-back; chairs face their own desk from the
    # outside. The seat front overlaps the tabletop edge by 23 cm.
    row_pair_centers = (3.17, 0.0, -3.17)
    room_half_width = 6.13
    room_half_depth = 5.47
    wall_thickness = 0.16

    ground = world.find("model[@name='ground_plane']")
    if ground is not None:
        ElementTree.SubElement(ground, 'pose').text = '0 0 0 0 0 0'
        for ground_size in ground.findall('link/*/geometry/plane/size'):
            ground_size.text = '12.42 11.10'

    wall_color = '0.97 0.97 0.96 1'
    _add_box_model(
        world, 'wall_north',
        (0, room_half_depth + wall_thickness / 2.0, 1.25, 0, 0, 0),
        (12.42, wall_thickness, 2.5), wall_color, roughness=0.92
    )
    _add_box_model(
        world, 'wall_south',
        (0, -room_half_depth - wall_thickness / 2.0, 1.25, 0, 0, 0),
        (12.42, wall_thickness, 2.5), wall_color, roughness=0.92
    )
    _add_box_model(
        world, 'wall_east',
        (room_half_width + wall_thickness / 2.0, 0, 1.25, 0, 0, 0),
        (wall_thickness, 10.94, 2.5), wall_color, roughness=0.92
    )
    _add_box_model(
        world, 'wall_west',
        (-room_half_width - wall_thickness / 2.0, 0, 1.25, 0, 0, 0),
        (wall_thickness, 10.94, 2.5), wall_color, roughness=0.92
    )

    partition_index = 1
    for pair_center in row_pair_centers:
        for desk_x in desk_x_positions:
            # The divider starts 30 cm above the floor and reaches 30 cm
            # above the 72 cm tabletop: z=0.30..1.02 m.
            _add_rounded_partition(
                world,
                f'desk_partition_{partition_index:02d}',
                (desk_x, pair_center, 0.66, 0.0, 0.0, 0.0),
            )
            partition_index += 1

    desk_index = 1
    for pair_center in row_pair_centers:
        for row_offset, chair_offset in (
            ((0.385, 0.385), (-0.385, -0.385))
        ):
            desk_y = pair_center + row_offset
            front_sign = 1.0 if chair_offset > 0 else -1.0
            for desk_x in desk_x_positions:
                desk_name = f'demo_desk_{desk_index:02d}'
                monitor_name = f'desk_monitor_{desk_index:02d}'
                chair_name = f'office_chair_{desk_index:02d}'
                _add_demo_desk(
                    world, desk_name,
                    (desk_x, desk_y, 0.0, 0.0, 0.0, 0.0),
                    front_sign=front_sign,
                )
                _add_desk_monitor(
                    world,
                    monitor_name,
                    (
                        desk_x,
                        desk_y - front_sign * 0.285,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ),
                    front_sign=front_sign,
                )
                chair_xy = (desk_x, desk_y + chair_offset)
                _add_office_chair(
                    world, chair_name,
                    _chair_pose_toward(chair_xy, (desk_x, desk_y)),
                )
                desk_index += 1

    target = target_path or (
        Path(gettempdir()) / 'cleany_study_cafe.sdf'
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(root).write(
        target, encoding='unicode', xml_declaration=True
    )
    return target
