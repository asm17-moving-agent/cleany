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
_STUDY_CAFE_SPAWN_POSE = (0.0, -2.7, 0.38, 0.0, 0.0, 1.5708)
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
            ElementTree.SubElement(material, 'diffuse').text = color


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

    robot = world.find("model[@name='cleany_mecanum']")
    if robot is None:
        raise ValueError('robot template is missing cleany_mecanum')
    pose = robot.find('pose')
    if pose is None:
        raise ValueError('cleany_mecanum is missing its world pose')
    pose.text = ' '.join(map(str, _STUDY_CAFE_SPAWN_POSE))

    ground = world.find("model[@name='ground_plane']")
    if ground is not None:
        ElementTree.SubElement(ground, 'pose').text = '0 1.75 0 0 0 0'
        for ground_size in ground.findall('link/*/geometry/plane/size'):
            ground_size.text = '18 10.5'

    wall_color = '0.88 0.88 0.84 1'
    _add_box_model(
        world, 'wall_north', (0, 7, 1.25, 0, 0, 0),
        (18, 0.16, 2.5), wall_color
    )
    _add_box_model(
        world, 'wall_south', (0, -3.5, 1.25, 0, 0, 0),
        (18, 0.16, 2.5), wall_color
    )
    _add_box_model(
        world, 'wall_east', (9, 1.75, 1.25, 0, 0, 0),
        (0.16, 10.5, 2.5), wall_color
    )
    _add_box_model(
        world, 'wall_west', (-9, 1.75, 1.25, 0, 0, 0),
        (0.16, 10.5, 2.5), wall_color
    )

    standard_table_poses = [
        (-6.0, -1.0, 0, 0, 0, 0),
        (-3.8, -1.0, 0, 0, 0, 0),
        (-6.0, 4.7, 0, 0, 0, 0),
        (-3.8, 4.7, 0, 0, 0, 0),
        (-1.5, 5.8, 0, 0, 0, 0),
        (1.5, 5.8, 0, 0, 0, 0),
    ]
    for index, table_pose in enumerate(standard_table_poses, start=1):
        _add_standard_table(
            world, f'standard_table_{index:02d}', table_pose
        )

    shared_table_poses = [
        (-1.7, 1.4, 0, 0, 0, 0),
        (0.0, 1.4, 0, 0, 0, 0),
        (1.7, 1.4, 0, 0, 0, 0),
        (5.6, -0.9, 0, 0, 0, 1.5708),
        (5.6, 3.3, 0, 0, 0, 1.5708),
    ]
    adj_table_z_scale = 0.72 / 0.802432
    for index, table_pose in enumerate(shared_table_poses, start=1):
        _add_fuel_furniture(
            world, f'adj_table_{index:02d}', 'adj_table', table_pose,
            (1.6, 0.82, 0.72), 0.36,
            visual_scale=(1.0, 1.0, adj_table_z_scale),
        )

    chair_assignments: list[
        tuple[tuple[float, float], tuple[float, float]]
    ] = []
    for table_x, table_y in (
        (-6.0, -1.0), (-3.8, -1.0), (-6.0, 4.7), (-3.8, 4.7)
    ):
        chair_assignments.extend(
            [
                ((table_x, table_y - 0.78), (table_x, table_y)),
                ((table_x, table_y + 0.78), (table_x, table_y)),
            ]
        )
    for table_x in (-1.7, 0.0, 1.7):
        chair_assignments.extend(
            [
                ((table_x, 0.58), (table_x, 1.4)),
                ((table_x, 2.22), (table_x, 1.4)),
            ]
        )
    for table_y in (-0.9, 3.3):
        chair_assignments.extend(
            [
                ((4.82, table_y - 0.42), (5.6, table_y)),
                ((4.82, table_y + 0.42), (5.6, table_y)),
                ((6.38, table_y - 0.42), (5.6, table_y)),
                ((6.38, table_y + 0.42), (5.6, table_y)),
            ]
        )
    for table_x in (-1.5, 1.5):
        chair_assignments.extend(
            [
                ((table_x - 0.38, 4.98), (table_x, 5.8)),
                ((table_x + 0.38, 4.98), (table_x, 5.8)),
            ]
        )
    chair_poses = [
        _chair_pose_toward(chair_xy, table_xy)
        for chair_xy, table_xy in chair_assignments
    ]
    for index, chair_pose in enumerate(chair_poses, start=1):
        _add_fuel_furniture(
            world, f'wooden_chair_{index:02d}', 'wooden_chair',
            chair_pose, (0.42, 0.42, 0.80), 0.40
        )

    shelf_poses = [
        (-7.8, 6.55, 0, 0, 0, 0),
        (-6.7, 6.55, 0, 0, 0, 0),
        (6.7, 6.55, 0, 0, 0, 0),
        (7.8, 6.55, 0, 0, 0, 0),
        (-7.8, 1.7, 0, 0, 0, 1.5708),
        (7.8, 1.7, 0, 0, 0, 1.5708),
    ]
    for index, shelf_pose in enumerate(shelf_poses, start=1):
        _add_fuel_furniture(
            world, f'square_shelf_{index:02d}', 'square_shelf',
            shelf_pose, (0.78, 0.40, 0.78), 0.39
        )

    for index, planter_xy in enumerate(
        [
            (-8.1, -2.8), (8.1, -2.8), (-8.1, 5.5),
            (8.1, 5.5), (-2.7, 3.6), (3.0, 4.6),
        ],
        start=1,
    ):
        _add_planter(world, f'planter_{index:02d}', planter_xy)

    target = target_path or (
        Path(gettempdir()) / 'cleany_study_cafe.sdf'
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(root).write(
        target, encoding='unicode', xml_declaration=True
    )
    return target
