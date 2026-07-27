from __future__ import annotations

from math import asin, atan2, cos, pi, sin, sqrt
from pathlib import Path
from tempfile import gettempdir


_MARKER = '<!-- CLEANY_ARTICULATED_ROLLERS -->'
_ROLLER_RADIUS = 0.008
_ROLLER_LENGTH = 0.03
_ROLLER_CENTER_RADIUS = 0.0555


def articulated_roller_sdf() -> str:
    """Generate the 48 roller links and joints from the MuJoCo wheel layout."""
    wheel_sets = (
        ('rear_left', 'rear_left_wheel', -1.0),
        ('rear_right', 'rear_right_wheel', 1.0),
        ('front_left', 'front_left_wheel', 1.0),
        ('front_right', 'front_right_wheel', -1.0),
    )
    fragments: list[str] = []
    for prefix, parent, handedness in wheel_sets:
        for index in range(12):
            angle = index * pi / 6.0
            x = _ROLLER_CENTER_RADIUS * cos(angle)
            y = _ROLLER_CENTER_RADIUS * sin(angle)
            axis_x = -sin(angle) / sqrt(2.0)
            axis_y = cos(angle) / sqrt(2.0)
            axis_z = handedness / sqrt(2.0)
            roll = -asin(axis_y)
            pitch = atan2(axis_x, axis_z)
            name = f'{prefix}_roller_{index:02d}'
            fragments.append(
                f'''<link name="{name}">
  <pose relative_to="{name}_joint">0 0 0 0 0 0</pose>
  <inertial><mass>0.01</mass><inertia><ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz></inertia></inertial>
  <collision name="collision"><geometry><capsule><radius>{_ROLLER_RADIUS}</radius><length>{_ROLLER_LENGTH}</length></capsule></geometry><surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>
  <visual name="visual"><geometry><capsule><radius>{_ROLLER_RADIUS}</radius><length>{_ROLLER_LENGTH}</length></capsule></geometry><material><diffuse>0.06 0.06 0.07 1</diffuse></material></visual>
</link>
<joint name="{name}_joint" type="revolute">
  <pose relative_to="{parent}">{x:.6f} {y:.6f} 0 {roll:.6f} {pitch:.6f} 0</pose>
  <parent>{parent}</parent><child>{name}</child>
  <axis><xyz>0 0 1</xyz><dynamics><damping>0.00002</damping><friction>0.0001</friction></dynamics></axis>
</joint>'''
            )
    return '\n'.join(fragments)


def materialize_articulated_roller_world(template_path: Path) -> Path:
    template = template_path.read_text(encoding='utf-8')
    if template.count(_MARKER) != 1:
        raise ValueError('world template must contain one roller marker')

    world = template.replace(_MARKER, articulated_roller_sdf())
    target = Path(gettempdir()) / 'cleany_mecanum_articulated_rollers.sdf'
    target.write_text(world, encoding='utf-8')
    return target
