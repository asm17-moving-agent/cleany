from __future__ import annotations

from math import asin, atan2, cos, pi, sin, sqrt
from pathlib import Path
from tempfile import gettempdir


_WHEEL_HANDEDNESS = {
    'rear_left': -1.0,
    'rear_right': 1.0,
    'front_left': 1.0,
    'front_right': -1.0,
}
_ROLLER_RADIUS = 0.008
_ROLLER_LENGTH = 0.03
_ROLLER_CENTER_RADIUS = 0.0555


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

    target = Path(gettempdir()) / 'cleany_mecanum_fixed_roller_visuals.sdf'
    target.write_text(world, encoding='utf-8')
    return target
