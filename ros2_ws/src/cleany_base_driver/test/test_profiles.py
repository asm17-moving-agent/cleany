from pathlib import Path
import importlib.util
import pytest
import yaml

from cleany_base_driver.core import Geometry, Limits, wheel_speeds

LAUNCH = Path(__file__).parents[1] / 'launch' / 'base_driver.launch.py'
spec = importlib.util.spec_from_file_location('base_driver_launch', LAUNCH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
CONFIG = Path(__file__).parents[4] / 'configs' / 'robot'


def test_profile_mock_is_explicit_and_synthetic_never_real():
    synthetic = CONFIG / 'base_synthetic.yaml'
    assert module.load_profile(synthetic, True)[1][0] > 0
    with pytest.raises(RuntimeError, match='requires mock'):
        module.load_profile(synthetic, False)
    with pytest.raises(RuntimeError, match='requires synthetic'):
        module.load_profile(CONFIG / 'base_hardware.yaml', True)


def test_confirmed_real_geometry_and_initial_lifted_wheel_limits():
    profile, required = module.load_profile(CONFIG / 'base_hardware.yaml', False)
    assert profile['geometry'] == {
        'wheel_radius_m': 0.0635,
        'wheelbase_m': 0.350,
        'wheel_separation_m': 0.610,
    }
    assert profile['limits'] == {
        'linear_x_mps': 0.5,
        'linear_y_mps': 0.5,
        'angular_z_rad_s': 1.0,
        'wheel_rad_s': 10.0,
        'command_timeout_s': 0.3,
    }
    assert required == (0.0635, 0.350, 0.610, 0.5, 0.5, 1.0, 10.0, 0.3)


@pytest.mark.parametrize('field', [
    'linear_x_mps', 'linear_y_mps', 'angular_z_rad_s',
    'wheel_rad_s', 'command_timeout_s',
])
@pytest.mark.parametrize('missing', [False, True])
def test_hardware_profile_still_requires_every_limit(tmp_path, field, missing):
    profile = yaml.safe_load((CONFIG / 'base_hardware.yaml').read_text())
    if missing:
        del profile['limits'][field]
    else:
        profile['limits'][field] = None
    config = tmp_path / 'incomplete_hardware.yaml'
    config.write_text(yaml.safe_dump(profile))
    with pytest.raises(RuntimeError, match='explicitly reviewed'):
        module.load_profile(config, False)


def test_hardware_profile_rejects_limit_above_mcu_ceiling(tmp_path):
    profile = yaml.safe_load((CONFIG / 'base_hardware.yaml').read_text())
    profile['limits']['wheel_rad_s'] = 10.1
    config = tmp_path / 'over_limit_hardware.yaml'
    config.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='exceeds MCU contract'):
        module.load_profile(config, False)


@pytest.fixture
def hardware_motion():
    _, required = module.load_profile(CONFIG / 'base_hardware.yaml', False)
    return Geometry(*required[:3]), Limits(*required[3:])


def test_wheel_ceiling_is_independent_of_axis_limits(hardware_motion):
    geometry, limits = hardware_motion
    rotation_coefficient = (geometry.wheelbase + geometry.separation) / 2
    assert limits.linear_x / geometry.wheel_radius < limits.wheel_rad_s
    assert limits.linear_y / geometry.wheel_radius < limits.wheel_rad_s
    assert limits.angular_z * rotation_coefficient / geometry.wheel_radius < limits.wheel_rad_s


@pytest.mark.parametrize('x,y,signs', [
    (0.5, 0., (1., 1., 1., 1.)),
    (-0.5, 0., (-1., -1., -1., -1.)),
    (0., 0.5, (-1., 1., 1., -1.)),
    (0., -0.5, (1., -1., -1., 1.)),
])
@pytest.mark.parametrize('multiplier', [1., 10.])
def test_initial_translation_is_bounded_by_axis_limit(hardware_motion, x, y, signs, multiplier):
    geometry, limits = hardware_motion
    result = wheel_speeds(x * multiplier, y * multiplier, 0., geometry, limits)
    expected = tuple(sign * 0.5 / geometry.wheel_radius for sign in signs)
    assert result == pytest.approx(expected)
    circumferential_speed = max(map(abs, result)) * geometry.wheel_radius
    assert circumferential_speed <= 0.5
    assert circumferential_speed == pytest.approx(0.5)


@pytest.mark.parametrize('fraction', [-2., -1., -0.5, 0.5, 1., 2.])
def test_initial_yaw_ceiling(hardware_motion, fraction):
    geometry, limits = hardware_motion
    yaw = fraction * limits.angular_z
    clamped = max(-limits.angular_z, min(limits.angular_z, yaw))
    rotation = (geometry.wheelbase + geometry.separation) * clamped / 2
    wheel = rotation / geometry.wheel_radius
    result = wheel_speeds(0., 0., yaw, geometry, limits)
    assert result == pytest.approx((-wheel, wheel, -wheel, wheel))
    assert max(map(abs, result)) <= limits.wheel_rad_s
    assert max(map(abs, result)) * geometry.wheel_radius <= 0.5


def test_initial_combined_motion_clamps_axes_and_preserves_wheel_ratios(hardware_motion):
    geometry, limits = hardware_motion
    rotation = (geometry.wheelbase + geometry.separation) * limits.angular_z / 2
    x, y = limits.linear_x, limits.linear_y
    raw = (x - y - rotation, x + y + rotation, x + y - rotation, x - y + rotation)
    scale = limits.wheel_rad_s / max(map(abs, raw))
    result = wheel_speeds(50., 50., 50., geometry, limits)
    assert result == pytest.approx(tuple(value * scale for value in raw))
    assert max(map(abs, result)) == pytest.approx(limits.wheel_rad_s)
    assert max(map(abs, result)) * geometry.wheel_radius == pytest.approx(0.635)
