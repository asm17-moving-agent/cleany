from pathlib import Path
import importlib.util
import pytest

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
    with pytest.raises(RuntimeError, match='explicitly reviewed'):
        module.load_profile(CONFIG / 'base_hardware.yaml', False)


def test_confirmed_real_geometry_does_not_infer_limits():
    import yaml
    profile = yaml.safe_load((CONFIG / 'base_hardware.yaml').read_text())
    assert profile['geometry'] == {
        'wheel_radius_m': 0.0635,
        'wheelbase_m': 0.350,
        'wheel_separation_m': 0.610,
    }
    assert all(value is None for value in profile['limits'].values())
