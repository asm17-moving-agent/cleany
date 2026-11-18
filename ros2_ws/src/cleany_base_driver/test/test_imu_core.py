"""Fixed bias, sensor input and synchronized acquisition-time checks."""
import math

import pytest

from cleany_base_driver.imu_core import FactoryCalibration, ImuAdapter, ImuValidationError


BIAS = (.01, -.02, .03)
COVARIANCE = (1e-6, 1e-7, 0., 1e-7, 2e-6, 0., 0., 0., 3e-6)


def calibration():
    return FactoryCalibration(BIAS, COVARIANCE)


def process(adapter, stamp=1_000_000_000, ros=None, steady=None, **changes):
    values = dict(
        stamp_ns=stamp, frame_id='imu_link', angular_velocity=(.11, -.22, .33),
        linear_acceleration=(0., 0., 9.80665),
        receipt_ros_ns=stamp + 1_000_000 if ros is None else ros,
        receipt_steady_ns=stamp if steady is None else steady,
    )
    values.update(changes)
    return adapter.process(**values)


@pytest.mark.parametrize('acceleration', [(0., 0., 9.80665), (0., 0., 0.)])
def test_fixed_factory_bias_does_not_learn_from_motion_or_restart(acceleration):
    factory = calibration()
    adapter = ImuAdapter(factory)
    assert process(adapter, linear_acceleration=acceleration) == pytest.approx((.1, -.2, .3))
    for i in range(1, 101):
        assert process(adapter, stamp=1_000_000_000 + i * 10_000_000,
                       angular_velocity=BIAS) == (0., 0., 0.)
    assert factory.bias == BIAS
    assert factory.covariance == COVARIANCE
    assert process(ImuAdapter(factory)) == pytest.approx((.1, -.2, .3))


def test_rejected_duplicates_and_old_samples_do_not_advance_baseline():
    adapter = ImuAdapter(calibration())
    process(adapter)
    with pytest.raises(ImuValidationError, match='duplicate'):
        process(adapter)
    with pytest.raises(ImuValidationError, match='duplicate'):
        process(adapter, stamp=999_000_000, ros=1_002_000_000, steady=1_001_000_000)
    process(adapter, stamp=1_010_000_000)
    assert adapter.last_stamp == 1_010_000_000


@pytest.mark.parametrize('stamp,ros,pattern', [
    (1_000_000_000, 1_101_000_000, 'backlog'),
    (1_200_000_000, 1_099_000_000, 'ahead'),
    (0, 1, 'timestamp'),
    (-1, 1, 'timestamp'),
    (2**31 * 1_000_000_000, 2**31 * 1_000_000_000, 'timestamp'),
])
def test_invalid_acquisition_time(stamp, ros, pattern):
    adapter = ImuAdapter(calibration())
    with pytest.raises(ImuValidationError, match=pattern):
        process(adapter, stamp=stamp, ros=ros)
    assert adapter.last_stamp is None


def test_clock_jump_withholds_discontinuous_time_until_adapter_restart():
    adapter = ImuAdapter(calibration())
    process(adapter)
    with pytest.raises(ImuValidationError, match='clock jump'):
        process(adapter, stamp=2_000_000_000, ros=2_001_000_000, steady=1_001_000_000)
    assert adapter.last_stamp == 1_000_000_000
    with pytest.raises(ImuValidationError, match='restart IMU bringup'):
        process(adapter, stamp=2_010_000_000, ros=2_011_000_000, steady=1_011_000_000)
    process(ImuAdapter(calibration()), stamp=2_010_000_000)


@pytest.mark.parametrize('changes,pattern', [
    ({'frame_id': 'base_link'}, 'frame'),
    ({'angular_velocity': (math.nan, 0., 0.)}, 'finite'),
    ({'angular_velocity': (0., math.inf, 0.)}, 'finite'),
    ({'angular_velocity': (0., 0.)}, '3 finite'),
    ({'linear_acceleration': (0., 0., math.nan)}, 'finite'),
])
def test_invalid_sensor_data_does_not_advance_time(changes, pattern):
    adapter = ImuAdapter(calibration())
    process(adapter)
    with pytest.raises(ImuValidationError, match=pattern):
        process(adapter, stamp=1_010_000_000, **changes)
    assert adapter.last_stamp == 1_000_000_000
    process(adapter, stamp=1_010_000_000)


@pytest.mark.parametrize('bias,covariance', [
    ((0., 0.), COVARIANCE), ((math.nan, 0., 0.), COVARIANCE),
    ((True, 0., 0.), COVARIANCE), (BIAS, (0.,) * 9),
    (BIAS, (1e-6, 1e-3, 0., 0., 2e-6, 0., 0., 0., 3e-6)),
    (BIAS, (1e-6, 1e-3, 0., 1e-3, 2e-6, 0., 0., 0., 3e-6)),
    (BIAS, (1e-6, 0., 0., 0., 2e-6, 0., 0., 0., -3e-6)),
    (BIAS, (math.inf,) * 9),
])
def test_invalid_factory_coefficients(bias, covariance):
    with pytest.raises(ValueError):
        FactoryCalibration(bias, covariance)


@pytest.mark.parametrize('params', [
    {'frame_id': ''}, {'frame_id': '/imu_link'}, {'frame_id': 'imu link'},
    {'max_transport_delay_s': 0.}, {'max_transport_delay_s': math.inf},
    {'max_clock_jump_s': -1.}, {'max_clock_jump_s': math.nan},
])
def test_invalid_adapter_configuration(params):
    with pytest.raises(ValueError):
        ImuAdapter(calibration(), **params)
