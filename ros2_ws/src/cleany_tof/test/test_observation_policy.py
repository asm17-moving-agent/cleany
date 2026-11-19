"""15 Hz detector scenarios; synthetic geometry plus unmodified bench frames."""
from dataclasses import replace
import math
import json
from pathlib import Path
import numpy as np
import pytest
from cleany_tof.core import Settings, observe, desk_plane, project
from cleany_tof.observation import Observation, ZoneState as Z, classify_zones
from cleany_tof.proximity import ProximityDetector, limit_velocity


def empty(settings=None):
    # Real no-target slots can retain 338 mm; these are not measured obstacles.
    return observe([.338]*64, [.030]*64, [255]*64, [0]*64, [False]*64, settings or Settings())


def returns(indices=(27, 28), x=.8):
    points = np.zeros((64, 3))
    states = np.full(64, Z.NO_RETURN)
    for i in indices:
        points[i], states[i] = (x, 0., 0.), Z.RANGE
    return Observation(points, states)


def confirmed(detector, observation):
    for now in (0., 1/15, 2/15):
        result = detector.update(observation, now)
    return result


def test_no_return_is_normal_non_detection_without_depth_proof():
    s = Settings(calibrated=True)
    o = empty(s)
    assert o.available and not o.ranges.any() and (o.states == Z.NO_RETURN).all()
    assert not o.points.any()
    d = ProximityDetector(s).update(o, 0.)
    assert limit_velocity((.2, 0., 0.), (0., 0., 0.), d, .1, s) == ((.2, 0., 0.), 'CLEAR')
    with pytest.raises(ValueError, match='insufficient'):
        desk_plane(o, s)


@pytest.mark.parametrize('status,count,valid,state', [(255, 0, False, Z.NO_RETURN),
    (255, 1, False, Z.UNRELIABLE), (4, 0, False, Z.UNRELIABLE), (3, 1, False, Z.UNRELIABLE),
    (6, 1, False, Z.UNRELIABLE), (10, 1, False, Z.UNRELIABLE), (13, 1, False, Z.UNRELIABLE),
    (14, 1, False, Z.UNRELIABLE), (0, 0, False, Z.NOT_UPDATED), (5, 1, True, Z.RANGE)])
def test_status_count_and_firmware_valid_remain_distinct(status, count, valid, state):
    states = classify_zones([1.]*64, [.001]*64, [status]*64, [count]*64, [valid]*64)
    assert (states == state).all()


def test_poor_quality_is_ignored_individually_and_unusable_frames_stop():
    s = Settings(calibrated=True)
    for d, sigma in ((1., .1), (math.nan, .001), (4.1, .001), (-1., .001)):
        assert (classify_zones([d]*64, [sigma]*64, [5]*64, [1]*64, [True]*64) == Z.UNRELIABLE).all()
    o = empty(s)
    states = o.states.copy()
    states[27] = Z.UNRELIABLE
    detector = ProximityDetector(s)
    result = detector.update(Observation(o.points, states), 0.)
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), result, .1, s)[0][0] == .1
    states[:] = Z.UNRELIABLE
    result = detector.update(Observation(o.points, states), 1/15)
    assert not result.available
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), result, .1, s)[1] == 'TOF_UNAVAILABLE'


def test_adjacent_object_confirms_after_three_15hz_samples_and_limits_motion():
    s = Settings(calibrated=True)
    detector = ProximityDetector(s)
    for now in (0., 1/15):
        d = detector.update(returns(), now)
        assert len(d.points) == 0 and d.pending_zones == 2
    d = detector.update(returns(), 2/15)
    assert len(d.points) == 2 and d.pending_zones == 0
    output, reason = limit_velocity((.25, 0., 0.), (0., 0., 0.), d, .05, s)
    assert 0 < output[0] < .25 and reason == 'LIMITED'
    assert limit_velocity((.1, 0., 0.), (.5, 0., 0.), d, .05, s)[1] == 'BRAKING_ENVELOPE'
    assert limit_velocity((-.1, 0., 0.), (0., 0., 0.), d, .05, s)[0][0] == -.1
    assert 0 < limit_velocity((0., 0., .3), (0., 0., 0.), d, .05, s)[0][2] < .3


@pytest.mark.parametrize('indices', [(27,), (0, 63), (7, 8), (27, 36)])
def test_singletons_disconnected_diagonal_and_row_wrap_noise_do_not_confirm(indices):
    d = confirmed(ProximityDetector(Settings()), returns(indices))
    assert not d.emergency and not len(d.points)


def test_adjacent_zones_must_also_be_close_in_physical_space():
    o = returns()
    points = o.points.copy()
    points[28, 1] = .5
    assert not len(confirmed(ProximityDetector(Settings()), Observation(points, o.states)).points)


def test_flash_and_changing_spatial_support_restart_confirmation():
    detector = ProximityDetector(Settings())
    detector.update(returns(), 0.)
    detector.update(empty(), 1/15)
    assert not len(detector.update(returns(), 2/15).points)
    assert not len(detector.update(returns((40, 41)), 3/15).points)
    assert not len(detector.update(returns(), 4/15).points)


def test_repeated_frame_cannot_advance_filter_and_long_gap_resets_it():
    detector = ProximityDetector(Settings())
    first = detector.update(returns(), 0.)
    for _ in range(10):
        assert detector.update(returns(), 0.) is first
    assert not len(detector.update(returns(), .3).points)
    assert not len(detector.update(returns(), .3+1/15).points)
    assert len(detector.update(returns(), .3+2/15).points) == 2
    assert not len(detector.update(returns(), .1).points)  # Clock reset.


def test_emergency_single_return_stops_without_confirmation_and_has_hysteresis():
    s = Settings(calibrated=True)
    detector = ProximityDetector(s)
    d = detector.update(returns((27,), x=.66), 0.)
    assert d.emergency
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), d, .01, s)[1] == 'PROXIMITY_STOP'
    # Just outside the entry threshold still holds the emergency latch.
    for now in (1/15, 2/15, 3/15, 4/15):
        assert detector.update(returns((27,), x=.70), now).emergency
    for now in (5/15, 6/15):
        assert detector.update(empty(), now).emergency
    assert not detector.update(empty(), 8/15).emergency


def test_roi_excludes_returns_and_proximity_distance_hysteresis_releases():
    s = Settings(calibrated=True, guard_rows=(3,), guard_columns=(3, 4))
    detector = ProximityDetector(s)
    assert not detector.update(returns((0,), .55), 0.).emergency
    detector.reset()
    confirmed(detector, returns())
    edge = s.half_length+s.margin+3*s.max_sigma_m+s.obstacle_clearance_m
    d = detector.update(returns(x=edge+.02), 3/15)
    assert len(d.points) == 2  # Within the active track's exit threshold.
    d = detector.update(returns(x=edge+.06), 4/15)
    assert d.held and limit_velocity((.1, 0., 0.), (0., 0., 0.), d, .01, s)[1] == 'OBSTACLE_HOLD'
    detector.update(empty(), 5/15)
    detector.update(empty(), 6/15)
    assert not detector.update(empty(), 7/15).held


def test_missing_confirmed_returns_hold_zero_then_release_without_old_base_points():
    s = Settings(calibrated=True)
    detector = ProximityDetector(s)
    confirmed(detector, returns())
    d = detector.update(empty(), 3/15)
    assert d.held and len(d.points) == 0
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), d, .01, s)[0] == (0., 0., 0.)
    detector.update(empty(), 4/15)
    detector.update(empty(), 5/15)
    assert not detector.update(empty(), 6/15).held
    detector.reset()
    assert not len(detector.update(returns(), 7/15).points)


def test_partial_no_return_keeps_using_remaining_confirmed_distance_points():
    s = Settings(calibrated=True)
    detector = ProximityDetector(s)
    confirmed(detector, returns())
    d = detector.update(returns((27,)), 3/15)
    assert not d.held and len(d.points) == 1
    output, reason = limit_velocity((.25, 0., 0.), (0., 0., 0.), d, .01, s)
    assert 0 < output[0] < .25 and reason == 'LIMITED'


def test_desk_plane_uses_distributed_valid_subset_ignoring_bad_or_missing_zones():
    s = Settings(calibrated=True)
    rays, _ = project([1.]*64, [.001]*64, [5]*64, [1]*64, [True]*64, s)
    o = empty(s)
    states = o.states.copy()
    points = o.points.copy()
    for row in (2, 3, 4):
        for col in (1, 3, 4, 6):
            i = row*8+col
            points[i], states[i] = rays[i]/rays[i, 0], Z.RANGE
    states[3*8+3] = Z.UNRELIABLE
    assert desk_plane(Observation(points, states), s).distance == pytest.approx(1.)
    # Too few columns, one row only or a narrow patch cannot justify a desk.
    for mask in (np.arange(64) % 8 == 1, np.arange(64)//8 != 2):
        sparse = states.copy()
        sparse[mask] = Z.NO_RETURN
        with pytest.raises(ValueError, match='insufficient'):
            desk_plane(Observation(points, sparse), s)
    with pytest.raises(ValueError, match='span'):
        desk_plane(Observation(points, states), replace(s, min_plane_span_m=2.))


@pytest.mark.parametrize('changes', [dict(min_cluster_zones=0), dict(guard_rows=()),
    dict(guard_columns=(0, 0)), dict(confirm_s=-1.), dict(release_s=0.),
    dict(emergency_clearance_m=.8), dict(min_plane_columns=9), dict(min_plane_rows=5)])
def test_invalid_detection_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        Settings(**changes)


def test_real_standalone_log_fixture_preserves_non_detection_and_low_quality():
    from cleany_tof.transport import Stream
    fixture = json.loads((Path(__file__).parent/'data/standalone_frames.json').read_text())
    for r in fixture['frames']:
        Stream().frame(r, r['time_us']/1e6)
        o = observe([v/1000 for v in r['distance_mm']], [v/1000 for v in r['sigma_mm']],
                    r['target_status'], r['target_count'], r['valid'], Settings())
        missing = (np.asarray(r['target_count']) == 0) & (np.asarray(r['target_status']) == 255)
        assert (o.states[missing] == Z.NO_RETURN).all() and not o.points[missing].any()
        assert (o.states[np.isin(r['target_status'], (3, 4, 13))] == Z.UNRELIABLE).all()
    first = fixture['frames'][0]
    assert not any(first['valid']) and 6 in first['target_status']
