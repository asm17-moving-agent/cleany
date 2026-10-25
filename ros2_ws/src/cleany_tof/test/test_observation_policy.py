from dataclasses import replace
import math
import numpy as np
import pytest
from cleany_tof.core import Settings, observe, desk_plane, project
from cleany_tof.observation import Observation, ZoneState as Z, classify_zones
from cleany_tof.proximity import limit_velocity, model_id, relevant_zones, TofRegion, load_settings


def empty(settings=None):
    # These real log-style distance slots are junk, not 0.338 m obstacles.
    return observe([.338]*64, [.030]*64, [255]*64, [0]*64, [False]*64, settings or Settings())


def certificate(settings, requested=(.1, 0., 0.), measured=(0., 0., 0.), **changes):
    result = dict(contract='tof_sweep_v1', model_id=model_id(settings), covered=True,
                  reason='COVERED', requested=list(requested), measured=list(measured))
    result.update(changes)
    return result


def test_no_return_is_live_but_has_no_measured_point_or_free_distance():
    o = empty()
    assert o.available and not o.ranges.any() and (o.states == Z.NO_RETURN).all()
    assert not o.points.any()
    s = Settings(calibrated=True)
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), o, .1, s)[1] == 'NO_RETURN_UNCOVERED'
    with pytest.raises(ValueError, match='insufficient'):
        desk_plane(o, s)


@pytest.mark.parametrize('status,count,valid,state', [(255, 0, False, Z.NO_RETURN),
    (255, 1, False, Z.UNRELIABLE), (4, 0, False, Z.UNRELIABLE), (3, 1, False, Z.UNRELIABLE),
    (6, 1, False, Z.UNRELIABLE), (10, 1, False, Z.UNRELIABLE), (13, 1, False, Z.UNRELIABLE),
    (14, 1, False, Z.UNRELIABLE), (0, 0, False, Z.NOT_UPDATED), (5, 1, True, Z.RANGE)])
def test_status_count_and_firmware_valid_have_distinct_meanings(status, count, valid, state):
    states = classify_zones([1.]*64, [.001]*64, [status]*64, [count]*64, [valid]*64)
    assert (states == state).all()


def test_poor_quality_and_out_of_range_never_become_no_return():
    for d, sigma in ((1., .1), (math.nan, .001), (4.1, .001), (-1., .001)):
        assert (classify_zones([d]*64, [sigma]*64, [5]*64, [1]*64, [True]*64) == Z.UNRELIABLE).all()
    assert not observe([1.]*64, [.1]*64, [5]*64, [1]*64, [True]*64, Settings()).available


def test_empty_space_can_use_independent_sweep_proof_at_limited_speed():
    s = Settings(calibrated=True)
    o = empty(s)
    cmd = (.2, 0., 0.)
    proof = certificate(s, requested=cmd)
    output, reason = limit_velocity(cmd, (0., 0., 0.), o, .1, s, proof)
    assert output == pytest.approx((.08, 0., 0.)) and reason == 'NO_RETURN_DEPTH_COVERED'
    for invalid in (None, certificate(s, covered=False), certificate(s, model_id='wrong'),
                    certificate(s, requested=(0., .1, 0.)), certificate(s, requested=(.01, 0., 0.))):
        assert limit_velocity(cmd, (0., 0., 0.), o, .1, s, invalid)[0] == (0., 0., 0.)
    # Measured inertia is part of the independent certificate too.
    assert limit_velocity(cmd, (.3, 0., 0.), o, .1, s, proof)[0] == (0., 0., 0.)


def test_direction_and_measured_motion_choose_unknown_coverage():
    s = Settings(calibrated=True)
    o = empty(s)
    assert not relevant_zones((-.1, 0., 0.), (0., 0., 0.), s).any()
    assert limit_velocity((-.1, 0., 0.), (0., 0., 0.), o, .1, s)[0] == (-.1, 0., 0.)
    assert limit_velocity((-.1, 0., 0.), (.1, 0., 0.), o, .1, s)[0] == (0., 0., 0.)
    assert relevant_zones((0., 0., .01), (0., 0., 0.), s).all()
    rear = replace(s, sensor_yaw=math.pi)
    assert not relevant_zones((.1, 0., 0.), (0., 0., 0.), rear).any()
    assert relevant_zones((0., .1, 0.), (0., 0., 0.), s).any()
    assert not relevant_zones((0., .1, 0.), (0., 0., 0.), s).all()


def test_bad_zone_cannot_borrow_proof_and_single_near_return_still_stops():
    s = Settings(calibrated=True)
    o = empty(s)
    states = o.states.copy()
    states[27] = Z.UNRELIABLE
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), Observation(o.points, states), .1, s,
                          certificate(s))[1] == 'UNRELIABLE_COVERAGE'
    states[27] = Z.RANGE
    points = o.points.copy()
    points[27] = (.6, 0., 0.)
    assert limit_velocity((.1, 0., 0.), (0., 0., 0.), Observation(points, states), .1, s,
                          certificate(s))[1] == 'PROXIMITY_STOP'


def test_region_follows_mount_and_does_not_claim_full_body_coverage():
    centers = np.array([[1., 0., 0.], [-1., 0., 0.], [1., 2., 0.]])
    assert TofRegion(Settings()).intersects(centers, .1).tolist() == [True, False, False]
    assert TofRegion(Settings(sensor_yaw=math.pi)).intersects(centers[:2], .1).tolist() == [False, True]


def test_desk_plane_allows_missing_returns_only_with_real_column_support():
    s = Settings(calibrated=True)
    rays, _ = project([1.]*64, [.001]*64, [5]*64, [1]*64, [True]*64, s)
    d = 1./rays[:, 0]
    status, count, valid = [5]*64, [1]*64, [True]*64
    for r in range(8):
        for c in (0, 7):
            status[r*8+c], count[r*8+c], valid[r*8+c] = 255, 0, False
    o = observe(d, [.001]*64, status, count, valid, s)
    assert desk_plane(o, s).distance == pytest.approx(1.)
    status[3*8+3], valid[3*8+3] = 4, False
    with pytest.raises(ValueError, match='unreliable desk'):
        desk_plane(observe(d, [.001]*64, status, count, valid, s), s)


def test_legacy_global_coverage_config_must_be_migrated(tmp_path):
    p = tmp_path/'old.yaml'
    p.write_text('/**:\n  ros__parameters:\n    required_zones: [0]\n')
    with pytest.raises(ValueError, match='obsolete'):
        load_settings(str(p))


def test_real_standalone_log_fixture_preserves_missing_and_low_quality_states():
    import json
    from pathlib import Path
    from cleany_tof.transport import Stream
    fixture = json.loads((Path(__file__).parent/'data/standalone_frames.json').read_text())
    for r in fixture['frames']:
        Stream().frame(r, r['time_us']/1e6)
        o = observe([v/1000 for v in r['distance_mm']], [v/1000 for v in r['sigma_mm']],
                    r['target_status'], r['target_count'], r['valid'], Settings())
        missing = (np.asarray(r['target_count']) == 0) & (np.asarray(r['target_status']) == 255)
        assert (o.states[missing] == Z.NO_RETURN).all() and not o.points[missing].any()
        bad = np.isin(r['target_status'], (3, 4, 13))
        assert (o.states[bad] == Z.UNRELIABLE).all()
    first = fixture['frames'][0]
    assert not any(first['valid']) and 6 in first['target_status']


def test_side_closing_cannot_be_hidden_by_receding_longitudinal_motion():
    s = Settings(calibrated=True, half_width=.1)
    assert relevant_zones((-.3, .1, 0.), (0., 0., 0.), s)[0]
    # A sensor mounted beyond the side must consider that leading face even
    # when its optical axis points back towards the robot.
    outside = replace(s, sensor_y=.2, sensor_yaw=-math.pi/2)
    assert relevant_zones((0., .1, 0.), (0., 0., 0.), outside).all()
