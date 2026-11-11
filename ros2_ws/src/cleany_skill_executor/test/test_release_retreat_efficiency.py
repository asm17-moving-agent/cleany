import pytest
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit, bounded_path


def test_small_retreat_reduces_minimum_wait_without_exceeding_dynamic_limits():
    names = ('q',)
    limits = {'q': PhysicalJointLimit(-2., 2., 2., 2., 2048., 1., 0., 0., 4095.)}
    durations = []
    for floor in (1., .25):
        samples, duration = bounded_path(names, [(0.,), (.018,)], limits,
            velocity=.8, acceleration=1.2, minimum_duration_sec=floor)
        assert samples[0].positions == (0.,) and samples[-1].positions == pytest.approx((.018,))
        assert max(abs(s.derivatives[0]/duration) for s in samples) <= .8
        # A rest-to-rest two-point cubic has max acceleration 6*delta/T^2.
        assert 6*.018/duration**2 <= 1.2
        durations.append(duration)
    assert durations[0] == 1. and .25 <= durations[1] < .4


from types import SimpleNamespace as NS

import math

from geometry_msgs.msg import Pose

from cleany_skill_executor.collection_pipeline import CollectionPipeline

@pytest.mark.parametrize('lateral,below,angle,skip', [
    (0., 0., 0., True), (.004, 0., 1., True),
    (.006, 0., 0., False), (.001, .001, 0., False), (.001, 0., 3., False),
])
def test_release_retreat_skips_small_residual_only_at_required_height(lateral, below, angle, skip):
    actual, target, destination = Pose(), Pose(), Pose()
    for pose in (actual, target, destination):
        pose.position.x, pose.position.z, pose.orientation.w = .45, .53, 1.
    actual.position.x -= lateral
    actual.position.z -= below
    actual.orientation.z = math.sin(math.radians(angle)/2)
    actual.orientation.w = math.cos(math.radians(angle)/2)
    values = {'hardware_release_retreat_position_tolerance_m': .005,
              'hardware_release_retreat_orientation_tolerance_deg': 2.}
    paths, events = [], []
    node = NS(config={'return_to_pick_height_offset_m': .02},
              real=NS(value=values.__getitem__), pose=lambda _: actual,
              cartesian_pose=CollectionPipeline.cartesian_pose,
              pickup_destination_pose=lambda _: destination,
              linear=lambda *args, **kwargs: paths.append((args, kwargs)),
              event=lambda name, **fields: events.append((name, fields)))
    CollectionPipeline.pickup_release_retreat(node, 'right', target)
    assert bool(paths) is not skip
    assert any(name == 'retreat_after_release_already_reached' for name, _ in events) is skip
    if paths:
        assert paths[0][0][1].position.z >= destination.position.z
