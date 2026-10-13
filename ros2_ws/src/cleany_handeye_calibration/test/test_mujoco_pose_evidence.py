from pathlib import Path

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import JointPose
from cleany_handeye_calibration.mujoco_pose_evidence import (
    MujocoPoseEvidenceEvaluator,
)


SCENE_PATH = (
    Path(__file__).parents[2]
    / 'cleany_mujoco_sim'
    / 'scenes'
    / 'handeye.xml.in'
)


def _evaluator():
    from cleany_mujoco_sim.scene_loader import materialize_control_scene

    return MujocoPoseEvidenceEvaluator(
        materialize_control_scene(SCENE_PATH),
        minimum_camera_depth_m=0.18,
        image_border_fraction=0.08,
    )


def test_scene_evidence_accepts_visible_clear_pose():
    result = _evaluator().evaluate(
        JointPose(
            LEFT_ARM_JOINT_NAMES,
            (-0.794, 2.002, 1.933, 1.412, 0.179),
        )
    )

    assert result.target_visible
    assert result.camera_front
    assert result.minimum_camera_depth_m > 0.18
    assert result.minimum_collision_distance_m > 0.30


def test_scene_evidence_rejects_zero_pose_visibility():
    result = _evaluator().evaluate(
        JointPose(LEFT_ARM_JOINT_NAMES, (0.0,) * 5)
    )

    assert not result.target_visible


def test_separation_certificate_is_conservative_and_keeps_overlap_closed():
    import mujoco
    import pytest
    from cleany_handeye_calibration.mujoco_pose_evidence import (
        convex_separation_lower_bound,
    )

    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <asset><mesh name="cube" vertex="-.05 -.05 -.05 -.05 -.05 .05
        -.05 .05 -.05 -.05 .05 .05 .05 -.05 -.05 .05 -.05 .05
        .05 .05 -.05 .05 .05 .05"/></asset>
      <worldbody>
        <geom name="a" type="mesh" mesh="cube"/>
        <body pos=".5 .1 0" euler="0 0 .4">
          <geom name="b" type="box" size=".05 .05 .05"/>
        </body>
        <geom name="overlap" type="sphere" size=".04"/>
        <geom name="unsupported" type="capsule" size=".02 .1" pos="2 0 0"/>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
           for name in ('a', 'b', 'overlap', 'unsupported')}
    a, b = ids['a'], ids['b']
    bound = convex_separation_lower_bound(model, data, a, b)
    # Full mesh support must respect the rotated box, not its unrotated size.
    assert .3 < bound <= .5 - .05
    assert convex_separation_lower_bound(model, data, a, ids['overlap']) == 0.
    assert convex_separation_lower_bound(model, data, a, ids['unsupported']) == 0.
    model.geom_margin[a] = .02
    model.geom_margin[b] = .03
    assert convex_separation_lower_bound(model, data, a, b) == pytest.approx(
        bound - .05,
    )
    model.geom_margin[a] = -.02
    model.geom_margin[b] = 0.
    assert convex_separation_lower_bound(model, data, a, b) == pytest.approx(bound)


def test_clear_pose_remains_clear_when_engine_returns_spurious_zero():
    # Exact CAD/control-scene regression: two distant pairs return zero in
    # MuJoCo 3.4 despite support gaps of 0.467 m and 0.560 m respectively.
    from cleany_mujoco_sim.scene_loader import materialize_control_scene

    evaluator = MujocoPoseEvidenceEvaluator(
        materialize_control_scene(SCENE_PATH),
        minimum_camera_depth_m=.05,
        image_border_fraction=.05,
    )
    result = evaluator.evaluate(JointPose(LEFT_ARM_JOINT_NAMES, (
        -1.5750541401922051, 1.0852839827929772, .863463681560594,
        .8057062641169179, -1.7714713196743206,
    )))
    assert result.minimum_collision_distance_m > .10
    assert result.target_visible
