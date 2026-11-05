from types import SimpleNamespace

from cleany_interfaces.msg import DetectedObject2D, DetectedObject3D, GraspCandidate

from cleany_skill_executor.core.target_filter import TargetDecision
from cleany_skill_executor.validation_evidence import write_validation_evidence


def test_writes_scene_3x3_json_and_summary_grid(tmp_path):
    detection = DetectedObject2D()
    detection.object_id = 1
    detection.label = 'can'
    detection.confidence = 0.9
    detection.x_max = 20.0
    detection.y_max = 20.0
    detection.median_depth_m = 0.4
    detection.estimated_width_m = 0.05
    detection.estimated_height_m = 0.08
    obj = DetectedObject3D()
    obj.object_id = 1
    obj.label = 'can'
    obj.obb_size.x = 0.04
    obj.obb_size.y = 0.04
    obj.obb_size.z = 0.1
    candidate = GraspCandidate()
    candidate.tcp_pose.orientation.w = 1.0
    candidate.required_opening_m = 0.04
    candidate.grasp_depth_m = 0.02
    candidate.score = 0.8
    selection = SimpleNamespace(
        selected_candidate_index=0,
        selected_candidate=candidate,
        selected_arm='left',
        pregrasp_position_error_m=0.001,
        grasp_position_error_m=0.001,
        pregrasp_approach_error_degrees=2.0,
        pregrasp_closing_error_degrees=3.0,
        grasp_approach_error_degrees=2.0,
        grasp_closing_error_degrees=3.0,
        trajectory_max_approach_error_degrees=4.0,
        trajectory_max_closing_error_degrees=5.0,
        planner_id='RRTConnectkConfigDefault',
        gripper_joint_position=0.5,
    )
    decision = TargetDecision(1, 'can', True, None, 'eligible', 0.4, 0.05)

    json_path, grid_path = write_validation_evidence(
        tmp_path,
        snapshot_id='snapshot-1',
        query='find objects',
        detections=[detection],
        decisions=[decision],
        selected_detection=detection,
        selected_object=obj,
        candidates=[candidate],
        selection=selection,
        latencies=[('detect', 0.1)],
        phase1_debug=None,
        phase2_debug=None,
        grasp_debug=None,
    )

    assert tmp_path.joinpath('validate_pick_summary_grid.png').is_file()
    assert tmp_path.joinpath('snapshot-1', 'pipeline_grid_3x3.png').is_file()
    assert tmp_path.joinpath('snapshot-1', 'result.json').is_file()
    assert grid_path.endswith('pipeline_grid_3x3.png')
    assert json_path.endswith('result.json')
