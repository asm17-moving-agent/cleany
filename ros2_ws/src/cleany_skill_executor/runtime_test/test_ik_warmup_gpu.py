"""Real GPU captures from the CAD model; no ROS requests or actuator clients."""
import json
import os
from pathlib import Path
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_RUN_CUROBO') != '1',
    reason='Enable with make test-motion-gpu in the pinned CUDA environment')


@pytest.mark.parametrize('include_aim', [False, True], ids=['vlm_grasp_only', 'grasp_and_aim'])
def test_primary_warmup_reuses_all_captured_arm_tip_buffers(tmp_path, include_aim):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.context import Context
    from sensor_msgs.msg import JointState
    import xacro
    from cleany_skill_executor.core.urdf_fk import UrdfChain
    from cleany_skill_executor.hybrid_adapter import HybridConfig, HybridGraspAdapter
    from cleany_skill_executor.curobo_model import CuRoboConfig
    from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig

    description = Path(get_package_share_directory('cleany_description'))/'urdf/cleany.urdf.xacro'
    xml = xacro.process_file(str(description), mappings={
        'include_head_camera': 'true', 'include_wheel_joints': 'false'}).toxml()
    context = Context()
    rclpy.init(context=context, domain_id=179)
    node = rclpy.create_node('isolated_ik_warmup_probe', context=context)
    adapter = HybridGraspAdapter(node, MoveItAdapterConfig(base_frame='base_link'),
        gpu_config=CuRoboConfig(num_seeds=16, spheres_per_link=8,
            geometry_cache_directory=os.environ.get('CLEANY_GPU_GEOMETRY_CACHE', '')),
        hybrid_config=HybridConfig(progressive_pregrasp=True, defer_candidate_ik_retries=True,
            group_gripper_openings=True, pregrasp_retry_passes=0))
    try:
        adapter.set_robot_description(xml)
        joints = JointState(name=list(adapter._state_joint_names), position=[0.]*len(adapter._state_joint_names))
        values = dict(zip(joints.name, joints.position))
        adapter.set_current_state(joints)
        adapter.set_deadline(time.monotonic()+300.)
        began = time.monotonic()
        adapter.warmup(['left', 'right'], include_aim=include_aim, continue_work=lambda: True)
        warmup_sec = time.monotonic()-began
        engine = adapter._gpus[16]._engine
        expected = {(arm, f'{arm}_{tip}', 8) for arm in ('left', 'right')
                    for tip in (('grasp_tcp', 'pregrasp_aim_tip') if include_aim else ('grasp_tcp',))}
        assert set(engine._batch_ik_caches) == expected
        buffers = {key: id(value[0][key[:2]]) for key, value in engine._batch_ik_caches.items()}
        assert not adapter._pregrasp_cache and set(adapter._gpus) == {16}
        adapter.set_current_state(joints)
        adapter.set_deadline(time.monotonic()+300.)
        rows = []
        for arm, tip, _ in sorted(expected):
            position, _ = UrdfChain(xml, 'base_link', tip).pose(values)
            seeds = adapter._gpus[16]._seed_bases(arm, None)
            began = time.monotonic()
            engine.solve_batch(arm, tip, [tuple(position)]*8, [seeds]*8, values, {})
            rows.append({'arm': arm, 'tip': tip, 'reused_sec': time.monotonic()-began})
        assert buffers == {key: id(value[0][key[:2]]) for key, value in engine._batch_ik_caches.items()}
        # Changing the gripper, head and opposite arm must update every locked
        # transform exactly as a full RobotConfig rebuild, preserving CUDA buffers.
        import torch
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        lock_updates = []
        for arm, tip, _ in sorted(expected):
            solver = engine._batch_ik_caches[(arm, tip, 8)][0][(arm, tip)]
            fixed_address = solver.kinematics.kinematics_config.fixed_transforms.data_ptr()
            for angle in (.25, -.15):
                changed = dict(values)
                other = 'right' if arm == 'left' else 'left'
                changed.update({f'{arm}_gripper_joint': angle+.8,
                                f'{other}_gripper_joint': angle+.5,
                                f'{other}_shoulder_yaw_joint': angle,
                                'head_pan_joint': angle, 'head_tilt_joint': angle/2})
                position, _ = UrdfChain(xml, 'base_link', tip).pose(changed)
                seeds = adapter._gpus[16]._seed_bases(arm, None)
                began = time.monotonic()
                engine.solve_batch(arm, tip, [tuple(position)]*8, [seeds]*8, changed, {})
                update_solve_sec = time.monotonic()-began
                began = time.monotonic()
                robot = adapter._robot_model.robot_config(engine._urdf, arm, changed, tip)
                reference = CudaRobotModel(RobotConfig.from_dict(robot, engine.tensor).kinematics)
                rebuild_sec = time.monotonic()-began
                current = solver.kinematics.kinematics_config
                assert current.fixed_transforms.data_ptr() == fixed_address
                torch.testing.assert_close(current.fixed_transforms,
                    reference.kinematics_config.fixed_transforms, atol=3e-5, rtol=0)
                torch.testing.assert_close(current.lock_jointstate.position,
                    reference.kinematics_config.lock_jointstate.position)
                torch.testing.assert_close(current.cspace.retract_config,
                    reference.kinematics_config.cspace.retract_config)
                q = engine.tensor.to_device([[changed[name] for name in solver.joint_names]])
                actual, baseline = solver.kinematics.get_state(q), reference.get_state(q)
                torch.testing.assert_close(actual.ee_position, baseline.ee_position, atol=3e-5, rtol=0)
                torch.testing.assert_close(actual.link_spheres_tensor,
                    baseline.link_spheres_tensor, atol=3e-5, rtol=0)
                lock_updates.append(dict(arm=arm, tip=tip, update_and_solve_sec=update_solve_sec,
                                         reference_rebuild_sec=rebuild_sec))
        assert engine.timings.get('locked_joint_fast_updates', 0) == len(lock_updates)
        assert engine.timings.get('locked_joint_full_updates', 0) == 0
        measurements = {'warmup_sec': warmup_sec, 'include_aim': include_aim, 'reuse': rows,
            'locked_joint_updates': lock_updates,
            'geometry_cache_hits': adapter._robot_model.geometry_cache_hits,
            'geometry_cache_misses': adapter._robot_model.geometry_cache_misses}
        (tmp_path/'warmup-timings.json').write_text(json.dumps(measurements, indent=2))
        print('IK_WARMUP_MEASUREMENTS=' + json.dumps(measurements))
    finally:
        adapter.close()
        node.destroy_node()
        context.shutdown()
