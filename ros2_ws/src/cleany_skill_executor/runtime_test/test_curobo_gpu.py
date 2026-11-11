"""Opt-in real CUDA tests; no controller or real robot is started."""

import os
import signal
import subprocess
import time
from pathlib import Path
import xml.etree.ElementTree as ET
from uuid import uuid4

import numpy as np
import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES,
    JointSolution,
    REQUIRED_JOINT_NAMES,
)
from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel

pytestmark = pytest.mark.skipif(
    os.environ.get("CLEANY_RUN_CUROBO") != "1",
    reason="set CLEANY_RUN_CUROBO=1 in the pinned CUDA environment",
)


@pytest.fixture(scope="module")
def engine():
    from ament_index_python.packages import get_package_share_directory
    import xacro
    from cleany_skill_executor.curobo_engine import CuRoboEngine

    description = Path(get_package_share_directory("cleany_description"))
    moveit = Path(get_package_share_directory("cleany_moveit_config"))
    urdf = xacro.process_file(
        str(description / "urdf/cleany.urdf.xacro"),
        mappings={"include_head_camera": "false", "include_wheel_joints": "false",
                  "optimized_arm_collision_meshes": "true"},
    ).toxml()
    config = CuRoboConfig(num_seeds=16)
    result = CuRoboEngine(
        RobotModel(urdf, (moveit / "config/cleany.srdf").read_text(), config), config
    )
    yield result
    result.close()


def cpu_fk(urdf, tip, state):
    """Independent URDF chain multiplication, without cuRobo or MoveIt."""
    from scipy.spatial.transform import Rotation

    joints = {
        j.find("child").get("link"): j for j in ET.fromstring(urdf).findall("joint")
    }
    chain = []
    while tip != "base_link":
        joint = joints[tip]
        chain.append(joint)
        tip = joint.find("parent").get("link")
    result = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find("origin")
        transform = np.eye(4)
        if origin is not None:
            transform[:3, :3] = Rotation.from_euler(
                "xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]
            ).as_matrix()
            transform[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        if joint.get("type") == "revolute":
            rotation = np.eye(4)
            axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            rotation[:3, :3] = Rotation.from_rotvec(
                axis * state[joint.get("name")]
            ).as_matrix()
            transform = transform @ rotation
        result = result @ transform
    return result


@pytest.mark.parametrize("arm", ["left", "right"])
def test_gpu_fk_and_position_ik_match_authoritative_urdf(engine, arm):
    from scipy.spatial.transform import Rotation

    state = dict.fromkeys(REQUIRED_JOINT_NAMES, 0.0)
    joints = JointSolution(ARM_JOINT_NAMES[arm], (0.2, 0.65, 0.8, -0.4, 0.3))
    state.update(zip(joints.names, joints.positions))
    for tip in (f"{arm}_grasp_tcp", f"{arm}_pregrasp_aim_tip"):
        expected = cpu_fk(engine.model.xml, tip, state)
        actual = engine.fk(arm, tip, [joints], state)[0]
        np.testing.assert_allclose(actual.position, expected[:3, 3], atol=2e-6)
        np.testing.assert_allclose(
            Rotation.from_quat(actual.orientation_xyzw).as_matrix(),
            expected[:3, :3],
            atol=2e-5,
        )
        solutions = engine.solve(
            arm,
            tip,
            actual.position,
            [JointSolution(joints.names, (0.0, 0.5, 0.6, -0.2, 0.0))],
            state,
        )
        assert solutions
        poses = engine.fk(arm, tip, solutions, state)
        assert (
            max(np.linalg.norm(np.array(p.position) - actual.position) for p in poses)
            < 0.002
        )


@pytest.mark.parametrize("arm", ["left", "right"])
def test_gpu_generates_finite_joint_path(engine, arm):
    state = dict.fromkeys(REQUIRED_JOINT_NAMES, 0.0)
    start = JointSolution(ARM_JOINT_NAMES[arm], (0.0, 0.65, 0.8, -0.4, 0.0))
    state.update(zip(start.names, start.positions))
    goal = JointSolution(start.names, (0.1, 0.7, 0.85, -0.4, 0.1))
    world = {
        "cuboid": {
            "far_box": {
                "dims": [0.1, 0.1, 0.1],
                "pose": [5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 0.0],
            }
        }
    }
    result = engine.plan(arm, start, goal, state, world, 0.1, 0.1, 10.0, 3)
    assert result is not None, engine.last_plan_status
    names, positions = result
    assert set(names) == set(start.names)
    assert len(positions) > 1 and np.isfinite(positions).all()
    order = [names.index(n) for n in start.names]
    np.testing.assert_allclose(positions[0, order], start.positions, atol=0.002)
    np.testing.assert_allclose(positions[-1, order], goal.positions, atol=0.002)
    motion = engine._motion[arm][0]
    collision_config = motion.kinematics.get_self_collision_config()
    before_spheres = motion.kinematics.get_state(
        engine.tensor.to_device([[state[n] for n in names]])
    ).link_spheres_tensor.clone()
    other = "right" if arm == "left" else "left"
    state[f"{other}_gripper_joint"] = 1.2
    updated = engine.plan(arm, start, goal, state, world, 0.1, 0.1, 10.0, 3)
    assert updated is not None, engine.last_plan_status
    assert motion.kinematics.get_self_collision_config() is collision_config
    after_spheres = motion.kinematics.get_state(
        engine.tensor.to_device([[state[n] for n in names]])
    ).link_spheres_tensor
    assert not engine.torch.allclose(before_spheres, after_spheres)
    # A blocking world must still reject the same motion after locked joints
    # have changed, and a subsequent empty world must clear the old obstacle.
    blocked = {
        "cuboid": {
            "block": {
                "dims": [3.0, 3.0, 3.0],
                "pose": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            }
        }
    }
    assert engine.plan(arm, start, goal, state, blocked, 0.1, 0.1, 10.0, 1) is None
    assert engine.plan(arm, start, goal, state, {}, 0.1, 0.1, 10.0, 3) is not None


@pytest.fixture(scope="module")
def ros_node(tmp_path_factory):
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import GetPlanningScene

    logs = tmp_path_factory.mktemp("curobo-moveit") / "mock.log"
    domain_id = 20 + uuid4().int % 180
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
    with logs.open("w") as output:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "cleany_moveit_config",
                "mock_planning.launch.py",
                "use_rviz:=false",
                'include_head_camera:='+os.environ.get('CLEANY_TEST_MOCK_HEAD', 'false'),
                'head_tilt_initial:='+os.environ.get('CLEANY_TEST_MOCK_HEAD_TILT', '0.405'),
            ],
            env=environment,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init(domain_id=domain_id)
        node = Node("curobo_moveit_validation_test")
        try:
            client = node.create_client(GetPlanningScene, "/get_planning_scene")
            deadline = time.monotonic() + 45.0
            while not client.wait_for_service(timeout_sec=0.2):
                assert process.poll() is None, logs.read_text()[-6000:]
                assert time.monotonic() < deadline, logs.read_text()[-6000:]
            yield node
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10.0)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_gpu_adapter_with_real_moveit_services(engine, ros_node, arm, record_property):
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
    from cleany_skill_executor.physical_moveit_adapter import (
        MoveItAdapterConfig,
        MoveItGraspAdapter,
    )

    adapter = CuRoboGraspAdapter(
        ros_node,
        MoveItAdapterConfig(planning_timeout_sec=10.0, planning_attempts=3),
        engine=engine,
        gpu_config=engine.config,
    )
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.0] * 12)
    state.header.stamp = ros_node.get_clock().now().to_msg()
    start = JointSolution(ARM_JOINT_NAMES[arm], (0.0, 0.65, 0.8, -0.4, 0.0))
    for name, value in zip(start.names, start.positions):
        state.position[state.name.index(name)] = value
    adapter.set_current_state(state)
    adapter.set_deadline(time.monotonic() + 120.0)
    actual_gpu = adapter.forward_kinematics(arm, start)
    actual_moveit = MoveItGraspAdapter.forward_kinematics(adapter, arm, start)
    np.testing.assert_allclose(actual_gpu.position, actual_moveit.position, atol=2e-6)
    goal = JointSolution(start.names, (0.1, 0.7, 0.85, -0.4, 0.1))
    assert adapter.state_is_valid(arm, start)
    assert adapter.state_is_valid(arm, goal)
    began = time.monotonic()
    path = adapter.plan(arm, goal, start)
    assert path is not None
    print(
        f"\n{arm}: cuRobo + GPU validation {time.monotonic() - began:.3f}s, {len(path.points)} points"
    )
    target = adapter.forward_kinematics(arm, goal)
    aligned = adapter.solve_aligned_ik(arm, target, start)
    assert aligned is not None and adapter._feasible_alignment(aligned)
    assert adapter.state_is_valid(arm, aligned.solution)
    # Independent reference checks are test-only, never called by GPU mode.
    assert all(
        MoveItGraspAdapter.state_is_valid(
            adapter, arm, JointSolution(path.joint_names, q)
        )
        for q in path.points
    )
    adapter._validity_client = None  # Any inherited CPU collision call now fails.
    began = time.monotonic()
    assert adapter.plan(arm, goal, start) is not None
    gpu_warm = time.monotonic() - began
    record_property("gpu_warm_seconds", gpu_warm)
    for key, value in adapter.last_timings.items():
        record_property(key, value)
    baseline = MoveItGraspAdapter(
        ros_node, MoveItAdapterConfig(planning_timeout_sec=10.0)
    )
    baseline.set_current_state(state)
    began = time.monotonic()
    assert baseline.plan(arm, goal, start) is not None
    cpu_warm = time.monotonic() - began
    print(
        f"{arm}: warm GPU + GPU validation={gpu_warm:.3f}s, OMPL={cpu_warm:.3f}s, stages={adapter.last_timings}"
    )
    record_property("ompl_warm_seconds", cpu_warm)


def test_gpu_node_parameters_select_the_real_backend(ros_node):
    from rclpy.parameter import Parameter
    from cleany_skill_executor.physical_grasp_selection_node import GraspSelectionNode

    node = GraspSelectionNode(
        parameter_overrides=[Parameter("motion_backend", value="curobo")]
    )
    try:
        assert node._adapter.planner_id == "curobo_v0.7.8_gpu_validated"
        assert node._adapter._engine.torch.cuda.is_available()
    finally:
        node._adapter._engine.close()
        node.destroy_node()


def test_gpu_convex_acceptance_handles_containment_base_and_contact_pairs():
    from types import SimpleNamespace
    from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
    from curobo.types.base import TensorDeviceType
    from cleany_skill_executor.curobo_collision import (
        GpuCollisionValidator,
        world_bodies,
    )
    import torch

    bodies = world_bodies(
        {
            "cuboid": {
                "base/primitive/0": {
                    "dims": [1.0, 1.0, 1.0],
                    "pose": [0, 0, 0, 1, 0, 0, 0],
                },
                "finger/primitive/0": {
                    "dims": [0.1, 0.1, 0.1],
                    "pose": [0, 0, 0, 1, 0, 0, 0],
                },
            }
        }
    )
    model = SimpleNamespace(
        bodies=bodies, collision_links=["base", "finger"], ignore={}
    )
    validator = GpuCollisionValidator(model, TensorDeviceType(), 0.0001)
    acm = AllowedCollisionMatrix()
    xyz = torch.zeros((3, 2, 3), device="cuda")
    xyz[1, 1, 0] = 0.54  # surface overlap
    xyz[2, 1, 0] = 0.7  # separated
    quaternion = torch.zeros((3, 2, 4), device="cuda")
    quaternion[:, :, 0] = 1
    fk = SimpleNamespace(links_position=xyz, links_quaternion=quaternion)
    validator.prepare({}, acm)
    assert validator.valid(fk).cpu().tolist() == [False, False, True]
    # Permit base/finger self-contact, but NOT base/target world contact.
    acm.entry_names = ["base", "finger", "target"]
    acm.entry_values = [
        AllowedCollisionEntry(enabled=row)
        for row in ([False, True, False], [True, False, True], [False, True, False])
    ]
    world = {
        "cuboid": {
            "target/primitive/0": {
                "dims": [0.05, 0.05, 0.05],
                "pose": [0, 0, 0, 1, 0, 0, 0],
            }
        }
    }
    validator.prepare(world, acm)
    assert not validator.valid(fk).any().item()  # target entirely inside base
    validator.prepare({}, acm)
    assert validator.valid(fk).all().item()  # removed obstacle is really gone


@pytest.mark.parametrize("arm", ["left", "right"])
def test_gpu_collision_aware_ik_rejects_blocked_goal(engine, arm):
    state = dict.fromkeys(REQUIRED_JOINT_NAMES, 0.0)
    q = JointSolution(ARM_JOINT_NAMES[arm], (0.0, 0.65, 0.8, -0.4, 0.0))
    state.update(zip(q.names, q.positions))
    target = engine.fk(arm, f"{arm}_grasp_tcp", [q], state)[0]
    world = {"cuboid": {"block": {"dims": [3.0] * 3, "pose": [0, 0, 0, 1, 0, 0, 0]}}}
    assert (
        engine.solve(arm, f"{arm}_grasp_tcp", target.position, [q], state, world) == []
    )
    assert engine.solve(arm, f"{arm}_grasp_tcp", target.position, [q], state, {})


def test_gpu_complete_robot_collision_agrees_conservatively_with_moveit(
    engine, ros_node, record_property
):
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
    from cleany_skill_executor.physical_moveit_adapter import (
        MoveItAdapterConfig,
        MoveItGraspAdapter,
    )

    adapter = CuRoboGraspAdapter(
        ros_node, MoveItAdapterConfig(), engine=engine, gpu_config=engine.config
    )
    names = tuple(REQUIRED_JOINT_NAMES)
    state = dict.fromkeys(names, 0.0)
    rng = np.random.default_rng(20260907)
    limits = np.array([engine.model.limits[n] for n in names])
    points = rng.uniform(limits[:, 0], limits[:, 1], size=(96, len(names)))
    nominal = dict(state)
    for arm in ARM_JOINT_NAMES:
        nominal.update(zip(ARM_JOINT_NAMES[arm], (0.0, 0.65, 0.8, -0.4, 0.0)))
    points = np.vstack(([nominal[n] for n in names], points))
    adapter.set_current_state(
        JointState(name=list(names), position=[nominal[n] for n in names])
    )
    scene = adapter._snapshot()
    gpu_valid = engine.valid_states(
        names, points, state, {}, scene.allowed_collision_matrix
    )
    reference = []
    for point in points:
        adapter.set_current_state(JointState(name=list(names), position=point.tolist()))
        values = dict(zip(names, point))
        reference.append(
            all(
                MoveItGraspAdapter.state_is_valid(
                    adapter,
                    arm,
                    JointSolution(joints, tuple(values[n] for n in joints)),
                )
                for arm, joints in ARM_JOINT_NAMES.items()
            )
        )
    cpu_valid = np.array(reference)
    assert gpu_valid[0] and cpu_valid[0]
    assert cpu_valid.sum() >= 3 and (~cpu_valid).sum() >= 3
    assert not np.any(gpu_valid & ~cpu_valid), "GPU missed a MoveIt mesh collision"
    record_property("states", len(points))
    record_property("gpu_accepted", int(gpu_valid.sum()))
    record_property("moveit_accepted", int(cpu_valid.sum()))
    record_property("conservative_rejections", int((~gpu_valid & cpu_valid).sum()))
    print(
        f"collision comparison: {len(points)} states, GPU accepted={gpu_valid.sum()}, MoveIt accepted={cpu_valid.sum()}"
    )


def test_warm_planning_resource_samples(engine, ros_node, record_property):
    """Small warm sample, not a production benchmark or isolated GPU memory comparison."""
    import cProfile
    import io
    import json
    import pstats
    import resource
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
    from cleany_skill_executor.physical_moveit_adapter import (
        MoveItAdapterConfig,
        MoveItGraspAdapter,
    )

    config = MoveItAdapterConfig(planning_timeout_sec=10.0, planning_attempts=3)
    gpu = CuRoboGraspAdapter(ros_node, config, engine=engine, gpu_config=engine.config)
    cpu = MoveItGraspAdapter(ros_node, config)
    names = ARM_JOINT_NAMES["left"]
    start = JointSolution(names, (0.0, 0.65, 0.8, -0.4, 0.0))
    goal = JointSolution(names, (0.1, 0.7, 0.85, -0.4, 0.1))
    state = dict.fromkeys(REQUIRED_JOINT_NAMES, 0.0)
    state.update(zip(names, start.positions))
    for adapter in (gpu, cpu):
        adapter.set_current_state(
            JointState(name=list(state), position=list(state.values()))
        )
        assert adapter.plan("left", goal, start) is not None
    profile = cProfile.Profile()
    results = {"gpu": [], "ompl": []}
    for index in range(5):
        for label, adapter in (("gpu", gpu), ("ompl", cpu)):
            before = resource.getrusage(resource.RUSAGE_SELF)
            began = time.monotonic()
            if index == 4 and label == "gpu":
                profile.enable()
            assert adapter.plan("left", goal, start) is not None
            profile.disable()
            elapsed = time.monotonic() - began
            after = resource.getrusage(resource.RUSAGE_SELF)
            results[label].append(
                {
                    "wall_sec": elapsed,
                    "client_cpu_sec": after.ru_utime
                    + after.ru_stime
                    - before.ru_utime
                    - before.ru_stime,
                }
            )
    results["gpu_stages"] = gpu.last_timings
    results["torch_allocated_mib"] = engine.torch.cuda.memory_allocated() / 2**20
    results["torch_reserved_mib"] = engine.torch.cuda.memory_reserved() / 2**20
    results["process_peak_rss_mib"] = (
        resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    )
    # CPU time is CLIENT ONLY; MoveIt server CPU is deliberately not inferred.
    record_property("resource_samples_json", json.dumps(results))
    print("warm resource samples: " + json.dumps(results))
    summary = io.StringIO()
    pstats.Stats(profile, stream=summary).sort_stats("cumtime").print_stats(12)
    print(summary.getvalue())
