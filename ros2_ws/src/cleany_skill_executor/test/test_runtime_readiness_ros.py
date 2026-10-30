import json
import time
import pytest

def test_single_participant_waits_for_servers_and_live_feedback(tmp_path, monkeypatch):
    import rclpy
    from rclpy.action import ActionClient, ActionServer
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from cleany_skill_executor.runtime_readiness import (
        FEEDBACK, REQUIRED_ACTIONS, REQUIRED_SERVICES, ReadinessProbe)
    import cleany_skill_executor.runtime_readiness as readiness

    rclpy.init(domain_id=189)
    nodes, action_clients, action_servers = [], [], []
    executor = SingleThreadedExecutor()
    try:
        inspector = Node('perception_inspector')
        move_group = Node('move_group')
        controller = Node('controller_manager')
        probe = ReadinessProbe(gui=False)
        nodes.extend((inspector, move_group, controller, probe))
        move_group.declare_parameter('allow_trajectory_execution', False)
        def controllers(request, response):
            response.controller = [ControllerState(name=name, state='active') for name in
                                  ('left_arm_controller', 'left_gripper_controller')]
            return response
        controller.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
        def unused(request, response):
            pytest.fail('Readiness must not call motion, camera control or stop services')
        warmup_ready = [False]
        def warmup(request, response):
            response.success = warmup_ready[0]
            response.message = 'IK warmup ready' if response.success else 'IK warmup pending'
            return response
        for name in REQUIRED_SERVICES:
            if name != '/controller_manager/list_controllers':
                inspector.create_service(Trigger, name,
                    warmup if name == '/grasp/warmup_status' else unused)
        for name in REQUIRED_ACTIONS:
            action_clients.append(ActionClient(inspector, FollowJointTrajectory, name))
        publishers = {name: inspector.create_publisher(kind, name, 1) for name, kind in FEEDBACK.items()}
        guard = inspector.create_publisher(String, '/real_control/status', 1)
        enabled = [False]
        def publish():
            guard.publish(String(data='{"state": "ready"}'))
            if enabled[0]:
                for name, kind in FEEDBACK.items():
                    publishers[name].publish(kind())
        inspector.create_timer(.05, publish)
        for node in nodes: executor.add_node(node)
        def wait_for(predicate, timeout=10.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
                pending = probe.pending()
                if predicate(pending): return pending
            pytest.fail('Readiness did not converge: ' + repr(pending))
        pending = wait_for(lambda p: any('(server)' in item for item in p))
        began = time.monotonic()
        for name in REQUIRED_ACTIONS:
            action_servers.append(ActionServer(inspector, FollowJointTrajectory, name,
                execute_callback=lambda goal: pytest.fail('No actuator goal is allowed')))
        wait_for(lambda p: 'Primary GPU IK warmup' in p and not any('(server)' in item for item in p))
        assert probe.feedback_after is None
        warmup_ready[0] = True
        wait_for(lambda p: set(p) == set(FEEDBACK))
        original = readiness.get_action_server_names_and_types_by_node
        failures = []

        def discover(node, name, namespace):
            if not failures:
                failures.append(name)
                raise readiness._rclpy.RCLError(
                    'Failed to get action server names and type: error not set')
            return original(node, name, namespace)

        monkeypatch.setattr(readiness, 'get_action_server_names_and_types_by_node', discover)
        pending = probe.pending()
        assert any('Action server discovery for /' in item for item in pending)
        assert probe.feedback_after is None
        wait_for(lambda p: set(p) == set(FEEDBACK))
        enabled[0] = True
        wait_for(lambda p: not p)
        (tmp_path/'readiness.json').write_text(json.dumps({
            'wall_sec': time.monotonic()-began, 'action_servers': len(action_servers),
            'motion_goals': 0, 'injected_discovery_errors': len(failures)}, indent=2))
    finally:
        for action in action_servers: action.destroy()
        for action in action_clients: action.destroy()
        executor.shutdown()
        for node in reversed(nodes): node.destroy_node()
        rclpy.shutdown()
