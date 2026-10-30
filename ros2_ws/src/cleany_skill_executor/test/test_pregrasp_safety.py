from types import SimpleNamespace

import pytest
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError, REQUIRED_JOINT_NAMES
from cleany_skill_executor.pregrasp_pipeline import check_joint_snapshot, check_start_state, check_path_start, PregraspPipeline
from cleany_skill_executor.sensor_scene import SensorSceneMonitor


def monitor(monkeypatch):
    import cleany_skill_executor.sensor_scene as module
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    values = {'require_sensor_scene': True, 'sensor_scene_maximum_age_sec': 2.,
              'sensor_scene_startup_timeout_sec': 1.,
              'sensor_scene_receipt_topic': '/receipt', 'sensor_scene_topic': '/scene',
              'sensor_scene_service': '/scene_service', 'sensor_scene_snapshot_period_sec': 1.}
    node = SimpleNamespace(declare_parameter=lambda *_: None,
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        create_subscription=lambda *_, **__: None,
        create_client=lambda *_, **__: None, create_timer=lambda *_, **__: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))))
    return SensorSceneMonitor(node), clock


def receipt(seconds):
    header = Header(frame_id='optical')
    header.stamp.sec = seconds
    return header


def populated():
    scene = PlanningScene()
    scene.world.octomap.octomap.id = 'OcTree'
    scene.world.octomap.octomap.data = [1, 2]
    return scene


def test_map_required_even_with_fresh_receipt(monkeypatch):
    item, _ = monitor(monkeypatch)
    item.on_receipt(receipt(10))
    with pytest.raises(InfrastructureError, match='OctoMap'):
        item.check()
    item.on_scene(populated())
    item.check()
    with pytest.raises(InfrastructureError, match='advanced'):
        item.check(after_stamp_ns=10_000_000_000)


def test_replay_future_and_state_diffs_cannot_renew_map(monkeypatch):
    item, clock = monitor(monkeypatch)
    item.on_scene(populated())
    item.on_receipt(receipt(10))
    clock[0] = 13.
    item.on_receipt(receipt(10))
    item.on_receipt(receipt(14))
    item.on_scene(PlanningScene(is_diff=True))
    with pytest.raises(InfrastructureError, match='fresh'):
        item.check()
    item.on_receipt(receipt(13))
    item.on_scene(populated())
    item.check(after_stamp_ns=12_000_000_000)
    item.on_scene(PlanningScene())
    with pytest.raises(InfrastructureError, match='OctoMap'):
        item.check()


def test_measured_state_and_start_drift_guard():
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.]*12)
    state.header.stamp.sec = 10
    values = check_joint_snapshot(state, 10_100_000_000)
    check_start_state(values, values)
    whole_robot = {**values, 'head_pan_joint': 0.1, 'head_tilt_joint': -0.2}
    check_start_state(whole_robot, whole_robot)
    with pytest.raises(RuntimeError, match='stale'):
        check_joint_snapshot(state, 9_000_000_000)
    moved = dict(values, right_wrist_roll_joint=.03)
    with pytest.raises(RuntimeError, match='moved'):
        check_start_state(moved, values)
    state.position[0] = float('nan')
    with pytest.raises(RuntimeError, match='malformed'):
        check_joint_snapshot(state, 10_100_000_000)


def test_start_bridge_permission_applies_only_to_commanded_joints():
    planned = dict.fromkeys(REQUIRED_JOINT_NAMES, 0.)
    moved = dict(planned, left_wrist_roll_joint=.025)
    check_start_state(moved, planned, commanded=('left_wrist_roll_joint',), commanded_tolerance=.03)
    moved['right_wrist_roll_joint'] = .025
    with pytest.raises(RuntimeError, match='moved'):
        check_start_state(moved, planned, commanded=('left_wrist_roll_joint',), commanded_tolerance=.03)


@pytest.mark.parametrize('terminal', [True, False])
def test_communication_recovery_requires_terminal_action_cancellation(terminal):
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus
    from cleany_skill_executor.core.hardware_execution import CommunicationRecovered
    result = Future()
    error = CommunicationRecovered(1)
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                             cancel_goal_async=lambda: 'cancel')
    calls = []
    real = SimpleNamespace(_active_recovery_generation=0)

    def wait(future, timeout, **kwargs):
        calls.append(future)
        if future == 'send':
            return handle
        if future == 'cancel':
            assert real._active_recovery_generation is None
            return object()
        if len(calls) == 2:
            raise error
        if not terminal:
            raise RuntimeError('cancel result unavailable')
        assert real._active_recovery_generation is None
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        return result.result()

    client = SimpleNamespace(wait_for_server=lambda **_: True, send_goal_async=lambda _: 'send')
    node = SimpleNamespace(wait=wait, real=real,
                           get_logger=lambda: SimpleNamespace(error=lambda _: None))
    with pytest.raises(CommunicationRecovered):
        PregraspPipeline.run_action(node, client, object(), 1., monitor=True)
    assert error.cancellation_confirmed is terminal
    assert calls == ['send', result, 'cancel', result]


def test_simulator_gate_rejects_real_or_additional_hardware():
    hardware = SimpleNamespace(component=[SimpleNamespace(
        class_type='real_robot/Hardware', state=SimpleNamespace(label='active'))])
    node = SimpleNamespace(hardware=object(), call=lambda *_: hardware)
    with pytest.raises(RuntimeError, match='MuJoCo'):
        PregraspPipeline.verify_simulator(node, 'left')
    hardware.component *= 2
    for item in hardware.component:
        item.class_type = 'mujoco_ros2_control/MujocoSystemInterface'
    with pytest.raises(RuntimeError, match='exactly one'):
        PregraspPipeline.verify_simulator(node, 'left')


def test_unknown_mode_is_rejected_before_node_initialization():
    with pytest.raises(ValueError, match='Unknown execution mode'):
        PregraspPipeline(mode='unsupported')


def test_trajectory_cannot_jump_to_a_different_start():
    trajectory = SimpleNamespace(joint_names=['joint'],
                                 points=[SimpleNamespace(positions=[.1])])
    with pytest.raises(RuntimeError, match='planning start'):
        check_path_start(trajectory, {'joint': 0.})
    check_path_start(trajectory, {'joint': .1})


def test_action_timeout_keeps_result_available_until_cancel_confirmed():
    from rclpy.task import Future
    result = Future()
    calls = []
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                             cancel_goal_async=lambda: 'cancel')
    def wait(future, timeout, **kwargs):
        calls.append((future, kwargs))
        if future == 'send':
            return handle
        if future is result and len(calls) == 2:
            assert kwargs['cancel_on_timeout'] is False
            raise RuntimeError('test timeout')
        if future is result:
            result.set_result(object())
        return object()
    client = SimpleNamespace(wait_for_server=lambda **_: True,
                             send_goal_async=lambda _: 'send')
    node = SimpleNamespace(wait=wait)
    with pytest.raises(RuntimeError, match='test timeout'):
        PregraspPipeline.run_action(node, client, object(), 1.)
    assert [future for future, _ in calls] == ['send', result, 'cancel', result]
    assert not result.cancelled()


@pytest.mark.parametrize('frozen,scene_checks', [(True, 0), (False, 1)])
def test_physical_action_monitor_accepts_a_deliberately_frozen_scene(
        frozen, scene_checks, monkeypatch):
    from cleany_skill_executor import pregrasp_pipeline as module
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    checks = []
    measurements = []
    guard_checks = []

    class Future:
        calls = 0

        def add_done_callback(self, callback):
            self.callback = callback

        def done(self):
            self.calls += 1
            return self.calls > 1

        @staticmethod
        def result():
            return 'complete'

    node = SimpleNamespace(
        scene_monitor=SimpleNamespace(check=lambda: checks.append(True)),
        real=SimpleNamespace(
            sensor_scene_frozen=frozen,
            monitor=lambda: guard_checks.append(True),
        ),
        measured=lambda: measurements.append(True),
    )

    assert PregraspPipeline.wait(
        node, Future(), 1., monitor=True) == 'complete'
    assert len(checks) == scene_checks
    assert measurements == [True]
    assert guard_checks == [True]


def test_pipeline_wait_wakes_on_service_completion_without_polling_sleep(monkeypatch):
    from concurrent.futures import Future
    from cleany_skill_executor import pregrasp_pipeline as module

    future = Future()
    waits, notifications = [], []

    class Completion:
        def set(self):
            notifications.append(True)

        def wait(self, timeout):
            waits.append(timeout)
            future.set_result('service response')

    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.threading, 'Event', Completion)
    monkeypatch.setattr(module.time, 'sleep', lambda _: pytest.fail('response should wake the waiter'))
    assert PregraspPipeline.wait(SimpleNamespace(), future, 1.) == 'service response'
    assert len(waits) == 1 and 0 < waits[0] <= .01
    assert notifications == [True]


@pytest.mark.parametrize('cancel_on_timeout', [True, False])
def test_pipeline_response_wait_keeps_timeout_and_cancellation(monkeypatch, cancel_on_timeout):
    from concurrent.futures import Future
    from cleany_skill_executor import pregrasp_pipeline as module

    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    future = Future()
    with pytest.raises(RuntimeError, match='timed out'):
        PregraspPipeline.wait(SimpleNamespace(), future, 0., cancel_on_timeout=cancel_on_timeout)
    assert future.cancelled() is cancel_on_timeout


def test_measured_simulation_state_waits_for_clock_without_accepting_future(monkeypatch):
    import threading
    from cleany_skill_executor import pregrasp_pipeline as module
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.]*12)
    state.header.stamp.sec = 10
    clock = [9_999_000_000]
    node = SimpleNamespace(mode='mujoco', lock=threading.Lock(), state=state,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock[0])),
        get_parameter=lambda _: SimpleNamespace(value=.5))
    monkeypatch.setattr(module.time, 'sleep', lambda _: clock.__setitem__(0, 10_000_000_000))
    assert PregraspPipeline.measured(node) == state
    clock[0] = 9_000_000_000
    with pytest.raises(RuntimeError, match='age=-1.000'):
        PregraspPipeline.measured(node)


def test_simulation_startup_needs_current_inputs_and_capture_time_tf():
    from cleany_skill_executor.pregrasp_sim_ready import SimulationInputsReady
    from sensor_msgs.msg import CameraInfo
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.]*12)
    state.header.stamp.sec = 10
    available = [True]
    info = CameraInfo(header=state.header)
    node = SimpleNamespace(joints=state,
        infos={topic: info for topic in ('/camera/color/camera_info', '/camera/depth/camera_info',
                                         '/left_wrist_camera/camera_info', '/right_wrist_camera/camera_info')},
        tf=SimpleNamespace(can_transform=lambda *_: available[0]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)))
    assert SimulationInputsReady.inputs_ready(node)
    available[0] = False
    assert not SimulationInputsReady.inputs_ready(node)
    available[0] = True
    state.header.stamp.sec = 11
    assert not SimulationInputsReady.inputs_ready(node)
