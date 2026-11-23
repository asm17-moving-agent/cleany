import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription

import cleany_robot_interface.control_feedback_startup as module


def test_startup_requires_continuous_feedback_and_rejects_faults(monkeypatch):
    now = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    probe = NS(guard_state=None, guard_at=0., sample_stamp_ns=None, stable_since=None,
               get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(now[0] * 1e9))))
    receive_guard = module.ControlFeedbackStartupProbe.receive_guard
    receive_joints = module.ControlFeedbackStartupProbe.receive_joints
    ready = module.ControlFeedbackStartupProbe.ready

    def publish(at):
        now[0] = at
        sec, ns = divmod(round(at * 1e9), 1_000_000_000)
        receive_joints(probe, NS(header=NS(stamp=NS(sec=sec, nanosec=ns))))
        receive_guard(probe, NS(data='{"state":"ready"}'))
        return ready(probe)

    assert not ready(probe)
    for i in range(10):
        assert not publish(10. + i / 20)
    assert not publish(10.8)  # The acquisition gap restarts the stability window.
    for i in range(1, 20):
        assert not publish(10.8 + i / 20)
    assert publish(11.85)
    now[0] += .3
    assert not ready(probe)
    receive_guard(probe, NS(data='{"state":"fault"}'))
    with pytest.raises(RuntimeError, match='guard is not ready: fault'):
        ready(probe)


@pytest.mark.parametrize('cold,deferred', [(False, False), (True, False), (True, True)])
def test_moveit_waits_for_both_spawners_and_successful_probe(monkeypatch, tmp_path, cold, deferred):
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('real_control_launch', package / 'launch/real_control.launch.py')
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    model = tmp_path / 'robot.urdf'
    joints = ''.join(f'<joint name="motor_{i}"><command_interface name="position"/></joint>' for i in range(12))
    model.write_text(f'<robot name="test"><ros2_control>{joints}</ros2_control></robot>')
    profile = NS(model=model, controllers=tmp_path / 'controllers.yaml', controller_names=('left_arm_controller',),
                 state=NS(camera_pose_config=None), moveit_controllers=tmp_path / 'moveit.yaml',
                 joint_limits=tmp_path / 'limits.yaml')
    monkeypatch.setattr(launch_module, 'prepare_control_profile', lambda *_: profile)
    monkeypatch.setattr(launch_module, 'get_package_share_directory', lambda _: str(tmp_path))
    recorded = []
    original = launch_module.OnProcessExit

    def record(**kwargs):
        recorded.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(launch_module, 'OnProcessExit', record)
    context = LaunchContext()
    context.launch_configurations.update(repository_root=str(tmp_path), profile_file=str(model),
        control_file=str(profile.controllers), output_root=str(tmp_path), staged_moveit='true')
    for entity in launch_module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    if cold:
        context.launch_configurations.update(preload_cold_controllers='true', read_only='false',
            defer_cold_activation=str(deferred).lower(), allow_trajectory_execution='false')
        from cleany_robot_interface import calibrated_cad_model
        monkeypatch.setattr(calibrated_cad_model, 'require_cad_motion_ready', lambda *_: None)
    actions = launch_module.setup(context)
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)
    spawners = [r for r in recorded if getattr(r['on_exit'], '__name__', '') == 'spawned']
    assert len(spawners) == 2
    spawn = spawners[0]['on_exit']
    if cold:
        spawners.reverse()
    assert spawners[0]['target_action'] in actions and spawners[1]['target_action'] not in actions
    assert isinstance(spawn(NS(action=spawners[0]['target_action'], returncode=1), context)[0], EmitEvent)
    assert spawn(NS(action=spawners[0]['target_action'], returncode=0), context) == [spawners[1]['target_action']]
    probe, = spawn(NS(action=spawners[1]['target_action'], returncode=0), context)
    argv = [''.join(part.perform(context) for part in argument)
            for argument in probe.process_description.cmd]
    assert ('--cold-controllers' in argv) == (cold and not deferred)
    gate = next(r['on_exit'] for r in recorded if r['target_action'] is probe)
    assert isinstance(gate(NS(returncode=1), context)[0], EmitEvent)
    assert isinstance(gate(NS(returncode=0), context)[0], IncludeLaunchDescription)

@pytest.mark.parametrize('missing', ['first_read', 'every_read', 'activation'])
def test_only_unanswered_state_reads_are_retried(monkeypatch, missing):
    from collections import Counter

    now = [0.]
    calls, abandoned = Counter(), Counter()
    active = [False]
    controller = 'left_arm_controller'

    def client_for(_type, path):
        name = path.rsplit('/', 1)[-1]

        def request(message):
            calls[name] += 1
            if name == 'list_controllers':
                reply = NS(controller=[
                    NS(name='joint_state_broadcaster', state='active'),
                    NS(name=controller, state='active' if active[0] else 'inactive'),
                ])
                replied = missing != 'every_read' and not (missing == 'first_read' and calls[name] == 1)
            elif name == 'list_hardware_components':
                reply = NS(component=[NS(name='FeetechSystem', state=NS(label='inactive'))])
                replied = True
            elif name == 'set_hardware_component_state':
                reply = NS(ok=True, state=message.target_state)
                replied = missing != 'activation'
            else:
                active[0] = True
                reply, replied = NS(ok=True), True
            return NS(done=lambda: replied, result=lambda: reply)

        return NS(service_is_ready=lambda: True, call_async=request,
                  remove_pending_request=lambda _: abandoned.update([name]))

    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once',
                        lambda _, timeout_sec: now.__setitem__(0, now[0] + timeout_sec))
    node = NS(ready=lambda: True, create_client=client_for, destroy_client=lambda _: None,
              get_logger=lambda: NS(warning=lambda _: None))
    if missing == 'first_read':
        module.activate_cold_controllers(node, [controller], 12.)
        assert calls == dict(list_controllers=3, list_hardware_components=1,
                             set_hardware_component_state=1, switch_controller=1)
        assert abandoned == dict(list_controllers=1)
    else:
        service = 'list_controllers' if missing == 'every_read' else 'set_hardware_component_state'
        with pytest.raises(RuntimeError, match=service + ' reply'):
            module.activate_cold_controllers(node, [controller], 12.)
        if missing == 'every_read':
            assert calls == dict(list_controllers=3)
            assert 6. <= now[0] < 7.
        else:
            assert calls == dict(list_controllers=1, list_hardware_components=1,
                                 set_hardware_component_state=1)
        assert abandoned[service] == calls[service]
