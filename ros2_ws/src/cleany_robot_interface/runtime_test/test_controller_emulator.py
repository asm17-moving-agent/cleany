"""ROS controller + real C++ wire driver tests against isolated PTYs, never USB motors."""
import json
import math
import multiprocessing
import os
from pathlib import Path
import select
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

os.environ['ROS_DOMAIN_ID'] = '167'

import pytest
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger
import yaml

from cleany_robot_interface.control_profile import prepare_control_profile
from cleany_robot_interface.control_state_guard import ControlStateGuard
from rclpy.executors import SingleThreadedExecutor

ROOT = Path(__file__).resolve().parents[4]


class Servos:
    def __init__(self, elements):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)
        self.registers, self.writes = {}, []
        self.stall = False
        self.drop_reads = False
        self.drop_read_count = 0
        self.running = True
        self.read_count = 0
        self.load_offsets = {}  # encoder ticks of simulated static load error
        for element in elements:
            p = {v.get('name'): float(v.text) for v in element.findall('param')}
            motor = int(p['motor_id'])
            mem = bytearray(256)
            def set_value(address, value, size=2):
                mem[address:address+size] = int(value).to_bytes(size, 'little')
            set_value(3, p['model'])
            set_value(5, motor, 1)
            set_value(13, 80, 1)
            set_value(16, 1000)
            set_value(28, 250)
            set_value(63, 30, 1)
            set_value(62, 120, 1)
            if motor <= 12:
                set_value(9, p['range_min']); set_value(11, p['range_max'])
                h = int(p['homing_offset'])
                set_value(31, abs(h) | (0x800 if h < 0 else 0))
                bounds = sorted(p['zero_tick'] + p['direction']*(v-p['offset_rad'])*4096/math.tau
                                for v in (p['lower'], p['upper']))
                raw = round((max(bounds[0], p['range_min']) + min(bounds[1], p['range_max']))/2)
            else:
                raw = round(p['zero_tick'])
            set_value(56, raw)
            self.registers[motor] = mem
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def update(self, motor, address, data):
        self.writes.append({'id': motor, 'address': address, 'data': list(data)})
        mem = self.registers[motor]
        mem[address:address+len(data)] = data
        if mem[40] == 1 and not self.stall:
            goal = int.from_bytes(mem[42:44], 'little')
            measured = goal-self.load_offsets.get(motor, 0)
            mem[56:58] = measured.to_bytes(2, 'little')

    def run(self):
        buffer = bytearray()
        while self.running:
            if not select.select([self.master], [], [], .02)[0]:
                continue
            try:
                buffer.extend(os.read(self.master, 4096))
            except OSError:
                continue
            while len(buffer) >= 4:
                if buffer[:2] != b'\xff\xff':
                    del buffer[0]
                    continue
                size = buffer[3] + 4
                if len(buffer) < size:
                    break
                packet = buffer[:size]
                del buffer[:size]
                assert sum(packet[2:]) & 255 == 255
                motor, instruction, params = packet[2], packet[4], packet[5:-1]
                result = b''
                if instruction == 2:
                    self.read_count += 1
                    if self.drop_reads:
                        continue
                    if self.drop_read_count > 0:
                        self.drop_read_count -= 1
                        continue
                    address, length = params
                    result = bytes(self.registers[motor][address:address+length])
                elif instruction == 3:
                    self.update(motor, params[0], params[1:])
                elif instruction == 131:
                    address, width = params[:2]
                    for i in range(2, len(params), width+1):
                        self.update(params[i], address, params[i+1:i+1+width])
                    continue
                else:
                    raise AssertionError('Unexpected instruction')
                response = bytearray([255, 255, motor, len(result)+2, 0]) + result
                response.append(~sum(response[2:]) & 255)
                os.write(self.master, response)

    def close(self):
        self.running = False
        self.thread.join(1)
        os.close(self.master); os.close(self.slave)


class ProcessServos(Servos):
    """Keep the PTY responder independent of the Python ROS executor's GIL."""

    def __init__(self, elements):
        self.context = multiprocessing.get_context('fork')
        self._running = self.context.RawValue('b', False)
        self._drop_reads = self.context.RawValue('b', False)
        self._drop_read_count = self.context.RawValue('i', 0)
        self._read_count = self.context.RawValue('i', 0)
        super().__init__(elements)
        self.running = False
        self.thread.join(1)
        assert not self.thread.is_alive()
        self.registers = {motor: self.context.RawArray('B', values)
                          for motor, values in self.registers.items()}
        self.wire_records = self.context.Queue()
        self.running = True
        self.process = self.context.Process(target=self._serve, daemon=True)
        self.process.start()

    @property
    def running(self):
        return bool(self._running.value)

    @running.setter
    def running(self, value):
        self._running.value = value

    @property
    def drop_reads(self):
        return bool(self._drop_reads.value)

    @drop_reads.setter
    def drop_reads(self, value):
        self._drop_reads.value = value

    @property
    def drop_read_count(self):
        return self._drop_read_count.value

    @drop_read_count.setter
    def drop_read_count(self, value):
        self._drop_read_count.value = value

    @property
    def read_count(self):
        return self._read_count.value

    @read_count.setter
    def read_count(self, value):
        self._read_count.value = value

    def _serve(self):
        try:
            self.run()
        finally:
            self.wire_records.put(self.writes)

    def close(self):
        self.running = False
        self.writes = self.wire_records.get(timeout=5)
        self.process.join(2)
        assert not self.process.is_alive()
        self.wire_records.close()
        os.close(self.master)
        os.close(self.slave)


class Runtime:
    def __init__(self, directory, read_only, control_file=None, shared_bus=False,
                 serial_timeout_ms=100, max_state_age_s=.5, serial_read_retries=0,
                 serial_read_recovery_budget_ms=60, servo_type=Servos, start_controllers=True):
        self.directory, self.read_only = directory, read_only
        state = json.loads((ROOT/'configs/robot/camera_preview.json').read_text())
        arm = yaml.safe_load((ROOT/state['arm_config']).read_text())
        arm['calibration_dir'] = str(
            ROOT/'ros2_ws/src/cleany_description/config/calibration/jetson_20261009/motors')
        arm_file = directory/'synthetic_arm.yaml'
        arm_file.write_text(yaml.safe_dump(arm))
        state['arm_config'] = str(arm_file)
        state_file = directory/'synthetic_state.json'
        state_file.write_text(json.dumps(state))
        self.profile = prepare_control_profile(state_file, ROOT,
            directory/'profile', control_file or ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml', read_only)
        tree = ET.parse(self.profile.model)
        elements = tree.findall('ros2_control/joint')
        self.arms = servo_type(elements if shared_bus else elements[:12])
        self.head = self.arms if shared_bus else servo_type(elements[12:])
        self.process = None
        try:
            hardware = tree.find('ros2_control/hardware')
            hardware.find("param[@name='arm_port']").text = self.arms.path
            hardware.find("param[@name='head_port']").text = self.head.path
            # PTYs remain responsive despite non-realtime Python/CI scheduling.
            hardware.find("param[@name='serial_timeout_ms']").text = str(serial_timeout_ms)
            hardware.find("param[@name='max_state_age_s']").text = str(max_state_age_s)
            # Legacy controller trials use a 100ms single-attempt allowance.
            # Recovery trials explicitly use the real 20ms/60ms retry profile.
            hardware.find("param[@name='serial_read_retries']").text = str(serial_read_retries)
            hardware.find("param[@name='serial_read_recovery_budget_ms']").text = str(serial_read_recovery_budget_ms)
            config = yaml.safe_load(self.profile.controllers.read_text())
            config['controller_manager']['ros__parameters']['robot_description'] = ET.tostring(tree.getroot(), encoding='unicode')
            parameters = directory/'manager.yaml'
            parameters.write_text(yaml.safe_dump(config))
            self.log = (directory/'manager.log').open('w')
            self.process = subprocess.Popen(['ros2', 'run', 'controller_manager', 'ros2_control_node',
                '--ros-args', '--params-file', str(parameters)], stdout=self.log, stderr=subprocess.STDOUT,
                start_new_session=True)
            rclpy.init()
            self.node = rclpy.create_node('controller_emulator_probe')
            self.guard = ControlStateGuard()
            self.guard.controllers = list(self.profile.controller_names)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor.add_node(self.guard)
            self.states = []
            self.dynamic_states = []
            self.subscription = self.node.create_subscription(JointState, '/joint_states', self.states.append, 10)
            self.dynamic_subscription = self.node.create_subscription(DynamicJointState,
                '/joint_state_broadcaster/dynamic_joint_states', self.dynamic_states.append, 10)
            self.list = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
            assert self.list.wait_for_service(timeout_sec=15), (directory/'manager.log').read_text()
            spawners = [(['joint_state_broadcaster'], [])]
            if start_controllers:
                spawners.append((list(self.profile.controller_names),
                                 ['--inactive'] if read_only else ['--activate-as-group']))
            for names, options in spawners:
                result = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', *names,
                                         '--controller-manager-timeout', '20', *options],
                                        text=True, capture_output=True, timeout=35)
                assert result.returncode == 0, result.stdout + result.stderr + (directory/'manager.log').read_text()
            self.spin(.3)
            assert self.states
            self.clients = {name: ActionClient(self.node, FollowJointTrajectory, f'/{name}/follow_joint_trajectory')
                            for name in self.profile.controller_names if start_controllers}
            assert all(c.wait_for_server(timeout_sec=5) for c in self.clients.values())
        except BaseException:
            self.close()
            raise

    def spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def wait(self, future, timeout=10):
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        assert future.done(), (self.directory/'manager.log').read_text()[-4000:]
        return future.result()

    def goal(self, name, delta=.03, duration=2, bad_joint=False):
        config = yaml.safe_load(self.profile.controllers.read_text())[name]['ros__parameters']
        joints = config['joints']
        self.spin(.1)
        actual = dict(zip(self.states[-1].name, self.states[-1].position))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['wrong_joint'] if bad_joint else joints
        for seconds, shift in ((0, 0), (duration, delta)):
            point = JointTrajectoryPoint()
            point.positions = [actual[j]+shift for j in joints]
            point.velocities = [0.0]*len(joints)
            point.accelerations = [0.0]*len(joints)
            point.time_from_start.sec = seconds
            goal.trajectory.points.append(point)
        feedback = []
        future = self.clients[name].send_goal_async(goal, feedback_callback=feedback.append)
        return self.wait(future), feedback

    def close(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        if hasattr(self, 'node'):
            self.node.destroy_node()
        if hasattr(self, 'guard'):
            self.guard.destroy_node()
            self.executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        self.arms.close()
        if self.head is not self.arms:
            self.head.close()
        if hasattr(self, 'log'):
            self.log.close()
        (self.directory/'wire.json').write_text(json.dumps({
            'read_only': self.read_only, 'arm_reads': self.arms.read_count, 'head_reads': self.head.read_count,
            'arm_writes': self.arms.writes, 'head_writes': self.head.writes}, indent=2))


class PoweredHoldServos(Servos):
    """Already powered with a small static load error at the right elbow."""
    goal_offset = 6
    mismatched_ram = False

    def __init__(self, elements):
        super().__init__(elements)
        self.initial_goals = {}
        for element in elements:
            values = {v.get('name'): float(v.text) for v in element.findall('param')}
            motor = int(values['motor_id'])
            if motor > 12:
                continue
            mem = self.registers[motor]
            raw = int.from_bytes(mem[56:58], 'little')
            offset = self.goal_offset if motor == 9 else 0
            goal = raw + offset
            self.initial_goals[motor] = goal
            self.load_offsets[motor] = offset
            for address, size, value in (
                    (40, 1, 1), (41, 1, int(values['acceleration_raw'])),
                    (42, 2, goal), (44, 2, 0), (46, 2, int(values['speed_raw'])),
                    (48, 2, int(values['torque_limit_raw']))):
                mem[address:address+size] = value.to_bytes(size, 'little')
            if self.mismatched_ram and motor == 9:
                mem[48:50] = (1).to_bytes(2, 'little')


@pytest.mark.parametrize('start_controllers', [False, True])
def test_powered_recovery_preserves_loaded_hold_without_reinitializing_ram(tmp_path, start_controllers):
    runtime = Runtime(tmp_path, False, servo_type=PoweredHoldServos,
                      start_controllers=start_controllers)
    try:
        runtime.spin(.3)
        assert 'Adopted powered hold' in (tmp_path/'manager.log').read_text()
        assert not any(w['address'] in (40, 41, 44, 46, 48) for w in runtime.arms.writes)
        for motor, goal in runtime.arms.initial_goals.items():
            assert int.from_bytes(runtime.arms.registers[motor][42:44], 'little') == goal
        if not start_controllers:
            assert not runtime.arms.writes  # no idle loop goal reset before JTC claims joints
        else:
            handle, _ = runtime.goal('right_arm_controller', delta=.03, duration=1)
            assert handle.accepted
            assert runtime.wait(handle.get_result_async()).result.error_code == 0
            runtime.spin(.1)
            next_stop_raw = int.from_bytes(runtime.arms.registers[9][56:58], 'little')
        assert not runtime.head.writes
    finally:
        runtime.close()
    final_goal = int.from_bytes(runtime.arms.registers[9][42:44], 'little')
    if start_controllers:
        assert final_goal == next_stop_raw  # new motion clears the startup stop anchor
    else:
        assert final_goal == runtime.arms.initial_goals[9]


@pytest.mark.parametrize('fault', ['ram', 'distant_goal'])
def test_powered_recovery_rejects_incompatible_existing_hold(tmp_path, fault):
    class UnsafeHoldServos(PoweredHoldServos):
        mismatched_ram = fault == 'ram'
        goal_offset = 33 if fault == 'distant_goal' else 6

    with pytest.raises(AssertionError):
        Runtime(tmp_path, False, servo_type=UnsafeHoldServos, start_controllers=False)
    log = (tmp_path/'manager.log').read_text()
    expected = 'powered recovery RAM mismatch' if fault == 'ram' else 'powered recovery goal is not holding'
    assert expected in log
    wire = json.loads((tmp_path/'wire.json').read_text())
    assert not any(w['address'] in (40, 41, 44, 46, 48) for w in wire['arm_writes'])


def test_repeated_stop_and_shutdown_do_not_relatch_each_sagged_position(tmp_path):
    runtime = Runtime(tmp_path, False)
    try:
        runtime.arms.load_offsets[9] = 6
        runtime.spin(.2)
        stop = runtime.node.create_client(Trigger, '/real_control/stop')
        assert stop.wait_for_service(timeout_sec=5)
        assert runtime.wait(stop.call_async(Trigger.Request())).success
        runtime.spin(.6)
        assert runtime.guard.actions_cancelled and runtime.guard.stop_confirmed
        goal = int.from_bytes(runtime.arms.registers[9][42:44], 'little')
        writes = len(runtime.arms.writes)
        runtime.spin(.2)
        assert int.from_bytes(runtime.arms.registers[9][42:44], 'little') == goal
        assert len(runtime.arms.writes) == writes
    finally:
        runtime.close()
    assert int.from_bytes(runtime.arms.registers[9][42:44], 'little') == goal
    assert not any(w['address'] == 42 for w in runtime.arms.writes[writes:])


def test_readonly_controllers_cannot_activate_or_execute(tmp_path):
    runtime = Runtime(tmp_path, True)
    try:
        status = runtime.wait(runtime.list.call_async(ListControllers.Request()))
        assert {c.state for c in status.controller if c.name in runtime.clients} == {'inactive'}
        for name in runtime.clients:
            handle, _ = runtime.goal(name)
            assert not handle.accepted
        switch = runtime.node.create_client(SwitchController, '/controller_manager/switch_controller')
        assert switch.wait_for_service(timeout_sec=5)
        request = SwitchController.Request()
        request.activate_controllers = list(runtime.clients)
        request.strictness = 2
        request.timeout.sec = 2
        assert not runtime.wait(switch.call_async(request)).ok
        assert len(runtime.states[-1].name) == 14
        assert not runtime.states[-1].velocity and not runtime.states[-1].effort
        assert not runtime.arms.writes and not runtime.head.writes
    finally:
        runtime.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_shared_bus_selected_arm_only(tmp_path, side):
    """14 IDs on one real wire driver: five writable motors, no head/gripper writes."""
    config = yaml.safe_load((ROOT / f'ros2_ws/src/cleany_robot_interface/test/data/feetech_operational_{side}_arm.yaml').read_text())
    config['command_joints'] = [j for j in config['command_joints'] if not j.endswith('_gripper_joint')]
    path = tmp_path / 'selected.yaml'
    path.write_text(yaml.safe_dump(config))
    runtime = Runtime(tmp_path, False, path, shared_bus=True)
    try:
        assert runtime.arms is runtime.head
        assert set(runtime.arms.registers) == set(range(1, 15))
        assert set(runtime.clients) == {f'{side}_arm_controller'}
        handle, feedback = runtime.goal(f'{side}_arm_controller')
        assert handle.accepted
        result = runtime.wait(handle.get_result_async())
        assert result.status == 4 and result.result.error_code == 0
        assert feedback and len(runtime.states[-1].name) == 14
        expected = set(range(1, 6)) if side == 'left' else set(range(7, 12))
        assert {w['id'] for w in runtime.arms.writes} == expected
        assert {w['address'] for w in runtime.arms.writes} <= {40, 41, 42, 44, 46, 48}
        for motor in expected:
            assert int.from_bytes(runtime.arms.registers[motor][46:48], 'little') == 800
            assert int.from_bytes(runtime.arms.registers[motor][48:50], 'little') == 800
            assert runtime.arms.registers[motor][41] == 254
    finally:
        runtime.close()


def test_shared_bus_readonly_has_no_writes(tmp_path):
    runtime = Runtime(tmp_path, True, shared_bus=True)
    try:
        assert len(runtime.states[-1].name) == 14
        assert not runtime.arms.writes
        handle, _ = runtime.goal('left_arm_controller')
        assert not handle.accepted
        assert not runtime.arms.writes
    finally:
        runtime.close()


def test_single_read_timeout_holds_and_recovers_but_persistent_loss_stops(tmp_path):
    runtime = Runtime(tmp_path, False, shared_bus=True, serial_timeout_ms=20,
                      max_state_age_s=.1, serial_read_retries=1)
    try:
        handle, _ = runtime.goal('left_arm_controller', delta=.03, duration=2)
        assert handle.accepted
        runtime.arms.drop_read_count = 1
        runtime.spin(.3)
        text = (tmp_path/'manager.log').read_text()
        assert 'Bounded READ recovery' in text
        assert 'LATCHED hardware fault' not in text
        counts = [dict(zip(row.interface_names, row.values)).get('communication_recovery_count')
            for message in runtime.dynamic_states for row in message.interface_values]
        assert 1. in counts
        result = runtime.wait(handle.get_result_async())
        assert result.status == 4 and result.result.error_code == 0
        assert all(runtime.arms.registers[motor][40] == 1 for motor in range(1, 13))
        assert not any(w['address'] == 40 and w['data'] == [0] for w in runtime.arms.writes)
        runtime.arms.drop_reads = True
        runtime.spin(.6)
        assert 'LATCHED hardware fault' in (tmp_path/'manager.log').read_text()
        assert runtime.guard.fault
    finally:
        runtime.close()




def test_trajectory_success_cancel_reject_and_latched_disconnect(tmp_path):
    runtime = Runtime(tmp_path, False)
    try:
        handles = [runtime.goal(name) for name in runtime.clients]
        for handle, feedback in handles:
            assert handle.accepted
            result = runtime.wait(handle.get_result_async())
            assert result.status == 4 and result.result.error_code == 0
            assert feedback
        handle, _ = runtime.goal('left_arm_controller', bad_joint=True)
        assert not handle.accepted
        handle, _ = runtime.goal('right_arm_controller', delta=.02, duration=4)
        assert handle.accepted
        runtime.spin(.3)
        assert runtime.wait(handle.cancel_goal_async()).goals_canceling
        assert runtime.wait(handle.get_result_async()).status == 5
        runtime.spin(.2)
        assert 'LATCHED hardware fault' not in (tmp_path/'manager.log').read_text()
        assert not runtime.head.writes
        assert all(w['address'] in (40, 41, 42, 44, 46, 48) for w in runtime.arms.writes)
        # A bus fault must latch: reconnecting alone may not resume target streaming.
        handle, _ = runtime.goal('left_arm_controller', delta=.02, duration=4)
        assert handle.accepted
        runtime.arms.drop_reads = True
        runtime.spin(1.5)
        runtime.arms.drop_reads = False
        runtime.spin(1.5)
        assert runtime.wait(handle.get_result_async()).status in (5, 6)
        assert runtime.guard.fault and runtime.guard.actions_cancelled and runtime.guard.stop_confirmed
        assert all(math.isfinite(value) for value in runtime.states[-1].position)
        before = len(runtime.arms.writes)
        runtime.spin(.3)
        assert len(runtime.arms.writes) == before
        assert 'LATCHED hardware fault' in (tmp_path/'manager.log').read_text()
    finally:
        runtime.close()


@pytest.mark.parametrize('fault', ['tracking', 'software_stop'])
def test_stalled_goal_and_software_stop_return_failure(tmp_path, fault):
    runtime = Runtime(tmp_path, False)
    try:
        if fault == 'tracking':
            runtime.arms.stall = True
            # In-motion path tolerance is disabled for the supervised profile;
            # a stalled joint must fail the final goal tolerance instead.
            handle, _ = runtime.goal('left_arm_controller', delta=.15, duration=3)
            assert handle.accepted
            result = runtime.wait(handle.get_result_async())
            assert result.status == 6
            assert result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        else:
            handle, _ = runtime.goal('right_arm_controller', delta=.02, duration=4)
            assert handle.accepted
            stop = runtime.node.create_client(Trigger, '/real_control/stop')
            assert stop.wait_for_service(timeout_sec=5)
            assert runtime.wait(stop.call_async(Trigger.Request())).success
            assert runtime.wait(handle.get_result_async()).status in (5, 6)
            runtime.spin(.3)
            assert runtime.guard.actions_cancelled and runtime.guard.stop_confirmed
            switch = runtime.node.create_client(SwitchController, '/controller_manager/switch_controller')
            assert switch.wait_for_service(timeout_sec=5)
            request = SwitchController.Request()
            request.activate_controllers = list(runtime.clients)
            request.strictness = 2
            request.timeout.sec = 2
            assert not runtime.wait(switch.call_async(request)).ok
        assert not runtime.head.writes
    finally:
        runtime.close()


def test_single_wrist_trial_never_writes_other_motors(tmp_path):
    runtime = Runtime(tmp_path, False, ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_commissioning_left_wrist.yaml')
    try:
        handle, _ = runtime.goal('left_arm_controller', delta=math.radians(2), duration=3)
        assert handle.accepted
        result = runtime.wait(handle.get_result_async())
        assert result.status == 4 and result.result.error_code == 0
        handle, _ = runtime.goal('left_arm_controller', delta=-math.radians(2), duration=3)
        assert handle.accepted
        assert runtime.wait(handle.get_result_async()).status == 4
        assert 'LATCHED hardware fault' not in (tmp_path/'manager.log').read_text()
        assert {w['id'] for w in runtime.arms.writes} == {5}
        assert not runtime.head.writes
        assert runtime.arms.registers[5][40] == 1
        assert all(mem[40] == 0 for motor, mem in runtime.arms.registers.items() if motor != 5)
    finally:
        runtime.close()


@pytest.mark.parametrize('terminal', ['goal_timeout', 'cancel'])
def test_lagging_wrist_hold_does_not_fault_and_software_stop_completes(tmp_path, terminal):
    runtime = Runtime(tmp_path, False, ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_commissioning_left_wrist.yaml')
    try:
        # Error stays below the path tolerance but exceeds the final goal tolerance.
        # Real first trial similarly stopped short; an ideal servo hid the hold jump.
        runtime.arms.stall = True
        handle, _ = runtime.goal('left_arm_controller', delta=.02, duration=3)
        assert handle.accepted
        if terminal == 'cancel':
            runtime.spin(2.5)
            assert runtime.wait(handle.cancel_goal_async()).goals_canceling
        result = runtime.wait(handle.get_result_async())
        if terminal == 'goal_timeout':
            assert result.status == 6
            assert result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        else:
            assert result.status == 5
        runtime.spin(.3)
        assert 'LATCHED hardware fault' not in (tmp_path/'manager.log').read_text()
        assert int.from_bytes(runtime.arms.registers[5][42:44], 'little') == int.from_bytes(
            runtime.arms.registers[5][56:58], 'little')
        stop = runtime.node.create_client(Trigger, '/real_control/stop')
        assert stop.wait_for_service(timeout_sec=5)
        assert runtime.wait(stop.call_async(Trigger.Request())).success
        runtime.spin(.5)
        assert runtime.guard.stop_confirmed
        status = runtime.wait(runtime.list.call_async(ListControllers.Request()))
        assert next(c.state for c in status.controller if c.name == 'left_arm_controller') == 'inactive'
        assert {w['id'] for w in runtime.arms.writes} == {5}
        assert not runtime.head.writes
    finally:
        runtime.close()


def test_left_arm_profile_holds_gripper_and_never_writes_right_or_head(tmp_path):
    runtime = Runtime(tmp_path, False, ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_commissioning_left_arm.yaml')
    try:
        gripper_start = bytes(runtime.arms.registers[6][56:58])
        handle, _ = runtime.goal('left_arm_controller', delta=.01, duration=2)
        assert handle.accepted
        result = runtime.wait(handle.get_result_async())
        assert result.status == 4 and result.result.error_code == 0
        assert {w['id'] for w in runtime.arms.writes} == set(range(1,7))
        assert not runtime.head.writes
        assert all(mem[40] == (1 if motor <= 6 else 0)
                   for motor, mem in runtime.arms.registers.items())
        assert all(bytes(w['data']) == gripper_start for w in runtime.arms.writes
                   if w['id'] == 6 and w['address'] == 42)
        assert 'LATCHED hardware fault' not in (tmp_path/'manager.log').read_text()
    finally:
        runtime.close()




def test_readonly_gripper_telemetry_uses_existing_read_packets(tmp_path):
    from control_msgs.msg import DynamicJointState
    runtime = Runtime(tmp_path, True)
    try:
        runtime.arms.registers[6][60:62] = (321).to_bytes(2,'little')
        runtime.arms.registers[12][60:62] = (1124).to_bytes(2,'little')
        runtime.arms.registers[6][69:71] = (18).to_bytes(2,'little')
        runtime.arms.registers[12][69:71] = (0x8000 | 20).to_bytes(2,'little')
        messages = []
        subscription = runtime.node.create_subscription(DynamicJointState,
            '/joint_state_broadcaster/dynamic_joint_states', messages.append, 10)
        runtime.spin(.5)
        assert messages
        data = {n:dict(zip(v.interface_names,v.values)) for n,v in
                zip(messages[-1].joint_names,messages[-1].interface_values)}
        assert data['left_gripper_joint']['load_raw']==321
        assert data['right_gripper_joint']['load_raw']==1124
        assert data['left_gripper_joint']['current_raw'] == 18
        assert data['right_gripper_joint']['current_raw'] == 20
        assert data['right_gripper_joint']['load_magnitude_raw'] == 100
        for side in ('left','right'):
            row=data[side+'_gripper_joint']
            assert row['torque_enabled']==0 and row['motor_status']==0 and row['temperature']==30
            assert math.isfinite(row['position'])
            assert row['current_limit_raw'] == 250
            assert row['protocol_error_code'] == 0
            assert row['moving_flag'] == 0
            assert row['temperature_raw'] == 30
            assert row['motor_status_raw'] == 0
            assert 0 <= runtime.node.get_clock().now().nanoseconds-row['feedback_stamp_ns'] < .5e9
        for name, row in data.items():
            if name.startswith('head_'):
                continue
            assert row['voltage_raw'] == 120
            assert row['goal_position_raw'] == 0
            assert math.isnan(row['sent_goal_raw'])  # no write in readonly mode
            assert row['torque_enabled'] == 0
        assert runtime.arms.writes==[] and runtime.head.writes==[]
        runtime.node.destroy_subscription(subscription)
    finally:
        runtime.close()


@pytest.mark.parametrize('timing', ['future_header', 'immediate', 'explicit_hold'])
def test_loaded_arm_action_boundary_preserves_wire_goal_without_future_header(tmp_path, timing):
    """Use the installed JTC, actual driver, and fake serial motors, never USB.

    The legacy future-header case is a positive reproduction control: the
    other two must not reset the wire target to the sagged measured position.
    """
    runtime = Runtime(tmp_path, False)
    try:
        runtime.arms.load_offsets[2] = 20
        first, _ = runtime.goal('left_arm_controller', delta=.02, duration=1)
        assert first.accepted
        assert runtime.wait(first.get_result_async()).result.error_code == 0
        runtime.spin(.2)
        config = yaml.safe_load(runtime.profile.controllers.read_text())
        names = config['left_arm_controller']['ros__parameters']['joints']
        elements = ET.parse(runtime.profile.model).findall('ros2_control/joint')
        held = {}
        for element in elements[:5]:
            p = {v.get('name'): float(v.text) for v in element.findall('param')}
            raw = int.from_bytes(runtime.arms.registers[int(p['motor_id'])][42:44], 'little')
            held[element.get('name')] = p['offset_rad']+p['direction']*(raw-p['zero_tick'])*math.tau/4096
        held_raw = int.from_bytes(runtime.arms.registers[2][42:44], 'little')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        now_ns = runtime.node.get_clock().now().nanoseconds
        if timing != 'immediate':
            epoch = now_ns+(800_000_000 if timing == 'future_header' else 0)
            goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = divmod(epoch, 10**9)
        for ns in (0, 800_000_000, 1_200_000_000):
            point = JointTrajectoryPoint(positions=[held[n] for n in names], velocities=[0.]*5)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            goal.trajectory.points.append(point)
        offset = len(runtime.arms.writes)
        handle = runtime.wait(runtime.clients['left_arm_controller'].send_goal_async(goal))
        assert handle.accepted
        assert runtime.wait(handle.get_result_async()).result.error_code == 0
        wire = [int.from_bytes(bytes(w['data']), 'little')
                for w in runtime.arms.writes[offset:] if w['id'] == 2 and w['address'] == 42]
        assert wire
        if timing == 'future_header':
            assert min(wire) <= held_raw-10  # reproduces the old hidden reset
        else:
            assert all(abs(value-held_raw) <= 1 for value in wire)
        assert runtime.arms.registers[2][40] == 1
    finally:
        runtime.close()


def test_executor_cancels_and_reconnects_remaining_path_after_read_recovery(tmp_path):
    from types import SimpleNamespace as NS
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    from cleany_skill_executor.core.hardware_execution import physical_limits
    from cleany_skill_executor.core.grasp_selection import ARM_JOINT_NAMES
    from trajectory_msgs.msg import JointTrajectory

    runtime = Runtime(tmp_path, False, shared_bus=True, serial_timeout_ms=20,
                      max_state_age_s=.1, serial_read_retries=1,
                      servo_type=ProcessServos)
    stop_spin = threading.Event()
    spinner = None
    injector = None
    try:
        node = runtime.node
        execution = HardwareExecution(node)
        def measured():
            from cleany_skill_executor.pregrasp_pipeline import check_joint_snapshot
            state = runtime.states[-1]
            check_joint_snapshot(state, node.get_clock().now().nanoseconds)
            return state
        node.measured = measured
        # Scene receipts and collision-service replies are synthetic. ROS Actions,
        # PTY packets, C++ driver recovery and DynamicJointState are real code.
        node.call = lambda *_, **__: NS(valid=True, contacts=[])
        node.scene_monitor = NS(wait=lambda **_: None)
        execution.verify = lambda arm: physical_limits(runtime.profile.model.read_text(), arm)
        # DDS client destruction can monopolize Python's GIL and starve this
        # same-process PTY responder. Reuse real clients until runtime shutdown.
        def create_motion_client(action_type, name):
            controller = name.strip('/').split('/')[0]
            client = runtime.clients[controller]
            return NS(wait_for_server=client.wait_for_server,
                      send_goal_async=client.send_goal_async, destroy=lambda: None)
        node.create_motion_client = create_motion_client

        def wait(future, timeout, *, monitor=False, **_):
            deadline = time.monotonic()+timeout
            while not future.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError('test ROS operation timed out')
                if monitor or getattr(node, 'carrying', None) is not None:
                    execution.monitor()
                time.sleep(.005)
            return future.result()

        node.wait = wait
        submitted = []
        injected = threading.Event()

        def interrupt_moving_arm():
            deadline = time.monotonic()+3.
            while not stop_spin.is_set() and time.monotonic() < deadline:
                measured = dict(zip(runtime.states[-1].name, runtime.states[-1].position))
                if abs(measured[names[0]]-start[0]) >= .006:
                    runtime.arms.drop_read_count = 1
                    injected.set()
                    return
                time.sleep(.005)

        def action(client, goal, timeout, **options):
            nonlocal injector
            submitted.append(goal)
            if len(submitted) == 1:
                injector = threading.Timer(.2, interrupt_moving_arm)
                injector.start()
            return PregraspPipeline.run_action(node, client, goal, timeout, **options)
        node.run_action = action
        node.real = execution
        node.carrying = object()  # Match Jetson's monitoring during carry/cancel.

        def spin():
            while not stop_spin.is_set():
                runtime.executor.spin_once(timeout_sec=.005)
        spinner = threading.Thread(target=spin, daemon=True)
        spinner.start()
        time.sleep(.1)
        names = ARM_JOINT_NAMES['left']
        q = dict(zip(runtime.states[-1].name, runtime.states[-1].position))
        start = [q[name] for name in names]
        target = [start[0]+.05, *start[1:]]
        trajectory = JointTrajectory(joint_names=list(names), points=[
            JointTrajectoryPoint(positions=start), JointTrajectoryPoint(positions=target)])
        actual = execution.execute('left', trajectory, 'arm', terminal_check=False)
        result = dict(zip(actual.name, actual.position))
        assert result[names[0]] == pytest.approx(target[0], abs=.01)
        assert injected.is_set()
        assert len(submitted) == 2
        assert execution.last_execution_metrics['communication_resumes'][0]['generation'] == 1
        resumed_start = list(submitted[1].trajectory.points[0].positions)
        assert abs(resumed_start[0]-start[0]) >= .003
        assert resumed_start[-1] == pytest.approx(start[-1], abs=.003)
        assert all(runtime.arms.registers[motor][40] == 1 for motor in range(1, 13))
        assert 'LATCHED hardware fault' not in (tmp_path/'manager.log').read_text()
    finally:
        if injector is not None:
            injector.cancel()
            injector.join(1)
        stop_spin.set()
        if spinner is not None:
            spinner.join(2)
        runtime.close()


@pytest.mark.parametrize('side,motor,closed', [('left',6,1358), ('right',12,1372)])
def test_contact_close_keeps_final_goal_and_raw_evidence_distinguishes_empty(tmp_path, side, motor, closed):
    from control_msgs.msg import DynamicJointState, JointTolerance
    from cleany_skill_executor.core.gripper_contact import (
        ContactState, LiveContactHistory, load_contact_criteria,
    )
    runtime = Runtime(tmp_path, False)
    try:
        history = LiveContactHistory(load_contact_criteria(
            ROOT/'configs/robot/gripper_contact_decision.yaml')[side])
        joint = f'{side}_gripper_joint'
        def feedback(message):
            row = message.interface_values[message.joint_names.index(joint)]
            history.observe(dict(zip(row.interface_names, row.values)))
        subscription = runtime.node.create_subscription(DynamicJointState,
            '/joint_state_broadcaster/dynamic_joint_states', feedback, 10)
        mem = runtime.arms.registers[motor]
        assert int.from_bytes(mem[48:50], 'little') == 200
        runtime.arms.load_offsets[motor] = -340
        mem[60:62] = (0x400 | 200).to_bytes(2, 'little')
        mem[69:71] = (0x8000 | 18).to_bytes(2, 'little')
        element = ET.parse(runtime.profile.model).find(f"ros2_control/joint[@name='{joint}']")
        p = {v.get('name'):float(v.text) for v in element.findall('param')}
        target = p['offset_rad']+p['direction']*(closed-p['zero_tick'])*math.tau/4096
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint]
        start = runtime.states[-1].position[runtime.states[-1].name.index(joint)]
        goal.trajectory.points = [JointTrajectoryPoint(positions=[start]),
                                  JointTrajectoryPoint(positions=[target])]
        goal.trajectory.points[-1].time_from_start.sec = 2
        goal.goal_tolerance = [JointTolerance(name=joint, position=-1.)]
        handle = runtime.wait(runtime.clients[f'{side}_gripper_controller'].send_goal_async(goal))
        assert handle.accepted
        result = runtime.wait(handle.get_result_async())
        assert result.status == 4 and result.result.error_code == 0
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            runtime.spin(.05)
            decision = history.evaluate(now_ns=runtime.node.get_clock().now().nanoseconds)
            if decision.state == ContactState.CONTACT:
                break
        assert decision.state == ContactState.CONTACT, decision
        assert int.from_bytes(mem[42:44], 'little') == closed
        assert int.from_bytes(mem[56:58], 'little') == closed+340
        # Preserve the command while the encoder and electrical feedback become empty.
        runtime.arms.load_offsets[motor] = -6
        mem[56:58] = (closed+6).to_bytes(2, 'little')
        mem[69:71] = (1).to_bytes(2, 'little')
        mem[60:62] = (36).to_bytes(2, 'little')
        runtime.spin(.6)
        decision = history.evaluate(now_ns=runtime.node.get_clock().now().nanoseconds)
        assert decision.state == ContactState.EMPTY, decision
        assert int.from_bytes(mem[42:44], 'little') == closed
        # A high current is confirmed and latches the driver fault, without dropping torque.
        mem[69:71] = (250).to_bytes(2, 'little')
        runtime.spin(.4)
        assert 'current protection bound' in (tmp_path/'manager.log').read_text()
        assert mem[40] == 1
        assert history.evaluate(now_ns=runtime.node.get_clock().now().nanoseconds).state == ContactState.UNKNOWN
        runtime.node.destroy_subscription(subscription)
    finally:
        runtime.close()
