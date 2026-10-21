"""Device-free failures, source reuse, visual measurement and command/report tests."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import threading
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from cleany_robot_interface.core import JointMapping
from cleany_robot_interface.tcp_waypoints import Dataset, digest, validate_control_contract
from tcp_waypoints_session import ROOT, BoardPreview, CameraMeasurement, TeachBus, create_session, register_gripper_reference
from tcp_waypoints_cli import closed_gripper, write_report
import tcp_waypoints_cli as cli


@pytest.fixture(params=['left', 'right'])
def session(tmp_path, request):
    """Uses the machine's frozen wrist calibration, not any camera or USB port."""
    config = json.loads((ROOT / 'configs/robot/tcp_waypoints.json').read_text())
    extrinsics = yaml.safe_load((ROOT / config['camera_extrinsics'][request.param]).read_text())
    if not Path(extrinsics['intrinsics_file']).exists():
        pytest.skip('Local wrist calibration inputs are not installed')
    config['operating_profiles'] = {side: str(ROOT / f'ros2_ws/src/cleany_robot_interface/test/data/feetech_operational_{side}_arm.yaml') for side in ('left', 'right')}
    config_path = tmp_path / 'synthetic_policy.json'
    config_path.write_text(json.dumps(config))
    return create_session(request.param, tmp_path / 'session', config_path)


def test_both_sessions_reuse_operational_limits_and_five_joint_scope(session):
    operating = yaml.safe_load(Path(session.manifest['operating_profile_source']).read_text())
    control = yaml.safe_load((session.root / 'control.yaml').read_text())
    assert control['joint_defaults'] == operating['joint_defaults']
    assert control['controller'] == operating['controller']
    assert len(session.command_joints) == 5
    assert all(j.startswith(session.side + '_') for j in session.command_joints)
    assert not any(j.endswith('_gripper_joint') for j in session.command_joints)
    assert not session.manifest['head_commands'] and not session.manifest['gripper_commands']
    assert session.manifest['fixed_board_required'] and not session.manifest['absolute_tcp_accuracy_verified']
    assert session.manifest['arm_port'] == session.manifest['head_port']
    session.verify()
    validate_control_contract(session)


def test_corrupted_command_scope_cannot_open_motors(session):
    tree = ET.parse(session.root / 'profile/robot_control.urdf')
    gripper = tree.find(f"ros2_control/joint[@name='{session.side}_gripper_joint']")
    ET.SubElement(gripper, 'command_interface', name='position')
    tree.write(session.root / 'profile/robot_control.urdf', encoding='unicode')
    with pytest.raises(ValueError, match='five-joint'):
        validate_control_contract(session)


class MemoryBus(TeachBus):
    def __init__(self, session):
        self.dataset = session
        self.control = yaml.safe_load((session.root / 'control.yaml').read_text())
        self.maps = [JointMapping(**row) for row in json.loads((session.root / 'mapping.json').read_text())]
        self.selected = [j for j in self.maps if j.name in session.command_joints]
        self.ids = {j.motor_id for j in self.selected}
        self.registers = {(j.motor_id, a): value for j in self.selected for a, value in
            ((33, 0), (63, 30), (13, 80), (16, 1000), (56, round((j.range_min + j.range_max)/2)), (40, 0), (42, 0))}
        self.writes = []

    def read_register(self, motor, address, size):
        return self.registers.get((motor, address), 0)

    def write_register(self, motor, address, size, value):
        # Preserve the production whitelist by supplying an SDK-shaped adapter.
        class Packet:
            def write1ByteTxRx(_, port, motor, address, value):
                self.writes.append((motor, address, value))
                self.registers[motor, address] = value
                return 0, 0
            write2ByteTxRx = write1ByteTxRx
        class Reader:
            packet = Packet()
            port = None
        self.reader = Reader()
        TeachBus.write_register(self, motor, address, size, value)


def test_hold_reuses_ram_settings_then_release_selected_only(session):
    bus = MemoryBus(session)
    bus.hold()
    assert bus.powered()
    settings = bus.control['joint_defaults']
    for joint in bus.selected:
        i = joint.motor_id
        assert bus.registers[i, 42] == bus.registers[i, 56]
        assert bus.registers[i, 46] == settings['speed_raw']
        assert bus.registers[i, 48] == settings['torque_limit_raw']
        assert bus.registers[i, 41] == settings['acceleration_raw']
    bus.release()
    assert not bus.powered()
    assert {row[0] for row in bus.writes} == bus.ids
    assert {row[1] for row in bus.writes} <= {40, 41, 42, 44, 46, 48}
    for motor, address in ((6, 40), (12, 40), (13, 40), (14, 42), (next(iter(bus.ids)), 13)):
        with pytest.raises(ValueError, match='Unauthorized'):
            bus.write_register(motor, address, 1 if address in (40, 13) else 2, 0)


@pytest.mark.parametrize('fault', ['temperature', 'mode', 'protection', 'moving_powered'])
def test_hold_failure_blocks_every_write(session, fault):
    bus = MemoryBus(session)
    motor = bus.selected[-1].motor_id
    if fault == 'temperature':
        bus.registers[motor, 63] = 80
    elif fault == 'mode':
        bus.registers[motor, 33] = 1
    elif fault == 'protection':
        bus.registers[motor, 16] = 100
    else:
        bus.registers[motor, 40] = 1
        bus.registers[motor, 42] = bus.registers[motor, 56] + 100
    with pytest.raises(ValueError):
        bus.hold()
    assert not bus.writes


def test_closed_tcp_rejects_open_gripper(session):
    q = {session.side + '_gripper_joint': session.manifest['closed_gripper_joint_rad']}
    closed_gripper(session, q)
    q[session.side + '_gripper_joint'] += .1
    with pytest.raises(ValueError, match='Close'):
        closed_gripper(session, q)
    q[session.side + '_gripper_joint'] = float('nan')
    with pytest.raises(ValueError, match='Close'):
        closed_gripper(session, q)


def gripper_observation(session, extra_ticks=16):
    maps = [JointMapping(**row) for row in json.loads((session.root / 'mapping.json').read_text())]
    q, joints = {}, {}
    for joint in maps:
        raw = round(joint.zero_tick) + (extra_ticks if joint.name == session.side + '_gripper_joint' else 0)
        q[joint.name] = joint.radians(raw)
        joints[joint.name] = {'id': joint.motor_id, 'raw_tick': raw}
    for i, name in enumerate(('head_pan_joint', 'head_tilt_joint')):
        q[name], joints[name] = 0., {'id': 13+i, 'raw_tick': 2048}
    stamp = time.monotonic_ns()
    return {'positions_rad': q, 'joints': joints, 'host_monotonic_start_ns': stamp,
            'host_monotonic_end_ns': stamp+1}


def test_physical_closed_reference_preserves_tolerance_and_freezes_before_points(session):
    original = session.manifest['closed_gripper_joint_rad']
    observation = gripper_observation(session)
    with pytest.raises(ValueError, match='Close'):
        closed_gripper(session, observation['positions_rad'])
    before = (session.root / 'session.json').read_bytes()
    with pytest.raises(ValueError, match='confirmation'):
        session.register_closed_gripper(observation)
    assert (session.root / 'session.json').read_bytes() == before
    observation.update(operator_confirmed_closed=True, stationary_checks_passed=True)
    session.register_closed_gripper(observation)
    loaded = Dataset.load(session.root)
    assert loaded.manifest['cad_closed_gripper_joint_rad'] == original
    assert loaded.manifest['closed_gripper_joint_rad'] == observation['positions_rad'][session.side + '_gripper_joint']
    assert not loaded.manifest['gripper_closed_reference']['motor_calibration_changed']
    assert json.loads((session.root / 'policy.json').read_text())['closed_gripper_tolerance_rad'] == .01
    closed_gripper(loaded, observation['positions_rad'])
    q = observation['positions_rad'].copy()
    q[session.side + '_gripper_joint'] += .02
    with pytest.raises(ValueError, match='Close'):
        closed_gripper(loaded, q)
    loaded.commit(1, {**observation, 'stationary_checks_passed': True, 'torque_hold_verified': True,
        'base_T_tcp': np.eye(4).tolist(), 'board_T_tcp': np.eye(4).tolist()}, b'image')
    before = (session.root / 'session.json').read_bytes()
    with pytest.raises(ValueError, match='before the first point'):
        loaded.register_closed_gripper(observation)
    assert (session.root / 'session.json').read_bytes() == before
    assert loaded.point(1)['session_sha256'] == digest(session.root / 'session.json')


def test_moving_or_out_of_bounds_gripper_cannot_change_reference(session, monkeypatch):
    import tcp_waypoints_session as adapters
    original_load = adapters.load_json
    def short_policy(path):
        value = original_load(path)
        return {**value, 'hold_seconds': .15, 'measurement_timeout_s': .5} if path.name == 'policy.json' else value
    monkeypatch.setattr(adapters, 'load_json', short_policy)
    calls = []
    def moving():
        calls.append(1)
        return gripper_observation(session, 16 if len(calls) % 2 else 48)
    before = (session.root / 'session.json').read_bytes()
    with pytest.raises(ValueError, match='Confirm'):
        register_gripper_reference(session, moving, confirmed_closed=False)
    assert not calls
    with pytest.raises(RuntimeError, match='hold the robot'):
        register_gripper_reference(session, moving, confirmed_closed=True)
    assert (session.root / 'session.json').read_bytes() == before
    invalid = gripper_observation(session, 2000)
    invalid.update(operator_confirmed_closed=True, stationary_checks_passed=True)
    with pytest.raises(ValueError, match='valid calibrated'):
        session.register_closed_gripper(invalid)
    assert (session.root / 'session.json').read_bytes() == before


def test_real_heldout_image_reconstructs_board_tcp_without_camera(session, monkeypatch):
    extrinsics = yaml.safe_load((session.root / 'camera_extrinsics.yaml').read_text())
    sample = next(Path(extrinsics['validation_run']).glob('sample_*'))
    image = cv2.imread(str(sample / 'rgb.png'))
    assert image is not None

    class Camera:
        current = image
        def open(self, *_): return True
        def set(self, *_): pass
        def read(self):
            time.sleep(.02)
            return True, self.current.copy()
        def release(self): pass

    monkeypatch.setattr(cv2, 'VideoCapture', Camera)
    camera = CameraMeasurement(session)
    shown = []
    camera.preview = SimpleNamespace(show=lambda frame, found, status: shown.append((frame.copy(), status.copy())))
    maps = [JointMapping(**row) for row in json.loads((session.root / 'mapping.json').read_text())]
    q = {name: 0.0 for name in session.manifest['state_joints']}
    q[session.side + '_gripper_joint'] = session.manifest['closed_gripper_joint_rad']

    def read():
        stamp = time.monotonic_ns()
        return {'host_monotonic_start_ns': stamp, 'host_monotonic_end_ns': stamp,
            'positions_rad': q, 'joints': {j.name: {'raw_tick': 2000} for j in maps}}

    try:
        camera.poll_preview()
        assert shown[-1][1]['board_visible'] and shown[-1][1]['pnp_valid']
        assert camera.preview_status['corners'] >= 8
        # Losing the board must immediately clear a previous green/OK status.
        camera.camera.current = np.zeros_like(image)
        camera.poll_preview()
        assert not shown[-1][1]['board_visible'] and not shown[-1][1]['pnp_valid']
        camera.camera.current = image
        import tcp_waypoints_session as adapters
        with monkeypatch.context() as rejected:
            def reject(*_):
                raise ValueError('PnP quality rejected')
            rejected.setattr(adapters, 'board_pose', reject)
            camera.poll_preview()
            assert shown[-1][1]['board_visible'] and not shown[-1][1]['pnp_valid']
            assert shown[-1][1]['reason'] == 'PnP quality rejected'
        measured, png = camera.measure(read)
        assert measured['stationary_checks_passed']
        assert len(measured['corner_ids']) >= 8
        assert measured['pnp']['best']['rmse_px'] <= 1
        assert not measured['hardware_exposure_synchronized']
        assert len(png) > 1000
        assert shown[-1][1]['pnp_valid'] and 'verified' in shown[-1][1]['phase']
        # The displayed annotations must never enter the stored measurement image.
        np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR), image)
        # The visual transform remains unchanged even if encoder FK is intentionally changed.
        q[session.command_joints[0]] += .2
        another, _ = camera.measure(read)
        np.testing.assert_allclose(another['board_T_tcp'], measured['board_T_tcp'], atol=1e-10)
        assert not np.allclose(another['base_T_tcp'], measured['base_T_tcp'])
    finally:
        camera.close()


def test_missing_board_leaves_error_csv_blank(tmp_path):
    write_report(tmp_path, {'points': [{'order': 1, 'point': 1, 'status': 'failed', 'error': 'board hidden'}]})
    csv = (tmp_path / 'errors.csv').read_text()
    assert '1,1,failed,,,,,,' in csv
    assert '0.0' not in csv


@pytest.mark.parametrize('preview', [False, True])
def test_ssh_commands_hold_save_duplicate_stop_release(session, monkeypatch, preview):
    """Exercise the actual teaching server and CLI requests without device acquisition."""
    buses = []

    class Bus(MemoryBus):
        def __init__(self, dataset, *, release_only=False):
            super().__init__(dataset)
            self.release_only = release_only
            if buses:
                self.registers = buses[-1].registers
            self.closed = False
            buses.append(self)
        def read(self):
            return gripper_observation(session)
        def close(self):
            self.closed = True

    class Camera:
        instances = []
        def __init__(self, _):
            self.preview_status = None
            self.closed = False
            self.instances.append(self)
        def poll_preview(self):
            self.preview_status = {'board_visible': True, 'pnp_valid': True, 'corners': 24}
        def measure(self, read):
            return {'positions_rad': read()['positions_rad'], 'base_T_tcp': np.eye(4).tolist(),
                    'board_T_tcp': np.eye(4).tolist(), 'stationary_checks_passed': True}, b'test PNG'
        def close(self): self.closed = True

    class Viewer:
        instances = []
        def __init__(self, side):
            self.side, self.closed = side, False
            self.instances.append(self)
        def close(self): self.closed = True

    monkeypatch.setattr(cli, 'TeachBus', Bus)
    monkeypatch.setattr(cli, 'CameraMeasurement', Camera)
    monkeypatch.setattr(cli, 'BoardPreview', Viewer)
    errors = []

    def serve():
        try:
            cli.teach(session, preview=preview)
        except Exception as error:
            errors.append(error)

    server = threading.Thread(target=serve, daemon=True)
    server.start()
    deadline = time.monotonic() + 5
    while not (session.root / 'control.sock').exists():
        assert time.monotonic() < deadline
        time.sleep(.01)
    try:
        status = cli.request(session, 'status')
        assert status['phase'] == 'teaching'
        if preview:
            deadline = time.monotonic() + 2
            while status['board'] is None:
                assert time.monotonic() < deadline
                status = cli.request(session, 'status')
            assert status['board']['pnp_valid']
            assert Camera.instances[0].preview is Viewer.instances[0]
        assert not buses[0].writes  # Preview itself cannot change any torque or motor setting.
        with pytest.raises(RuntimeError, match='Confirm'):
            cli.request(session, 'reference-gripper')
        with pytest.raises(RuntimeError, match='Run hold'):
            cli.request(session, 'save', 1)
        cli.request(session, 'release')
        cli.request(session, 'hold')
        before = list(buses[0].writes)
        reference = cli.request(session, 'reference-gripper', confirmed_closed=True)
        assert reference['closed_gripper_reference_registered'] and not reference['motor_writes']
        assert buses[0].writes == before
        assert cli.request(session, 'save', 1)['saved'] == 1
        assert session.point(1)['torque_hold_verified']
        with pytest.raises(RuntimeError, match='new positive'):
            cli.request(session, 'save', 1)
        with pytest.raises(RuntimeError, match='before the first point'):
            cli.request(session, 'reference-gripper', confirmed_closed=True)
        response = cli.request(session, 'stop')
        assert response['process_shutdown_confirmed']
    finally:
        if server.is_alive():
            try:
                cli.request(session, 'stop')
            except Exception:
                pass
        server.join(5)
    assert not errors and not server.is_alive()
    assert buses[0].closed and buses[0].powered()  # Stop retains the verified hold.
    assert len(Camera.instances) == 1 and Camera.instances[0].closed
    assert len(Viewer.instances) == int(preview)
    assert not preview or Viewer.instances[0].closed
    # The standalone explicit release command must take over only after server shutdown.
    monkeypatch.setattr(sys, 'argv',
                        ['tcp-waypoints', 'release', '--session', str(session.root)])
    cli.main()
    assert buses[-1].closed and not buses[-1].powered()
    assert buses[-1].release_only


def test_preview_display_failure_cannot_open_bus_or_leave_socket(session, monkeypatch):
    def unavailable(_):
        raise RuntimeError('Cannot access DISPLAY')
    def unexpected(_):
        pytest.fail('A failed display must not acquire hardware')
    monkeypatch.setattr(cli, 'BoardPreview', unavailable)
    monkeypatch.setattr(cli, 'TeachBus', unexpected)
    monkeypatch.setattr(cli, 'CameraMeasurement', unexpected)
    with pytest.raises(RuntimeError, match='DISPLAY'):
        cli.teach(session, preview=True)
    assert not cli.control_socket_path(session).exists()
    assert not (session.root / 'control.sock').is_symlink()


def test_preview_requires_display_before_highgui(monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setattr(cv2, 'namedWindow', lambda *_: pytest.fail('No GUI call without a display'))
    with pytest.raises(RuntimeError, match='DISPLAY'):
        BoardPreview('left')


def test_preview_draws_on_a_copy_and_quit_preserves_raw_frame(monkeypatch):
    viewer = BoardPreview.__new__(BoardPreview)
    viewer.title = 'test preview'
    frame = np.full((480, 640, 3), 80, dtype=np.uint8)
    original = frame.copy()
    found = {'marker_ids': None, 'ids': None}
    status = {'pnp_valid': False, 'board_visible': False, 'corners': 0, 'minimum_corners': 8,
              'corner_hull_fraction': 0., 'reason': 'No board', 'phase': 'Preview only'}
    shown = []
    monkeypatch.setattr(cv2, 'imshow', lambda title, view: shown.append(view.copy()))
    monkeypatch.setattr(cv2, 'waitKey', lambda _: ord('q'))
    with pytest.raises(KeyboardInterrupt):
        viewer.show(frame, found, status)
    np.testing.assert_array_equal(frame, original)
    assert shown[0].shape == (620, 640, 3)
    assert np.any(shown[0][480:] != 0)


def test_standalone_release_does_not_depend_on_other_arm_or_head(session, monkeypatch):
    import tcp_waypoints_session as adapters
    maps = [JointMapping(**row) for row in json.loads((session.root / 'mapping.json').read_text())]
    selected = {j.motor_id: j for j in maps if j.name in session.command_joints}
    values = {(i, address): value for i, j in selected.items() for address, value in
        ((3, j.model), (31, abs(j.homing_offset) | (0x800 if j.homing_offset < 0 else 0)),
         (9, j.range_min), (11, j.range_max), (40, 1))}
    opened, written = [], []

    class Packet:
        def read2ByteTxRx(self, port, motor, address):
            assert motor in selected, 'Nonselected motor must never be queried by standalone release'
            return values[motor, address], 0, 0
        read1ByteTxRx = read2ByteTxRx
        def write1ByteTxRx(self, port, motor, address, value):
            written.append((motor, address, value))
            values[motor, address] = value
            return 0, 0

    class Reader:
        def __init__(self, config):
            assert set(config['motor_ids'].values()) == set(selected)
            opened.append(config['serial_port'])
            self.packet, self.port = Packet(), None
        def close(self): pass

    monkeypatch.setattr(adapters, 'JointReader', Reader)
    bus = TeachBus(session, release_only=True)
    try:
        bus.release()
        assert not bus.powered()
        for address, size, value in ((40, 1, 1), (42, 2, 2000), (48, 2, 800)):
            with pytest.raises(ValueError, match='torque OFF only'):
                bus.write_register(next(iter(selected)), address, size, value)
    finally:
        bus.close()
    assert opened == [session.manifest['arm_port']]
    assert {i for i, _, _ in written} == set(selected)
    assert all((a, v) == (40, 0) for _, a, v in written)


def test_feedback_ready_before_controller_activation_is_waited_for(session):
    """Broadcaster and arm spawners may complete in either order."""
    client = cli.MoveItReplay.__new__(cli.MoveItReplay)
    client.dataset = session
    client.policy = json.loads((session.root / 'policy.json').read_text())
    client.guard = {'state': 'ready'}
    client.channel = SimpleNamespace(stopped=False)
    client.process = None
    client.action = SimpleNamespace(wait_for_server=lambda timeout_sec: True)
    q = {j: 0. for j in session.manifest['state_joints']}
    q[session.side + '_gripper_joint'] = session.manifest['closed_gripper_joint_rad']
    client.check = lambda: q.copy()
    client.valid = lambda _: None
    queries = []

    def query(kind, path, payload):
        if path.endswith('list_controllers'):
            queries.append(path)
            return SimpleNamespace(controller=[SimpleNamespace(name=session.side + '_arm_controller',
                state='inactive' if len(queries) == 1 else 'active')])
        model = (session.root / 'profile/robot_control.urdf').read_text()
        values = [SimpleNamespace(string_value=model)]
        if path.startswith('/move_group'):
            values.extend([SimpleNamespace(bool_value=False), SimpleNamespace(bool_value=False)])
        return SimpleNamespace(values=values)

    client.query = query
    client.ready([])
    assert len(queries) == 2
    assert client.baseline == q


def test_long_session_paths_and_existing_server_protection(tmp_path):
    root = tmp_path / ('long_session_name_' * 10)
    root.mkdir()
    store = Dataset(root, {'side': 'left'})
    server = cli.ControlSocket(store)
    try:
        assert len(str(server.path).encode()) < 108
        assert (root / 'control.sock').is_socket()
        with pytest.raises(OSError):
            cli.ControlSocket(store)
        assert server.path.exists() and (root / 'control.sock').exists()
    finally:
        server.close()
    assert not server.path.exists() and not (root / 'control.sock').is_symlink()
