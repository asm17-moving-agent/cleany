"""TCP session preparation and device adapters. No automatic startup on import."""
from dataclasses import asdict
import ctypes
import ctypes.util
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

from cleany_robot_interface.control_profile import prepare_control_profile
from cleany_robot_interface.core import JointMapping
from cleany_robot_interface.state_profile import head_positions
from cleany_robot_interface.tcp_waypoints import Dataset, board_tcp, digest, transform, validate_control_contract
from cleany_handeye_calibration.headcam_core import make_board, detect
from cleany_handeye_calibration.headcam_handeye import board_pose
from cleany_handeye_calibration.offline_fk import UrdfOfflineFk
from capture import JointReader
from handeye_observer import StillWindow
from auto_capture import AutoCaptureGate, view_signature

ROOT = Path(__file__).resolve().parents[1]


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def create_session(side: str, root: Path, config_file: Path) -> Dataset:
    if side not in ('left', 'right'):
        raise ValueError('Select left or right')
    root = root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    config = load_json(config_file)
    operating_path = Path(config['operating_profiles'][side]).expanduser()
    if not operating_path.is_absolute():
        operating_path = ROOT / operating_path
    operating = yaml.safe_load(operating_path.read_text())
    command_joints = [j for j in operating['command_joints'] if not j.endswith('_gripper_joint')]
    if len(command_joints) != 5 or any(not j.startswith(side + '_') for j in command_joints):
        raise ValueError('Operating profile must describe the selected five-joint arm')
    # Preserve its actual velocity, acceleration, torque and controller tolerances.
    control = {**operating, 'command_joints': command_joints, 'allow_torque_on_recovery': True}
    (root / 'operating_source.yaml').write_bytes(operating_path.read_bytes())
    (root / 'control.yaml').write_text(yaml.safe_dump(control, sort_keys=False))
    profile = prepare_control_profile(ROOT / config['state_profile'], ROOT,
                                      root / 'profile', root / 'control.yaml', False)
    tree = ET.parse(profile.model)
    arm = yaml.safe_load(profile.state.arm_config.read_text())
    head = load_json(profile.state.head_config)
    head_map = load_json(profile.state.head_mapping)
    # Freeze the mathematical mapping directly from the exact control URDF.
    maps = []
    for element in tree.findall('ros2_control/joint')[:12]:
        p = {v.get('name'): float(v.text) for v in element.findall('param')}
        maps.append(asdict(JointMapping(element.get('name'), int(p['motor_id']), int(p['model']),
            p['zero_tick'], int(p['direction']), p['offset_rad'], int(p['homing_offset']),
            int(p['range_min']), int(p['range_max']))))
    camera_path = ROOT / config['camera_configs'][side]
    extrinsic_path = ROOT / config['camera_extrinsics'][side]
    tcp_path = ROOT / config['tcp_candidates'][side]
    camera, extrinsics = load_json(camera_path), yaml.safe_load(extrinsic_path.read_text())
    intrinsic_path = Path(extrinsics['intrinsics_file']).expanduser()
    intrinsic = load_json(intrinsic_path)
    training_camera = load_json(intrinsic_path.parent / 'manifest.json')['config']
    for key in ('camera_device', 'width', 'height', 'dictionary', 'squares_x', 'squares_y',
                'square_length_m', 'marker_length_m'):
        if camera[key] != training_camera[key]:
            raise ValueError('Camera/board differs from frozen intrinsic calibration: ' + key)
    if not camera.get('square_size_physically_verified'):
        raise ValueError('Board square length must be physically verified')
    tcp = yaml.safe_load(tcp_path.read_text())
    if (tcp['parent_frame'] != f'{side}_gripper_frame'
            or extrinsics['parent_frame'] != tcp['parent_frame']
            or extrinsics['child_frame'] != f'{side}_wrist_camera_optical_frame'):
        raise ValueError('TCP/camera frame contract mismatch')
    gripper_T_tcp = np.eye(4)
    gripper_T_tcp[:3, 3] = tcp['xyz_m']
    if side == 'left' and any(float(v) != 0 for v in str(tcp['rpy']).split()):
        raise ValueError('TCP candidate must use the gripper orientation')
    if side == 'right' and tcp['quaternion_xyzw'] != [0, 0, 0, 1]:
        raise ValueError('TCP candidate must use the gripper orientation')
    for name, data in [('camera.json', camera), ('intrinsics.json', intrinsic),
                       ('head_mapping.json', head_map), ('policy.json', config),
                       ('mapping.json', maps), ('tcp_candidate.json', tcp)]:
        (root / name).write_text(json.dumps(data, indent=2) + '\n')
    (root / 'camera_extrinsics.yaml').write_bytes(extrinsic_path.read_bytes())
    manifest = {
        'schema': 1, 'side': side, 'created_unix_ns': time.time_ns(),
        'command_joints': command_joints,
        'state_joints': [e.get('name') for e in tree.findall('ros2_control/joint')],
        'arm_port': arm['port'], 'head_port': head['serial_port'], 'baudrate': arm['baudrate'],
        'gripper_T_camera': transform(extrinsics['parent_T_child']).tolist(),
        'gripper_T_tcp': transform(gripper_T_tcp).tolist(),
        'closed_gripper_joint_rad': tcp['closed_gripper_joint_rad'],
        'model_sha256': digest(profile.model),
        'operating_profile_source': str(operating_path),
        'source_sha256': {str(p): digest(p) for p in
            (config_file, operating_path, camera_path, extrinsic_path, intrinsic_path, tcp_path)},
        'frozen_sha256': {str(p.relative_to(root)): digest(p) for p in root.rglob('*') if p.is_file()},
        'head_commands': False, 'gripper_commands': False,
        'fixed_board_required': True, 'tcp_definition': 'CAD closed-fingertip midpoint candidate',
        'absolute_tcp_accuracy_verified': False, 'physical_execution_verified': False,
    }
    (root / 'session.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return Dataset.load(root)


class TeachBus:
    """Exclusive bus owner; only the selected five motors accept RAM writes."""
    def __init__(self, dataset: Dataset, *, release_only: bool = False):
        validate_control_contract(dataset)
        self.release_only = release_only
        self.dataset = dataset
        self.policy = load_json(dataset.root / 'policy.json')
        self.control = yaml.safe_load((dataset.root / 'control.yaml').read_text())
        self.maps = [JointMapping(**row) for row in load_json(dataset.root / 'mapping.json')]
        self.selected = [j for j in self.maps if j.name in dataset.command_joints]
        self.ids = {j.motor_id for j in self.selected}
        self.head_map = load_json(dataset.root / 'head_mapping.json')
        self.reader = self.head_reader = None
        try:
            ids = {j.name: j.motor_id for j in (self.selected if release_only else self.maps)}
            head_ids = {'head_pan_joint': 13, 'head_tilt_joint': 14}
            shared = not release_only and Path(dataset.manifest['arm_port']).resolve() == Path(dataset.manifest['head_port']).resolve()
            self.reader = JointReader({'serial_port': dataset.manifest['arm_port'],
                'baudrate': dataset.manifest['baudrate'], 'motor_ids': {**ids, **(head_ids if shared else {})}})
            if not shared and not release_only:
                self.head_reader = JointReader({'serial_port': dataset.manifest['head_port'],
                    'baudrate': dataset.manifest['baudrate'], 'motor_ids': head_ids})
            for j in (self.selected if release_only else self.maps):
                homing = self.read_register(j.motor_id, 31, 2)
                homing = -(homing & 0x7ff) if homing & 0x800 else homing
                if (self.read_register(j.motor_id, 3, 2), homing,
                        self.read_register(j.motor_id, 9, 2), self.read_register(j.motor_id, 11, 2)) != (
                        j.model, j.homing_offset, j.range_min, j.range_max):
                    raise ValueError(j.name + ': hardware calibration/model mismatch')
            if not release_only:
                head_reader = self.head_reader or self.reader
                for motor in (13, 14):
                    model, status, error = head_reader.packet.read2ByteTxRx(head_reader.port, motor, 3)
                    if status or error or model != 777:
                        raise ValueError(f'Head ID {motor}: hardware model/read mismatch')
        except BaseException:
            self.close()
            raise

    def read_register(self, motor: int, address: int, size: int) -> int:
        method = self.reader.packet.read1ByteTxRx if size == 1 else self.reader.packet.read2ByteTxRx
        value, status, error = method(self.reader.port, motor, address)
        if status or error:
            raise RuntimeError(f'ID {motor}: read {address} failed {status}/{error}')
        return value

    def write_register(self, motor: int, address: int, size: int, value: int) -> None:
        if getattr(self, 'release_only', False) and (address, size, value) != (40, 1, 0):
            raise ValueError('Release-only connection permits torque OFF only')
        if motor not in self.ids or (address, size) not in ((40, 1), (41, 1), (42, 2), (44, 2), (46, 2), (48, 2)):
            raise ValueError('Unauthorized motor/register write')
        with (self.dataset.root / 'teach_wire.jsonl').open('a') as stream:
            stream.write(json.dumps({'unix_ns': time.time_ns(), 'id': motor, 'address': address,
                                     'size': size, 'value': value}) + '\n')
        method = self.reader.packet.write1ByteTxRx if size == 1 else self.reader.packet.write2ByteTxRx
        status, error = method(self.reader.port, motor, address, value)
        if status or error or self.read_register(motor, address, size) != value:
            raise RuntimeError(f'ID {motor}: write/readback {address} failed {status}/{error}')

    def read(self) -> dict:
        if self.release_only:
            raise ValueError('Release-only connection does not acquire robot feedback')
        record = self.reader.read()
        if self.head_reader:
            head = self.head_reader.read()
            record['joints'].update(head['joints'])
            record['host_monotonic_end_ns'] = head['host_monotonic_end_ns']
        values = {j.name: j.radians(record['joints'][j.name]['raw_tick']) for j in self.maps}
        raw = [record['joints'][j]['raw_tick'] for j in ('head_pan_joint', 'head_tilt_joint')]
        values.update(zip(('head_pan_joint', 'head_tilt_joint'), head_positions(raw, self.head_map)))
        record['positions_rad'] = values
        return record

    def powered(self) -> bool:
        return all(self.read_register(j.motor_id, 40, 1) == 1 for j in self.selected)

    def hold(self) -> None:
        d = self.control['joint_defaults']
        targets = {}
        for j in self.selected:
            motor = j.motor_id
            if self.read_register(motor, 33, 1) != 0:
                raise ValueError('Position mode required')
            temperature_limit = min(self.control['temperature_limit_c'], self.read_register(motor, 13, 1))
            if self.read_register(motor, 63, 1) >= temperature_limit:
                raise ValueError('Motor temperature protection reached')
            if self.read_register(motor, 16, 2) < d['torque_limit_raw']:
                raise ValueError('EEPROM torque protection is below requested RAM limit')
            raw = self.read_register(motor, 56, 2)
            if not j.range_min <= raw <= j.range_max:
                raise ValueError('Current position outside calibrated motor bounds')
            # Re-seeding an already powered motor is allowed only at a current-position hold.
            if self.read_register(motor, 40, 1) and abs(self.read_register(motor, 42, 2) - raw) > 3:
                raise ValueError('Motor is powered with a moving/stale goal; release before teaching')
            targets[motor] = raw
        for motor, raw in targets.items():
            for address, size, value in ((42, 2, raw), (44, 2, 0), (41, 1, d['acceleration_raw']),
                                         (46, 2, d['speed_raw']), (48, 2, d['torque_limit_raw'])):
                self.write_register(motor, address, size, int(value))
        if any(abs(self.read_register(i, 56, 2) - raw) > 3 for i, raw in targets.items()):
            raise RuntimeError('Arm moved before torque-on; hold still and retry')
        for motor in targets:
            self.write_register(motor, 40, 1, 1)

    def release(self) -> None:
        errors = []
        for motor in sorted(self.ids):
            for attempt in range(3):
                try:
                    self.write_register(motor, 40, 1, 0)
                    break
                except (RuntimeError, OSError) as error:
                    if attempt == 2:
                        errors.append(str(error))
        if errors:
            raise RuntimeError('Torque-off NOT confirmed: ' + '; '.join(errors))

    def close(self) -> None:
        # Closing communications deliberately preserves the current RAM hold.
        for reader in (self.reader, self.head_reader):
            if reader:
                reader.close()
        self.reader = self.head_reader = None


class BoardPreview:
    """Display copies of the acquisition frames; never opens another camera or bus."""
    def __init__(self, side: str):
        # Check X before HighGUI: some backends abort the process on a bad display.
        if not os.environ.get('DISPLAY'):
            raise RuntimeError('Preview needs DISPLAY; in SSH set DISPLAY and XAUTHORITY for the robot desktop')
        library = ctypes.util.find_library('X11')
        if not library:
            raise RuntimeError('Preview needs the X11 library')
        x11 = ctypes.CDLL(library)
        x11.XOpenDisplay.argtypes, x11.XOpenDisplay.restype = [ctypes.c_char_p], ctypes.c_void_p
        x11.XCloseDisplay.argtypes, x11.XCloseDisplay.restype = [ctypes.c_void_p], ctypes.c_int
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError('Cannot access DISPLAY; check the desktop DISPLAY and XAUTHORITY')
        x11.XCloseDisplay(display)
        self.title = f'Cleany TCP teaching | {side.upper()} wrist | Q / Esc stop'
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.title, 960, 930)

    def show(self, frame: np.ndarray, found: dict, status: dict) -> None:
        view = frame.copy()
        if found['marker_ids'] is not None:
            cv2.aruco.drawDetectedMarkers(view, found['marker_corners'], found['marker_ids'])
        if found['ids'] is not None:
            cv2.aruco.drawDetectedCornersCharuco(view, found['corners'], found['ids'])
        panel = np.zeros((140, view.shape[1], 3), dtype=np.uint8)
        color = (70, 220, 70) if status['pnp_valid'] else (0, 180, 255) if status['board_visible'] else (80, 80, 255)
        state = 'BOARD OK' if status['pnp_valid'] else 'BOARD VISIBLE' if status['board_visible'] else 'BOARD NOT READY'
        lines = [f"{state} | corners {status['corners']}/{status['minimum_corners']} minimum | area {status['corner_hull_fraction']:.1%}",
                 status['reason'][:86], status['phase'],
                 'SSH: hold, then save N | Q / Esc stops; current torque state is kept']
        for i, line in enumerate(lines):
            cv2.putText(panel, line, (8, 24 + i*30), cv2.FONT_HERSHEY_SIMPLEX, .44, color, 1, cv2.LINE_AA)
        cv2.imshow(self.title, np.vstack((view, panel)))
        if cv2.waitKey(1) & 0xff in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt

    def close(self) -> None:
        cv2.destroyWindow(self.title)
        cv2.waitKey(1)


def register_gripper_reference(dataset: Dataset, read, *, confirmed_closed: bool) -> dict:
    """Read a stationary physical closure; no motor writes or camera acquisition."""
    if not confirmed_closed:
        raise ValueError('Confirm that the empty gripper is physically fully closed with --confirm-closed')
    if any(directory.exists() and any(directory.iterdir()) for directory in
           (dataset.root / 'points', dataset.root / 'runs')):
        raise ValueError('Register the closed gripper before the first point/run; use a new session otherwise')
    policy = load_json(dataset.root / 'policy.json')
    window = StillWindow(policy['hold_seconds'], policy['max_span_ticks'], policy['max_gap_seconds'])
    deadline = time.monotonic() + policy['measurement_timeout_s']
    while time.monotonic() < deadline:
        observation = read()
        if window.update(observation):
            record = {**observation, 'operator_confirmed_closed': True, 'stationary_checks_passed': True,
                      'stationary_joint_window': window.samples, 'registered_unix_ns': time.time_ns()}
            dataset.register_closed_gripper(record)
            name = dataset.side + '_gripper_joint'
            return {'closed_gripper_reference_registered': True, 'side': dataset.side,
                    'raw_tick': observation['joints'][name]['raw_tick'],
                    'closed_gripper_joint_rad': dataset.manifest['closed_gripper_joint_rad'],
                    'tolerance_rad': policy['closed_gripper_tolerance_rad'], 'motor_writes': False}
        time.sleep(.02)
    raise RuntimeError('Gripper reference unavailable: hold the robot and gripper still')


class CameraMeasurement:
    def __init__(self, dataset: Dataset):
        self.dataset = dataset
        self.config = load_json(dataset.root / 'camera.json')
        self.policy = load_json(dataset.root / 'policy.json')
        self.intrinsics = load_json(dataset.root / 'intrinsics.json')
        self.dictionary, self.board = make_board(self.config)
        self.fk = UrdfOfflineFk(dataset.root / 'profile/robot_control.urdf',
                               tip_frame=f'{dataset.side}_gripper_frame')
        self.preview = None
        self.preview_status = None
        self.camera = cv2.VideoCapture()
        if not self.camera.open(self.config['camera_device'], cv2.CAP_V4L2):
            raise RuntimeError('Cannot open the pinned wrist camera: ' + self.config['camera_device'])
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['width'])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['height'])
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read_frame(self) -> tuple[np.ndarray, int]:
        ok, frame = self.camera.read()
        received = time.monotonic_ns()
        if not ok or frame is None or frame.shape[:2] != (self.config['height'], self.config['width']):
            raise RuntimeError('Wrist image failed or resolution differs from intrinsics')
        return frame, received

    def inspect_frame(self, frame: np.ndarray) -> tuple[dict, dict]:
        found = detect(frame, self.dictionary, self.board, self.config['minimum_corners'])
        count = 0 if found['ids'] is None else len(found['ids'])
        area = 0 if found['corners'] is None else cv2.contourArea(cv2.convexHull(found['corners'])) / (frame.shape[0]*frame.shape[1])
        visible = bool(found['valid'] and area >= self.config['minimum_corner_hull_fraction'])
        if count < self.config['minimum_corners']:
            reason = 'Show more of the fixed board; enough spread corners are required'
        elif not visible:
            reason = 'Corners need to cover a larger, non-collinear area of the image'
        else:
            reason = 'Board detected; pose quality has not been checked yet'
        return found, {'board_visible': visible, 'pnp_valid': False, 'corners': count,
            'minimum_corners': self.config['minimum_corners'], 'corner_hull_fraction': area,
            'reason': reason, 'phase': 'Preview only; save verifies torque, gripper and stationary feedback',
            'observed_monotonic_ns': time.monotonic_ns()}

    def show_preview(self, frame: np.ndarray, found: dict, status: dict) -> None:
        self.preview_status = status
        if self.preview is not None:
            self.preview.show(frame, found, status)

    def poll_preview(self) -> None:
        frame, _ = self.read_frame()
        found, status = self.inspect_frame(frame)
        if status['board_visible']:
            try:
                pnp = board_pose(found, self.board, self.intrinsics, self.policy)
                status.update(pnp_valid=True, pnp_rmse_px=pnp['best']['rmse_px'],
                              reason='Fixed board pose OK; hold the arm, then use save N')
            except ValueError as error:
                status['reason'] = str(error)
        self.show_preview(frame, found, status)

    def measure(self, read, cancelled=lambda: False) -> tuple[dict, bytes]:
        p = self.policy
        window = StillWindow(p['hold_seconds'], p['max_span_ticks'], p['max_gap_seconds'])
        gate = AutoCaptureGate(p['hold_seconds'], p['stable_rms_px'], 0, p['max_gap_seconds'])
        deadline, reason = time.monotonic() + p['measurement_timeout_s'], 'Waiting for the fixed board'
        while time.monotonic() < deadline:
            if cancelled():
                raise RuntimeError('Measurement stopped')
            before = read()
            frame, received = self.read_frame()
            after = read()
            found, status = self.inspect_frame(frame)
            area, valid = status['corner_hull_fraction'], status['board_visible']
            if after['host_monotonic_end_ns'] - before['host_monotonic_start_ns'] > p['max_gap_seconds'] * 1e9:
                window.update(None)
                gate.update(None, time.monotonic())
                status.update(reason='Joint/image timing gap too large; waiting for fresh feedback', phase='Saving: waiting')
                self.show_preview(frame, found, status)
                continue
            window.update(before)
            still = window.update(after)
            signature = view_signature(found, self.board) if valid else None
            image_still, _ = gate.update(signature, time.monotonic())
            status['phase'] = f"Saving: arm {'STILL' if still else 'WAIT'} | image {'STILL' if image_still else 'WAIT'} | need {p['hold_seconds']}s"
            if not valid or not still or not image_still:
                reason = status['reason'] if not valid else 'Hold arm and board still; waiting for stationary arm/image window'
                status['reason'] = reason
                self.show_preview(frame, found, status)
                continue
            try:
                pnp = board_pose(found, self.board, self.intrinsics, p)
            except ValueError as error:
                reason = str(error)
                status['reason'] = reason
                self.show_preview(frame, found, status)
                continue
            status.update(pnp_valid=True, pnp_rmse_px=pnp['best']['rmse_px'], reason='Board and stationary window verified',
                          phase='Capture verified; committing the unannotated image')
            self.show_preview(frame, found, status)
            q = after['positions_rad']
            base_T_gripper = self.fk.compute(tuple(q), tuple(q.values())).as_homogeneous_matrix()
            measurement = {
                'positions_rad': q, 'joint_before_frame': before, 'joint_after_frame': after,
                'stationary_joint_window': window.samples, 'stationary_checks_passed': True,
                'host_receive_monotonic_ns': received, 'hardware_exposure_synchronized': False,
                'base_T_tcp': (base_T_gripper @ transform(self.dataset.manifest['gripper_T_tcp'])).tolist(),
                'board_T_tcp': board_tcp(pnp['best']['camera_T_board'],
                    self.dataset.manifest['gripper_T_camera'], self.dataset.manifest['gripper_T_tcp']).tolist(),
                'pnp': pnp, 'corner_ids': found['ids'].flatten().tolist(),
                'image_points_px': found['corners'].reshape(-1, 2).tolist(), 'corner_hull_fraction': area,
            }
            ok, encoded = cv2.imencode('.png', frame)
            if not ok:
                raise RuntimeError('PNG encoding failed')
            return measurement, encoded.tobytes()
        raise RuntimeError('TCP measurement unavailable: ' + reason)

    def close(self) -> None:
        self.camera.release()
