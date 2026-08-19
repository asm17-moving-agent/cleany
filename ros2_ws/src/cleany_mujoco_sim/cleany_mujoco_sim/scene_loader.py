from __future__ import annotations

import atexit
import html
import shutil
import struct
import tempfile
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path
from typing import Any

import mujoco
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from cleany_mujoco_sim.camera_contract import (
    CAMERA_FOVY_DEG,
    CAMERA_HEIGHT,
    CAMERA_NAME,
    CAMERA_WIDTH,
)

_SCENE_MODEL_TOKEN = '@CLEANY_MJCF_PATH@'
_HANDEYE_CAMERA_CONTRACT_TOKEN = '@CLEANY_HANDEYE_CAMERA_CONTRACT@'
_HANDEYE_CHARUCO_TEXTURE_TOKEN = '@CLEANY_CHARUCO_TEXTURE_PATH@'
_DESCRIPTION_MESHDIR = 'meshdir="../meshes/"'
_MATERIALIZED_DIRECTORIES: list[Path] = []
_CONTROL_INITIAL_KEYFRAME = 'handeye_ros2_control_home'
_CHARUCO_TEXTURE_WIDTH = 1400
_CHARUCO_TEXTURE_HEIGHT = 1000
_CHARUCO_BOARD_WIDTH_M = 0.210
_CHARUCO_BOARD_HEIGHT_M = 0.150
_CHARUCO_INK_GEOMETRY_COUNT = 211
_WHEEL_DCMOTOR_ACTUATORS = frozenset(
    {
        'rear_left_drive',
        'rear_right_drive',
        'front_left_drive',
        'front_right_drive',
    }
)


def _package_share(package_name: str) -> Path:
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        source_root = Path(__file__).resolve().parents[2]
        source_package = source_root / package_name
        if source_package.is_dir():
            return source_package
        raise


def default_scene_template_path() -> Path:
    share_dir = _package_share('cleany_mujoco_sim')
    return share_dir / 'scenes' / 'default.xml.in'


def default_scene_path() -> Path:
    return materialize_scene(default_scene_template_path())


def resolve_scene_path(scene_path: Path) -> Path:
    if scene_path.suffix == '.in':
        return materialize_scene(scene_path)
    return scene_path


def materialize_scene(template_path: Path) -> Path:
    return _materialize_scene(template_path, control_compatible=False)


def materialize_control_scene(template_path: Path) -> Path:
    """Materialize a MuJoCo 3.4-compatible arm-control scene.

    The canonical model uses MuJoCo 3.7 ``dcmotor`` actuators for the mobile
    base.  The Humble ``mujoco_ros2_control`` binary vendors MuJoCo 3.4, and
    the calibration backend neither commands nor depends on those actuators.
    Only the temporary model copy used by this function drops them.  It also
    adds a zero-state keyframe so that the 0.0.3 hardware plugin initializes
    every remaining actuator command, including MJCF-only head actuators, to a
    finite value.  The canonical model and the default custom simulator path
    remain unchanged.
    """
    return _materialize_scene(template_path, control_compatible=True)


def resolve_control_scene_path(scene_path: Path) -> Path:
    if scene_path.suffix == '.in':
        return materialize_control_scene(scene_path)
    return scene_path


def _materialize_scene(
    template_path: Path,
    *,
    control_compatible: bool,
) -> Path:
    if not template_path.is_file():
        raise FileNotFoundError(
            f'MuJoCo scene template not found: {template_path}'
        )

    description_share = _package_share('cleany_description')
    description_model = description_share / 'mjcf' / 'cleany.xml'
    description_meshes = description_share / 'meshes'
    if not description_model.is_file():
        raise FileNotFoundError(
            f'Cleany MJCF model not found: {description_model}'
        )
    if not description_meshes.is_dir():
        raise FileNotFoundError(
            f'Cleany mesh directory not found: {description_meshes}'
        )

    scene_text = template_path.read_text(encoding='utf-8')
    if _SCENE_MODEL_TOKEN not in scene_text:
        raise ValueError(
            f'MuJoCo scene template is missing {_SCENE_MODEL_TOKEN}'
        )
    apply_handeye_camera_contract = (
        _HANDEYE_CAMERA_CONTRACT_TOKEN in scene_text
    )
    materialize_charuco_texture = (
        _HANDEYE_CHARUCO_TEXTURE_TOKEN in scene_text
    )

    materialized_dir = Path(
        tempfile.mkdtemp(prefix='cleany_mujoco_scene_')
    )
    _MATERIALIZED_DIRECTORIES.append(materialized_dir)
    model_dir = materialized_dir / 'mjcf'
    model_dir.mkdir()
    model_path = model_dir / 'cleany.xml'

    model_text = description_model.read_text(encoding='utf-8')
    if _DESCRIPTION_MESHDIR not in model_text:
        raise ValueError(
            'Cleany MJCF must declare meshdir="../meshes/" so the simulator '
            'can resolve installed description assets'
        )
    absolute_meshdir = html.escape(
        str(description_meshes.resolve()),
        quote=True,
    )
    if control_compatible:
        materialized_model = _control_compatible_model_text(
            model_text,
            absolute_meshdir=description_meshes.resolve(),
        )
    else:
        materialized_model = model_text.replace(
            _DESCRIPTION_MESHDIR,
            f'meshdir="{absolute_meshdir}"',
            1,
        )
    if apply_handeye_camera_contract:
        materialized_model = _handeye_camera_model_text(materialized_model)
    model_path.write_text(materialized_model, encoding='utf-8')

    model_include_path = html.escape(str(model_path.resolve()), quote=True)
    scene_text = scene_text.replace(
        _SCENE_MODEL_TOKEN,
        model_include_path,
    )
    if _SCENE_MODEL_TOKEN in scene_text:
        raise ValueError(
            f'Unresolved MuJoCo scene token: {_SCENE_MODEL_TOKEN}'
        )
    scene_text = scene_text.replace(
        _HANDEYE_CAMERA_CONTRACT_TOKEN,
        (
            f'{CAMERA_NAME}:{CAMERA_WIDTH}x{CAMERA_HEIGHT}'
            f'@fovy{CAMERA_FOVY_DEG:g}'
        ),
    )
    if materialize_charuco_texture:
        texture_path = materialized_dir / 'charuco_render_texture.png'
        _write_charuco_texture(scene_text, texture_path)
        scene_text = scene_text.replace(
            _HANDEYE_CHARUCO_TEXTURE_TOKEN,
            html.escape(str(texture_path.resolve()), quote=True),
        )
    if _HANDEYE_CHARUCO_TEXTURE_TOKEN in scene_text:
        raise ValueError(
            'Unresolved MuJoCo scene token: '
            f'{_HANDEYE_CHARUCO_TEXTURE_TOKEN}'
        )

    scene_path = materialized_dir / template_path.name.removesuffix('.in')
    scene_path.write_text(scene_text, encoding='utf-8')
    return scene_path


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    body = chunk_type + payload
    return (
        struct.pack('>I', len(payload))
        + body
        + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)
    )


def _grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    if len(pixels) != width * height:
        raise ValueError('grayscale texture payload has an invalid size')
    scanlines = b''.join(
        b'\x00' + pixels[row * width:(row + 1) * width]
        for row in range(height)
    )
    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return (
        b'\x89PNG\r\n\x1a\n'
        + _png_chunk(b'IHDR', header)
        + _png_chunk(b'IDAT', zlib.compress(scanlines, level=9))
        + _png_chunk(b'IEND', b'')
    )


def _write_charuco_texture(scene_text: str, texture_path: Path) -> None:
    """Rasterize the exact vector ink boxes into a lossless temp texture.

    At 640x480 the many sub-pixel box edges corrupt enough ArUco bits to make
    the vector-only render undecodable.  The vector geometry remains the
    canonical board source; this deterministic raster is merely its render
    representation and is generated only in the temporary hand-eye scene.
    """

    root = ET.fromstring(scene_text)
    target = root.find(".//body[@name='charuco_target']")
    if target is None:
        raise ValueError('hand-eye scene is missing charuco_target')
    ink_geometries = tuple(
        geom
        for geom in target.findall('./geom')
        if geom.attrib.get('name', '').startswith('charuco_ink_')
    )
    expected_names = tuple(
        f'charuco_ink_{index:03d}'
        for index in range(_CHARUCO_INK_GEOMETRY_COUNT)
    )
    actual_names = tuple(
        geometry.attrib['name'] for geometry in ink_geometries
    )
    if actual_names != expected_names:
        raise ValueError(
            'hand-eye scene must contain the exact ordered 211-vector '
            'ChArUco ink source'
        )

    pixels = bytearray(b'\xff') * (
        _CHARUCO_TEXTURE_WIDTH * _CHARUCO_TEXTURE_HEIGHT
    )
    for geometry in ink_geometries:
        if geometry.attrib.get('type') != 'box':
            raise ValueError('ChArUco ink geometry must use boxes')
        try:
            position = tuple(
                float(value) for value in geometry.attrib['pos'].split()
            )
            half_size = tuple(
                float(value) for value in geometry.attrib['size'].split()
            )
        except (KeyError, ValueError) as error:
            raise ValueError(
                'ChArUco ink geometry must declare numeric pos and size'
            ) from error
        if len(position) != 3 or len(half_size) != 3:
            raise ValueError('ChArUco ink pos and size must have three values')
        x_min = position[0] - half_size[0]
        x_max = position[0] + half_size[0]
        y_min = position[1] - half_size[1]
        y_max = position[1] + half_size[1]
        x0 = round(
            x_min / _CHARUCO_BOARD_WIDTH_M * _CHARUCO_TEXTURE_WIDTH
        )
        x1 = round(
            x_max / _CHARUCO_BOARD_WIDTH_M * _CHARUCO_TEXTURE_WIDTH
        )
        y0 = round(
            (_CHARUCO_BOARD_HEIGHT_M - y_max)
            / _CHARUCO_BOARD_HEIGHT_M
            * _CHARUCO_TEXTURE_HEIGHT
        )
        y1 = round(
            (_CHARUCO_BOARD_HEIGHT_M - y_min)
            / _CHARUCO_BOARD_HEIGHT_M
            * _CHARUCO_TEXTURE_HEIGHT
        )
        if not (
            0 <= x0 < x1 <= _CHARUCO_TEXTURE_WIDTH
            and 0 <= y0 < y1 <= _CHARUCO_TEXTURE_HEIGHT
        ):
            raise ValueError('ChArUco ink geometry exceeds the board bounds')
        black_run = b'\x00' * (x1 - x0)
        for row in range(y0, y1):
            start = row * _CHARUCO_TEXTURE_WIDTH + x0
            pixels[start:start + len(black_run)] = black_run

    texture_path.write_bytes(
        _grayscale_png(
            _CHARUCO_TEXTURE_WIDTH,
            _CHARUCO_TEXTURE_HEIGHT,
            bytes(pixels),
        )
    )


def _handeye_camera_model_text(model_text: str) -> str:
    """Patch only the temporary hand-eye include for the release renderer."""

    root = ET.fromstring(model_text)
    cameras = root.findall(f".//camera[@name='{CAMERA_NAME}']")
    if len(cameras) != 1:
        raise ValueError(
            f'Cleany MJCF must define exactly one {CAMERA_NAME} camera'
        )
    camera = cameras[0]
    try:
        source_fovy = float(camera.attrib['fovy'])
    except (KeyError, ValueError) as error:
        raise ValueError(
            f'{CAMERA_NAME} must declare a numeric fovy'
        ) from error
    if source_fovy != CAMERA_FOVY_DEG:
        raise ValueError(
            f'{CAMERA_NAME} fovy changed: expected {CAMERA_FOVY_DEG}, '
            f'got {source_fovy}'
        )
    source_resolution = camera.attrib.get('resolution')
    expected_resolution = f'{CAMERA_WIDTH} {CAMERA_HEIGHT}'
    if source_resolution not in (None, expected_resolution):
        raise ValueError(
            f'{CAMERA_NAME} resolution changed: expected '
            f'{expected_resolution}, got {source_resolution}'
        )
    camera.set('fovy', f'{CAMERA_FOVY_DEG:g}')
    camera.set('resolution', expected_resolution)
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode') + '\n'


def _control_compatible_model_text(
    model_text: str,
    *,
    absolute_meshdir: Path,
) -> str:
    root = ET.fromstring(model_text)
    compiler = root.find('./compiler')
    if compiler is None or compiler.attrib.get('meshdir') != '../meshes/':
        raise ValueError(
            'Cleany MJCF must declare meshdir="../meshes/" so the simulator '
            'can resolve installed description assets'
        )
    compiler.set('meshdir', str(absolute_meshdir))

    actuator_group = root.find('./actuator')
    if actuator_group is None:
        raise ValueError('Cleany MJCF is missing its actuator group')
    wheel_actuators = {
        actuator.attrib.get('name'): actuator
        for actuator in actuator_group.findall('./dcmotor')
    }
    if set(wheel_actuators) != _WHEEL_DCMOTOR_ACTUATORS:
        raise ValueError(
            'Cleany MJCF wheel dcmotor set changed: '
            f'expected {sorted(_WHEEL_DCMOTOR_ACTUATORS)}, '
            f'got {sorted(name for name in wheel_actuators if name)}'
        )
    for actuator in wheel_actuators.values():
        actuator_group.remove(actuator)

    drive_defaults = root.find("./default/default[@class='pg42_drive']")
    if drive_defaults is None:
        raise ValueError('Cleany MJCF is missing the pg42_drive defaults')
    default_dcmotors = drive_defaults.findall('./dcmotor')
    if len(default_dcmotors) != 1:
        raise ValueError(
            'Cleany MJCF must define exactly one pg42_drive dcmotor default'
        )
    drive_defaults.remove(default_dcmotors[0])

    keyframe_group = root.find('./keyframe')
    if keyframe_group is None:
        keyframe_group = ET.SubElement(root, 'keyframe')
    if root.find(
        f"./keyframe/key[@name='{_CONTROL_INITIAL_KEYFRAME}']"
    ) is not None:
        raise ValueError(
            f'Cleany MJCF already defines {_CONTROL_INITIAL_KEYFRAME}'
        )
    ET.SubElement(
        keyframe_group,
        'key',
        {'name': _CONTROL_INITIAL_KEYFRAME},
    )

    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode') + '\n'


def load_model(scene_path: Path) -> tuple[Any, Any]:
    resolved_scene_path = resolve_scene_path(scene_path)
    model = mujoco.MjModel.from_xml_path(str(resolved_scene_path))
    data = mujoco.MjData(model)
    return model, data


@atexit.register
def _cleanup_materialized_scenes() -> None:
    for directory in _MATERIALIZED_DIRECTORIES:
        shutil.rmtree(directory, ignore_errors=True)
