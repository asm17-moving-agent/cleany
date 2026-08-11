from __future__ import annotations

import atexit
import html
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

_SCENE_MODEL_TOKEN = '@CLEANY_MJCF_PATH@'
_DESCRIPTION_MESHDIR = 'meshdir="../meshes/"'
_MATERIALIZED_DIRECTORIES: list[Path] = []
_CONTROL_INITIAL_KEYFRAME = 'handeye_ros2_control_home'
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
    model_path.write_text(materialized_model, encoding='utf-8')

    scene_text = template_path.read_text(encoding='utf-8')
    if _SCENE_MODEL_TOKEN not in scene_text:
        raise ValueError(
            f'MuJoCo scene template is missing {_SCENE_MODEL_TOKEN}'
        )
    model_include_path = html.escape(str(model_path.resolve()), quote=True)
    scene_text = scene_text.replace(
        _SCENE_MODEL_TOKEN,
        model_include_path,
    )
    if _SCENE_MODEL_TOKEN in scene_text:
        raise ValueError(
            f'Unresolved MuJoCo scene token: {_SCENE_MODEL_TOKEN}'
        )

    scene_path = materialized_dir / template_path.name.removesuffix('.in')
    scene_path.write_text(scene_text, encoding='utf-8')
    return scene_path


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
