from __future__ import annotations

import atexit
import html
import shutil
import tempfile
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
    model_path.write_text(
        model_text.replace(
            _DESCRIPTION_MESHDIR,
            f'meshdir="{absolute_meshdir}"',
            1,
        ),
        encoding='utf-8',
    )

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


def load_model(scene_path: Path) -> tuple[Any, Any]:
    resolved_scene_path = resolve_scene_path(scene_path)
    model = mujoco.MjModel.from_xml_path(str(resolved_scene_path))
    data = mujoco.MjData(model)
    return model, data


@atexit.register
def _cleanup_materialized_scenes() -> None:
    for directory in _MATERIALIZED_DIRECTORIES:
        shutil.rmtree(directory, ignore_errors=True)
