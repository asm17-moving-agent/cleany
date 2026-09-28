"""Generate YOLO instance-segmentation labels from offline MuJoCo renders.

The renderer's object IDs are used only while preparing training data. They
are not available to, or used by, the running perception node.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml

from cleany_mujoco_sim.scene_loader import materialize_control_scene


CLASS_NAMES = ('cup', 'computer mouse', 'crumpled tissue', 'lego brick')
OBJECT_NAMES = ('cup', 'mouse', 'tissue', 'lego')
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def _robot_observation_pose(
    model: mujoco.MjModel, data: mujoco.MjData
) -> None:
    positions = {'head_tilt_joint': 1.0}
    suffixes = ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
                'wrist_pitch', 'wrist_roll', 'gripper')
    for arm, angles in (
        ('left', (-1.53, 3.35, 3.12, -1.63, 1.58, -0.35)),
        ('right', (1.58, 3.35, 3.12, -1.63, 1.58, -0.35)),
    ):
        positions.update({
            f'{arm}_{name}_joint': value
            for name, value in zip(suffixes, angles, strict=True)
        })
    for name, position in positions.items():
        joint = model.joint(name)
        data.qpos[joint.qposadr[0]] = position
        data.qvel[joint.dofadr[0]] = 0.0


def _object_geom_ids(model: mujoco.MjModel) -> tuple[np.ndarray, ...]:
    groups = []
    for name in OBJECT_NAMES:
        prefix = f'study_cafe_{name}_'
        ids = [index for index in range(model.ngeom)
               if model.geom(index).name.startswith(prefix)
               and model.geom(index).name.endswith('_visual')]
        if not ids:
            raise ValueError(f'No visible MuJoCo geoms for {name}')
        groups.append(np.asarray(ids, dtype=np.int32))
    return tuple(groups)


def _polygon(mask: np.ndarray) -> str | None:
    contours, _ = cv2.findContours(mask.astype(np.uint8),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    contour = cv2.approxPolyDP(
        contour, 0.002 * cv2.arcLength(contour, True), True
    )
    points = contour.reshape(-1, 2)
    if len(points) < 3 or cv2.contourArea(contour) < 100:
        return None
    if (points[:, 0].min() < 3 or points[:, 0].max() > IMAGE_WIDTH - 4
            or points[:, 1].min() < 3
            or points[:, 1].max() > IMAGE_HEIGHT - 4):
        return None
    return ' '.join(f'{x / IMAGE_WIDTH:.6f} {y / IMAGE_HEIGHT:.6f}'
                    for x, y in points)


def _randomize_objects(
    model: mujoco.MjModel, data: mujoco.MjData,
    baseline_qpos: np.ndarray, rng: np.random.Generator,
) -> None:
    data.qpos[:] = baseline_qpos
    positions: list[np.ndarray] = []
    for name in OBJECT_NAMES:
        joint = model.joint(f'study_cafe_{name}_freejoint')
        start = joint.qposadr[0]
        anchor = baseline_qpos[start:start + 3]
        for _ in range(100):
            position = anchor[:2] + rng.uniform((-0.07, -0.04), (0.07, 0.04))
            if all(np.linalg.norm(position - other) >= 0.095
                   for other in positions):
                break
        else:
            raise RuntimeError('Could not sample separated object positions')
        positions.append(position)
        data.qpos[start:start + 2] = position
        angle = float(rng.uniform(-0.7, 0.7))
        yaw = np.array((np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)))
        orientation = np.empty(4)
        mujoco.mju_mulQuat(
            orientation, yaw, baseline_qpos[start + 3:start + 7]
        )
        data.qpos[start + 3:start + 7] = orientation
    head_tilt = model.joint('head_tilt_joint').qposadr[0]
    data.qpos[head_tilt] += rng.uniform(-0.055, 0.055)
    mujoco.mj_forward(model, data)


def generate(
    output: Path, train_count: int, val_count: int, seed: int
) -> dict:
    if train_count <= 0 or val_count <= 0:
        raise ValueError('Training and validation counts must be positive')
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f'Dataset output must be empty: {output}')
    package = Path(__file__).resolve().parents[2] / 'cleany_mujoco_sim'
    scene_template = package / 'scenes/study_cafe_grasp_execution.xml.in'
    scene = materialize_control_scene(
        scene_template,
        sorting_bins_config=package / 'config/robot_top_bins.yaml',
    )
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(
        model, data, model.key('handeye_ros2_control_home').id
    )
    for _ in range(2000):
        mujoco.mj_step(model, data)
    _robot_observation_pose(model, data)
    mujoco.mj_forward(model, data)
    baseline_qpos = data.qpos.copy()
    geom_groups = _object_geom_ids(model)
    rng = np.random.default_rng(seed)
    options = mujoco.MjvOption()
    options.geomgroup[3:] = 0
    counts = {'train': train_count, 'val': val_count}
    for split in counts:
        (output / 'images' / split).mkdir(parents=True, exist_ok=True)
        (output / 'labels' / split).mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(
        model, width=IMAGE_WIDTH, height=IMAGE_HEIGHT
    ) as renderer:
        for split, count in counts.items():
            for index in range(count):
                for _ in range(30):
                    _randomize_objects(model, data, baseline_qpos, rng)
                    renderer.disable_segmentation_rendering()
                    renderer.update_scene(data, camera='head_realsense_rgb',
                                          scene_option=options)
                    rgb = renderer.render()
                    renderer.enable_segmentation_rendering()
                    renderer.update_scene(data, camera='head_realsense_rgb',
                                          scene_option=options)
                    segmentation = renderer.render()
                    labels = []
                    for class_id, geom_ids in enumerate(geom_groups):
                        mask = (
                            (segmentation[..., 1]
                             == int(mujoco.mjtObj.mjOBJ_GEOM))
                            & np.isin(segmentation[..., 0], geom_ids)
                        )
                        polygon = _polygon(mask)
                        if polygon is not None:
                            labels.append(f'{class_id} {polygon}')
                    if len(labels) == len(CLASS_NAMES):
                        break
                else:
                    raise RuntimeError(
                        f'Four visible objects unavailable: {split}/{index}'
                    )
                gain = rng.uniform(0.85, 1.15)
                bias = rng.uniform(-10, 10)
                rgb = np.clip(
                    rgb.astype(np.float32) * gain + bias, 0, 255
                ).astype(np.uint8)
                stem = f'{split}_{index:04d}'
                cv2.imwrite(str(output / 'images' / split / f'{stem}.png'),
                            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                (output / 'labels' / split / f'{stem}.txt').write_text(
                    '\n'.join(labels) + '\n', encoding='utf-8')
    config = {'path': str(output.resolve()), 'train': 'images/train',
              'val': 'images/val', 'names': dict(enumerate(CLASS_NAMES))}
    (output / 'dataset.yaml').write_text(
        yaml.safe_dump(config, sort_keys=False), encoding='utf-8'
    )
    summary = {'seed': seed, 'counts': counts, 'classes': CLASS_NAMES,
               'source': str(scene_template),
               'ground_truth_use': 'offline training only'}
    (output / 'manifest.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8'
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--train-count', type=int, default=80)
    parser.add_argument('--val-count', type=int, default=20)
    parser.add_argument('--seed', type=int, default=20260928)
    args = parser.parse_args()
    print(json.dumps(generate(args.output.expanduser(), args.train_count,
                              args.val_count, args.seed), indent=2))


if __name__ == '__main__':
    main()
