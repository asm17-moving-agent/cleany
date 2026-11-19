"""고정 장면의 짝지은 microbenchmark이며 전체 분류 과정의 benchmark가 아닙니다."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import statistics
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np

from cleany_mujoco_sim.scene_loader import materialize_control_scene
from cleany_mujoco_sim.tabletop_performance import PERFORMANCE_PROFILES


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=3000)
    parser.add_argument('--frames', type=int, default=30)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if min(args.steps, args.frames, args.repeats) < 1:
        parser.error('steps, frames, and repeats must be positive')
    package = Path(__file__).resolve().parents[1]
    launch = package.parent / 'cleany_skill_executor/launch/study_cafe_nearest_grasp_demo.launch.py'
    # 별도의 불일치하는 기본 자세를 만들지 않고 실제 launch의 home 자세를 재사용합니다.
    initial = {f'{name}_joint': float(value) for name, value in re.findall(
        r"'((?:left|right)_\w+|head_tilt)_initial': '(-?[0-9.]+)'", launch.read_text())}
    if len(initial) != 13:
        raise ValueError('Expected 12 arm/gripper joints and head tilt in launch')
    models = {name: mujoco.MjModel.from_xml_path(str(materialize_control_scene(
        package / 'scenes/study_cafe_grasp_execution.xml.in',
        initial_joint_positions=initial,
        sorting_bins_config=package / 'config/robot_top_bins.yaml',
        performance_profile=name,
    ))) for name in PERFORMANCE_PROFILES}
    report = {'mujoco_version': mujoco.__version__, 'scope': 'stationary home; Python MuJoCo, no ROS/perception/GUI',
              'steps': args.steps, 'frames_per_camera': args.frames, 'repeats': args.repeats,
              'profiles': {}}
    baseline = models['baseline']
    for name, model in models.items():
        report['profiles'][name] = {
            'geoms': model.ngeom, 'explicit_contact_pairs': model.npair,
            'masked_geoms': int(np.count_nonzero(
                ((baseline.geom_contype != 0) | (baseline.geom_conaffinity != 0))
                & (model.geom_contype == 0) & (model.geom_conaffinity == 0))),
            'shadow_map_size': int(model.vis.quality.shadowsize),
            'timestep_seconds': float(model.opt.timestep),
            'physics_ms_per_step': [], 'render_ms_per_frame': {camera: [] for camera in (
                'head_realsense_rgb', 'left_wrist_rgb', 'right_wrist_rgb')},
        }
    # 준비 및 순서 편향을 줄이도록 프로파일 순서를 번갈아 적용하며 병렬 측정하지 않습니다.
    for repeat in range(args.repeats):
        order = PERFORMANCE_PROFILES if repeat % 2 == 0 else PERFORMANCE_PROFILES[::-1]
        for name in order:
            model = models[name]
            data = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, data, model.key('handeye_ros2_control_home').id)
            mujoco.mj_step(model, data, nstep=500)
            started = time.perf_counter()
            mujoco.mj_step(model, data, nstep=args.steps)
            elapsed = time.perf_counter()-started
            values = report['profiles'][name]
            values['physics_ms_per_step'].append(elapsed*1000/args.steps)
            with mujoco.Renderer(model, height=480, width=640) as renderer:
                option = mujoco.MjvOption()
                option.geomgroup[3:] = 0
                for camera in values['render_ms_per_frame']:
                    for _ in range(3):
                        renderer.update_scene(data, camera=camera, scene_option=option)
                        renderer.render()
                    started = time.perf_counter()
                    for _ in range(args.frames):
                        renderer.update_scene(data, camera=camera, scene_option=option)
                        renderer.render()
                    values['render_ms_per_frame'][camera].append(
                        (time.perf_counter()-started)*1000/args.frames)
            print(f'Completed repeat {repeat+1}: {name}', flush=True)
    for values in report['profiles'].values():
        values['physics_median_ms'] = statistics.median(values['physics_ms_per_step'])
        values['physics_only_realtime_factor'] = values['timestep_seconds']/(values['physics_median_ms']/1000)
        values['render_median_ms'] = {camera: statistics.median(times)
                                      for camera, times in values['render_ms_per_frame'].items()}
    payload = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload+'\n', encoding='utf-8')
    print(payload)


if __name__ == '__main__':
    main()
