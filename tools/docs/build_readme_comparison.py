#!/usr/bin/env python3
"""Pair recorded camera checkpoints with the same run's planned arm poses."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]


def font(size: int):
    path = subprocess.check_output(['fc-match', '-f', '%{file}', 'sans-serif'], text=True).strip()
    return ImageFont.truetype(path, size)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scene = json.loads((args.input/'scene.json').read_text())
    plan = json.loads((args.input/'moveit-result.json').read_text())
    plan_input = json.loads((args.input/'plan-input.json').read_text())
    receipt = json.loads((args.input/'execution/receipt.json').read_text())
    records = [json.loads(line) for line in (args.input/'execution/camera-frames.jsonl').read_text().splitlines()]
    source = ROOT/'ros2_ws/src/cleany_description/mjcf/cleany.xml'
    root = ET.parse(source).getroot()
    compiler = root.find('compiler')
    compiler.set('meshdir', str((source.parent/compiler.get('meshdir')).resolve()))
    world = root.find('worldbody')
    # The legacy MJCF chassis is 0.38 m above world; the saved scene uses base_link.
    base_offset = np.fromstring(world.find('body').get('pos'), sep=' ')
    rotation = np.asarray(scene['box']['rotation_base_from_box'])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    ET.SubElement(world, 'geom', type='box', size=' '.join(map(str, np.asarray(scene['box']['size_m'])/2)),
                  pos=' '.join(map(str, np.asarray(scene['box']['center_m'])+base_offset)),
                  quat=' '.join(map(str, quat)), rgba='.18 .21 .23 1', contype='0', conaffinity='0')
    table_center = np.asarray(scene['table_origin_m'])+base_offset-.015*np.asarray(scene['table_normal'])
    table_center[0] += .15
    ET.SubElement(world, 'geom', type='box', size='.38 .45 .015',
                  pos=' '.join(map(str, table_center)), quat=' '.join(map(str, quat)),
                  rgba='.65 .68 .62 1', contype='0', conaffinity='0')
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', type='skybox', builtin='gradient',
                  rgb1='.12 .17 .15', rgb2='.3 .36 .32', width='512', height='3072')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    model.vis.global_.offwidth, model.vis.global_.offheight = 640, 360
    model.vis.headlight.ambient[:] = [.3, .3, .3]
    model.vis.headlight.diffuse[:] = [.4, .4, .4]
    data = mujoco.MjData(model)
    for name, q in scene['q_rad'].items():
        data.qpos[model.joint(name).qposadr[0]] = q
    positions = plan['program']['approach']['trajectory'][-1]['positions']
    joint_names = ['right_shoulder_yaw_joint', 'right_shoulder_pitch_joint', 'right_elbow_pitch_joint',
                   'right_wrist_pitch_joint', 'right_wrist_roll_joint', 'right_gripper_joint']
    for motor, name in zip(range(7, 13), joint_names):
        data.qpos[model.joint(name).qposadr[0]] = positions[str(motor)]
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.30, -.12, .82]
    camera.distance, camera.azimuth, camera.elevation = 1.2, -65, -35
    options = mujoco.MjvOption()
    options.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    outputs = []
    with mujoco.Renderer(model, height=360, width=640) as renderer:
        for record in records:
            label = record['label']
            if label == 'after_close':
                data.qpos[model.joint('right_gripper_joint').qposadr[0]] = plan_input['box_grasp']['candidate']['contact_gripper_rad']
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=options)
            planned = Image.fromarray(renderer.render())
            actual = Image.open(args.input/f'execution/{label}.jpg').convert('RGB').resize((640, 360), Image.Resampling.LANCZOS)
            canvas = Image.new('RGB', (1320, 470), '#f3f7f3')
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 18), 'Planned pose', font=font(24), fill='#294e43')
            draw.text((680, 18), 'Recorded frame', font=font(24), fill='#294e43')
            canvas.paste(planned, (10, 65)); canvas.paste(actual, (670, 65))
            elapsed = record['frame_unix']-receipt['start_unix']
            title = {'at_grasp':'At grasp', 'after_close':'After close'}[label]
            draw.text((20, 436), title, font=font(18), fill='#294e43')
            draw.text((680, 436), f'{title}  /  t + {elapsed:.3f} s', font=font(18), fill='#294e43')
            canvas.save(args.output/f'plan-execution-{label}.png')
            outputs.append(canvas)
    outputs[0].save(args.output/'plan-execution.gif', save_all=True,
                    append_images=outputs[1:], duration=2500, loop=0)
    sheet = Image.new('RGB', (1320, 956), '#e0eae2')
    sheet.paste(outputs[0], (0, 0)); sheet.paste(outputs[1], (0, 486))
    sheet.save(args.output/'plan-execution.png')
    sources = [source, args.input/'scene.json', args.input/'moveit-result.json', args.input/'plan-input.json',
               args.input/'execution/camera-frames.jsonl', args.input/'execution/receipt.json']
    sources += [args.input/f'execution/{r["label"]}.jpg' for r in records]
    (args.output/'plan-execution.json').write_text(json.dumps({
        'kind':'two recorded camera checkpoints paired with planned poses',
        'continuous_video': False, 'playback_hold_ms_per_checkpoint':2500,
        'pixel_registered': False, 'records':records,
        'run_passed':receipt['passed'], 'run_error':receipt['error'],
        'planned_close_angle_rad':plan_input['box_grasp']['candidate']['contact_gripper_rad'],
        'physical_pose_inferred_from_photo':False,
        'scene_geometry':'recorded OBB and plane; illustrative finite tabletop extent',
        'sources':[{'path':str(p), 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources],
    }, indent=2)+'\n')


if __name__ == '__main__':
    main()
