#!/usr/bin/env python3
"""Render CAD motion using the production Mecanum velocity conversion."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/cleany_base_driver'))
from cleany_base_driver.core import Geometry, Limits, wheel_speeds


def font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(subprocess.check_output(['fc-match', '-f', '%{file}', 'sans-serif'], text=True).strip(), size)


def model_with_floor(source: Path, width: int, height: int) -> mujoco.MjModel:
    tree = ET.parse(source)
    root = tree.getroot()
    compiler = root.find('compiler')
    assert compiler is not None
    for key in ('meshdir', 'texturedir'):
        if key in compiler.attrib:
            compiler.set(key, str((source.parent / compiler.get(key)).resolve()))
    asset = root.find('asset')
    assert asset is not None
    ET.SubElement(asset, 'texture', name='review_floor', type='2d', builtin='checker',
                  rgb1='.34 .40 .37', rgb2='.31 .37 .34', width='512', height='512')
    ET.SubElement(asset, 'texture', name='review_sky', type='skybox', builtin='gradient',
                  rgb1='.14 .19 .17', rgb2='.31 .38 .34', width='512', height='3072')
    ET.SubElement(asset, 'material', name='review_floor', texture='review_floor',
                  texrepeat='10 10', texuniform='true', reflectance='0')
    world = root.find('worldbody')
    assert world is not None
    ET.SubElement(world, 'geom', name='review_ground', type='plane', size='5 5 .1',
                  material='review_floor', contype='0', conaffinity='0')
    for light in world.findall('light'):
        light.set('diffuse', '.35 .35 .35')
        light.set('ambient', '.05 .05 .05')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    model.vis.headlight.ambient[:] = [.32, .32, .32]
    model.vis.headlight.diffuse[:] = [.45, .45, .45]
    model.vis.headlight.specular[:] = [.05, .05, .05]
    return model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=80)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    frames_dir = args.output.parent / 'review/mecanum-frames'
    frames_dir.mkdir(parents=True, exist_ok=True)
    package = ROOT / 'ros2_ws/src/cleany_description'
    source = package / 'mjcf/cleany.xml'
    config_path = ROOT / 'ros2_ws/src/cleany_base_driver/config/base_hardware.yaml'
    config = yaml.safe_load(config_path.read_text())
    g = config['geometry']
    geometry = Geometry(g['wheel_radius_m'], g['wheelbase_m'], g['wheel_separation_m'])
    l = config['limits']
    limits = Limits(l['linear_x_mps'], l['linear_y_mps'], l['angular_z_rad_s'],
                    l['wheel_rad_s'], l['command_timeout_s'])
    width, height, fps = 400, 420, 10
    model = model_with_floor(source, width, height)
    data = mujoco.MjData(model)
    home = data.qpos.copy()
    options = mujoco.MjvOption()
    options.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.20, .20, .50]
    camera.distance, camera.azimuth, camera.elevation = 2.6, 135, -25
    names = ['front_left_wheel_joint', 'front_right_wheel_joint',
             'rear_left_wheel_joint', 'rear_right_wheel_joint']
    angles = np.zeros((3, 4))
    states = np.zeros((3, 3))
    records = []
    modes = [('Forward / backward', 'vx', .24), ('Lateral', 'vy', .24),
             ('Rotation', 'wz', .55)]
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for i in range(args.frames):
            t = i / fps
            canvas = Image.new('RGB', (1200, 520), '#f3f7f3')
            draw = ImageDraw.Draw(canvas)
            for mode, (title, axis, amplitude) in enumerate(modes):
                command = np.zeros(3)
                command[mode] = amplitude * math.sin(2 * math.pi * t / (args.frames / fps))
                wheels = np.array(wheel_speeds(*command, geometry, limits))
                # The inverse check ensures the displayed velocity and wheel signs agree.
                radius, lever = geometry.wheel_radius, (geometry.wheelbase + geometry.separation) / 2
                recovered = radius * np.array([
                    wheels.sum(), -wheels[0] + wheels[1] + wheels[2] - wheels[3],
                    (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) / lever,
                ]) / 4
                if not np.allclose(recovered, command, atol=1e-9):
                    raise ValueError('Wheel conversion and displayed command disagree')
                if i:
                    states[mode] += recovered / fps
                    angles[mode] += wheels / fps
                data.qpos[:] = home
                data.qpos[:2] = states[mode, :2]
                yaw = states[mode, 2]
                data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
                for name, angle in zip(names, angles[mode]):
                    data.qpos[model.joint(name).qposadr[0]] = angle
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera, scene_option=options)
                canvas.paste(Image.fromarray(renderer.render()), (mode * width, 58))
                draw.text((mode * width + 20, 18), title, font=font(22), fill='#294e43')
                unit = 'rad/s' if mode == 2 else 'm/s'
                draw.text((mode * width + 20, 484), f'{axis} {command[mode]:+.2f} {unit}',
                          font=font(17), fill='#294e43')
                records.append({'frame': i, 'time_s': t, 'mode': axis,
                                'command': command.tolist(), 'wheel_rad_s': wheels.tolist(),
                                'base_xy_yaw': states[mode].tolist()})
            canvas.save(frames_dir / f'{i:04d}.png')
            if i == args.frames // 4:
                canvas.save(args.output / 'mecanum-motion.png')
            if i % 20 == 0:
                print(f'Rendered {i}/{args.frames}', flush=True)
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(fps),
                    '-i', str(frames_dir / '%04d.png'), '-c:v', 'libopenh264', '-b:v', '4M',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    str(args.output / 'mecanum-motion.mp4')], check=True)
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                    '-i', str(args.output / 'mecanum-motion.mp4'), '-filter_complex',
                    'scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=3',
                    '-loop', '0', str(args.output / 'mecanum-motion.gif')], check=True)
    (args.output / 'mecanum-motion.json').write_text(json.dumps({
        'kind': 'CAD kinematic simulation', 'fps': fps, 'frames': args.frames,
        'physics_contacts_simulated': False,
        'model': str(source), 'model_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'geometry_config': str(config_path), 'velocity_conversion': 'cleany_base_driver.core.wheel_speeds',
        'samples': records,
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
