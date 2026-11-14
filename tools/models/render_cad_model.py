#!/usr/bin/env python3
"""검토를 위해 실제 MuJoCo 모델을 CAD 등록 자세로 렌더링합니다."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--package',type=Path,default=Path('ros2_ws/src/cleany_description'))
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
manifest=json.loads((args.package/'config/geometry/cad_final.json').read_text())
model=mujoco.MjModel.from_xml_path(str(args.package/'mjcf/cleany.xml'))
model.vis.global_.offwidth=1200
model.vis.global_.offheight=1000
model.vis.headlight.ambient[:]=[.5,.5,.5]
data=mujoco.MjData(model)
for name,q in manifest['registration_angles_rad'].items():
    data.qpos[model.joint(name).qposadr[0]]=q
mujoco.mj_forward(model,data)
options=mujoco.MjvOption()
options.geomgroup[:]=[1,1,1,0,0,0]
camera=mujoco.MjvCamera()
camera.lookat[:]=[0,0,.65]
camera.distance=2.2
args.output.mkdir(parents=True,exist_ok=True)
with mujoco.Renderer(model,height=1000,width=1200) as renderer:
    for name,azimuth,elevation in [('isometric',135,-20),('front',180,-5),('rear',0,-10)]:
        camera.azimuth,camera.elevation=azimuth,elevation
        renderer.update_scene(data,camera=camera,scene_option=options)
        Image.fromarray(renderer.render()).save(args.output/f'{name}.png')
print(args.output)
