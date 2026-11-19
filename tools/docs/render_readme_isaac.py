#!/usr/bin/env python3
"""Capture independent Cleany environments in one real Isaac Sim stage."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import traceback

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--main', type=Path, required=True)
parser.add_argument('--urdf', type=Path, required=True)
parser.add_argument('--placement-config', type=Path, required=True)
parser.add_argument('--num-envs', type=int, default=16)
args = parser.parse_args()
if not 1 <= args.num_envs <= 25:
    raise ValueError('This presentation capture is limited to 25 environments')
args.output.mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'hide_ui': True, 'renderer': 'RayTracedLighting',
                     'width': 1600, 'height': 1000, 'limit_cpu_threads': 6})

import numpy as np
import omni.usd
import omni.physics.tensors
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.api import World
from isaacsim.core.rendering_manager import ViewportManager
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf, UsdLux, UsdShade


def cube(stage, path, xyz, size, color):
    prim = UsdGeom.Cube.Define(stage, path)
    prim.CreateSizeAttr(1.)
    prim.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    prim.AddScaleOp().Set(Gf.Vec3f(*size))
    prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    material = UsdShade.Material.Define(stage, path+'/Material')
    shader = UsdShade.Shader.Define(stage, path+'/Material/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.8)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(material)
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


def main():
    app_utils.enable_extension('isaacsim.asset.importer.urdf')
    app.update()
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    imported = URDFImporter().import_urdf(URDFImporterConfig(
        urdf_path=str(args.urdf), usd_path=str(args.output / 'usd'),
        ros_package_paths=[{'name': 'cleany_description',
                            'path': str(args.main / 'ros2_ws/src/cleany_description')}],
        fix_base=True, merge_fixed_joints=False, merge_mesh=False,
        collision_from_visuals=False, allow_self_collision=True,
        joint_drive_type='force', joint_target_type='position',
        override_joint_stiffness=1200., override_joint_damping=60.,
    ))
    if not omni.usd.get_context().open_stage(imported):
        raise RuntimeError('Could not open converted Cleany model')
    stage = omni.usd.get_context().get_stage()
    stage.GetPrimAtPath('/cleany').GetVariantSet('Physics').SetVariantSelection('physx')
    base = stage.GetPrimAtPath('/cleany/Geometry/base_link')
    base.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    base.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    base.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(
        [name for name in base.GetAppliedSchemas() if 'Articulation' not in name]))
    anchor = UsdPhysics.FixedJoint(stage.GetPrimAtPath('/cleany/Physics/root_joint'))
    anchor.GetBody0Rel().ClearTargets(True)
    UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(anchor.GetPrim())
    articulation.CreateEnabledSelfCollisionsAttr(True)
    articulation.CreateSolverPositionIterationCountAttr(32)
    articulation.CreateSolverVelocityIterationCountAttr(8)
    # Flattened importer prototypes can lose rendered children when referenced
    # across environments. Author independent presentation geometry instead.
    for _ in range(3):
        for prim in list(stage.Traverse()):
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
    template = args.output / 'robot-template.usda'
    stage.Flatten().Export(str(template))
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    bounds = json.loads(args.placement_config.read_text())
    columns = math.ceil(math.sqrt(args.num_envs))
    spacing = float(bounds['spacing_m'])
    center = (columns - 1) * spacing / 2
    base_height = float(bounds['base_height_above_floor_m'])
    cube(stage, '/World/Floor', [center, center, -base_height-.05],
         [columns*spacing+1.5, columns*spacing+1.5, .1], [.86, .9, .87])
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(350.)
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(750.)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, -20, -30))
    rng = np.random.default_rng(20261115)
    paths, placements = [], []
    for index in range(args.num_envs):
        origin = [(index % columns)*spacing, (index // columns)*spacing, 0.]
        env = f'/World/env_{index:02d}'
        UsdGeom.Xform.Define(stage, env).AddTranslateOp().Set(Gf.Vec3d(*origin))
        robot = stage.DefinePrim(env + '/cleany', 'Xform')
        robot.GetReferences().AddReference(str(template), '/cleany')
        fixed = UsdPhysics.FixedJoint(stage.GetPrimAtPath(env+'/cleany/Physics/root_joint'))
        fixed.GetLocalPos0Attr().Set(Gf.Vec3f(*origin)+fixed.GetLocalPos0Attr().Get())
        z = float(bounds['table_height_m'][0]) - base_height
        cube(stage, env+'/Table', [.6, -.27, z-.025], [.60, .78, .05], [.83, .75, .6])
        for j, (x, y) in enumerate([(.36, -.59), (.84, -.59), (.36, .05), (.84, .05)]):
            cube(stage, env+f'/Leg_{j}', [x, y, (z-base_height)/2-.025],
                 [.035, .035, z+base_height-.05], [.25, .31, .3])
        cup_x = float(rng.uniform(*bounds['cup_x_m']))
        cup_y = float(rng.uniform(*bounds['cup_y_m']))
        # This presentation fixture uses a visible cup proxy, not a grasp evaluation.
        cup = UsdGeom.Cylinder.Define(stage, env+'/Cup')
        cup.CreateRadiusAttr(.035)
        cup.CreateHeightAttr(.10)
        cup.CreateAxisAttr('Z')
        cup.AddTranslateOp().Set(Gf.Vec3d(cup_x, cup_y, z+.05))
        cup.CreateDisplayColorAttr([Gf.Vec3f(.94, .93, .86)])
        paths.append(env+'/cleany/Physics/root_joint')
        placements.append({'environment': index, 'origin_m': origin,
                           'cup_xy_m': [cup_x, cup_y], 'table_height_m': z+base_height})
    world = World(physics_dt=1/120, rendering_dt=1/30, stage_units_in_meters=1.,
                  backend='numpy', device='cpu')
    world.reset()
    physics = omni.physics.tensors.create_simulation_view('numpy',
        stage_id=omni.usd.get_context().get_stage_id())
    views = [physics.create_articulation_view(path) for path in paths]
    initial_layout = []
    for i, view in enumerate(views):
        if view.count != 1:
            raise RuntimeError(f'Environment {i} does not contain one articulation')
        q = view.get_dof_positions().copy()
        for j, name in enumerate(view.shared_metatype.dof_names):
            if name.endswith(('shoulder_pitch_joint', 'elbow_pitch_joint')):
                q[0, j] = 2.8 + .10*(i%3)
        view.set_dof_positions(q, np.array([0], dtype=np.int32))
        view.set_dof_position_targets(q, np.array([0], dtype=np.int32))
        initial_layout.append({'environment': i, 'names': list(view.shared_metatype.link_names),
                               'links': view.get_link_transforms().tolist()})
    print('INITIALIZED_ENVIRONMENTS', len(views), flush=True)
    (args.output/'initial-layout.json').write_text(json.dumps(initial_layout))
    ViewportManager.set_camera_view('/OmniverseKit_Persp',
        eye=[center+7, center-9, 8], target=[center+.1, center, .1])
    for _ in range(10):
        app.update()
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    capture = capture_viewport_to_file(get_active_viewport(), str(args.output/'isaac-initial.png'))
    for _ in range(20):
        app.update()
    app.run_coroutine(capture.wait_for_result())
    for _ in range(90):
        world.step(render=True)
    print('PHYSICS_COMPLETE', flush=True)
    for index, view in enumerate(views):
        names = list(view.shared_metatype.link_names)
        base = view.get_link_transforms()[0, names.index('base_link'), :3]
        if np.linalg.norm(base-np.array(placements[index]['origin_m'])) > .001:
            raise RuntimeError(f'Environment {index} has a displaced base')
    camera = get_active_viewport()
    capture = capture_viewport_to_file(camera, str(args.output/'isaac-parallel.png'))
    for _ in range(20):
        app.update()
    app.run_coroutine(capture.wait_for_result())
    stage.GetRootLayer().Export(str(args.output/'parallel-stage.usda'))
    (args.output/'render.json').write_text(json.dumps({
        'kind': 'new Isaac Sim parallel environment capture', 'environments': len(views),
        'independent_articulations': [view.count for view in views],
        'physics_steps': 90, 'dt_s': 1/120, 'trained_policy_executed': False,
        'success_rate_evaluated': False, 'cup_geometry': 'presentation cylinder proxy',
        'urdf': str(args.urdf), 'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
        'placement_config': str(args.placement_config), 'placements': placements,
        'initial_layout': initial_layout,
        'final_layout': [{'environment': i, 'links': v.get_link_transforms().tolist()}
                         for i, v in enumerate(views)],
    }, indent=2)+'\n')
    print('RENDER_COMPLETE', args.output, flush=True)


try:
    main()
except Exception:
    (args.output/'error.txt').write_text(traceback.format_exc())
    raise
finally:
    app.close()
