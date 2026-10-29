"""Render recorded PhysX link poses from three observer cameras; no new physics."""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, default=Path("/output"))
parser.add_argument("--limit", type=int, help="Preview only the first N frames")
parser.add_argument("--name", default="rendered")
parser.add_argument("--sample-period", type=float, default=.5)
args = parser.parse_args()
media = args.run / "media"
report = json.loads((args.run / "evaluations/mission.json").read_text())
assert report["passed"] and report["mission_result"]["navigation_result"] == "OK"
assert report["mission_result"]["return_result"] == "OK"
states = report["states"]
records = [json.loads(line) for line in (media / "motion.jsonl").read_text().splitlines()]
selected = []
last_time, last_phase = -1., ""
for record in records:
    prior = [event for event in states if event["wall_time"] <= record["wall_time"]]
    phase = prior[-1]["state"] if prior else ""
    if phase not in ("NAVIGATE_TO_TARGET", "RETURN_HOME"):
        continue
    if phase != last_phase or record["sim_time"]-last_time >= args.sample_period-1e-6:
        selected.append({**record, "phase": phase})
        last_time, last_phase = record["sim_time"], phase
assert len(selected) > 20, "No complete navigation recording"
if args.limit:
    selected = selected[:args.limit]

from isaacsim import SimulationApp
# The bundled Franka policy example checks remote assets during startup even
# for an offline replay. Keep the base rendering app without that unused demo.
base_experience = Path('/isaac-sim/apps/isaacsim.exp.base.kit')
replay_experience = base_experience.with_name('cleany.recorded.replay.kit')
replay_experience.write_text(base_experience.read_text().replace(
    '"isaacsim.robot.policy.examples" = {}', ''))
app = SimulationApp({"headless": True, "hide_ui": True, "renderer": "RaytracedLighting",
                     "limit_cpu_threads": 4, "disable_viewport_updates": True},
                    experience=str(replay_experience))
import carb.settings
import omni.usd
import omni.timeline
import omni.replicator.core as rep
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics


def main() -> None:
    # Captured RGB-D products contain process-local SyntheticData graphs. They
    # cannot be reused by new observer cameras. Strip them in a separate layer;
    # leave the original recorded stage and all world/robot geometry intact.
    replay_stage = Usd.Stage.Open(str(media / 'scene.usda'))
    replay_stage.RemovePrim('/Render')
    removed_physics = [str(prim.GetPath()) for prim in replay_stage.Traverse()
                       if prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.Scene)]
    for path in reversed(removed_physics):
        replay_stage.RemovePrim(path)
    replay_path = media / f'replay-{args.name}.usda'
    replay_stage.Export(str(replay_path))
    del replay_stage
    if not omni.usd.get_context().open_stage(str(replay_path)):
        raise RuntimeError("Could not load the recorded scene")
    app.update()
    app.update()
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().pause()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if str(prim.GetPath()).startswith('/CleanupLighting/'):
            intensity = prim.GetAttribute('inputs:intensity')
            if intensity:
                intensity.Set(0.)
    light = UsdLux.DomeLight.Define(stage, "/ReplayLighting/Dome")
    light.CreateIntensityAttr(1300.)
    sun = UsdLux.DistantLight.Define(stage, "/ReplayLighting/Key")
    sun.CreateIntensityAttr(1800.)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35., -25., 0.))
    # Presentation colors affect only this replay stage, never the test scene.
    for name, color in (("Floor", (.22, .27, .33)), ("Desk", (.62, .42, .24)),
                        ("Payload", (.95, .35, .08))):
        prim = stage.GetPrimAtPath("/Room/"+name)
        if prim.IsValid():
            UsdGeom.Gprim(prim).CreateDisplayColorAttr([color])
    views = [
        ("front", "FRONT / THREE-QUARTER", [3.6, 1.6, 2.5], [.85, 0., .65], [0., 0., 1.]),
        ("side", "SIDE", [.8, -1.85, 2.15], [.8, 0., .65], [0., 0., 1.]),
        ("top", "OVERHEAD", [.8, 0., 5.], [.8, 0., 0.], [0., 1., 0.]),
    ]
    scene = json.loads((args.run / "config/room.json").read_text())
    physical = report.get('scope', {}).get('physical_cleanup', False)
    payload_operation = None
    payload_samples, payload_times = [], []
    if physical:
        audit_path = args.run/'evaluations/grasp-physics.jsonl'
        audit = [json.loads(line) for line in audit_path.read_text().splitlines()]
        payload_samples = [np.asarray(row['payload'][0], dtype=float) for row in audit]
        payload_times = [row['simulation_s'] for row in audit]
        if not payload_times:
            raise RuntimeError('Physical cleanup replay requires recorded payload poses')
        payload_xf = UsdGeom.Xformable(stage.GetPrimAtPath('/CleanupPayload'))
        payload_xf.ClearXformOpOrder()
        payload_operation = payload_xf.AddTransformOp(opSuffix='replay')
        payload_xf.SetXformOpOrder([payload_operation], resetXformStack=True)
    hidden_replay_prims = []
    if scene["world"] == "facility18":
        # RTX can shade the back of the original ceiling despite doubleSided=False.
        # Hide only its replay visuals; the recorded physics scene stays intact.
        ceiling = stage.GetPrimAtPath("/Facility18/facility_ceiling")
        for prim in ceiling.GetChildren():
            if prim.GetName().startswith("visual_"):
                UsdGeom.Imageable(prim).MakeInvisible()
                hidden_replay_prims.append(str(prim.GetPath()))
        views = [
            ("front", "FACILITY 18F / FRONT TRACK", [2.2, 2.2, 2.6], [0., 0., .5], [0., 0., 1.]),
            ("side", "FACILITY 18F / SIDE TRACK", [2., -2., 4.], [0., 0., .5], [0., 0., 1.]),
            ("top", "FACILITY 18F / OVERHEAD TRACK", [0., 0., 5.5], [0., 0., 0.], [0., 1., 0.]),
        ]
    paths = ["/ReplayCameras/"+view[0] for view in views]
    cameras = [UsdGeom.Camera.Define(stage, path) for path in paths]
    width, height = 480, 360
    camera_operations = []
    for path, (_, _, eye, target, up) in zip(paths, views):
        camera = UsdGeom.Camera(stage.GetPrimAtPath(path))
        camera.CreateFocalLengthAttr(18.)
        camera.CreateHorizontalApertureAttr(32.)
        camera.CreateVerticalApertureAttr(24.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 100.))
        camera.CreateFStopAttr(0.)
        camera.CreateShutterOpenAttr(0.)
        camera.CreateShutterCloseAttr(0.)
        xf = UsdGeom.Xformable(camera)
        xf.ClearXformOpOrder()
        operation = xf.AddTransformOp()
        operation.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)).GetInverse())
        camera_operations.append(operation)
    products, annotators = [], []
    for path in paths:
        product = rep.create.render_product(path, (width, height))
        annotator = rep.AnnotatorRegistry.get_annotator('rgb')
        annotator.attach(product)
        products.append(product)
        annotators.append(annotator)
    operations = []
    link_paths = json.loads((media / "links.json").read_text())
    base_index = next(i for i, path in enumerate(link_paths) if path.endswith("/base_link"))
    for path in link_paths:
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
        xf.ClearXformOpOrder()
        xf.SetResetXformStack(True)
        attribute = xf.GetPrim().GetAttribute("xformOp:transform:replay")
        operation = UsdGeom.XformOp(attribute) if attribute else xf.AddTransformOp(opSuffix="replay")
        xf.SetXformOpOrder([operation], resetXformStack=True)
        operations.append(operation)
    font = ImageFont.truetype("/integration-font.ttf", 16)
    small = ImageFont.truetype("/integration-font.ttf", 13)
    rep.orchestrator.set_capture_on_play(False)
    # Explicit async capture is the SDK path for a frozen Fabric clock. step()
    # still waits for each render, so RGB must match this frame's authored poses.
    carb.settings.get_settings().set_bool('/omni/replicator/asyncRendering', True)
    output = media / args.name
    output.mkdir()
    all_frames = {name: [] for name, *_ in views}
    combined = []
    max_position_error = 0.
    max_payload_sample_gap = 0.
    for index, record in enumerate(selected):
        for operation, pose in zip(operations, record["links"]):
            matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]), Gf.Vec3d(*pose[3:6])))
            matrix.SetTranslateOnly(Gf.Vec3d(*pose[:3]))
            operation.Set(matrix)
        if payload_operation is not None:
            after = min(bisect.bisect_left(payload_times, record['sim_time']), len(payload_times)-1)
            before = max(0, after-1) if record['sim_time'] <= payload_times[-1] else after
            gap = max(abs(payload_times[i]-record['sim_time']) for i in (before, after))
            max_payload_sample_gap = max(max_payload_sample_gap, gap)
            if gap > .21:
                raise RuntimeError('Payload audit does not cover the recorded navigation time')
            start, end = payload_samples[before], payload_samples[after].copy()
            span = payload_times[after]-payload_times[before]
            alpha = np.clip((record['sim_time']-payload_times[before])/span, 0., 1.) if span else 0.
            # Audit RGB-D ticks are slower than motion recording. Interpolate
            # their read-only body poses; never rerun or constrain physics.
            if np.dot(start[3:], end[3:]) < 0.:
                end[3:] *= -1.
            payload_pose = (1.-alpha)*start+alpha*end
            payload_pose[3:] /= np.linalg.norm(payload_pose[3:])
            matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(float(payload_pose[6]), Gf.Vec3d(*payload_pose[3:6])))
            matrix.SetTranslateOnly(Gf.Vec3d(*payload_pose[:3]))
            payload_operation.Set(matrix)
        if scene["world"] == "facility18":
            base = np.asarray(record["links"][base_index][:3])
            base[2] = 0.
            for operation, (_, _, eye, target, up) in zip(camera_operations, views):
                operation.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*(base+eye)),
                    Gf.Vec3d(*(base+target)), Gf.Vec3d(*up)).GetInverse())
        rep.orchestrator.step(rt_subframes=2, delta_time=0., pause_timeline=True, wait_for_render=True)
        cache = UsdGeom.XformCache()
        for operation, pose in zip(operations, record["links"]):
            position = cache.GetLocalToWorldTransform(operation.GetAttr().GetPrim()).ExtractTranslation()
            error = float(np.linalg.norm(np.asarray(position)-np.asarray(pose[:3])))
            max_position_error = max(max_position_error, error)
        assert max_position_error < 1e-5, "Rendering changed recorded link positions"
        frames = [np.asarray(annotator.get_data()) for annotator in annotators]
        for _ in range(10):
            if all(frame.size > 0 for frame in frames):
                break
            rep.orchestrator.step(rt_subframes=2, delta_time=0., pause_timeline=True, wait_for_render=True)
            frames = [np.asarray(annotator.get_data()) for annotator in annotators]
        if any(frame.size == 0 for frame in frames):
            raise RuntimeError('RTX observer returned no frame')
        rgb = np.stack([frame[:, :, :3] for frame in frames])
        assert rgb.shape == (3, height, width, 3), rgb.shape
        montage = Image.new("RGB", (3*width, height+70), "#101827")
        for camera_index, (name, title, *_rest) in enumerate(views):
            frame = Image.new("RGB", (width, height+70), "#101827")
            frame.paste(Image.fromarray(rgb[camera_index]), (0, 32))
            draw = ImageDraw.Draw(frame)
            draw.text((12, 7), title, font=font, fill="white")
            phase = "TO DESK" if record["phase"] == "NAVIGATE_TO_TARGET" else "RETURN HOME"
            draw.text((12, height+37), f"{phase} | sim {record['sim_time']:.1f}s | replay {10*args.sample_period:g}x", font=small, fill="white")
            caption = ('Actual Nav2 motion; physical cup cleanup' if physical
                       else 'Actual Nav2 motion; cleanup mocked')
            draw.text((12, height+53), caption, font=small, fill="#a8bbd5")
            all_frames[name].append(frame)
            montage.paste(frame, (camera_index*width, 0))
        montage.save(output / f"frame_{index:04d}.png")
        combined.append(montage)
        if index % 20 == 0:
            print(f"RENDER {index+1}/{len(selected)}", flush=True)
    for name, frames in {**all_frames, "three-views": combined}.items():
        frames[0].save(output / (name+".gif"), save_all=True, append_images=frames[1:],
                       duration=100, loop=0, disposal=2, optimize=False)
    metadata = {"type": "Isaac RTX replay of recorded world-space PhysX link poses",
                "physics_during_render": False, "frames": len(selected), "fps": 10,
                "simulation_sample_period_s": args.sample_period, "nominal_playback_speed": 10*args.sample_period,
                "maximum_link_position_error_m": max_position_error,
                "hidden_replay_prims": hidden_replay_prims,
                "cameras": views, "tracking": "observer XY follow recorded base" if scene["world"] == "facility18" else "fixed", "source_sha256": hashlib.sha256((media / "motion.jsonl").read_bytes()).hexdigest(),
                "selection": [{"sim_time": r["sim_time"], "phase": r["phase"]} for r in selected]}
    metadata['renderer_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    metadata['experience_sha256'] = hashlib.sha256(replay_experience.read_bytes()).hexdigest()
    metadata['source_stage_sha256'] = hashlib.sha256((media/'scene.usda').read_bytes()).hexdigest()
    metadata['removed_runtime_prims'] = ['/Render']
    metadata['removed_physics_prims'] = removed_physics
    metadata['sensor_timeline_playing'] = False
    metadata['capture_delta_time_s'] = 0.
    metadata['explicit_async_capture'] = True
    metadata['capture_clock'] = 'Frozen render clock; recorded simulation timestamps label the authored poses'
    if physical:
        metadata['payload'] = {
            'source': 'Read-only PhysX audit; linear position and normalized quaternion interpolation',
            'maximum_sample_distance_s': max_payload_sample_gap,
            'sha256': hashlib.sha256(audit_path.read_bytes()).hexdigest()}
    (output / "render.json").write_text(json.dumps(metadata, indent=2))
    print(f"RENDER_DONE {output}", flush=True)


try:
    main()
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
