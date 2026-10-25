"""Render recorded PhysX link poses from three observer cameras; no new physics."""
from __future__ import annotations

import argparse
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
app = SimulationApp({"headless": True, "hide_ui": True, "renderer": "RaytracedLighting",
                     "limit_cpu_threads": 4, "disable_viewport_updates": True})
import omni.usd
import omni.timeline
import omni.replicator.core as rep
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.sensors.experimental.rtx import RtxCamera, TiledCameraSensor
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics


def main() -> None:
    if not omni.usd.get_context().open_stage(str(media / "scene.usda")):
        raise RuntimeError("Could not load the recorded scene")
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().pause()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
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
            ("front", "FACILITY 18F / FRONT TRACK", [3., 3., 3.8], [0., 0., .5], [0., 0., 1.]),
            ("side", "FACILITY 18F / HIGH SIDE", [-1., -1., 5.5], [0., 0., .5], [0., 0., 1.]),
            ("top", "FACILITY 18F / FULL FLOOR", [-6., -4.5, 26.], [-6., -4.5, 0.], [0., 1., 0.]),
        ]
    paths = ["/ReplayCameras/"+view[0] for view in views]
    cameras = [RtxCamera(path, tick_rate=0.) for path in paths]
    width, height = 480, 360
    sensor = TiledCameraSensor(paths, resolution=(height, width), annotators=["rgb"])
    order = [sensor.camera.paths.index(path) for path in paths]
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
    output = media / args.name
    output.mkdir()
    all_frames = {name: [] for name, *_ in views}
    combined = []
    max_position_error = 0.
    for index, record in enumerate(selected):
        for operation, pose in zip(operations, record["links"]):
            matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]), Gf.Vec3d(*pose[3:6])))
            matrix.SetTranslateOnly(Gf.Vec3d(*pose[:3]))
            operation.Set(matrix)
        if scene["world"] == "facility18":
            base = np.asarray(record["links"][base_index][:3])
            base[2] = 0.
            for operation, (_, _, eye, target, up) in zip(camera_operations[:2], views[:2]):
                operation.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*(base+eye)),
                    Gf.Vec3d(*(base+target)), Gf.Vec3d(*up)).GetInverse())
        RenderingManager.render()
        rep.orchestrator.step(rt_subframes=2, delta_time=0., pause_timeline=True, wait_for_render=True)
        cache = UsdGeom.XformCache()
        for operation, pose in zip(operations, record["links"]):
            position = cache.GetLocalToWorldTransform(operation.GetAttr().GetPrim()).ExtractTranslation()
            error = float(np.linalg.norm(np.asarray(position)-np.asarray(pose[:3])))
            max_position_error = max(max_position_error, error)
        assert max_position_error < 1e-5, "Rendering changed recorded link positions"
        data, _ = sensor.get_data("rgb")
        if data is None:
            raise RuntimeError("RTX camera returned no frame")
        rgb = data.numpy() if hasattr(data, "numpy") else np.asarray(data)
        rgb = rgb[order]
        assert rgb.shape == (3, height, width, 3), rgb.shape
        montage = Image.new("RGB", (3*width, height+70), "#101827")
        for camera_index, (name, title, *_rest) in enumerate(views):
            frame = Image.new("RGB", (width, height+70), "#101827")
            frame.paste(Image.fromarray(rgb[camera_index]), (0, 32))
            draw = ImageDraw.Draw(frame)
            draw.text((12, 7), title, font=font, fill="white")
            phase = "TO DESK" if record["phase"] == "NAVIGATE_TO_TARGET" else "RETURN HOME"
            draw.text((12, height+37), f"{phase} | sim {record['sim_time']:.1f}s | replay {10*args.sample_period:g}x", font=small, fill="white")
            draw.text((12, height+53), "Actual Nav2 motion; cleanup mocked", font=small, fill="#a8bbd5")
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
                "cameras": views, "tracking": "front/side XY follow recorded base" if scene["world"] == "facility18" else "fixed", "source_sha256": hashlib.sha256((media / "motion.jsonl").read_bytes()).hexdigest(),
                "selection": [{"sim_time": r["sim_time"], "phase": r["phase"]} for r in selected]}
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
