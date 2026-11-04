"""Render explicit CAD-relative deltas without sending robot commands."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from extract_freecad_kinematics import forward_kinematics
from freecad_gripper_linkage import resolved_deltas

ROOT = Path(__file__).resolve().parents[2]


def render(
    package: Path,
    deltas: dict[str, float],
    output: Path,
    frames: int,
    focus_gripper: str | None = None,
) -> None:
    import vtk
    from PIL import Image
    from vtk.util.numpy_support import vtk_to_numpy

    static = json.loads((package / "config/geometry/freecad_final.json").read_text())
    kinematics = json.loads(
        (package / "config/geometry/freecad_kinematics.json").read_text()
    )
    forward_kinematics(kinematics, deltas)
    origins = {
        "base_link": np.zeros(3),
        **{
            s["link"]: np.array(s["point_base_m"])
            for s in [*kinematics["links"], *kinematics.get("fixed_attachments", [])]
        },
    }
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.956, 0.967, 0.98)
    actors = []
    shown_parts = []
    for part in static["parts"]:
        if focus_gripper:
            prefix = focus_gripper.capitalize() + "Kinematic_CameraTool180_Pinc_"
            if not part["name"].startswith(prefix):
                continue
            role = part["name"].removeprefix(prefix)
            if role in ["Top_Plate", "servo", "servo_middle", "servo_other", "CamHorn"]:
                continue
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(package / part["mesh"]))
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(part["color_rgba"][:3])
        if focus_gripper:
            colors = {
                "Cam": (0.95, 0.55, 0.12),
                "Driving": (0.85, 0.22, 0.2),
                "Internal": (0.2, 0.4, 0.8),
                "External": (0.18, 0.65, 0.35),
                "Distal": (0.55, 0.3, 0.7),
            }
            for key, color in colors.items():
                if role.startswith(key):
                    actor.GetProperty().SetColor(color)
            if role == "Bottom_Plate":
                actor.GetProperty().SetOpacity(0.16)
        actor.GetProperty().SetAmbient(0.35)
        actor.GetProperty().SetDiffuse(0.65)
        renderer.AddActor(actor)
        actors.append((actor, kinematics["part_owners"][part["name"]]))
        shown_parts.append(part)
    for plane in [] if focus_gripper else static["image_planes"]:
        points = np.array(plane["vertices_base_link_m"]) + [0, 0, 0.38]
        source = vtk.vtkPlaneSource()
        source.SetOrigin(points[0])
        source.SetPoint1(points[1])
        source.SetPoint2(points[3])
        reader = vtk.vtkPNGReader()
        reader.SetFileName(str(package / "meshes/freecad_final/idle.png"))
        texture = vtk.vtkTexture()
        texture.SetInputConnection(reader.GetOutputPort())
        texture.InterpolateOn()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetTexture(texture)
        actor.GetProperty().SetLighting(False)
        renderer.AddActor(actor)
    light = vtk.vtkLight()
    light.SetPosition(3, -4, 5)
    light.SetFocalPoint(0, 0, 0.6)
    renderer.AddLight(light)
    camera = renderer.GetActiveCamera()
    camera.SetPosition(2.7, -3.6, 2.5)
    camera.SetFocalPoint(0, 0, 0.63)
    camera.SetViewUp(0, 0, 1)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(0.85)
    if focus_gripper:
        bounds = np.array([p["bounds_base_link_m"] for p in shown_parts])
        center = (bounds[:, 0].min(axis=0) + bounds[:, 1].max(axis=0)) / 2 + [
            0,
            0,
            0.38,
        ]
        camera.SetFocalPoint(center)
        camera.SetPosition(center + np.array([0.05, -0.08, 0.2]))
        camera.SetParallelScale(0.09)
        light.SetPosition(center + np.array([0.2, -0.2, 0.4]))
        light.SetFocalPoint(center)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(720, 850)
    window.SetMultiSamples(4)
    window.AddRenderer(renderer)
    images = []
    for index in range(frames):
        fraction = (1 - np.cos(2 * np.pi * index / frames)) / 2
        fk = forward_kinematics(
            kinematics, {name: value * fraction for name, value in deltas.items()}
        )
        for actor, owner in actors:
            transform = fk[owner].copy()
            transform[:3, 3] -= transform[:3, :3] @ origins[owner]
            transform[2, 3] += 0.38
            matrix = vtk.vtkMatrix4x4()
            for i in range(4):
                for j in range(4):
                    matrix.SetElement(i, j, transform[i, j])
            actor.SetUserMatrix(matrix)
        renderer.ResetCameraClippingRange()
        window.Render()
        capture = vtk.vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.Update()
        data = capture.GetOutput()
        width, height, _ = data.GetDimensions()
        pixels = vtk_to_numpy(data.GetPointData().GetScalars()).reshape(
            height, width, -1
        )
        images.append(Image.fromarray(np.flipud(pixels).copy()))
    output.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(output, save_all=True, append_images=images[1:], loop=0, duration=90)
    images[frames // 2].save(output.with_suffix(".png"))
    window.Finalize()
    output.with_suffix(".json").write_text(
        json.dumps(
            {
                "coordinate": kinematics["coordinate"],
                "peak_delta_radians": deltas,
                "peak_resolved_joint_deltas": resolved_deltas(kinematics, deltas),
                "focus_gripper": focus_gripper,
                "view_style": "covers omitted; diagnostic link colors"
                if focus_gripper
                else "source colors",
                "note": "Display inputs only; not limits, physical zeros, or verified safe motion.",
                "source_sha256": kinematics["source_sha256"],
                "kinematics_sha256": hashlib.sha256(
                    (package / "config/geometry/freecad_kinematics.json").read_bytes()
                ).hexdigest(),
                "frames": frames,
                "robot_commands": False,
            },
            indent=2,
        )
        + "\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--package", type=Path, default=ROOT / "ros2_ws/src/cleany_description"
    )
    parser.add_argument(
        "--delta",
        action="append",
        required=True,
        help="cad_delta_joint=degrees; preview displacement only",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--focus-gripper", choices=["right", "left"])
    args = parser.parse_args()
    if args.frames < 2:
        parser.error("--frames must be at least 2")
    deltas = {}
    for entry in args.delta:
        name, degrees = entry.split("=", 1)
        deltas[name] = float(np.deg2rad(float(degrees)))
    render(args.package, deltas, args.output, args.frames, args.focus_gripper)


if __name__ == "__main__":
    main()
