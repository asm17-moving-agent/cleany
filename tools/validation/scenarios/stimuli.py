"""Explicit Gazebo/ROS test stimuli; never modify runtime safety parameters."""

from __future__ import annotations
import math
import json
import hashlib
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def add_obstacle(world_file: Path) -> str:
    """Stage an obstacle off-route while keeping it movable as a separate entity."""
    tree = ET.parse(world_file)
    world = tree.getroot().find("world")
    name = "validation_obstacle"
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "50 50 .4 0 0 0"
    link = ET.SubElement(model, "link", name="body")
    for kind in ("collision", "visual"):
        shape = ET.SubElement(link, kind, name=kind)
        box = ET.SubElement(ET.SubElement(shape, "geometry"), "box")
        ET.SubElement(box, "size").text = ".25 .65 .8"
    # Keep this controllable entity out of the immutable static scene grouping.
    ET.SubElement(world, "frame", name="validation_obstacle_frame", attached_to=name)
    tree.write(world_file, encoding="utf-8", xml_declaration=True)
    return world.get("name")


def move_obstacle(world: str, x: float, y: float, yaw: float) -> dict:
    req = f'name: "validation_obstacle" position: {{x: {x} y: {y} z: .4}} orientation: {{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}'
    command = [
        "ign",
        "service",
        "-s",
        f"/world/{world}/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        req,
    ]
    response = subprocess.run(command, text=True, capture_output=True, timeout=6)
    if response.returncode or "true" not in response.stdout:
        raise RuntimeError(
            f"Gazebo set_pose failed: {response.stdout} {response.stderr}"
        )
    return dict(command=command, response=response.stdout)


def entity_request(world: str, service: str, request_type: str, request: str) -> dict:
    command = [
        "ign",
        "service",
        "-s",
        f"/world/{world}/{service}",
        "--reqtype",
        request_type,
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        request,
    ]
    result = transport_request(command)
    result["acknowledged"] = result["returncode"] == 0 and bool(
        re.search(r"\bdata:\s*true\b", result["response"])
    )
    return result


def transport_request(command: list[str]) -> dict:
    """Retain partial output and timeout evidence without retrying a mutation."""
    start = time.monotonic()
    try:
        response = subprocess.run(command, text=True, capture_output=True, timeout=6)
        result = {
            "command": command,
            "response": response.stdout,
            "stderr": response.stderr,
            "returncode": response.returncode,
            "process_timeout": False,
        }
    except subprocess.TimeoutExpired as error:

        def text(value):
            return (
                value.decode(errors="replace")
                if isinstance(value, bytes)
                else value or ""
            )

        result = {
            "command": command,
            "response": text(error.stdout),
            "stderr": text(error.stderr),
            "returncode": None,
            "process_timeout": True,
        }
    return {**result, "wall_start": start, "wall_end": time.monotonic()}


def switch_chair(
    world: str,
    restored_model: ET.Element,
    remove_name: str,
    all_chairs: set[str],
    output: Path,
) -> list[dict]:
    """Reconcile ambiguous replies against the scene before retrying mutations."""
    restored_name = restored_model.get("name")
    if (
        restored_name not in all_chairs
        or remove_name not in all_chairs
        or restored_name == remove_name
    ):
        raise ValueError("Invalid chair transition")
    output.mkdir(parents=True, exist_ok=False)
    evidence = {
        "restored": restored_name,
        "removed": remove_name,
        "requests": [],
        "scenes": [],
    }

    def save():
        temporary = output / "transition.json.tmp"
        temporary.write_text(json.dumps(evidence, indent=2))
        temporary.replace(output / "transition.json")

    def observe(label):
        names, scene = read_scene(world, output / (label + ".pbtxt"))
        evidence["scenes"].append(scene)
        save()
        return names

    sdf = ET.Element("sdf", version="1.8")
    sdf.append(restored_model)
    operations = [
        (
            "create",
            "ignition.msgs.EntityFactory",
            "sdf: "
            + json.dumps(ET.tostring(sdf, encoding="unicode"))
            + " allow_renaming: false",
        ),
        ("remove", "ignition.msgs.Entity", f'name: "{remove_name}" type: MODEL'),
    ]
    try:
        before = observe("before")
        if before & all_chairs != all_chairs - {restored_name}:
            raise RuntimeError("Gazebo chair transition precondition mismatch")
        for service, request_type, request in operations:
            expected = (
                before | {restored_name}
                if service == "create"
                else before - {remove_name}
            )
            for attempt in range(1, 4):
                reply = entity_request(world, service, request_type, request)
                evidence["requests"].append(
                    {"service": service, "attempt": attempt, **reply}
                )
                save()
                observed = observe(f"{service}-{attempt}")
                if observed == expected:
                    evidence["requests"][-1]["scene_confirmed"] = True
                    save()
                    before = observed
                    break
                if observed != before:
                    raise RuntimeError(
                        f"Gazebo {service} changed unexpected scene models"
                    )
                # The scene proves that the requested change is not visible.
                # Duplicate queued creates cannot rename or add another chair.
                if attempt == 3:
                    raise RuntimeError(
                        f"Gazebo {service} not applied after three attempts"
                    )
                time.sleep(0.1)
        evidence["verified"] = True
        save()
        return evidence["requests"]
    except (Exception, KeyboardInterrupt) as error:
        evidence["error"] = repr(error)
        save()
        raise


def scene_model_names(text: str) -> set[str]:
    """Read top-level model names from transport's protobuf text response."""
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[A-Za-z_][A-Za-z_0-9]*|[{}:]', text)
    stack = []
    names = set()
    models = 0
    for index, token in enumerate(tokens):
        if token == "{":
            if index == 0:
                raise ValueError("Malformed scene response")
            stack.append(tokens[index - 1])
            if stack == ["model"]:
                models += 1
        elif token == "}":
            if not stack:
                raise ValueError("Malformed scene response")
            stack.pop()
        elif stack == ["model"] and token == "name":
            if tokens[index + 1 : index + 2] != [":"] or index + 2 >= len(tokens):
                raise ValueError("Missing model name")
            name = json.loads(tokens[index + 2])
            if not isinstance(name, str) or name in names:
                raise ValueError("Invalid or duplicate model name")
            names.add(name)
    if stack or not names or models != len(names):
        raise ValueError("Incomplete scene response")
    return names


def read_scene(world: str, output: Path | None = None) -> tuple[set[str], dict]:
    """Retry only failed read requests; preserve every reply and reject malformed scenes."""
    command = [
        "ign",
        "service",
        "-s",
        f"/world/{world}/scene/info",
        "--reqtype",
        "ignition.msgs.Empty",
        "--reptype",
        "ignition.msgs.Scene",
        "--timeout",
        "3000",
        "--req",
        "",
    ]
    attempts = []
    for attempt in range(1, 4):
        response = transport_request(command)
        raw = response.pop("response")
        response["scene_sha256"] = hashlib.sha256(raw.encode()).hexdigest()
        if output:
            output.parent.mkdir(parents=True, exist_ok=True)
            raw_file = output.with_name(output.name + f".attempt-{attempt}")
            with raw_file.open("x") as stream:
                stream.write(raw)
            response["scene_file"] = str(raw_file)
        attempts.append(response)
        if output:
            with output.with_name(output.name + f".attempt-{attempt}.json").open(
                "x"
            ) as stream:
                json.dump(response, stream, indent=2)
        if response["returncode"] == 0 and raw.strip():
            models = scene_model_names(raw)
            if output:
                with output.open("x") as stream:
                    stream.write(raw)
            return models, {
                "model_count": len(models),
                "command": command,
                "query_attempts": attempts,
                "scene_file": str(output) if output else None,
                "scene_sha256": response["scene_sha256"],
            }
        if attempt < 3:
            time.sleep(0.1)
    raise RuntimeError(
        "Gazebo scene query failed after three attempts: " + json.dumps(attempts)
    )


def verify_chair_models(
    world: str, all_chairs: set[str], removed: str, output: Path | None = None
) -> dict:
    """Verify that only the requested chair is absent from the live scene."""
    models, evidence = read_scene(world, output)
    observed = models & all_chairs
    expected = all_chairs - {removed}
    if observed != expected:
        raise RuntimeError(
            f"Gazebo chair mismatch: missing={sorted(expected-observed)}, extra={sorted(observed-expected)}"
        )
    return {"removed": removed, "observed_chairs": sorted(observed), **evidence}


def obstacle_ahead(current: dict, previous: dict, distance: float) -> list[float]:
    """Use observed world displacement; omni travel need not follow body yaw."""
    dx = current["x"] - previous["x"]
    dy = current["y"] - previous["y"]
    if (
        not all(
            math.isfinite(v) for v in (current["x"], current["y"], dx, dy, distance)
        )
        or distance <= 0
        or math.hypot(dx, dy) <= 0.001
    ):
        raise ValueError("A finite observed motion direction is required")
    angle = math.atan2(dy, dx)
    return [
        current["x"] + distance * math.cos(angle),
        current["y"] + distance * math.sin(angle),
        angle,
    ]
