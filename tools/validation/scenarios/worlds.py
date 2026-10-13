"""Create seat-specific worlds while preserving every other model."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def chair_name(seat: str) -> str:
    """좌석 ID를 해당 fixture의 Gazebo 의자 모델 이름으로 변환한다."""
    if re.fullmatch(r"seat-\d{2}", seat):
        return "office_chair_" + seat[5:]
    match = re.fullmatch(r"seat-([am]\d)-(\d{2})", seat)
    if match:
        room, number = match.groups()
        return "facility_chair_" + room.upper() + str(int(number))
    raise ValueError(f"Unsupported seat ID: {seat}")


def restored_tree(source: Path, reference: Path) -> ET.ElementTree:
    """기준 월드에서 빠진 의자를 복원해 좌석별 실행의 출발점을 맞춘다."""
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    original = ET.parse(reference).getroot().find("world")
    if world is None or original is None:
        raise ValueError("SDF must contain a world")
    for number in range(1, 49):
        name = f"office_chair_{number:02d}"
        if world.find(f"model[@name='{name}']") is None:
            model = original.find(f"model[@name='{name}']")
            if model is None:
                raise ValueError(f"Missing reference chair: {name}")
            world.append(copy.deepcopy(model))
    return tree


def prepare(source: Path, reference: Path, seat: str, output: Path) -> dict:
    """대상 의자 하나만 제거하고 다른 모델이 바뀌지 않았는지 확인한다."""
    tree = restored_tree(source, reference)
    world = tree.getroot().find("world")
    name = chair_name(seat)
    matches = world.findall(f"model[@name='{name}']")
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one chair for {seat}: {name}")
    before = {m.get("name"): ET.tostring(m) for m in world.findall("model")}
    world.remove(matches[0])
    after = {m.get("name"): ET.tostring(m) for m in world.findall("model")}
    if set(before) - set(after) != {name} or any(after[n] != before[n] for n in after):
        raise ValueError("Unexpected model changes")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    evidence = dict(
        seat=seat,
        removed_chair=name,
        model_count_before=len(before),
        model_count_after=len(after),
        sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
    )
    output.with_suffix(".manifest.json").write_text(json.dumps(evidence, indent=2))
    return evidence
