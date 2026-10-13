"""Prepare a separate simulation fixture without modifying its source assets."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import yaml
import xml.etree.ElementTree as ET
from assets import localize_and_hash
from worlds import restored_tree

CONFIG_FILES = {"config/wheel_odometry.yaml", "config/nav2_safety.yaml"}


def merge(base: dict, overlay: dict) -> dict:
    """원본을 보존하면서 지정된 설정만 재귀적으로 덮어쓴다."""
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict):
            previous = result.get(key, {})
            if not isinstance(previous, dict):
                raise ValueError(f"Cannot merge mapping into scalar: {key}")
            result[key] = merge(previous, value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def prepare(base: Path, output: Path, profile: Path) -> dict:
    """수정 설정은 새 파일로 만들고 나머지 입력은 원본에 연결한다."""
    base, output, profile = base.resolve(), output.absolute(), profile.resolve()
    if not base.is_dir() or output.resolve().is_relative_to(base):
        raise ValueError("Output must be separate from an existing source fixture")
    if output.exists() or output.is_symlink():
        raise FileExistsError(output)
    raw_profile = profile.read_bytes()
    patches = yaml.safe_load(raw_profile)
    if not isinstance(patches, dict) or set(patches) != CONFIG_FILES:
        raise ValueError(
            "Profile must contain the odometry and navigation configuration overlays"
        )

    # Validate and serialize before creating output. Modified configs are new
    # regular files even when the baseline points to another fixture by symlink.
    prepared, configs = {}, {}
    for name, overlay in patches.items():
        raw = (base / name).read_bytes()
        document = yaml.safe_load(raw)
        if not isinstance(document, dict) or not isinstance(overlay, dict):
            raise ValueError(f"Configuration must be a mapping: {name}")
        encoded = yaml.safe_dump(merge(document, overlay), sort_keys=False).encode()
        prepared[name] = encoded
        configs[name] = {
            "source_sha256": digest(raw),
            "prepared_sha256": digest(encoded),
        }
    tree = (
        restored_tree(base / "world.sdf", base / "world.study.sdf")
        if (base / "world.study.sdf").exists()
        else ET.parse(base / "world.sdf")
    )
    assets = localize_and_hash(tree, base, Path(__file__).resolve().parents[3])
    output.mkdir(parents=True, exist_ok=False)
    (output / "config").mkdir()
    for entry in base.iterdir():
        if entry.name in {"config", "profile-manifest.json"}:
            continue
        (output / entry.name).symlink_to(
            entry.resolve(), target_is_directory=entry.is_dir()
        )
    for entry in (base / "config").iterdir():
        if "config/" + entry.name not in prepared:
            (output / "config" / entry.name).symlink_to(
                entry.resolve(), target_is_directory=entry.is_dir()
            )
    for name, encoded in prepared.items():
        (output / name).write_bytes(encoded)
    manifest = {
        "source_fixture": str(base),
        "profile": str(profile),
        "profile_sha256": digest(raw_profile),
        "configs": configs,
        "asset_hashes": assets,
        "scope": "Simulation fixture preparation only; no mission success is inferred.",
    }
    (output / "profile-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--profile", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.base, args.output, args.profile), indent=2))


if __name__ == "__main__":
    main()
