"""Provision the validated real53 runtime once, independently of app releases."""

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path


VARIANT = Path("real53/m")
ENGINE = "yoloe-26m-seg-real53.engine"
ENCODER = "mobileclip2_b.ts"
VERSION_FILES = (ENGINE, "manifest.json", "validation.json", "profile.yaml", "input.lock.json")


def check_digest(path: Path, expected: str) -> None:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected:
        raise ValueError(f"SHA-256 mismatch: {path}")


def active_version(root: Path) -> Path:
    parent = (root / VARIANT).resolve(strict=True)
    active = parent / "active"
    version = active.resolve(strict=True)
    if not active.is_symlink() or version.parent != parent or not version.is_dir():
        raise ValueError(f"Expected active to select a version in {parent}")
    return version


def verify(root: Path) -> None:
    version = active_version(root)
    manifest = json.loads((version / "manifest.json").read_text())
    validation = json.loads((version / "validation.json").read_text())
    lock = json.loads((version / "input.lock.json").read_text())
    engine_hash = manifest["artifacts"][ENGINE]["sha256"]
    if (manifest["status"] != "validated" or validation["passed"] is not True
            or validation["candidate_sha256"] != engine_hash
            or validation["lock_sha256"] != manifest["lock_sha256"]):
        raise ValueError(f"Engine validation does not match the manifest: {version}")
    check_digest(version / ENGINE, engine_hash)
    check_digest(version / "validation.json", manifest["validation"]["sha256"])
    check_digest(version / "input.lock.json", manifest["lock_sha256"])
    check_digest(version / "profile.yaml", lock["profile"]["sha256"])
    check_digest(root / ENCODER, lock["source"]["text_encoder_sha256"])


def provision(source: Path, destination: Path) -> None:
    """Publish a complete first version. Later deployments keep the selected version."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    with (destination.parent / ".yoloe-install.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if os.path.lexists(destination):
            verify(destination)
            return
        version = active_version(source)
        with tempfile.TemporaryDirectory(dir=destination.parent, prefix=".yoloe-") as temp:
            staged = Path(temp) / "yoloe"
            target = staged / VARIANT / version.name
            target.mkdir(parents=True)
            for filename in VERSION_FILES:
                shutil.copyfile(version / filename, target / filename)
            shutil.copyfile(source / ENCODER, staged / ENCODER)
            (target.parent / "active").symlink_to(target.name)
            verify(staged)
            staged.rename(destination)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="YOLOE export root used on first install")
    parser.add_argument("destination", type=Path, help="persistent YOLOE runtime root")
    args = parser.parse_args()
    provision(args.source, args.destination)
