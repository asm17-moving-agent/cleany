import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest

from tools.ros2 import provision_yoloe as model


@pytest.fixture
def export(tmp_path):
    root = tmp_path / "export"
    version = root / model.VARIANT / "test-version"
    version.mkdir(parents=True)
    (version.parent / "active").symlink_to(version.name)

    def write(path, content):
        path.write_bytes(content)
        return hashlib.sha256(content).hexdigest()

    engine = write(version / model.ENGINE, b"engine")
    profile = write(version / "profile.yaml", b"profile")
    encoder = write(root / model.ENCODER, b"encoder")
    lock = write(version / "input.lock.json", json.dumps({
        "profile": {"sha256": profile}, "source": {"text_encoder_sha256": encoder},
    }).encode())
    validation = write(version / "validation.json", json.dumps({
        "passed": True, "candidate_sha256": engine, "lock_sha256": lock,
    }).encode())
    (version / "manifest.json").write_text(json.dumps({
        "status": "validated", "lock_sha256": lock, "validation": {"sha256": validation},
        "artifacts": {model.ENGINE: {"sha256": engine}},
    }))
    return root


def test_invalid_bundle_does_not_publish(export, tmp_path):
    (export / model.ENCODER).write_bytes(b"damaged")
    destination = tmp_path / "models/yoloe"
    with pytest.raises(ValueError, match="SHA-256"):
        model.provision(export, destination)
    assert not destination.exists()


def test_concurrent_first_install_and_redeploy_without_export(export, tmp_path):
    destination = tmp_path / "models/yoloe"
    command = [sys.executable, model.__file__, str(export), str(destination)]
    with subprocess.Popen(command) as first, subprocess.Popen(command) as second:
        assert first.wait() == second.wait() == 0
    assert (destination / model.VARIANT / "active").readlink() == Path("test-version")
    model.provision(tmp_path / "missing-export", destination)
    (destination / model.ENCODER).write_bytes(b"damaged")
    with pytest.raises(ValueError, match="SHA-256"):
        model.provision(export, destination)
