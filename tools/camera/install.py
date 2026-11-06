#!/usr/bin/env python3
"""Install the host camera initialization files on Jetson Orin."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile


REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER = Path("usr/local/libexec/cleany/camera-orientation.py")
UNIT = Path("etc/systemd/system/cleany-camera-orientation@.service")
RULE = Path("etc/udev/rules.d/70-cleany-camera-orientation.rules")
FILES = (
    (Path("tools/camera/orientation.py"), HELPER, 0o755),
    (
        Path("configs/host/systemd/cleany-camera-orientation@.service"),
        UNIT,
        0o644,
    ),
    (Path("configs/host/udev/70-cleany-camera-orientation.rules"), RULE, 0o644),
)


def require_jetson_host() -> None:
    """Reject live installation outside the supported native host."""
    if os.geteuid() != 0:
        raise RuntimeError("Run this installation with sudo on the Jetson host.")
    release = platform.freedesktop_os_release()
    if (
        release.get("ID") != "ubuntu"
        or release.get("VERSION_ID") != "22.04"
        or platform.machine() != "aarch64"
    ):
        raise RuntimeError("Live installation requires Ubuntu 22.04 aarch64.")
    compatible = Path("/proc/device-tree/compatible").read_text().split("\0")
    if "nvidia,tegra234" not in compatible:
        raise RuntimeError("Live installation requires a Jetson Orin host.")
    if Path("/proc/1/comm").read_text().strip() != "systemd":
        raise RuntimeError("Run on the native systemd host, not in a container.")
    for command in ("systemctl", "udevadm"):
        if shutil.which(command) is None:
            raise RuntimeError(f"Required host command is missing: {command}")


def check_destination(root: Path, relative: Path) -> Path:
    """Refuse symlink destinations, including intermediate directories."""
    target = root
    for component in relative.parts:
        target = target / component
        if target.is_symlink():
            raise RuntimeError(f"Refusing symlink installation path: {target}")
        if root == Path("/") and target.exists():
            metadata = target.stat()
            if metadata.st_uid != 0 or metadata.st_mode & 0o022:
                raise RuntimeError(
                    f"Installation path must be root-owned and safe: {target}"
                )
    return target


def install_file(root: Path, source: Path, relative: Path, mode: int) -> None:
    target = check_destination(root, relative)
    target.parent.mkdir(parents=True, exist_ok=True, mode=0o755)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as output:
            temporary = Path(output.name)
            output.write(source.read_bytes())
            output.flush()
            os.fsync(output.fileno())
            os.fchmod(output.fileno(), mode)
            if root == Path("/"):
                os.fchown(output.fileno(), 0, 0)
        os.replace(temporary, target)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def install(root: Path) -> None:
    # Preflight all sources and destinations before replacing any live file.
    for source, relative, _ in FILES:
        if not (REPO_ROOT / source).is_file():
            raise RuntimeError(f"Missing installation source: {source}")
        check_destination(root, relative)
    for source, relative, mode in FILES:
        install_file(root, REPO_ROOT / source, relative, mode)
    if root == Path("/"):
        subprocess.run(["systemctl", "daemon-reload"], check=True)
        subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
        print("Installed. Reconnect the camera before starting capture applications.")
    else:
        print(f"Staged installation at {root}; no host commands were run.")


def uninstall(root: Path) -> None:
    targets = [check_destination(root, path) for path in (RULE, UNIT, HELPER)]
    # Remove the trigger first, then finish any running helper before deleting it.
    targets[0].unlink(missing_ok=True)
    if root == Path("/"):
        subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
        subprocess.run(
            ["systemctl", "stop", "cleany-camera-orientation@*.service"],
            check=True,
        )
    for target in targets[1:]:
        target.unlink(missing_ok=True)
    if root == Path("/"):
        subprocess.run(["systemctl", "daemon-reload"], check=True)
    print(f"Removed camera initialization files from {root}.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("install", "uninstall"))
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("/"),
        help="Stage files under this directory without running host commands.",
    )
    args = parser.parse_args(argv)
    try:
        root = args.root.resolve()
        if root == Path("/"):
            require_jetson_host()
        elif root.exists() and not root.is_dir():
            raise RuntimeError(f"Staging root is not a directory: {root}")
        if args.action == "install":
            install(root)
        else:
            uninstall(root)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Camera installation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
