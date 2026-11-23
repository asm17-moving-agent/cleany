"""Publish a versioned Jetson field dependency tree and pin an inactive release to it."""

import argparse
import fcntl
import hashlib
import os
import stat
import subprocess
import tempfile
from pathlib import Path

STORE = Path("/opt/cleany/shared/field-deps")
OVERLAYS = ("moveit-octomap-lock", "moveit-perception", "moveit-scene-runtime")


def fingerprint(root: Path) -> str:
    """Identify install contents, ownership, modes, attributes and relative links."""
    digest = hashlib.sha256()

    def record(value: bytes) -> None:
        digest.update(len(value).to_bytes(8, "big"))
        digest.update(value)

    for path in [root, *sorted(root.rglob("*"))]:
        info = path.lstat()
        record(os.fsencode(path.relative_to(root)))
        record(f"{info.st_mode}:{info.st_uid}:{info.st_gid}".encode())
        attrs = sorted(os.listxattr(path, follow_symlinks=False))
        record(str(len(attrs)).encode())
        for key in attrs:
            record(os.fsencode(key))
            record(os.getxattr(path, key, follow_symlinks=False))
        if stat.S_ISLNK(info.st_mode):
            target = path.readlink()
            if target.is_absolute() or not path.resolve().is_relative_to(
                root.resolve()
            ):
                raise ValueError(f"Dependency link escapes its install tree: {path}")
            record(os.fsencode(target))
        elif stat.S_ISREG(info.st_mode):
            file_hash = hashlib.sha256()
            with path.open("rb") as handle:
                for block in iter(lambda: handle.read(1024 * 1024), b""):
                    file_hash.update(block)
            record(file_hash.digest())
        elif not stat.S_ISDIR(info.st_mode):
            raise ValueError(f"Unsupported dependency entry: {path}")
    return digest.hexdigest()


def attach(link: Path, version: Path) -> None:
    if link.exists() and not link.is_symlink():
        raise ValueError(f"Existing dependency directory requires migration: {link}")
    if link.is_symlink() and link.readlink() == version:
        return
    with tempfile.TemporaryDirectory(prefix=".pin-", dir=link.parent) as link_dir:
        new_link = Path(link_dir) / "deps"
        new_link.symlink_to(version, target_is_directory=True)
        os.replace(new_link, link)


def inherit(source: Path, link: Path, store: Path = STORE) -> Path:
    """Pin the exact version selected by the active release without copying its files."""
    with (store / ".publish.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not source.is_symlink():
            raise ValueError(
                f"No published field dependencies at {source}; supply --deps-from"
            )
        version = source.readlink()
        if (
            version.parent != store
            or version.is_symlink()
            or not version.is_dir()
            or fingerprint(version) != version.name
        ):
            raise ValueError(f"Invalid or damaged shared dependency version: {version}")
        attach(link, version)
    return version


def pin(install: Path, overlays: Path, link: Path, store: Path = STORE) -> Path:
    """Stage on the store filesystem. Concurrent publishers reuse a verified tree."""
    store.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".stage-", dir=store) as temp:
        staged = Path(temp) / "deps"
        staged.mkdir()
        # Keep the deployed install-root mode independent of the invoking umask.
        staged.chmod(0o775)
        sources = [install, *(overlays / name for name in OVERLAYS)]
        for source, name in zip(sources, ("install", *OVERLAYS)):
            if not source.is_dir():
                raise ValueError(f"Missing field dependency: {source}")
            subprocess.run(
                [
                    "rsync",
                    "-aHAX",
                    "--exclude=__pycache__",
                    "--",
                    f"{source}/",
                    f"{staged / name}/",
                ],
                check=True,
            )
        digest = fingerprint(staged)
        version = store / digest
        with (store / ".publish.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if link.exists() and not link.is_symlink():
                raise ValueError(
                    f"Existing dependency directory requires migration: {link}"
                )
            if version.is_symlink():
                raise ValueError(f"Invalid shared dependency version: {version}")
            if version.exists():
                if fingerprint(version) != digest:
                    raise ValueError(f"Damaged shared dependency version: {version}")
            else:
                staged.rename(version)
            attach(link, version)
        return version


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inherit", type=Path, metavar="ACTIVE_PIN")
    parser.add_argument("install", nargs="?", type=Path)
    parser.add_argument("overlays", nargs="?", type=Path)
    parser.add_argument("link", type=Path)
    args = parser.parse_args()
    if args.inherit and args.install is None and args.overlays is None:
        print(inherit(args.inherit, args.link))
    elif not args.inherit and args.install and args.overlays:
        print(pin(args.install, args.overlays, args.link))
    else:
        parser.error("expected --inherit ACTIVE_PIN LINK or INSTALL OVERLAYS LINK")
