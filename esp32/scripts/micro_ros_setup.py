#!/usr/bin/env python3
"""고정된 micro-ROS 소스를 가져와 ESP-IDF 컴포넌트를 생성합니다."""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import venv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONTROLLER = ROOT / "esp32"
LOCK = CONTROLLER / "micro_ros.lock.json"
OUTPUT = CONTROLLER / "micro_ros"  # 버전 관리에서 제외하는 초기 준비 출력입니다.
PIO_VENV = CONTROLLER / ".venv"


def component_fingerprint(project: Path, environment: str) -> str:
    """인터페이스 원본, 메타데이터와 ESP-IDF 빌드 환경 식별자를 포함합니다."""
    digest = hashlib.sha256()
    digest.update(f"{project.resolve()}:{environment}".encode())
    files = [LOCK, CONTROLLER / "app-colcon.meta",
             CONTROLLER / "sdkconfig.defaults", project / "platformio.ini"]
    interfaces = ROOT / "ros2_ws/src/cleany_base_interfaces"
    files += [interfaces / "CMakeLists.txt", interfaces / "package.xml"]
    files += sorted((interfaces / "msg").glob("*.msg"))
    files += sorted((interfaces / "srv").glob("*.srv"))
    if environment == "esp32-s3-integrated":
        subset = CONTROLLER / "mcu_interfaces/cleany_interfaces"
        files += [subset / name for name in ("CMakeLists.txt", "package.xml", "msg/TofFrame.msg")]
    for file in files:
        digest.update(str(file).encode())
        digest.update(file.read_bytes())
    return digest.hexdigest()


def refresh_component(project: Path, environment: str) -> None:
    """입력이나 IDF 빌드 환경이 바뀌면 파생 타입 지원 코드를 재생성합니다."""
    component = OUTPUT / "micro_ros_espidf_component"
    defaults = hashlib.sha256((CONTROLLER / "sdkconfig.defaults").read_bytes()).hexdigest()
    defaults_stamp = project / f".pio/cleany-sdkconfig-defaults-{environment}"
    if not defaults_stamp.exists() or defaults_stamp.read_text() != defaults:
        # sdkconfig.defaults는 최초 생성에만 적용되고 기존 sdkconfig에는 반영되지 않습니다.
        # 검토된 기본값이 바뀌면 파생 설정을 초기화합니다.
        config = project / f"sdkconfig.{environment}"
        if config.exists():
            config.unlink()
        cache = project / ".pio/build" / environment / "CMakeCache.txt"
        if cache.exists():
            cache.unlink()
        defaults_stamp.parent.mkdir(parents=True, exist_ok=True)
        defaults_stamp.write_text(defaults)
    integrated = environment == "esp32-s3-integrated"
    subset = CONTROLLER / "mcu_interfaces/cleany_interfaces"
    subset_link = project / "extra_ros_packages/cleany_interfaces"
    if integrated:
        subset_link.parent.mkdir(parents=True, exist_ok=True)
        link_generated_config(subset_link, subset)
    elif subset_link.is_symlink() and subset_link.resolve() == subset.resolve():
        subset_link.unlink()
    fingerprint = component_fingerprint(project, environment)
    stamp = component / ".cleany-build-fingerprint"
    if stamp.exists() and stamp.read_text() == fingerprint:
        # upstream은 install 디렉터리 자체를 make 대상으로 사용합니다.
        # colcon 실패 후 남은 디렉터리가 재시도를 막을 수 있습니다.
        if not (component / "libmicroros.a").exists() or not (component / "include").is_dir():
            incomplete = component / "micro_ros_src/install"
            if incomplete.is_dir():
                shutil.rmtree(incomplete)
        return
    # upstream은 CMake 구성 중 libmicroros/includes를 빌드합니다.
    # 프로젝트나 환경이 바뀌면 sdkconfig가 같아도 다시 구성해야 합니다.
    cache = project / ".pio/build" / environment / "CMakeCache.txt"
    if cache.exists():
        cache.unlink()
    for output in ("libmicroros.a", "include", "esp32_toolchain.cmake",
                   "micro_ros_src/build", "micro_ros_src/install", "micro_ros_src/log"):
        path = component / output
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()
    # upstream은 최초 clone 단계에서만 추가 패키지를 복사합니다.
    source = component / "micro_ros_src/src"
    if source.exists():
        package = source / "extra_packages/cleany_base_interfaces"
        if package.exists():
            shutil.rmtree(package)
        shutil.copytree(ROOT / "ros2_ws/src/cleany_base_interfaces", package,
                        ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache"))
        tof_package = source / "extra_packages/cleany_interfaces"
        if tof_package.exists():
            shutil.rmtree(tof_package)
        if integrated:
            shutil.copytree(subset, tof_package)
    stamp.write_text(fingerprint)


def run(*args: str, cwd: Path | None = None) -> None:
    subprocess.run(args, cwd=cwd, check=True)


def checkout(repository: str, revision: str, destination: Path) -> None:
    if destination.exists():
        current = subprocess.check_output(
            ["git", "-C", str(destination), "rev-parse", "HEAD"], text=True
        ).strip()
        if current == revision:
            return
        run("git", "-C", str(destination), "fetch", "--depth=1", "origin", revision)
        run("git", "-C", str(destination), "checkout", "--detach", "FETCH_HEAD")
    else:
        run("git", "clone", "--depth=1", repository, str(destination))
        run("git", "-C", str(destination), "fetch", "--depth=1", "origin", revision)
        run("git", "-C", str(destination), "checkout", "--detach", "FETCH_HEAD")
    actual = subprocess.check_output(
        ["git", "-C", str(destination), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual != revision:
        raise RuntimeError(f"{destination} resolved to {actual}, expected {revision}")


def link_generated_config(link: Path, source: Path, previous_source: Path | None = None) -> None:
    if link.is_symlink() and link.resolve() == source.resolve():
        return
    if (previous_source is not None and link.is_symlink()
            and link.resolve() == previous_source.resolve()):
        link.unlink()
    if link.exists() or link.is_symlink():
        raise RuntimeError(f"unexpected generated link path: {link}")
    link.symlink_to(source)


def link_git_wrapper(link: Path) -> None:
    """알려진 두 저장소 배치만 펌웨어 보조 도구 구조로 이전합니다."""
    source = Path(__file__).with_name("micro_ros_git.py")
    for previous in (ROOT / "tools/micro_ros_git.py",
                     ROOT / "tools/micro_ros/micro_ros_git.py"):
        if link.is_symlink() and link.resolve() == previous.resolve():
            link_generated_config(link, source, previous)
            return
    link_generated_config(link, source)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="verify existing pinned checkouts")
    parser.add_argument("--agent-build", action="store_true")
    args = parser.parse_args()
    if not args.check and sys.version_info[:2] != (3, 10):
        raise RuntimeError(f"firmware setup requires Python 3.10, got {sys.version.split()[0]}")
    lock = json.loads(LOCK.read_text())
    if args.check:
        paths = {
            "component": OUTPUT / "micro_ros_espidf_component",
            "agent": OUTPUT / "agent",
            "micro_ros_msgs": OUTPUT / "agent" / "src" / "micro_ros_msgs",
        }
        for name in paths:
            spec = lock[name]
            path = paths[name]
            actual = subprocess.check_output(
                ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
            ).strip()
            if actual != spec["revision"]:
                raise SystemExit(f"{name}: expected {spec['revision']}, got {actual}")
        component = paths["component"]
        for area in ("micro_ros_dev", "micro_ros_src"):
            for path in (component / area / "src").glob("*"):
                if not (path / ".git").exists():
                    continue
                expected = lock["component_sources"][path.name][1]
                actual = subprocess.check_output(
                    ["/usr/bin/git", "-C", str(path), "rev-parse", "HEAD"], text=True
                ).strip()
                if actual != expected:
                    raise SystemExit(f"{path.name}: expected {expected}, got {actual}")
        return
    OUTPUT.mkdir(exist_ok=True)
    checkout(**lock["component"], destination=OUTPUT / "micro_ros_espidf_component")
    checkout(**lock["agent"], destination=OUTPUT / "agent")
    agent_msgs = OUTPUT / "agent" / "src" / "micro_ros_msgs"
    agent_msgs.parent.mkdir(parents=True, exist_ok=True)
    checkout(**lock["micro_ros_msgs"], destination=agent_msgs)
    git_bin = OUTPUT / "bin"
    git_bin.mkdir(exist_ok=True)
    wrapper = git_bin / "git"
    link_git_wrapper(wrapper)
    if not (PIO_VENV / "bin" / "python").exists():
        venv.EnvBuilder(with_pip=True).create(PIO_VENV)
    run(str(PIO_VENV / "bin" / "pip"), "install", "platformio==6.1.19",
        "catkin_pkg==1.1.1", "colcon-common-extensions==0.3.0",
        "colcon-core==0.21.3", "vcstool==0.3.0", "lark-parser==0.12.0",
        "empy==3.3.4")
    # micro-ROS 컴포넌트의 Makefile이 ROS 소스 패키지를 자체 colcon workspace로
    # 복사하므로 인터페이스의 기준은 계속 이 소스에 둡니다.
    for project in (CONTROLLER, CONTROLLER / "microros_smoke"):
        package_dir = project / "extra_ros_packages" / "cleany_base_interfaces"
        package_dir.parent.mkdir(parents=True, exist_ok=True)
        if package_dir.is_symlink() or package_dir.exists():
            if package_dir.is_symlink() and package_dir.resolve() == (ROOT / "ros2_ws/src/cleany_base_interfaces").resolve():
                continue
            raise RuntimeError(f"unexpected existing generated package path: {package_dir}")
        package_dir.symlink_to(ROOT / "ros2_ws/src/cleany_base_interfaces", target_is_directory=True)
    for generated_link, target in (
        (CONTROLLER / "microros_smoke" / "app-colcon.meta", CONTROLLER / "app-colcon.meta"),
        (CONTROLLER / "microros_smoke" / "sdkconfig.defaults", CONTROLLER / "sdkconfig.defaults"),
    ):
        previous = (CONTROLLER / "sdkconfig.microros.defaults"
                    if generated_link.name == "sdkconfig.defaults" else None)
        link_generated_config(generated_link, target, previous)
    if args.agent_build:
        agent = OUTPUT / "agent"
        run(str(PIO_VENV / "bin" / "colcon"), "build", "--packages-up-to",
            "micro_ros_agent", cwd=agent)
        return
    print(f"PlatformIO: {PIO_VENV / 'bin/platformio'} (6.1.19); run firmware targets via Make")


if __name__ == "__main__":
    main()
