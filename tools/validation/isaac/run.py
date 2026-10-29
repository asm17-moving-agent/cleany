"""Run an isolated Isaac/Mission attempt and preserve failures as evidence."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

from build import source_digest

SOURCE = Path(__file__).resolve().parents[3]


def write(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+"\n")


def memory() -> dict:
    gpu = float(subprocess.check_output(["nvidia-smi", "--query-gpu=memory.free",
                                        "--format=csv,noheader,nounits"], text=True, timeout=5).splitlines()[0])
    host = int(next(line.split()[1] for line in Path("/proc/meminfo").read_text().splitlines()
                    if line.startswith("MemAvailable:")))/1024
    return {"gpu_free_mib": gpu, "host_available_mib": host}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New, empty attempt directory")
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--ros-image", default="localhost/cleany-mission-isaac-ros:humble")
    parser.add_argument("--isaac-image", default="localhost/cleany-isaac-visual:6.1.0")
    parser.add_argument("--domain", type=int, default=174)
    parser.add_argument("--cache-root", type=Path, help="Dedicated cache shared by this task's sequential attempts")
    parser.add_argument("--scenario", choices=["success", "navigation_failure", "manipulation_failure", "cancel"], default="success")
    parser.add_argument("--timeout", type=float, default=660.)
    parser.add_argument("--record-motion", action="store_true", help="Save exact PhysX link poses for offline multi-view GIF rendering")
    parser.add_argument("--profile", action="store_true", help="Measure adapter stages and host/process CPU/GPU load")
    parser.add_argument("--native-profile", action="store_true", help="Capture a bounded native CPU trace during motion")
    parser.add_argument("--publication-profile", choices=["legacy", "split"], default="split")
    parser.add_argument("--physics-sync", choices=["usd", "fabric"], default="fabric")
    parser.add_argument("--require-no-recovery", action="store_true", help="Reject navigation recovery, planner failure or costmap-clear events")
    parser.add_argument("--world", choices=["room", "facility18"], default="room")
    parser.add_argument("--target-id", type=str)
    parser.add_argument("--chair-mesh", type=Path, default=Path.home()/".ignition/fuel/fuel.gazebosim.org/openrobotics/models/officechairgrey/1/meshes/OfficeChairGrey.obj")
    parser.add_argument("--facility-map", type=Path, help="Existing facility_integration YAML; image must be next to it")
    parser.add_argument("--cleanup", action="store_true", help="Sensor-driven physical desk collection")
    parser.add_argument('--cleanup-backend', choices=['physical', 'mock'], default='physical',
                        help='Explicit Mission cleanup test double; physical remains the default')
    parser.add_argument("--cleanup-object", choices=['cup','tissue'], default='cup')
    parser.add_argument("--cleanup-profile", choices=['canonical','slender-cup-95mm'], default='canonical')
    parser.add_argument("--model-directory", type=Path)
    args = parser.parse_args()
    if args.cleanup_profile != 'canonical' and (not args.cleanup or args.cleanup_object != 'cup'):
        parser.error("The slender cup profile requires --cleanup --cleanup-object cup")
    physical = args.cleanup and args.cleanup_backend == 'physical'
    if args.cleanup_backend == 'mock' and not args.cleanup:
        parser.error('--cleanup-backend requires --cleanup')
    if args.cleanup and args.world != 'facility18':
        parser.error('Cleanup fixture requires Facility 18')
    if physical and (args.scenario != 'success' or not args.model_directory):
        parser.error('Physical cleanup requires success scenario and a downloaded model directory')
    if args.target_id and not __import__("re").fullmatch(r"[A-Za-z0-9_-]+", args.target_id):
        parser.error("Invalid target ID")
    if args.world == "facility18" and not args.facility_map:
        parser.error("--facility-map is required for facility18")
    if not 1 <= args.domain <= 232 or args.domain == 73:
        parser.error("Use an isolated DDS domain in 1..232 (73 is used by existing Isaac work)")
    output, build = args.output.resolve(), args.build_root.resolve()
    if output.exists():
        parser.error("Output must be a new directory; each attempt preserves its own journal")
    if not (build / "install/setup.bash").is_file():
        parser.error("Build the ROS dependencies before running")
    build_record = json.loads((build / "manifest.json").read_text())
    if build_record["status"] != "passed" or build_record["source_digest"] != source_digest(SOURCE):
        parser.error("ROS build is stale or failed; rerun build.py")
    output.mkdir(parents=True)
    for name in ("config", "logs", "evaluations", "provenance", "cache"):
        (output/name).mkdir()
    names = [f"cleany-e2e-{args.domain}-{part}" for part in ("isaac", "ros", "validate")]
    cache = (args.cache_root or build / f"isaac-cache-{args.domain}").resolve()
    cache.mkdir(parents=True, exist_ok=True)
    for name in ("driver", "compute"):
        (cache / name).mkdir(exist_ok=True)
    processes = []
    report = {"status": "running", "started_at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
              "source_commit": subprocess.check_output(["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip(),
              "scenario": args.scenario, "domain": args.domain, "world": args.world,
              "target_id": args.target_id, "record_motion": args.record_motion, "profile": args.profile,
              "cleanup": args.cleanup, "cleanup_object": args.cleanup_object,
              "cleanup_profile": args.cleanup_profile,
              'cleanup_backend': args.cleanup_backend if args.cleanup else 'mock',
              "native_profile": args.native_profile,
              "publication_profile": args.publication_profile,
              "physics_sync": args.physics_sync,
              "cache_root": str(cache), "commands": [],
              "source_sha256": {str(p.relative_to(SOURCE)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for root in (SOURCE/"tools/validation/isaac", SOURCE/"containers/isaac")
                  for p in root.rglob("*") if p.is_file() and "__pycache__" not in p.parts}}
    for key in ("ros_image", "isaac_image"):
        report[key] = subprocess.check_output(["podman", "image", "inspect", "--format", "{{.Id}}",
                                               getattr(args, key)], text=True).strip()
    navigation = json.loads((Path(__file__).parent / "navigation_source.json").read_text())
    report["navigation_source"] = {key: navigation[key] for key in ("branch", "commit", "strategy")}
    if report["ros_image"] != build_record["image"]:
        parser.error("ROS image differs from the build image; rebuild first")
    write(output / "provenance/build.json", build_record)
    snapshot = output / "provenance/source"
    shutil.copytree(SOURCE / "tools/validation/isaac", snapshot,
                    ignore=shutil.ignore_patterns("__pycache__"))
    report["git_status"] = subprocess.check_output(["git", "-C", str(SOURCE), "status", "--porcelain"], text=True)
    mounts = ["-v", f"{SOURCE}:/main:ro", "-v", f"{output}:/output:rw", "-v", f"{snapshot}:/integration:ro"]
    if physical:
        mounts += ["-v", f"{args.model_directory.resolve()}:/models:rw"]
        for asset in ('yoloe/yoloe-26l-seg.pt', 'yoloe/mobileclip2_b.ts', 'qwen3vl-4b/config.json'):
            if not (args.model_directory/asset).is_file():
                parser.error(f"Missing downloaded model asset: {asset}")
    env = ["--network", "host", "-e", f"ROS_DOMAIN_ID={args.domain}", "-e", "ROS_LOCALHOST_ONLY=0",
           "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp", "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/output/config/fastdds.xml"]
    prefix = ["podman", "run", "--rm", "--entrypoint", "bash"]
    ros_setup = "source /opt/ros/humble/setup.bash && source /build/install/setup.bash && "
    start_time = time.monotonic()
    with ExitStack() as stack:
        lock = stack.enter_context((build / f".isaac-domain-{args.domain}.lock").open("a"))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        build_lock = stack.enter_context((build/".build.lock").open("a"))
        fcntl.flock(build_lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        resource_log = stack.enter_context((output / "evaluations/resources.jsonl").open("w", buffering=1))

        def check():
            resources = memory()
            resource_log.write(json.dumps({"elapsed": time.monotonic()-start_time, **resources})+"\n")
            if resources["gpu_free_mib"] < 1024 or resources["host_available_mib"] < 4096:
                raise RuntimeError("Insufficient memory headroom; stopping only this attempt")
            if time.monotonic()-start_time > args.timeout:
                raise TimeoutError("Attempt exceeded wall-clock deadline")

        def start(command, label):
            report["commands"].append(command)
            write(output / "provenance/run.json", report)
            log = stack.enter_context((output / f"logs/{label}.log").open("w"))
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        try:
            resources = memory()
            if resources["gpu_free_mib"] < 2048 or resources["host_available_mib"] < 10000:
                raise RuntimeError(f"Insufficient startup headroom: {resources}")
            if args.profile:
                start([sys.executable, str(snapshot / "profile_system.py"), "--output", str(output),
                       "--domain", str(args.domain), "--timeout", str(args.timeout)], "profile-system")
            shutil.copyfile(SOURCE/"containers/isaac"/("fastdds-rgbd.xml" if args.cleanup else "fastdds.xml"), output/"config/fastdds.xml")
            if args.world == "facility18":
                (output / "assets").mkdir()
                shutil.copyfile(args.chair_mesh, output / "assets/OfficeChairGrey.obj")
                shutil.copyfile(args.facility_map, output / "assets/facility_integration.yaml")
                shutil.copyfile(args.facility_map.with_suffix(".pgm"), output / "assets/facility_integration.pgm")
            prepare = start(prefix+mounts+[report["ros_image"], "-c",
                "source /opt/ros/humble/setup.bash && python3 /integration/prepare.py --world "+args.world+
                (" --target-id "+args.target_id if args.target_id else "")+
                " --publication-profile "+args.publication_profile+
                " --physics-sync "+args.physics_sync+(" --cleanup --cleanup-object "+args.cleanup_object+
                " --cleanup-profile "+args.cleanup_profile+" --cleanup-backend "+args.cleanup_backend
                if args.cleanup else "")], "prepare")
            if prepare.wait(timeout=90):
                raise RuntimeError("Model export failed; see logs/prepare.log")
            isaac = start(prefix+["--name", names[0], "--user", "0", "--device", "nvidia.com/gpu=all",
                "--security-opt", "label=disable", "--shm-size=2g"]+env+mounts+
                ["-v", f"{cache}:/root/.cache/ov:rw", "-v", f"{cache}/driver:/isaac-sim/kit/cache/nv_shadercache:rw",
                 "-v", f"{cache}/compute:/root/.nv/ComputeCache:rw", "-e", "ACCEPT_EULA=Y", "-e", "PRIVACY_CONSENT=N",
                 "-e", "OMNI_KIT_ALLOW_ROOT=1", "-e", "ROS_DISTRO=humble", "-e", "PYTHONUNBUFFERED=1",
                 "-e", f"CLEANY_RECORD_MOTION={int(args.record_motion)}",
                 "-e", f"CLEANY_PROFILE={int(args.profile)}",
                 "-e", f"CLEANY_NATIVE_PROFILE={int(args.native_profile)}",
                 "-e", "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/humble/lib"]+
                ([] if args.cleanup else ["-e", "OMP_NUM_THREADS=4"])+[
                 report["isaac_image"], "-c", "cd /isaac-sim && ./python.sh /integration/scene.py"], "isaac")
            startup_deadline = time.monotonic()+180.
            while not (output / "evaluations/adapter-ready.json").exists():
                check()
                if isaac.poll() is not None:
                    raise RuntimeError("Isaac exited before readiness; see logs/isaac.log")
                if time.monotonic() >= startup_deadline:
                    raise TimeoutError("Isaac sensor publication did not become ready within 180s")
                time.sleep(2.)
            arm = start(prefix+["--name", names[2]]+env+mounts+["-v", f"{build}:/build:ro", report["ros_image"],
                "-c", ros_setup+"python3 /integration/validate_arm.py"], "arm")
            while arm.poll() is None:
                check()
                if isaac.poll() is not None:
                    raise RuntimeError("Isaac exited during arm validation")
                time.sleep(2.)
            if arm.returncode:
                raise RuntimeError("Arm control validation failed; see evaluations/arm.json")
            print("Isaac adapter and arm control ready; starting real Mission/Nav2", flush=True)
            inference_gpu = ["--device", "nvidia.com/gpu=all", "--security-opt", "label=disable"] if physical else []
            ros = start(prefix+["--name", names[1]]+inference_gpu+env+mounts+["-v", f"{build}:/build:ro", report["ros_image"],
                "-c", ros_setup+f"ros2 launch /integration/mission.launch.py scenario:={args.scenario}"], "mission")
            validator = start(prefix+["--name", names[2]]+env+mounts+["-v", f"{build}:/build:ro", report["ros_image"],
                "-c", ros_setup+f"python3 /integration/validate_mission.py --scenario {args.scenario}"+
                (" --require-no-recovery" if args.require_no_recovery else "")], "validate")
            while validator.poll() is None:
                check()
                if isaac.poll() is not None or ros.poll() is not None:
                    raise RuntimeError("Simulation or ROS launch exited during validation")
                time.sleep(2.)
            if validator.returncode:
                raise RuntimeError("Mission validation failed; see evaluations/mission.json and logs")
            from navigation_log import FAILURE_COUNTS, summarize_navigation_log
            log_text = (output / "logs/mission.log").read_text()
            navigation = summarize_navigation_log(log_text, require_no_recovery=args.require_no_recovery)
            write(output / "evaluations/navigation.json", navigation)
            if args.require_no_recovery and any(navigation[key] for key in FAILURE_COUNTS):
                raise RuntimeError(f"Navigation violated no-recovery checks: {navigation}")
            report["status"] = "passed"
        except BaseException as error:
            report.update(status="failed", error=repr(error))
            raise
        finally:
            (output / "stop").touch()
            for name in reversed(names):
                subprocess.run(["podman", "stop", "--ignore", "--time", "3", name],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
            for process in processes:
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
            report["elapsed_s"] = time.monotonic()-start_time
            write(output / "provenance/run.json", report)
            print(f"{report['status']}: {output}", flush=True)


if __name__ == "__main__":
    main()
