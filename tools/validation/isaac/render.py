"""Render a successful recorded run into three observer GIFs and one montage."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

from run import SOURCE, memory, write


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, help="Same cache override used for recording")
    parser.add_argument("--name", default="rendered", help="New media subdirectory")
    parser.add_argument("--limit", type=int, help="Preview only N frames")
    parser.add_argument("--sample-period", type=float, default=.5, help="Simulation seconds per frame; 10 FPS output")
    parser.add_argument("--timeout", type=float, default=900.)
    args = parser.parse_args()
    if not .1 <= args.sample_period <= 10.:
        parser.error("--sample-period must be between 0.1 and 10 seconds")
    if shutil.which("ffmpeg") is None:
        parser.error("Install host ffmpeg for compact GIF encoding")
    output, build = args.run.resolve(), args.build_root.resolve()
    if not args.name.replace("-", "").isalnum() or (output / "media" / args.name).exists():
        parser.error("Choose a new alphanumeric/hyphen media name")
    original = json.loads((output / "provenance/run.json").read_text())
    if original["status"] != "passed":
        parser.error("The recorded Mission must pass before rendering")
    font = Path(subprocess.check_output(["fc-match", "-f", "%{file}", "DejaVu Sans"], text=True))
    cache = args.cache_root or Path(original.get("cache_root", build / f"isaac-cache-{original['domain']}"))
    source = SOURCE / "tools/validation/isaac/render_motion.py"
    snapshot = output / f"provenance/render-{args.name}.py"
    shutil.copyfile(source, snapshot)
    name = "cleany-e2e-render-"+hashlib.sha256(str(output).encode()).hexdigest()[:8]
    command = ["podman", "run", "--rm", "--name", name, "--user", "0", "--network", "none",
        "--device", "nvidia.com/gpu=all", "--security-opt", "label=disable", "--shm-size=2g",
        "--entrypoint", "bash", "-e", "ACCEPT_EULA=Y", "-e", "PRIVACY_CONSENT=N",
        "-e", "OMNI_KIT_ALLOW_ROOT=1", "-e", "PYTHONUNBUFFERED=1",
        "-v", f"{output}:/output:rw", "-v", f"{SOURCE}:/main:ro",
        "-v", f"{snapshot}:/render.py:ro", "-v", f"{font}:/integration-font.ttf:ro",
        "-v", f"{cache}:/root/.cache/ov:rw", "-v", f"{cache}/driver:/isaac-sim/kit/cache/nv_shadercache:rw",
        "-v", f"{cache}/compute:/root/.nv/ComputeCache:rw", original["isaac_image"],
        "-c", f"cd /isaac-sim && ./python.sh /render.py --name {args.name} --sample-period {args.sample_period}"+
        (f" --limit {args.limit}" if args.limit else "")]
    record = {"status": "running", "command": command,
              "source_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest()}
    started = time.monotonic()
    with (build / f".isaac-domain-{original['domain']}.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        resources = memory()
        if resources["gpu_free_mib"] < 2500 or resources["host_available_mib"] < 10000:
            raise RuntimeError(f"Insufficient rendering headroom: {resources}")
        with (output / f"logs/render-{args.name}.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                while process.poll() is None:
                    resources = memory()
                    if resources["gpu_free_mib"] < 1024 or resources["host_available_mib"] < 4096:
                        raise RuntimeError("Insufficient rendering headroom")
                    if time.monotonic()-started > args.timeout:
                        raise TimeoutError("Rendering deadline exceeded")
                    time.sleep(2.)
                if process.returncode or not (output / "media" / args.name / "render.json").exists():
                    raise RuntimeError("Rendering failed; inspect the render log")
                frames = output / "media" / args.name
                encoding = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-framerate", "10",
                    "-i", str(frames / "frame_%04d.png"), "-filter_complex",
                    "[0:v]split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3",
                    "-loop", "0", str(frames / "three-views-compact.gif")]
                record["encoding_command"] = encoding
                subprocess.run(encoding, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=120)
                record["status"] = "passed"
            except BaseException as error:
                record.update(status="failed", error=repr(error))
                raise
            finally:
                subprocess.run(["podman", "stop", "--ignore", "--time", "3", name],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
                process.wait(timeout=15)
                record["elapsed_s"] = time.monotonic()-started
                write(output / f"provenance/render-{args.name}.json", record)
    print(f"{record['status']}: {output / 'media' / args.name}")


if __name__ == "__main__":
    main()
