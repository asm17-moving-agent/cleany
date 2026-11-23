"""Run the offline RTAB-Map worker as each complete recorder split becomes available."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import yaml


def run(args):
    root = args.root
    folder = root / "input"
    out = root / "results" / args.mode
    out.mkdir(parents=True, exist_ok=True)
    first = json.loads((folder / "manifest/00.json").read_text())["cameras"]
    channel = "rgb" if args.mode == "rgbd" else "left"
    p = first[channel]["P"]
    base = -first["right"]["P"][3] / first["right"]["P"][0]
    if args.mode == "stereo-depth":
        import numpy as np
        from reconstruct import matrices

        np.savetxt(out / "base-from-camera.txt", matrices(root)[2][:3])
    calibration = [p[0], p[5], p[2], p[6], base, 0.9]
    files = yaml.safe_load(args.metadata.read_text())["rosbag2_bagfile_information"]["files"]
    cmd = [str(args.binary), str(folder), str(out), args.mode, *map(str, calibration)]
    print("running", cmd, flush=True)
    with (out / "engine.log").open("w") as log:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=log, stderr=log, text=True)
        last = 0
        try:
            for idx, record in enumerate(files):
                while not (folder / f"manifest/{idx:02}.json").is_file():
                    if not args.wait_for_splits:
                        raise FileNotFoundError(f"Extract recorder split {idx} before replaying")
                    if proc.poll() is not None:
                        raise RuntimeError(f"RTAB-Map exited with {proc.returncode}")
                    time.sleep(2)
                limit = (
                    record["starting_time"]["nanoseconds_since_epoch"]
                    + record["duration"]["nanoseconds"]
                    - 2_000_000_000
                )
                if idx == len(files) - 1:
                    limit = 2**63 - 1
                for name in sorted((folder / channel).glob("*.jpg")):
                    timestamp = int(name.stem)
                    if last < timestamp < limit:
                        proc.stdin.write(name.stem + "\n")
                        last = timestamp
                proc.stdin.flush()
                print(f"consumed split {idx}", flush=True)
            proc.stdin.close()
            if proc.wait():
                raise RuntimeError("RTAB-Map failed. See engine.log")
        finally:
            if proc.poll() is None:
                proc.terminate()
                proc.wait()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("metadata", type=Path)
    p.add_argument("binary", type=Path)
    p.add_argument("mode", choices=("stereo", "stereo-filtered", "stereo-depth", "rgbd"))
    p.add_argument("--wait-for-splits", action="store_true", help="wait for a concurrent frame extractor")
    run(p.parse_args())
