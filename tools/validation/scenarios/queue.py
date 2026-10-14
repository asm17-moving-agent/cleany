"""Execute a reviewable serial list of parallel batches, preserving failures."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from paths import data_root, batch_directory


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--jobs", type=Path, required=True)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--output-root", type=Path)
    args = parser.parse_args()
    jobs = json.loads(args.jobs.read_text())
    records = []
    data = data_root(args.root, args.data_root)
    directory = (
        args.output_root.resolve() if args.output_root else data / "runs/scenarios"
    )
    status = directory / (args.jobs.stem + ".status.json")
    if status.exists():
        raise FileExistsError(f"Refusing to overwrite queue status: {status}")
    status.parent.mkdir(parents=True, exist_ok=True)
    for job in jobs:
        name = job["name"]
        if batch_directory(directory, name).exists():
            raise RuntimeError(f"Refusing to overwrite or restart existing job: {name}")
        cmd = [
            sys.executable,
            str(Path(__file__).parent / "parallel.py"),
            "--root",
            str(args.root),
            "--data-root",
            str(data),
            "--output-root",
            str(directory),
        ]
        for key, value in job.items():
            option = "--" + key.replace("_", "-")
            if value is True:
                cmd.append(option)
            elif value is not False and value is not None:
                cmd.extend([option, str(value)])
        record = {"job": job, "command": cmd, "start": time.time()}
        records.append(record)
        # Persist before launch so interrupted jobs remain in the run history.
        status.write_text(json.dumps(records, indent=2))
        print("JOB", name, flush=True)
        result = subprocess.run(cmd)
        record.update(end=time.time(), exit_code=result.returncode)
        status.write_text(json.dumps(records, indent=2))
        if result.returncode:
            # Do not conceal infrastructure errors or launch another stack over leftovers.
            raise RuntimeError(
                f"Infrastructure failure in {name}; inspect before continuing"
            )


if __name__ == "__main__":
    main()
