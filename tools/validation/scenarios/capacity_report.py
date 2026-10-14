"""Extract measured capacity and batch throughput from preserved raw evidence."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
from audit import assess


def resources(samples: list[dict]) -> dict:
    """Summarize whole-host utilization, including batch startup and shutdown."""
    cpu = []
    gpu = []
    available = []
    for a, b in zip(samples, samples[1:]):
        before = list(map(int, a["cpu"].split()[1:9]))
        after = list(map(int, b["cpu"].split()[1:9]))
        # CPU counters are cumulative; exclude idle and iowait from each interval.
        delta = [y - x for x, y in zip(before, after)]
        total = sum(delta)
        if total > 0:
            cpu.append(100 * (total - delta[3] - delta[4]) / total)
    for s in samples:
        if s.get("gpu"):
            gpu.append(float(s["gpu"].split(",")[0]))
        for line in s["memory"].splitlines():
            if line.startswith("MemAvailable:"):
                available.append(int(line.split()[1]) / 1024 / 1024)
    return {
        "scope": "whole batch including startup/shutdown; whole host CPU and GPU",
        "cpu_mean_percent": sum(cpu) / len(cpu) if cpu else None,
        "cpu_max_percent": max(cpu) if cpu else None,
        "gpu_max_percent": max(gpu) if gpu else None,
        "available_memory_min_gib": min(available) if available else None,
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("directory", type=Path)
    args = p.parse_args()
    rows = []
    pending = []
    for batch in sorted(args.directory.glob("capacity-*")):
        report_path = batch / "batch.json"
        if not report_path.exists():
            pending.append(batch.name)
            continue
        report = json.loads(report_path.read_text())
        workers = []
        for worker in sorted(batch.glob("worker-*")):
            if not worker.is_dir():
                continue
            result = json.loads((worker / "result.json").read_text())
            shutdown = json.loads((worker / "shutdown.json").read_text())
            validation = assess(result, shutdown)
            workers.append(
                {
                    "passed": validation["passed"],
                    "error": result.get("error"),
                    "missions": validation["missions"],
                }
            )
        finished = sum(1 for w in workers for m in w["missions"] if m["passed"])
        samples = json.loads((batch / "resources.json").read_text())
        rows.append(
            {
                "name": batch.name,
                "count": report["count"],
                "seats": report.get("seats", [report.get("seat")]),
                "all_passed": len(workers) == report["count"]
                and all(w["passed"] for w in workers)
                and all(c == 0 for c in report["exit_codes"]),
                "successful_missions": finished,
                "successful_missions_per_hour": finished
                * 3600
                / report["elapsed_wall"],
                "elapsed_wall_seconds": report["elapsed_wall"],
                "workers": workers,
                "resources": resources(samples),
            }
        )
    result = {"rows": rows, "batches_without_terminal_report": pending}
    (args.directory / "capacity_report.json").write_text(json.dumps(result, indent=2))
    for row in rows:
        print(
            row["name"],
            row["all_passed"],
            round(row["successful_missions_per_hour"], 1),
            [[round(m["rtf"], 3) for m in w["missions"]] for w in row["workers"]],
        )


if __name__ == "__main__":
    main()
