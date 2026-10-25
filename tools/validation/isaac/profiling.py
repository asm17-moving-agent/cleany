"""Optional wall/main-thread CPU timing of disjoint adapter loop stages."""
from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path
import time


def distribution(values: list[float]) -> dict:
    ordered = sorted(values)
    return {"count": len(values), "sum_s": sum(values),
            "mean_s": sum(values) / len(values),
            **{f"p{p}_s": ordered[min(len(ordered)-1, int((len(ordered)-1)*p/100))]
               for p in (50, 95, 99)}, "max_s": ordered[-1]}


class LoopProfiler:
    """Record bounded windows; CPU time excludes PhysX worker threads."""

    def __init__(self, path: Path, physics_hz: float, interval_s: float = 5.) -> None:
        self.output = path.open("w", buffering=1)
        self.period = 1. / physics_hz
        self.interval = interval_s
        self.window_start = time.perf_counter()
        self.window_epoch = time.time()
        self.stages = defaultdict(lambda: {"wall": [], "thread_cpu": []})
        self.loops = 0
        self.overruns = 0
        self.requested_sleep = 0.
        self.begin()

    def begin(self) -> None:
        self.last_wall = self.loop_start = time.perf_counter()
        self.last_cpu = time.thread_time()

    def mark(self, name: str) -> None:
        wall, cpu = time.perf_counter(), time.thread_time()
        self.stages[name]["wall"].append(wall-self.last_wall)
        self.stages[name]["thread_cpu"].append(cpu-self.last_cpu)
        self.last_wall, self.last_cpu = wall, cpu

    def before_sleep(self, requested: float) -> None:
        self.overruns += time.perf_counter()-self.loop_start > self.period
        self.requested_sleep += requested

    def finish(self) -> None:
        self.loops += 1
        if time.perf_counter()-self.window_start >= self.interval:
            self.flush()

    def flush(self) -> None:
        if not self.loops:
            return
        end = time.perf_counter()
        end_epoch = time.time()
        stages = {name: {clock: distribution(values) for clock, values in clocks.items()}
                  for name, clocks in self.stages.items()}
        elapsed = end-self.window_start
        measured = sum(item["wall"]["sum_s"] for item in stages.values())
        self.output.write(json.dumps({"wall_start": self.window_epoch, "wall_end": end_epoch,
            "wall_s": elapsed, "simulation_s": self.loops*self.period, "loops": self.loops,
            "rtf": self.loops*self.period/elapsed, "compute_overrun_loops": self.overruns,
            "requested_sleep_s": self.requested_sleep, "unattributed_wall_s": elapsed-measured,
            "stages": stages})+"\n")
        self.stages.clear()
        self.loops = self.overruns = 0
        self.requested_sleep = 0.
        self.window_start = end
        self.window_epoch = end_epoch

    def close(self) -> None:
        self.flush()
        self.output.close()
