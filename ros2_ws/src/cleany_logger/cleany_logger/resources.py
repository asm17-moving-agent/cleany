"""Bounded, read-only host metrics. No ROS imports or process control."""

from collections import deque
from pathlib import Path
import socket
import threading
import time

import psutil


class ResourceSampler:
    def __init__(self, interval=1.0, process_limit=20, history_limit=300):
        if (
            type(process_limit) is not int
            or type(history_limit) is not int
            or not 0.5 <= interval <= 60
            or not 1 <= process_limit <= 100
            or not 2 <= history_limit <= 600
        ):
            raise ValueError("Invalid resource observation limits")
        self.interval = interval
        self.process_limit = process_limit
        self.history = deque(maxlen=history_limit)
        self.previous = {}
        self.previous_at = None
        self.processes = {}
        self.cpu_previous = None

    def rate(self, key, values, elapsed):
        before = self.previous.get(key)
        self.previous[key] = values
        if (
            before is None
            or elapsed is None
            or elapsed <= 0
            or any(v < p for v, p in zip(values, before))
        ):
            return [None] * len(values)
        return [(v - p) / elapsed for v, p in zip(values, before)]

    def sample(self):
        now = time.monotonic()
        elapsed = None if self.previous_at is None else now - self.previous_at
        self.previous_at = now
        errors = []

        def read(name, fn, fallback=None):
            try:
                return fn()
            except (OSError, psutil.Error, ValueError, TypeError, AttributeError) as exc:
                errors.append(f"{name}: {type(exc).__name__}")
                return fallback

        cpu = read("cpu", lambda: psutil.cpu_times(percpu=True), [])
        cpu_percent = []
        for index, current in enumerate(cpu):
            previous = (
                self.cpu_previous[index]
                if self.cpu_previous and index < len(self.cpu_previous)
                else None
            )

            # guest time is already included in user/nice on Linux.
            def total(t):
                return sum(t) - getattr(t, "guest", 0) - getattr(t, "guest_nice", 0)

            delta = total(current) - total(previous) if previous else 0
            idle = current.idle + getattr(current, "iowait", 0)
            old_idle = previous.idle + getattr(previous, "iowait", 0) if previous else 0
            cpu_percent.append(
                max(0.0, min(100.0, 100 * (1 - (idle - old_idle) / delta)))
                if delta > 0
                else None
            )
        self.cpu_previous = cpu
        memory = read("memory", psutil.virtual_memory)
        swap = read("swap", psutil.swap_memory)
        disk = read("disk capacity", lambda: psutil.disk_usage("/"))
        io = read("disk io", lambda: psutil.disk_io_counters(perdisk=True), {})
        # Exclude partitions and virtual devices to avoid double-counting stacked IO.
        devices = {
            name: value
            for name, value in io.items()
            if (Path("/sys/block") / name).exists()
            and not name.startswith(("loop", "ram", "dm-", "md"))
        }
        if not devices:
            self.previous.pop("disk", None)
        rates = (
            self.rate(
                "disk",
                [
                    sum(v.read_bytes for v in devices.values()),
                    sum(v.write_bytes for v in devices.values()),
                ],
                elapsed,
            )
            if devices
            else [None, None]
        )
        net = read("network", lambda: psutil.net_io_counters(pernic=True), {})
        networks = []
        for name, value in net.items():
            rx, tx = self.rate(
                "net:" + name, [value.bytes_recv, value.bytes_sent], elapsed
            )
            networks.append(dict(name=name, rx_bytes_s=rx, tx_bytes_s=tx))
        active_net = {"net:" + name for name in net}
        self.previous = {
            k: v
            for k, v in self.previous.items()
            if not k.startswith("net:") or k in active_net
        }
        processes, next_processes = [], {}
        denied = 0
        # psutil caches Process instances; clear them before checking PID identity.
        if hasattr(psutil.process_iter, "cache_clear"):
            psutil.process_iter.cache_clear()
        for proc in psutil.process_iter():
            try:
                with proc.oneshot():
                    identity = (proc.pid, proc.create_time())
                    times = proc.cpu_times()
                    used = times.user + times.system
                    old = self.processes.get(identity)
                    percent = (
                        100 * (used - old) / elapsed
                        if old is not None and elapsed and elapsed > 0 and used >= old
                        else None
                    )
                    next_processes[identity] = used
                    processes.append(
                        dict(
                            pid=proc.pid,
                            name=proc.name(),
                            cpu_percent=percent,
                            rss_bytes=proc.memory_info().rss,
                        )
                    )
            except psutil.NoSuchProcess:
                continue
            except (psutil.AccessDenied, OSError):
                denied += 1
        self.processes = next_processes
        by_cpu = sorted(processes, key=lambda p: p["cpu_percent"] or 0, reverse=True)[
            : self.process_limit
        ]
        by_memory = sorted(processes, key=lambda p: p["rss_bytes"], reverse=True)[
            : self.process_limit
        ]
        temperatures = read("temperature", psutil.sensors_temperatures, {})
        temperatures = [
            dict(name=f"{name} / {v.label or i}", celsius=v.current)
            for name, values in temperatures.items()
            for i, v in enumerate(values)
        ][:32]
        gpu = []
        for path in sorted(
            Path("/sys/class/drm").glob("card[0-9]*/device/gpu_busy_percent")
        )[:8]:
            value = read("gpu", lambda p=path: float(p.read_text()))
            gpu.append(dict(name=path.parent.parent.name, percent=value))
        percent = (
            sum(v for v in cpu_percent if v is not None) / len(cpu_percent)
            if cpu_percent and all(v is not None for v in cpu_percent)
            else None
        )
        compact = dict(
            at=time.time(),
            cpu_percent=percent,
            memory_percent=memory.percent if memory else None,
            disk_read_bytes_s=rates[0],
            disk_write_bytes_s=rates[1],
            network_rx_bytes_s=sum(
                v["rx_bytes_s"] for v in networks if v["rx_bytes_s"] is not None
            )
            if networks and all(v["rx_bytes_s"] is not None for v in networks)
            else None,
            network_tx_bytes_s=sum(
                v["tx_bytes_s"] for v in networks if v["tx_bytes_s"] is not None
            )
            if networks and all(v["tx_bytes_s"] is not None for v in networks)
            else None,
        )
        self.history.append(compact)
        return dict(
            **compact,
            host=socket.gethostname(),
            interval_s=self.interval,
            cpu_cores=cpu_percent,
            memory=dict(
                total=memory.total,
                used=memory.total - memory.available,
                available=memory.available,
            )
            if memory
            else None,
            swap=dict(total=swap.total, used=swap.used, percent=swap.percent)
            if swap
            else None,
            disk=dict(path="/", total=disk.total, used=disk.used, percent=disk.percent)
            if disk
            else None,
            disk_devices=list(devices),
            networks=networks[:32],
            processes_cpu=by_cpu,
            processes_memory=by_memory,
            process_count=len(processes),
            denied_processes=denied,
            temperatures=temperatures,
            gpu=gpu,
            gpu_status="available" if gpu else "unsupported",
            errors=errors,
            history=list(self.history),
        )


class ResourceWorker:
    def __init__(self, hub, config):
        self.hub = hub
        self.sampler = ResourceSampler(
            config.get("resource_interval_s", 1.0),
            config.get("resource_process_limit", 20),
            config.get("resource_history_limit", 300),
        )
        self.stop = threading.Event()
        self.thread = threading.Thread(
            target=self.run, name="host-resources", daemon=True
        )

    def run(self):
        while not self.stop.is_set():
            try:
                self.hub.put("resources", "host", self.sampler.sample())
            except (OSError, psutil.Error, ValueError, TypeError) as exc:
                self.hub.put(
                    "resources",
                    "host",
                    dict(
                        host=socket.gethostname(),
                        at=time.time(),
                        interval_s=self.sampler.interval,
                        errors=[f"수집 실패: {type(exc).__name__}"],
                        history=[],
                    ),
                )
            self.stop.wait(self.sampler.interval)
