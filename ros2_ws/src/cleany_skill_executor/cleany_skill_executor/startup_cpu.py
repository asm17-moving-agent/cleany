"""Pace process starts using host CPU counters from Linux procfs."""
import argparse
import math
import os
import time


def cpu_limit() -> float:
    value = float(os.environ.get('CLEANY_COLLECTION_START_CPU_PERCENT', '80'))
    if not 0 < value <= 100:
        raise ValueError('CLEANY_COLLECTION_START_CPU_PERCENT must be in (0, 100]')
    return value


def cpu_ticks() -> tuple[int, int]:
    with open('/proc/stat', encoding='ascii') as source:
        tag, *values = source.readline().split()
    if tag != 'cpu' or len(values) < 4:
        raise ValueError('Aggregate CPU counters unavailable')
    # Guest ticks are already included in user and nice. Treat I/O wait as busy
    # so a stalled device cannot make startup appear to have extra capacity.
    ticks = [int(value) for value in values[:8]]
    return sum(ticks), ticks[3]


class StartupCpuWindow:
    """Require two consecutive one-second intervals at or below the configured load."""

    def __init__(self):
        self.limit = cpu_limit()
        self.at = time.monotonic()
        self.ticks = cpu_ticks()
        self.quiet_samples = 0
        self.percent = None

    def ready(self) -> bool:
        now = time.monotonic()
        if now - self.at >= 1.:
            previous, idle = self.ticks
            self.ticks = cpu_ticks()
            self.at = now
            elapsed, idle_elapsed = self.ticks[0] - previous, self.ticks[1] - idle
            self.percent = (100. * (1. - idle_elapsed / elapsed)
                            if elapsed > 0 and 0 <= idle_elapsed <= elapsed else None)
            if self.percent is not None and self.percent <= self.limit:
                self.quiet_samples += 1
            else:
                self.quiet_samples = 0
        return self.quiet_samples >= 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('stage')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    window = StartupCpuWindow()
    deadline = time.monotonic() + args.timeout
    print(f'Waiting for CPU <= {window.limit:g}% before {args.stage}', flush=True)
    while time.monotonic() < deadline:
        if window.ready():
            print(f'CPU settled before {args.stage}: {window.percent:.1f}%', flush=True)
            return 0
        time.sleep(min(.25, max(0., deadline - time.monotonic())))
    print(f'CPU did not settle before {args.stage} (last sample {window.percent}%)', flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
