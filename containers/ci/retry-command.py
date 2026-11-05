#!/usr/bin/env python3
"""Retry dependency preparation commands without changing their arguments."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--delay', type=float, default=10)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if args.attempts < 1 or args.delay < 0 or not command:
        parser.error('positive attempts, nonnegative delay and a command are required')
    for attempt in range(1, args.attempts + 1):
        result = subprocess.run(command, check=False)
        if result.returncode == 0:
            return 0
        if attempt < args.attempts:
            delay = args.delay * 2 ** (attempt - 1)
            print(f'{command[0]} failed (attempt {attempt}/{args.attempts}, '
                  f'exit {result.returncode}); retrying in {delay:g}s', file=sys.stderr, flush=True)
            time.sleep(delay)
    return result.returncode if result.returncode >= 0 else 128 - result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
