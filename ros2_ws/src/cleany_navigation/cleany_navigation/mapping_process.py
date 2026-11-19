"""Linux process identities used to avoid PID-reuse adoption or global cleanup."""

from pathlib import Path


def identity(pid: int) -> str | None:
    try:
        fields = (
            (Path("/proc") / str(pid) / "stat").read_text().split(") ", 1)[1].split()
        )
        return fields[19] if fields[0] != "Z" else None
    except (OSError, IndexError):
        return None


def descendants(pid: int) -> dict[str, str]:
    children = {}
    pending = {pid}
    while pending:
        next_generation = set()
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / "stat").read_text().split(") ", 1)[1].split()
                if int(fields[1]) in pending and fields[0] != "Z":
                    children[entry.name] = fields[19]
                    next_generation.add(int(entry.name))
            except (OSError, IndexError):
                pass
        pending = next_generation
    return children


def live_recorded(record: dict) -> list[int]:
    return [
        int(pid)
        for pid, token in record.items()
        if token is not None and identity(int(pid)) == token
    ]
