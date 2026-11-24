"""Load robot credentials when opening a Backend connection."""

from pathlib import Path


def device_headers(token_file: str) -> dict[str, str]:
    token = Path(token_file).expanduser().read_text().strip()
    if not token or not token.isascii() or any(c.isspace() or not c.isprintable() for c in token):
        raise ValueError("invalid device token file")
    return {"Authorization": "Bearer " + token}
