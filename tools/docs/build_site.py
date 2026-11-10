#!/usr/bin/env python3
"""Bundle repository Markdown into the local documentation reader."""

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/index.html"


def build() -> str:
    """Keep source paths so links also work in GitHub's Markdown viewer."""
    patterns = (
        "README.md", "docs/*.md", "docs/implementation/*.md",
        "docs/implementation/how-to/*.md", "docs/implementation/reference/*.md",
        "esp32/README.md",
        "esp32/*/README.md", "containers/*/README.md",
        "ros2_ws/src/*/README.md", "ros2_ws/src/*/docs/*.md",
        "tools/*/README.md", "tools/*/validation_protocol.md",
    )
    paths = sorted({p for pattern in patterns for p in ROOT.glob(pattern)})
    pages = {}
    for path in paths:
        body = path.read_text(encoding="utf-8")
        title = next((line[2:] for line in body.splitlines() if line.startswith("# ")), path.stem)
        pages[path.relative_to(ROOT).as_posix()] = {"title": title, "body": body}
    payload = json.dumps(pages, ensure_ascii=False, separators=(",", ":"))
    payload = payload.replace("<", "\\u003c").replace("\u2028", "\\u2028").replace("\u2029", "\\u2029")
    template = (ROOT / "docs/site.template.html").read_text(encoding="utf-8")
    marker = "/* DOCS_PAYLOAD */"
    if template.count(marker) != 1:
        raise ValueError("Expected one documentation payload marker")
    return template.replace(marker, "const pages = " + payload + ";")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Check without changing the output")
    args = parser.parse_args()
    result = build()
    if args.check:
        if not OUTPUT.is_file() or OUTPUT.read_text(encoding="utf-8") != result:
            raise SystemExit("Documentation reader is stale: python3 tools/docs/build_site.py")
        print("Documentation reader is up to date")
    else:
        OUTPUT.write_text(result, encoding="utf-8")
        print(f"Wrote {OUTPUT.relative_to(ROOT)} ({OUTPUT.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
