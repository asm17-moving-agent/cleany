#!/usr/bin/env python3
"""Build the bilingual README banners and verify their asset manifest."""

import argparse
import base64
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "docs/assets/readme"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Check without writing")
    args = parser.parse_args()
    manifest = json.loads((ASSETS / "sources.json").read_text(encoding="utf-8"))
    for asset in manifest["assets"]:
        path = ROOT / asset["asset"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != asset["sha256"]:
            raise SystemExit(f"Asset digest mismatch: {asset['asset']}")
    for entry in manifest["generated_banners"]:
        image = base64.b64encode((ROOT / entry["image"]).read_bytes()).decode("ascii")
        template = (ROOT / entry["template"]).read_text(encoding="utf-8")
        if template.count("{{CAD_IMAGE}}") != 1:
            raise SystemExit(f"Expected one CAD_IMAGE marker in {entry['template']}")
        banner = template.replace("{{CAD_IMAGE}}", "data:image/png;base64," + image)
        output = ROOT / entry["asset"]
        if args.check:
            if not output.exists() or output.read_text(encoding="utf-8") != banner:
                raise SystemExit(f"Stale banner: {entry['asset']}; run {entry['build']}")
        else:
            output.write_text(banner, encoding="utf-8")
    print(f"README banners and {len(manifest['assets'])} asset digests verified")


if __name__ == "__main__":
    main()
