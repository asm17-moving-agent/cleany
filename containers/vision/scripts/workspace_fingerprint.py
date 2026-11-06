#!/usr/bin/env python3
"""Fingerprint role workspace inputs; used only for successful build receipts."""
from __future__ import annotations
import hashlib
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def fingerprint(source: Path, role: str) -> str:
    packages = {path.parent.name: path.parent for path in source.glob('*/package.xml')}
    selected = set()
    def visit(name):
        if name in selected or name not in packages:
            return
        selected.add(name)
        for element in ET.parse(packages[name]/'package.xml').getroot():
            if 'depend' in element.tag and element.text:
                visit(element.text.strip())
    visit({'anygrasp': 'cleany_grasping', 'perception': 'cleany_perception'}[role])
    digest = hashlib.sha256(('v1/'+sys.version+'/'+os.environ.get('ROS_DISTRO', 'humble')).encode())
    for script in ('build-workspace', 'workspace_fingerprint.py'):
        digest.update(Path(__file__).with_name(script).read_bytes())
    for name in sorted(selected):
        for path in sorted(packages[name].rglob('*')):
            relative = path.relative_to(source)
            if any(part in ('__pycache__', '.pytest_cache', 'test', 'test_runtime') for part in relative.parts):
                continue
            if path.is_file():
                digest.update(str(relative).encode())
                digest.update(path.read_bytes())
    return digest.hexdigest()


if __name__ == '__main__':
    print(fingerprint(Path('/workspace/cleany/ros2_ws/src'), sys.argv[1]))
