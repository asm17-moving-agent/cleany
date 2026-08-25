#!/usr/bin/env python3
"""Rewrite Jazzy sqlite3 bag metadata so Humble can replay the bag."""

from __future__ import annotations

import argparse
import os
import shutil
import sqlite3
import tempfile
from pathlib import Path
from typing import Any

import yaml


_QOS_ENUMS = {
    'history': {
        'system_default': 0,
        'keep_last': 1,
        'keep_all': 2,
        'unknown': 3,
    },
    'reliability': {
        'system_default': 0,
        'reliable': 1,
        'best_effort': 2,
        'unknown': 3,
    },
    'durability': {
        'system_default': 0,
        'transient_local': 1,
        'volatile': 2,
        'unknown': 3,
    },
    'liveliness': {
        'system_default': 0,
        'automatic': 1,
        'manual_by_node': 2,
        'manual_by_topic': 3,
        'unknown': 4,
    },
}


def _numeric_qos(profiles: object) -> list[dict[str, Any]]:
    if not isinstance(profiles, list):
        raise ValueError('offered QoS profiles must be a list')
    converted: list[dict[str, Any]] = []
    for raw_profile in profiles:
        if not isinstance(raw_profile, dict):
            raise ValueError('each offered QoS profile must be a mapping')
        profile = dict(raw_profile)
        for field, values in _QOS_ENUMS.items():
            value = profile.get(field)
            if isinstance(value, str):
                try:
                    profile[field] = values[value]
                except KeyError as error:
                    raise ValueError(
                        f'unsupported QoS {field}: {value!r}'
                    ) from error
        converted.append(profile)
    return converted


def _qos_text(profiles: object) -> str:
    return yaml.safe_dump(
        _numeric_qos(profiles), sort_keys=False
    ).strip()


def prepare_humble_bag(bag_path: Path) -> Path:
    """Convert one Jazzy sqlite bag while retaining its original metadata."""
    metadata_path = bag_path / 'metadata.yaml'
    backup_path = bag_path / 'metadata.jazzy.yaml'
    document = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    information = document['rosbag2_bagfile_information']
    version = int(information['version'])
    if version <= 5:
        return metadata_path
    if information.get('storage_identifier') != 'sqlite3':
        raise ValueError('only sqlite3 bags can be prepared for Humble')

    relative_paths = [
        Path(value) for value in information['relative_file_paths']
    ]
    for relative_path in relative_paths:
        database_path = bag_path / relative_path
        with sqlite3.connect(database_path) as connection:
            rows = connection.execute(
                'SELECT id, offered_qos_profiles FROM topics'
            ).fetchall()
            for topic_id, qos_yaml in rows:
                connection.execute(
                    'UPDATE topics SET offered_qos_profiles = ? WHERE id = ?',
                    (_qos_text(yaml.safe_load(qos_yaml)), topic_id),
                )

    if not backup_path.exists():
        shutil.copy2(metadata_path, backup_path)

    information['version'] = 5
    information.pop('custom_data', None)
    information.pop('ros_distro', None)
    for entry in information['topics_with_message_count']:
        topic = entry['topic_metadata']
        topic['offered_qos_profiles'] = _qos_text(
            topic['offered_qos_profiles']
        )
        topic.pop('type_description_hash', None)

    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=bag_path,
        prefix='.metadata-',
        suffix='.yaml',
        delete=False,
    ) as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        temporary_path = Path(stream.name)
    os.replace(temporary_path, metadata_path)
    return metadata_path


def main() -> None:
    """Run the compatibility conversion from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_path', type=Path)
    options = parser.parse_args()
    print(prepare_humble_bag(options.bag_path))


if __name__ == '__main__':
    main()
