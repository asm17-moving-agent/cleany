"""Tests for Jazzy-to-Humble rosbag metadata conversion."""

import sqlite3
from pathlib import Path

from prepare_humble_bag import prepare_humble_bag

import yaml


QOS = [
    {
        'history': 'unknown',
        'depth': 0,
        'reliability': 'reliable',
        'durability': 'volatile',
        'liveliness': 'automatic',
    }
]


def test_prepare_humble_bag_preserves_jazzy_metadata(
    tmp_path: Path,
) -> None:
    """The conversion keeps evidence and is safe to run more than once."""
    database_path = tmp_path / 'input_0.db3'
    with sqlite3.connect(database_path) as connection:
        connection.execute(
            'CREATE TABLE topics('
            'id INTEGER PRIMARY KEY, name TEXT, type TEXT, '
            'serialization_format TEXT, offered_qos_profiles TEXT, '
            'type_description_hash TEXT)'
        )
        connection.execute(
            'INSERT INTO topics VALUES(1, ?, ?, ?, ?, ?)',
            ('/scan', 'sensor_msgs/msg/LaserScan', 'cdr',
             yaml.safe_dump(QOS), 'RIHS01_test'),
        )
    metadata = {
        'rosbag2_bagfile_information': {
            'version': 9,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': 1},
            'starting_time': {'nanoseconds_since_epoch': 1},
            'message_count': 1,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/scan',
                        'type': 'sensor_msgs/msg/LaserScan',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': QOS,
                        'type_description_hash': 'RIHS01_test',
                    },
                    'message_count': 1,
                }
            ],
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [database_path.name],
            'files': [],
            'custom_data': None,
            'ros_distro': 'jazzy',
        }
    }
    (tmp_path / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8'
    )

    prepare_humble_bag(tmp_path)
    prepare_humble_bag(tmp_path)

    converted = yaml.safe_load(
        (tmp_path / 'metadata.yaml').read_text(encoding='utf-8')
    )['rosbag2_bagfile_information']
    assert converted['version'] == 5
    assert 'ros_distro' not in converted
    topic = converted['topics_with_message_count'][0]['topic_metadata']
    assert 'type_description_hash' not in topic
    assert yaml.safe_load(topic['offered_qos_profiles'])[0] == {
        'history': 3,
        'depth': 0,
        'reliability': 1,
        'durability': 2,
        'liveliness': 1,
    }
    original = yaml.safe_load(
        (tmp_path / 'metadata.jazzy.yaml').read_text(encoding='utf-8')
    )['rosbag2_bagfile_information']
    assert original['version'] == 9
    with sqlite3.connect(database_path) as connection:
        qos_yaml = connection.execute(
            'SELECT offered_qos_profiles FROM topics'
        ).fetchone()[0]
    assert yaml.safe_load(qos_yaml)[0]['reliability'] == 1
