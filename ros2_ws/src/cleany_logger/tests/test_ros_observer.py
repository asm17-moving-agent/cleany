"""Opt-in shared observation checks with synthetic ROS execution records."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("CLEANY_RUN_ROS_TESTS") != "1", reason="ROS opt-in"
)


def test_active_raw_archive_reader_does_not_block_writer(tmp_path):
    import sqlite3
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from cleany_logger.recording import RawMessage
    from cleany_logger.rosbag import RosbagWriter

    path = tmp_path / 'bag'
    writer = RosbagWriter(path)
    message = RawMessage('logs', '/test', 'std_msgs/msg/String', 1., 1.,
        serialize_message(String(data='first')), 1_000_000_000)
    try:
        writer.write(message)
        db_path = next(path.glob('*.db3'))
        with sqlite3.connect(f'file:{db_path}?mode=ro', uri=True) as reader:
            assert reader.execute('PRAGMA journal_mode').fetchone()[0] == 'wal'
            reader.execute('BEGIN')
            assert reader.execute('SELECT count(*) FROM messages').fetchone()[0] == 1
            writer.write(message)
            reader.rollback()
            assert reader.execute('SELECT count(*) FROM messages').fetchone()[0] == 2
    finally:
        writer.close()


def test_diagnostic_bag_preserves_cdr_lookup_quota_and_export(tmp_path):
    import sqlite3
    import zipfile
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import MarkerArray
    from cleany_logger import Recorder
    from cleany_logger.archive import ArchiveReader
    from cleany_logger.recording import RawMessage
    from cleany_logger.rosbag import RosbagWriter, read_cdr

    recorder = Recorder(tmp_path)
    writer = RosbagWriter(recorder.bag_path)
    recorder.bag = writer
    sensor = RawMessage('slam', '/scan_raw', 'sensor_msgs/msg/LaserScan', 1., 1.,
                        serialize_message(LaserScan(ranges=[1., 2.])), 1_000_000_000)
    diagnostic = RawMessage('slam', '/trajectory_node_list', 'visualization_msgs/msg/MarkerArray',
                            1., 1., serialize_message(MarkerArray()), 1_000_000_000)
    writer.write(sensor)
    writer.write(diagnostic)
    assert read_cdr(recorder.bag_path, sensor.topic, sensor.timestamp_ns) == sensor.payload
    assert read_cdr(recorder.bag_path, diagnostic.topic, diagnostic.timestamp_ns) == diagnostic.payload
    with sqlite3.connect(next(recorder.bag_path.glob('*.db3'))) as db:
        assert db.execute('SELECT name FROM topics').fetchall() == [('/scan_raw',)]
    recorder.close()
    assert (recorder.bag_path / 'diagnostics/metadata.yaml').exists()
    expected = recorder.path.stat().st_size + sum(p.stat().st_size for p in recorder.bag_path.rglob('*') if p.is_file())
    assert recorder.session_bytes() == expected
    target = tmp_path / 'export.zip'
    ArchiveReader(tmp_path).export_bag(recorder.session_id, target)
    with zipfile.ZipFile(target) as archive:
        assert f'{recorder.session_id}/diagnostics/metadata.yaml' in archive.namelist()


def test_retained_manipulation_history_rebind_and_recording_filter(tmp_path):
    import rclpy
    from cleany_interfaces.msg import ManipulationExecutionRecord
    from cleany_logger import Recorder
    from cleany_logger.observer import ObservationNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    source = rclpy.create_node("manipulation_history_source")
    pub = source.create_publisher(ManipulationExecutionRecord, "/mock/manipulation/execution_events",
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for revision in range(20):
        pub.publish(ManipulationExecutionRecord(execution_id="retained-execution", revision=revision,
            accepted_at_ns=1_000_000_000, updated_at_ns=(revision + 1) * 1_000_000_000,
            stage="VALIDATING", substage=f"observed-{revision}", record_state="ACTIVE"))
    observer = ObservationNode(parameter_overrides=[Parameter("state_directory", value=str(tmp_path))])
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(source)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            snapshot = observer.hub.latest_data("manipulation", "executions")
            if snapshot.get("executions") and snapshot["executions"][0]["record"]["revision"] == 19:
                break
        run = snapshot["executions"][0]
        assert not run["partial"] and len(run["intervals"]) == 20
        assert observer.subs["/mock/manipulation/execution_events"][0].qos_profile.depth == 100
        published = observer.get_publisher_names_and_types_by_node(observer.get_name(), observer.get_namespace())
        assert all(name in ("/rosout", "/parameter_events") for name, _ in published)
        settings = observer.hub.recording.view()['settings']
        settings['sources']['manipulation_events_topic'] = '/other/manipulation/events'
        observer.hub.recording.configure(settings)
        observer.apply_sources()
        assert not observer.hub.latest_data('manipulation', 'executions')
        assert not observer.manipulation.snapshot()['executions']
        observer.receive('/other/manipulation/events', ManipulationExecutionRecord(
            execution_id='retained-execution', revision=0, stage='VALIDATING',
            substage='new-source', accepted_at_ns=1, updated_at_ns=1, record_state='ACTIVE'))
        rebound = observer.hub.latest_data('manipulation', 'executions')['executions']
        assert len(rebound) == 1
        assert rebound[0]['record']['revision'] == 0
        assert [interval['substage'] for interval in rebound[0]['intervals']] == ['new-source']
        summary = next(row for row in observer.hub.snapshot()['rows'] if row['kind'] == 'manipulation')
        assert summary['source_id'] == 'manipulation_events_topic'
        settings = observer.hub.recording.view()['settings']
        settings['source_enabled']['manipulation_events_topic'] = False
        observer.hub.recording.configure(settings)
        recorder = Recorder(tmp_path / 'filtered', controls=observer.hub.recording)
        try:
            recorder.append(observer.hub.snapshot())
            replay = recorder.replay(recorder.session_id, time.time())
            assert not any(row['kind'] == 'manipulation' for row in replay['rows'])
        finally:
            recorder.close()
    finally:
        observer.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
