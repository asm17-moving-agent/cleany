"""Opt-in shared observation checks with synthetic ROS execution records."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("CLEANY_RUN_ROS_TESTS") != "1", reason="ROS opt-in"
)


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
