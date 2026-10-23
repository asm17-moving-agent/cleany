"""Opt-in ROS graph/QoS/layer checks in an isolated DDS domain."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("CLEANY_RUN_ROS_TESTS") != "1", reason="ROS opt-in"
)


def test_retained_manipulation_history_is_aggregated_without_control_publishers(tmp_path, monkeypatch):
    import rclpy
    from cleany_interfaces.msg import ManipulationExecutionRecord
    from cleany_dev_monitor.node import MonitorNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda *args: None)
    rclpy.init()
    source = rclpy.create_node("manipulation_history_source")
    pub = source.create_publisher(ManipulationExecutionRecord, "/mock/manipulation/execution_events",
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for revision in range(20):
        pub.publish(ManipulationExecutionRecord(execution_id="retained-execution", revision=revision,
            accepted_at_ns=1_000_000_000, updated_at_ns=(revision + 1) * 1_000_000_000,
            stage="VALIDATING", substage=f"observed-{revision}", record_state="ACTIVE"))
    monitor = MonitorNode(parameter_overrides=[Parameter("state_directory", value=str(tmp_path))])
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            snapshot = monitor.hub.latest_data("manipulation", "executions")
            if snapshot.get("executions") and snapshot["executions"][0]["record"]["revision"] == 19:
                break
        run = snapshot["executions"][0]
        assert not run["partial"] and len(run["intervals"]) == 20
        assert monitor.subs["/mock/manipulation/execution_events"][0].qos_profile.depth == 100
        published = monitor.get_publisher_names_and_types_by_node(monitor.get_name(), monitor.get_namespace())
        assert all(name in ("/rosout", "/parameter_events") for name, _ in published)
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_qos_discovery_missing_transform_and_read_only_topics(tmp_path, monkeypatch):
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda *args: None)
    rclpy.init()
    monitor = MonitorNode(
        parameter_overrides=[Parameter("state_directory", value=str(tmp_path))]
    )
    source = rclpy.create_node("monitor_test_source")
    pub = source.create_publisher(
        String,
        "/monitor_test",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    monitor.hub.select("test", ["/monitor_test"])
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            pub.publish(String(data="hello"))
            executor.spin_once(timeout_sec=0.05)
            samples = [
                r
                for r in monitor.hub.snapshot()["rows"]
                if r["key"] == "/monitor_test" and r["kind"] == "sample"
            ]
            if samples:
                break
        assert samples[0]["data"]["data"] == "hello"
        graph = next(
            r["data"] for r in monitor.hub.snapshot()["rows"] if r["kind"] == "graph"
        )
        info = next(t for t in graph["topics"] if t["name"] == "/monitor_test")
        assert info["publishers"][0]["qos"]["reliability"] == "BEST_EFFORT"
        source.destroy_publisher(pub)
        monitor.discover()
        assert not source.get_publishers_info_by_topic("/monitor_test")
        monitor.hub.release("test")
        monitor.discover()
        assert "/monitor_test" not in monitor.subs
        assert not any(
            r["key"] == "/monitor_test" for r in monitor.hub.snapshot()["rows"]
        )
        grid = OccupancyGrid()
        grid.header.frame_id = "missing_frame"
        grid.info.width = grid.info.height = 1
        grid.info.resolution = 0.05
        grid.info.origin.orientation.w = 1.0
        grid.data = [100]
        monitor.layer("/map", grid, 0.0)
        layer = next(
            r["data"] for r in monitor.hub.snapshot()["rows"] if r["kind"] == "layer"
        )
        assert not layer["valid"] and "cells" not in layer
        published = monitor.get_publisher_names_and_types_by_node(
            monitor.get_name(), monitor.get_namespace()
        )
        assert all(name in ("/rosout", "/parameter_events") for name, _ in published)
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_camera_best_effort_subscription_and_expiry(tmp_path, monkeypatch):
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda *args: None)
    rclpy.init()
    monitor = MonitorNode()
    source = rclpy.create_node("camera_test_source")
    pub = source.create_publisher(Image, "/test_camera/image", qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    message = Image(height=2, width=2, encoding="rgb8", step=6, data=[255, 0, 0] * 4)
    try:
        deadline = time.monotonic() + 6
        frame = None
        while time.monotonic() < deadline:
            monitor.hub.cameras.request("/test_camera/image")
            pub.publish(message)
            executor.spin_once(timeout_sec=0.05)
            frame = monitor.hub.cameras.request("/test_camera/image")[0]
            if frame:
                break
        assert frame and frame.width == 2 and frame.jpeg.startswith(b"\xff\xd8")
        # Expire observation lease without any robot motion or control calls.
        monitor.hub.cameras.clock = lambda: time.monotonic() + 5
        monitor.discover()
        assert "/test_camera/image" not in monitor.subs
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_navigation_dds_diagnostics_geometry_and_actual_bt_messages(tmp_path, monkeypatch):
    import json
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from cleany_interfaces.msg import RuntimeDebugSnapshot
    from geometry_msgs.msg import Point32, PolygonStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
    from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore, CriticScore
    from geometry_msgs.msg import Pose2D
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda *args: None)
    rclpy.init()
    monitor = MonitorNode(parameter_overrides=[Parameter("use_sim_time", value=True)])
    source = rclpy.create_node("navigation_observation_source")
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    types = {"/clock": Clock, "/wheel/odom": Odometry, "/head_pan/state": String,
             "/reactive_guard/state": String, "/scan": LaserScan, "/evaluation": LocalPlanEvaluation,
             "/behavior_tree_log": BehaviorTreeLog, "/mission/debug_snapshot": RuntimeDebugSnapshot,
             **{t: Twist for t in monitor.config["velocity_topics"]}}
    publishers = {topic: source.create_publisher(cls, topic, qos_profile_sensor_data) for topic, cls in types.items()}
    monitor.hub.select("navigation-test", ["/scan", "/evaluation"])
    started = time.monotonic()
    try:
        deadline = started + 7
        while time.monotonic() < deadline:
            stamp = Clock().clock
            stamp.sec = 10
            publishers["/clock"].publish(Clock(clock=stamp))
            odom = Odometry(child_frame_id="base_link")
            odom.header.frame_id = "map"
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.
            odom.twist.twist.linear.x = .01
            publishers["/wheel/odom"].publish(odom)
            command = Twist()
            command.linear.x = .02
            for topic in monitor.config["velocity_topics"]:
                publishers[topic].publish(command)
            publishers["/head_pan/state"].publish(String(data=json.dumps({"pan":.5,"target":.4,"reason":"PAN_TARGET_OUT_OF_RANGE","motion_authority":False})))
            publishers["/reactive_guard/state"].publish(String(data=json.dumps({"decision":"PASS","requested":[.02,0,0],"output":[.02,0,0],"processing_s":.002})))
            scan = LaserScan(angle_min=0.,angle_increment=.1,range_min=.1,range_max=6.,ranges=[2.]*1024)
            scan.header.frame_id = "map"
            scan.header.stamp = stamp
            publishers["/scan"].publish(scan)
            evaluation = LocalPlanEvaluation(best_index=1)
            evaluation.header.frame_id = "map"
            evaluation.header.stamp = stamp
            for i in range(25):
                trajectory = TrajectoryScore(total=float(i))
                trajectory.traj.poses = [Pose2D(x=float(i),y=0.,theta=0.)]
                trajectory.scores = [CriticScore(name="PathDist",raw_score=float(i),scale=2.)]
                evaluation.twists.append(trajectory)
            publishers["/evaluation"].publish(evaluation)
            bt = BehaviorTreeLog(event_log=[BehaviorTreeStatusChange(node_name="FollowPath",previous_status="IDLE",current_status="RUNNING")])
            publishers["/behavior_tree_log"].publish(bt)
            runtime = RuntimeDebugSnapshot(schema_version=1,boot_id="navigation-test",snapshot_json=json.dumps({"state":"NAVIGATE_TO_TARGET","navigation":{"operation_id":"op13","target":"13","frame":"map","goal_pose":{"x":1.,"y":0.,"yaw":0.}}}))
            publishers["/mission/debug_snapshot"].publish(runtime)
            for _ in range(5):
                executor.spin_once(timeout_sec=.02)
            observed = {(r["kind"],r["key"]):r["data"] for r in monitor.hub.snapshot()["rows"]}
            if (("navigation","summary") in observed and observed[("navigation","summary")]["motion"]=="CREEPING"
                    and ("navigation","scan") in observed and ("navigation","controller_evaluation") in observed
                    and observed[("navigation","summary")]["bt_status"].get("FollowPath")=="RUNNING"):
                break
        assert observed[("navigation","summary")]["motion"] == "CREEPING"
        assert observed[("navigation","summary")]["reason_source"] is None
        assert observed[("navigation","pan")]["motion_authority"] is False
        assert len(observed[("navigation","scan")]["points"]) <= 512
        assert observed[("navigation","scan")]["sampled"]
        assert any(t["selected"] and t["index"]==1 for t in observed[("navigation","controller_evaluation")]["trajectories"])
        assert observed[("navigation","controller_evaluation")]["count"] == 25
        json.dumps(monitor.hub.snapshot(), allow_nan=False)
        # Repeated successful condition ticks are bounded; first failure is immediate.
        condition = BehaviorTreeLog(event_log=[BehaviorTreeStatusChange(node_name="IsPathValid", previous_status="IDLE", current_status="SUCCESS")])
        for _ in range(10):
            monitor.navigation_receive("/behavior_tree_log", condition, None)
        hits = [r for r in monitor.hub.snapshot()["rows"] if r["kind"] == "event" and r["data"].get("kind") == "nav2_bt" and r["data"]["data"]["node"] == "IsPathValid"]
        assert len(hits) == 1
        condition.event_log[0].current_status = "FAILURE"
        monitor.navigation_receive("/behavior_tree_log", condition, None)
        assert any(r["kind"] == "event" and r["data"].get("kind") == "nav2_bt" and r["data"]["data"]["status"] == "FAILURE" for r in monitor.hub.snapshot()["rows"])
        # Unsubscribe large geometry while fixed diagnostics remain active.
        monitor.hub.release("navigation-test")
        monitor.discover()
        assert "/scan" not in monitor.subs and "/evaluation" not in monitor.subs
        assert "/wheel/odom" in monitor.subs
        # Invalid measured frame cannot remain a fresh velocity source.
        odom.child_frame_id = "world"
        monitor.navigation_receive("/wheel/odom", odom, 10.)
        assert "odometry" not in monitor.navigation.observations
        # Non-finite covariance or polygon vertices must not make the snapshot unrecordable.
        amcl = PoseWithCovarianceStamped()
        amcl.header.frame_id = "map"
        amcl.pose.pose.orientation.w = 1.
        amcl.pose.covariance[0] = float("nan")
        monitor.navigation_receive(monitor.config["amcl_pose_topic"], amcl, 10.)
        zone = PolygonStamped()
        zone.header.frame_id = "map"
        zone.polygon.points = [Point32(x=1., y=0., z=0.), Point32(x=float("inf"), y=0., z=0.)]
        monitor.navigation_receive(monitor.config["stop_polygon_topic"], zone, 10.)
        observed = {(r["kind"],r["key"]):r["data"] for r in monitor.hub.snapshot()["rows"]}
        assert observed[("navigation","amcl_pose")]["pose"] == [0., 0., 0.]
        assert observed[("navigation","stop_polygon")]["points"] == [[1., 0., 0.]]
        json.dumps(monitor.hub.snapshot(), allow_nan=False)
        # A decode error is cleared by the next valid message on the same topic.
        monitor.receive("/clock", String())
        assert "/clock" in monitor.errors
        monitor.receive("/clock", Clock(clock=stamp))
        assert "/clock" not in monitor.errors
        published = monitor.get_publisher_names_and_types_by_node(monitor.get_name(),monitor.get_namespace())
        assert all(name in ("/rosout", "/parameter_events") for name,_ in published)
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_bt_snapshot_latched_discovery_preserves_full_tree(tmp_path, monkeypatch):
    import json
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    rclpy.init()
    source = rclpy.create_node('bt_tree_source')
    pub = source.create_publisher(String, '/navigation/bt_trace',
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    data = {'schema': 1, 'nodes': [{'id': str(i), 'parent': None, 'ports': {'path': '{path}'}} for i in range(40)],
            'states': {str(i): 'UNKNOWN' for i in range(40)}, 'history': [], 'xml_sha256': 'recorded'}
    pub.publish(String(data=json.dumps(data)))
    monitor = MonitorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(monitor)
    try:
        deadline = time.monotonic()+6
        found = []
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            found = [r for r in monitor.hub.snapshot()['rows'] if r['kind'] == 'nav2_bt_tree']
            if found:
                break
        assert found and found[0]['data'] == data
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


@pytest.mark.parametrize('key', ['point_cloud','voxel_layer','voxel_world','camera','lidar'])
def test_raw_point_cloud_recording_without_browser_and_off_release(tmp_path, monkeypatch, key):
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import PointCloud2, Image, LaserScan
    from nav2_msgs.msg import VoxelGrid
    from cleany_logger.observer import ObservationNode as MonitorNode
    from cleany_logger import Recorder
    from cleany_logger.recording import defaults

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    rclpy.init()
    monitor = MonitorNode()
    source = rclpy.create_node('raw_recording_source')
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    settings = defaults()
    settings['enabled'][key] = True
    monitor.hub.recording.configure(settings)
    message = (VoxelGrid(size_x=1,size_y=1,size_z=2,data=[1]) if key=='voxel_layer' else
               Image(height=2,width=2,encoding='rgb8',step=6,data=b'\x7f'*12) if key=='camera' else
               LaserScan(ranges=[1.,2.,3.]) if key=='lidar' else
               PointCloud2(height=1,width=10000,point_step=4,row_step=40000,data=b'\x00'*40000))
    topic = settings['topics'][key]
    pub = source.create_publisher(type(message), topic, qos_profile_sensor_data)
    recorder = Recorder(tmp_path, controls=monitor.hub.recording)
    try:
        deadline=time.monotonic()+6
        found=None
        while time.monotonic()<deadline:
            pub.publish(message)
            executor.spin_once(timeout_sec=.03)
            recorder.append_raw()
            found=recorder.db.execute("SELECT received FROM raw_index WHERE category=?", (key,)).fetchone()
            if found:
                break
        assert found and deserialize_message(recorder.raw_message(recorder.session_id, key, found[0])[0], type(message)) == message
        assert not monitor.hub.selections() and not monitor.hub.cameras.selected()
        monitor.hub.recording.configure(defaults())
        monitor.discover()
        assert topic not in monitor.subs
        assert not monitor.hub.recording.reserve(topic)
    finally:
        recorder.close()
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_editing_base_path_topic_replaces_subscription_and_discards_old_layer(tmp_path, monkeypatch):
    import rclpy
    from cleany_logger.observer import ObservationNode as MonitorNode
    from cleany_logger.recording import defaults
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    rclpy.init()
    monitor = MonitorNode(parameter_overrides=[Parameter('state_directory', value=str(tmp_path))])
    source = rclpy.create_node('editable_topic_test_source')
    old = source.create_publisher(Path, '/plan', 10)
    new = source.create_publisher(Path, '/custom/plan', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(monitor)
    def observe(publisher, x):
        message = Path()
        message.header.frame_id = 'map'
        point = PoseStamped()
        point.pose.position.x = x
        message.poses = [point]
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            monitor.discover()
            publisher.publish(message)
            executor.spin_once(timeout_sec=.05)
            if monitor.hub.latest_data('layer', 'path').get('points') == [[x, 0.]]:
                return
        raise AssertionError('Changed path topic did not arrive')
    try:
        observe(old, 1.)
        settings = defaults()
        settings['sources'] = {'path_topic': '/custom/plan'}
        monitor.hub.recording.configure(settings)
        monitor.discover()
        assert '/plan' not in monitor.subs
        assert monitor.hub.latest_data('layer', 'path') == {}
        observe(new, 2.)
        assert '/custom/plan' in monitor.subs
        assert monitor.hub.latest_data('config', 'monitor')['path_topic'] == '/custom/plan'
    finally:
        executor.shutdown()
        monitor.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_short_safety_transitions_survive_state_display_throttle(tmp_path, monkeypatch):
    import json
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    monkeypatch.setattr('cleany_logger.observer.time.monotonic', lambda: 100.)
    rclpy.init()
    monitor = MonitorNode(parameter_overrides=[Parameter('state_directory', value=str(tmp_path))])
    try:
        topic = monitor.config['reactive_guard_topic']
        for decision in ('STOP', 'PASS', 'STOP', 'PASS'):
            monitor.receive(topic, String(data=json.dumps({'decision': decision})))
        events = [r['data']['data']['decision'] for r in monitor.hub.snapshot()['rows']
                  if r['kind'] == 'event' and r['key'] == topic and r['data'].get('kind') == 'navigation_decision']
        assert events == ['STOP', 'PASS', 'STOP', 'PASS']
        assert monitor.hub.latest_data('navigation', 'reactive_guard')['decision'] == 'STOP'
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


def test_new_session_requests_retained_static_transforms_again(tmp_path, monkeypatch):
    import rclpy
    from cleany_logger import Recorder
    from cleany_logger.observer import ObservationNode as MonitorNode
    from cleany_logger.recording import defaults
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    rclpy.init()
    monitor = MonitorNode(parameter_overrides=[Parameter('state_directory', value=str(tmp_path))])
    source = rclpy.create_node('retained_static_source')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    publisher = source.create_publisher(TFMessage, '/tf_static', qos)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'sensor'
    transform.transform.rotation.w = 1.
    publisher.publish(TFMessage(transforms=[transform]))  # Publish only once.
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    recorder = None
    try:
        for _ in range(2):
            settings = defaults()
            settings['enabled']['camera'] = True
            with monitor.hub.recording.transition():
                monitor.hub.recording.configure(settings)
                recorder = Recorder(tmp_path, controls=monitor.hub.recording)
            deadline = time.monotonic() + 5
            found = None
            while time.monotonic() < deadline:
                monitor.discover()
                executor.spin_once(timeout_sec=.05)
                recorder.append_raw()
                found = recorder.db.execute("SELECT received FROM raw_index WHERE topic='/tf_static'").fetchone()
                if found:
                    break
            assert found, 'A new recording must receive retained /tf_static without another publication'
            recorder.close()
    finally:
        if recorder:
            recorder.close()
        executor.shutdown()
        monitor.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
