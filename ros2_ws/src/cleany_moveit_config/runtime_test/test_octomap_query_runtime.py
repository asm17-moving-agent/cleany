"""Exercise the real map updater and GetPlanningScene concurrently, without cameras."""
import hashlib
import os
from pathlib import Path
import struct
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('CLEANY_TEST_OCTOMAP_RUNTIME') != '1',
    reason='Enable with make test-manipulation-fixes (patched overlay required)',
)

from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState, PointCloud2, PointField
from runtime_support import call, moveit_runtime
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def test_octomap_queries_during_updates(tmp_path):
    prefix = Path(os.environ.get('CLEANY_MOVEIT_PATCH_PREFIX', str(Path.home()/'.local/share/cleany/moveit-octomap-lock')))
    assert (prefix/'octomap-read-lock.receipt').exists(), 'Build the lock patch first'
    with moveit_runtime(tmp_path, octomap=True) as (node, process):
        joint_pub = node.create_publisher(JointState, '/joint_states', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/perception/scene_cloud', 10)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'test_depth_optical'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        filtered = []
        subscription = node.create_subscription(PointCloud2, '/perception/scene_cloud_filtered',
            lambda msg: filtered.append((msg.header.stamp.sec,msg.header.stamp.nanosec)), qos_profile_sensor_data)
        ticks = [0]
        def publish():
            joints = JointState()
            joints.header.stamp = node.get_clock().now().to_msg()
            joints.name = [f'{side}_{suffix}_joint' for side in ('left','right') for suffix in
                ('shoulder_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll','gripper')]
            joints.name += ['rear_left_wheel_joint', 'rear_right_wheel_joint',
                            'front_left_wheel_joint', 'front_right_wheel_joint']
            joints.position = [0.]*len(joints.name)
            joint_pub.publish(joints)
            shift = .12 * ((ticks[0]//15)%2)
            ticks[0] += 1
            cloud = PointCloud2()
            cloud.header.stamp = joints.header.stamp
            cloud.header.frame_id = 'test_depth_optical'
            cloud.height = 1
            cloud.fields = [PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
            points = [( .60+shift+x*.012, -.10+y*.012, .30+z*.012) for x in range(8) for y in range(8) for z in range(8)]
            cloud.width = len(points);cloud.point_step=12;cloud.row_step=12*len(points);cloud.is_dense=True
            cloud.data = b''.join(struct.pack('<fff',*p) for p in points)
            cloud_pub.publish(cloud)
        timer = node.create_timer(.033, publish)
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.OCTOMAP
        # Spin while the planner initializes so the self-filter receives joints.
        # Full CAD self-filter registration can exceed 60 s on Jetson.
        startup = time.monotonic()+180
        while not client.service_is_ready():
            assert time.monotonic()<startup, 'MoveIt startup exceeded 180 s'
            assert process.poll() is None
            node.executor.spin_once(timeout_sec=.02)
        children = []
        for path in Path('/proc').iterdir():
            if not path.name.isdigit():
                continue
            try:
                if int((path/'stat').read_text().rsplit(')',1)[1].split()[1]) == process.pid:
                    children.append(path.name)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                pass
        assert any(str(prefix) in Path(f'/proc/{pid}/maps').read_text() and
                   'libmoveit_planning_scene_monitor' in Path(f'/proc/{pid}/maps').read_text() for pid in children), 'Patched library not loaded'
        end = time.monotonic()+20
        hashes = set(); queries=0
        while time.monotonic()<end:
            response = call(node, process, client, request, timeout=5)
            data = bytes(response.scene.world.octomap.octomap.data)
            if data:hashes.add(hashlib.sha256(data).hexdigest())
            queries += 1
        assert queries >= 50, queries
        assert len(set(filtered)) >= 10, len(set(filtered))
        assert len(hashes) >= 2, 'The map must actually change during the queries'
        import json
        metrics = {'queries': queries, 'filtered_updates': len(set(filtered)), 'distinct_maps': len(hashes)}
        (tmp_path/'query_metrics.json').write_text(json.dumps(metrics, indent=2))
        print(metrics)
