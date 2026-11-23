"""Load a map with an isolated Nav2 map server and inspect its OccupancyGrid."""

import hashlib
import json
import math
import subprocess

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.srv import GetMap


def main():
    rclpy.init()
    node = rclpy.create_node("cleany_map_check")
    server = subprocess.Popen([
        "/opt/ros/humble/lib/nav2_map_server/map_server", "--ros-args",
        "-p", "yaml_filename:=/tmp/candidate/source.yaml",
    ], stdout=subprocess.DEVNULL)

    def request(client, message):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"Cannot reach {client.srv_name}")
        future = client.call_async(message)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done():
            raise TimeoutError(client.srv_name)
        return future.result()

    try:
        lifecycle = node.create_client(ChangeState, "/map_server/change_state")
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            msg = ChangeState.Request()
            msg.transition.id = transition
            if not request(lifecycle, msg).success:
                raise ValueError("Nav2 rejected the map")
        grid = request(node.create_client(GetMap, "/map_server/map"), GetMap.Request()).map
        info = grid.info
        if grid.header.frame_id != "map" or not info.width or not info.height or len(grid.data) != info.width * info.height:
            raise ValueError("Nav2 returned an incomplete grid")
        p, q = info.origin.position, info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        geometry = (info.width, info.height, info.resolution, p.x, p.y, yaw)
        # Same wire representation as cleany_teleop.map_view.MapView.revision.
        digest = hashlib.sha256(str(geometry).encode() + bytes(v + 1 for v in grid.data))
        print(json.dumps({"width": info.width, "height": info.height, "grid_revision": digest.hexdigest()[:24]}))
    finally:
        server.terminate()
        try:
            server.wait(timeout=3)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
