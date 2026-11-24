"""Verify the portable map with Nav2 and rerender its matching Cartographer state."""

import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.qos import DurabilityPolicy, QoSProfile


def verify(folder: Path) -> dict:
    if os.getenv("ROS_LOCALHOST_ONLY") != "1" or os.getenv("ROS_DOMAIN_ID", "0") == "0":
        raise ValueError("Run this check on localhost in a separate ROS domain")
    yaml_path = folder / "map.yaml"
    config = yaml.safe_load(yaml_path.read_text())
    pgm = folder / config["image"]
    with tempfile.TemporaryDirectory() as tmp:
        carto = Path(get_package_prefix("cartographer_ros")) / "lib/cartographer_ros"
        subprocess.run(
            [
                str(carto / "cartographer_pbstream_to_ros_map"),
                f"-pbstream_filename={folder/'map.pbstream'}",
                "-map_filestem=map",
                f"-resolution={config['resolution']}",
            ],
            cwd=tmp,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        assert (Path(tmp) / "map.pgm").read_bytes() == pgm.read_bytes()
        assert yaml.safe_load((Path(tmp) / "map.yaml").read_text()) == config

    nav2 = Path(get_package_prefix("nav2_map_server")) / "lib/nav2_map_server/map_server"
    node_name = "floor19_export_check_server"
    server = subprocess.Popen(
        [
            str(nav2),
            "--ros-args",
            "-r",
            f"__node:={node_name}",
            "-p",
            f"yaml_filename:={yaml_path}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    rclpy.init()
    node = rclpy.create_node("floor19_export_check_client")
    try:
        client = node.create_client(ChangeState, f"/{node_name}/change_state")
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"No Nav2 lifecycle service (exit={server.poll()})")
        delivered = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, "/map", delivered.append, qos)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            req = ChangeState.Request()
            req.transition.id = transition
            response = client.call_async(req)
            rclpy.spin_until_future_complete(node, response, timeout_sec=12.0)
            assert response.done() and response.result().success
        end = time.monotonic() + 12
        while not delivered and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.25)
        if not delivered:
            raise RuntimeError("No occupancy grid received from Nav2")
        grid = delivered[0]
        pixels = np.asarray(Image.open(pgm), dtype=float)
        occupancy = 1 - pixels / 255
        expected = np.where(
            occupancy > config["occupied_thresh"],
            100,
            np.where(occupancy < config["free_thresh"], 0, -1),
        )
        assert grid.header.frame_id == "map"
        assert tuple(expected.shape) == (grid.info.height, grid.info.width)
        assert np.array_equal(np.flipud(expected).ravel(), grid.data)
        assert math.isclose(grid.info.resolution, config["resolution"], rel_tol=1e-6)
        assert np.allclose(
            [grid.info.origin.position.x, grid.info.origin.position.y],
            config["origin"][:2],
            rtol=0,
            atol=1e-6,
        )
        return dict(
            frame=grid.header.frame_id,
            width=grid.info.width,
            height=grid.info.height,
            resolution_m=config["resolution"],
            origin=config["origin"],
            exact_occupancy_cells=len(grid.data),
            pgm_sha256=hashlib.sha256(pgm.read_bytes()).hexdigest(),
            rerender_identical=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_folder", type=Path)
    print(json.dumps(verify(parser.parse_args().map_folder.resolve()), indent=2))
