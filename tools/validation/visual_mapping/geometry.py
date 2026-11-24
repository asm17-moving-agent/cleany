"""Geometry for the fixed-head Floor 19 offline reconstruction."""

import json
import sqlite3
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_track(path):
    d = np.genfromtxt(path, names=True, delimiter=",")
    return np.column_stack((d["time_ns"] / 1e9, d["x"], d["y"], np.unwrap(d["yaw"])))


def sample(track, times):
    return np.stack([np.interp(times, track[:, 0], track[:, i]) for i in (1, 2, 3)], axis=-1)


def relative(a, b):
    delta = b[..., :2] - a[..., :2]
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack(
        (
            c * delta[..., 0] + s * delta[..., 1],
            -s * delta[..., 0] + c * delta[..., 1],
            wrap(b[..., 2] - a[..., 2]),
        ),
        axis=-1,
    )


def transform(pose, xy):
    c, s = np.cos(pose[..., 2]), np.sin(pose[..., 2])
    return np.stack(
        (pose[..., 0] + c * xy[..., 0] - s * xy[..., 1], pose[..., 1] + s * xy[..., 0] + c * xy[..., 1]),
        axis=-1,
    )


def camera(root):
    record = json.loads((root / "input/manifest/00.json").read_text())
    frames = record["transforms"]
    # The recorded runtime model and the driver use different names for the head RGB optical frame.
    current = "cad_head_camera_optical_frame"
    base_T_rgb = np.eye(4)
    while current != "base_link":
        tf = frames[current]
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(tf["quat"]).as_matrix()
        matrix[:3, 3] = tf["xyz"]
        base_T_rgb = matrix @ base_T_rgb
        current = tf["parent"]
    k = np.asarray(record["cameras"]["rgb"]["K"]).reshape(3, 3)
    return k, base_T_rgb


def scan_cache(database, dest):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    with sqlite3.connect(database.as_uri() + "?mode=ro", uri=True) as con:

        def records(topic):
            return con.execute(
                "select data from messages where topic_id=(select id from topics where name=?) order by timestamp",
                (topic,),
            )

        tf = deserialize_message(next(records("/tf_static"))[0], TFMessage)
        m = next(
            t.transform
            for t in tf.transforms
            if (t.header.frame_id, t.child_frame_id) == ("base_link", "lidar_link")
        )
        q = m.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[:2, :2]
        origin = np.array([m.translation.x, m.translation.y])
        stamps, hits, intervals = [], [], []
        for (blob,) in records("/scan"):
            msg = deserialize_message(blob, LaserScan)
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
            idx = np.arange(0, len(msg.ranges), 3)
            angle = msg.angle_min + idx * msg.angle_increment
            rng = np.asarray(msg.ranges)[idx]
            rng[~np.isfinite(rng) | (rng < 0.30) | (rng > 12.0)] = np.nan
            hits.append(
                (np.stack((rng * np.cos(angle), rng * np.sin(angle)), axis=1) @ rotation.T + origin).astype(
                    "f4"
                )
            )
            intervals.append(float(msg.time_increment) * 3)
    np.savez(dest, times=np.array(stamps), xy=np.stack(hits), increments=np.array(intervals), origin=origin)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("database", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    scan_cache(args.database.resolve(), args.output)
