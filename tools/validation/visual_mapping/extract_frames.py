"""Stream timestamp-paired RGB-D and rectified stereo samples from one recorded split."""

import argparse
import collections
import ctypes
import io
import json
import sqlite3
import struct
import sys
import tarfile
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_msgs.msg import TFMessage

IMAGES = {
    "/camera/sdk/color/image_raw/compressed": "rgb",
    "/camera/aligned_depth_to_color/image_raw": "depth",
    "/camera/infra1/image_rect_raw": "left",
    "/camera/infra2/image_rect_raw": "right",
}
INFO = {
    "/camera/sdk/color/camera_info": "rgb",
    "/camera/aligned_depth_to_color/camera_info": "depth",
    "/camera/infra1/camera_info": "left",
    "/camera/infra2/camera_info": "right",
}
FAMILIES = {
    "rgb": ("rgb", "depth"),
    "depth": ("rgb", "depth"),
    "left": ("left", "right"),
    "right": ("left", "right"),
}


def decompressor():
    z = ctypes.CDLL("libzstd.so.1")
    z.ZSTD_getFrameContentSize.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    z.ZSTD_getFrameContentSize.restype = ctypes.c_ulonglong
    z.ZSTD_decompress.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
    z.ZSTD_decompress.restype = ctypes.c_size_t
    buf = ctypes.create_string_buffer(2**21)

    def unpack(blob):
        n = z.ZSTD_getFrameContentSize(blob, len(blob))
        if n > len(buf) or z.ZSTD_decompress(buf, len(buf), blob, len(blob)) != n:
            raise ValueError("Invalid compressed ROS message")
        return buf.raw[:n] if n > 1000000 else ctypes.string_at(buf, n)

    return unpack


def store(tar, name, payload):
    header = tarfile.TarInfo(name)
    header.size = len(payload)
    tar.addfile(header, io.BytesIO(payload))


def image_data(kind, raw):
    if kind == "rgb":
        return "jpg", bytes(deserialize_message(raw, CompressedImage).data)
    msg = deserialize_message(raw, Image)
    dtype = "<u2" if kind == "depth" else "u1"
    array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, -1)[:, : msg.width]
    if kind == "depth":
        # Keep millimeters unchanged at every second calibrated pixel.
        ok, image = cv2.imencode(".png", array[::2, ::2], [cv2.IMWRITE_PNG_COMPRESSION, 2])
        ext = "png"
    else:
        ok, image = cv2.imencode(".jpg", array, [cv2.IMWRITE_JPEG_QUALITY, 93])
        ext = "jpg"
    if not ok:
        raise ValueError("Image encode failed")
    return ext, image.tobytes()


def extract(bag, split, interval):
    db = bag / f"{bag.name}_{split}.db3"
    if not db.is_file():
        raise FileNotFoundError(db)
    counts = collections.Counter()
    info, tf, pending, completed = {}, {}, {}, set()
    cv2.setNumThreads(1)
    unpack = decompressor()
    with sqlite3.connect(db.as_uri() + "?mode=ro", uri=True) as conn, tarfile.open(
        mode="w|", fileobj=sys.stdout.buffer
    ) as tar:
        topics = dict(conn.execute("SELECT id,name FROM topics"))
        for ix, data in conn.execute("SELECT topic_id,data FROM messages ORDER BY id"):
            name = topics[ix]
            if name in IMAGES:
                kind = IMAGES[name]
                raw = unpack(data)
                sec, ns = struct.unpack_from("<iI", raw, 4)
                stamp = sec * 10**9 + ns
                key = (FAMILIES[kind][0], stamp // interval)
                counts[kind] += 1
                if key in completed:
                    continue
                for k in list(pending):
                    if k[1] < key[1] - 8:
                        del pending[k]
                sample = pending.setdefault(key, {}).setdefault(stamp, {})
                sample[kind] = raw
                if len(sample) == 2:
                    for channel, cdr in sample.items():
                        ext, img = image_data(channel, cdr)
                        store(tar, f"{channel}/{stamp}.{ext}", img)
                    completed.add(key)
                    counts[key[0] + "_pairs"] += 1
                    del pending[key]
            elif name in INFO and INFO[name] not in info:
                cam = deserialize_message(unpack(data), CameraInfo)
                info[INFO[name]] = dict(
                    frame=cam.header.frame_id,
                    width=cam.width,
                    height=cam.height,
                    K=list(cam.k),
                    P=list(cam.p),
                    D=list(cam.d),
                    model=cam.distortion_model,
                )
            elif name in (
                "/tf_static",
                "/runtime_model/tf_static",
                "/floor19/tf_static",
                "/runtime_model/tf",
            ) and (name.endswith("static") or counts[name] < 5):
                counts[name] += 1
                for m in deserialize_message(unpack(data), TFMessage).transforms:
                    t, q = m.transform.translation, m.transform.rotation
                    tf[m.child_frame_id] = dict(
                        parent=m.header.frame_id, xyz=[t.x, t.y, t.z], quat=[q.x, q.y, q.z, q.w], topic=name
                    )
        report = dict(
            source=str(db), split=split, interval_ns=interval, counts=counts, cameras=info, transforms=tf
        )
        store(tar, f"manifest/{split:02}.json", json.dumps(report, indent=2).encode())


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag", type=Path)
    p.add_argument("split", type=int)
    p.add_argument("--interval-ns", type=int, default=200000000)
    a = p.parse_args()
    extract(a.bag, a.split, a.interval_ns)
