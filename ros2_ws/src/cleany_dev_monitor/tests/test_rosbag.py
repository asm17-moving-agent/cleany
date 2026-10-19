"""Check the storage boundary against native rosbag2, not a lookalike file."""

import asyncio
import json
import sqlite3
import zipfile
import zlib
from io import BytesIO

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_dev_monitor.core import Hub, Recorder
from cleany_dev_monitor.recording import defaults
from cleany_dev_monitor.server import create_app


def raw_settings():
    settings = defaults()
    settings["enabled"].update(camera=True, point_cloud=True)
    return settings


def test_missing_rosbag_dependency_stops_explicitly_without_sqlite_fallback(tmp_path):
    def unavailable(path):
        raise ImportError("rosbag2_py missing")

    hub = Hub()
    hub.recording.configure(raw_settings())
    recorder = Recorder(tmp_path, controls=hub.recording, bag_factory=unavailable)
    topic = raw_settings()["topics"]["camera"]
    hub.recording.offer(
        hub.recording.reserve(topic), topic, "sensor_msgs/msg/Image", b"raw", 1.0
    )
    recorder.append_raw()
    assert "rosbag2_py missing" in recorder.reason
    assert not hub.recording.topics()
    assert recorder.db.execute("SELECT count(*) FROM raw_index").fetchone()[0] == 0
    assert recorder.raw_table(recorder.db) == "raw_index"
    loss = json.loads(
        recorder.db.execute(
            "SELECT value FROM metadata WHERE key='raw_gap'"
        ).fetchone()[0]
    )
    assert loss["camera"]["dropped"] == 1
    recorder.close()


def test_combined_quota_counts_other_bags_and_retains_them(tmp_path):
    recorder = Recorder(tmp_path, total_limit=100_000)
    bag = tmp_path / "bags" / "previous"
    bag.mkdir(parents=True)
    evidence = bag / "previous.db3"
    evidence.write_bytes(b"x" * 100_000)
    hub = Hub()
    hub.put("resources", "host", {"cpu_percent": 10})
    recorder.append(hub.snapshot())
    assert "size limit" in recorder.reason
    assert evidence.stat().st_size == 100_000
    assert recorder.disk_bytes() >= 100_000
    recorder.close()


def test_legacy_sqlite_payload_stays_readable(tmp_path):
    recorder = Recorder(tmp_path)
    recorder.db.execute("DROP TABLE raw_index")
    recorder.db.execute(
        "CREATE TABLE raw_messages(id INTEGER PRIMARY KEY, category TEXT, topic TEXT, type TEXT, received REAL, ros_time REAL, payload BLOB)"
    )
    recorder.db.execute(
        "INSERT INTO raw_messages VALUES (1,?,?,?,?,?,?)",
        (
            "camera",
            "/old/image",
            "sensor_msgs/msg/Image",
            12.0,
            2.0,
            zlib.compress(b"old cdr"),
        ),
    )
    recorder.db.commit()
    assert recorder.raw_message(recorder.session_id, "camera", 12.0) == (
        b"old cdr",
        "sensor_msgs/msg/Image",
        "/old/image",
    )
    assert (
        recorder.replay(recorder.session_id, 12.0)["rows"][0]["data"]["format"]
        == "legacy_sqlite"
    )
    recorder.close()


def test_camera_context_selection_and_tf_are_not_sampled(monkeypatch):
    hub = Hub()
    hub.recording.configure(raw_settings())
    assert {
        "/tf",
        "/tf_static",
        "/clock",
        "/camera/head/color/camera_info",
    } <= hub.recording.topics()
    monkeypatch.setattr("cleany_dev_monitor.recording.time.monotonic", lambda: 100.0)
    assert hub.recording.reserve("/tf")
    assert hub.recording.reserve("/tf")  # Distinct transforms may arrive in a burst.
    assert hub.recording.reserve("/tf_static")
    assert hub.recording.reserve("/tf_static")


def test_session_transition_seals_inflight_raw_callbacks():
    hub = Hub()
    hub.recording.configure(raw_settings())
    topic = raw_settings()["topics"]["camera"]
    ticket = hub.recording.reserve(topic)
    with hub.recording.transition():
        assert hub.recording.reserve(topic) is None
        hub.recording.offer(
            ticket, topic, "sensor_msgs/msg/Image", b"late previous session", 1.0
        )
        assert hub.recording.drain() == []
        hub.recording.configure(raw_settings())
        assert hub.recording.reserve(topic) is None
    assert hub.recording.reserve(topic) is not None


def test_navigation_context_respects_remapping_selection_and_native_bytes(tmp_path):
    pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message, deserialize_message
    from nav_msgs.msg import OccupancyGrid, Path

    hub = Hub()
    controls = hub.recording
    controls.source_config = {'map_topic': '/robot/map'}
    settings = defaults()
    settings['sources']['path_topic'] = '/robot/plan'
    settings['source_enabled']['local_path_topic'] = False
    controls.configure(settings)
    assert {'/robot/map', '/robot/plan', '/tf', '/tf_static', '/clock',
            '/joint_states', '/robot_description'} <= controls.topics()
    assert '/map' not in controls.topics() and '/local_plan' not in controls.context_topics()
    recorder = Recorder(tmp_path, controls=controls)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 1
    grid.info.resolution = 0.1
    grid.data = [100]
    for topic, message, typ in [('/robot/map', grid, 'nav_msgs/msg/OccupancyGrid'),
                                 ('/robot/plan', Path(), 'nav_msgs/msg/Path')]:
        ticket = controls.reserve(topic)
        assert ticket
        controls.offer(ticket, topic, typ, serialize_message(message), 10., timestamp_ns=10_000_000_000)
    recorder.append_raw()
    recorder.close()
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(recorder.resolve_bag(recorder.session_id)), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    saved = {}
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        saved[topic] = payload
    assert list(deserialize_message(saved['/robot/map'], OccupancyGrid).data) == [100]
    assert '/robot/plan' in saved
    settings['enabled'].update(motion=False, paths=False, costmaps=False)
    controls.configure(settings)
    assert controls.reserve('/robot/map') is None
    assert controls.reserve('/tf') is None


def test_native_bag_preserves_images_clouds_timestamps_qos_and_separate_exports(
    tmp_path,
):
    rosbag2_py = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Image, PointCloud2
    from tf2_msgs.msg import TFMessage

    async def scenario():
        hub = Hub()
        recorder = Recorder(tmp_path, controls=hub.recording)
        settings = raw_settings()
        messages = [
            (
                settings["topics"]["camera"],
                Image(
                    height=1,
                    width=2,
                    encoding="rgb8",
                    step=6,
                    data=b"\x01\x02\x03\x04\x05\x06",
                ),
                "sensor_msgs/msg/Image",
            ),
            (
                settings["topics"]["point_cloud"],
                PointCloud2(
                    height=1, width=1, point_step=16, row_step=16, data=b"\xff" * 16
                ),
                "sensor_msgs/msg/PointCloud2",
            ),
            ("/tf_static", TFMessage(), "tf2_msgs/msg/TFMessage"),
        ]
        async with (
            TestServer(create_app(hub, {}, recorder, tmp_path)) as server,
            ClientSession() as client,
        ):
            assert (
                await client.put(
                    server.make_url("/api/recording/settings"), json=settings
                )
            ).status == 200
            for index, (topic, message, typ) in enumerate(messages):
                hub.recording.offer(
                    hub.recording.reserve(topic),
                    topic,
                    typ,
                    serialize_message(message),
                    7.0 + index,
                    timestamp_ns=7_000_000_000 + index,
                    reliable=True,
                    latched=topic == "/tf_static",
                )
            recorder.append_raw()
            assert not recorder.reason
            assert (
                recorder.db.execute("SELECT count(*) FROM raw_index").fetchone()[0] == 3
            )
            columns = [
                row[1] for row in recorder.db.execute("PRAGMA table_info(raw_index)")
            ]
            assert "payload" not in columns
            session = recorder.session_id
            assert (
                await client.get(server.make_url(f"/api/recordings/{session}/bag"))
            ).status == 409
            assert (
                await client.put(
                    server.make_url("/api/recording/settings"),
                    json={**settings, "active": False},
                )
            ).status == 200
            assert recorder.bag is None and recorder.db is None
            assert (recorder.bag_path / "metadata.yaml").is_file()
            response = await client.get(
                server.make_url(f"/api/recordings/{session}/bag")
            )
            assert response.status == 200
            with zipfile.ZipFile(BytesIO(await response.read())) as archive:
                assert f"{session}/metadata.yaml" in archive.namelist()
                assert any(name.endswith(".db3") for name in archive.namelist())
                assert not any(name.endswith(".sqlite3") for name in archive.namelist())
            response = await client.get(
                server.make_url(f"/api/recordings/{session}/export")
            )
            exported = tmp_path / "export-check.sqlite3"
            exported.write_bytes(await response.read())
            with sqlite3.connect(exported) as db:
                assert recorder.raw_table(db) == "raw_index"
                assert (
                    dict(db.execute("SELECT key,value FROM metadata"))["bag_uri"]
                    == f"bags/{session}"
                )
            exported.unlink()
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(
                    uri=str(recorder.bag_path), storage_id="sqlite3"
                ),
                rosbag2_py.ConverterOptions("", ""),
            )
            actual = []
            while reader.has_next():
                actual.append(reader.read_next())
            assert len(actual) == 3
            for index, (topic, message, _) in enumerate(messages):
                assert actual[index][0] == topic
                assert deserialize_message(actual[index][1], type(message)) == message
                assert actual[index][2] == 7_000_000_000 + index
            static = next(
                topic
                for topic in reader.get_all_topics_and_types()
                if topic.name == "/tf_static"
            )
            assert json.loads(static.offered_qos_profiles)[0]["durability"] == 1
            previous = recorder.path.read_bytes()
            assert (
                await client.post(
                    server.make_url("/api/recording/session"), json=settings
                )
            ).status == 200
            assert recorder.session_id != session
            assert (tmp_path / f"{session}.sqlite3").read_bytes() == previous
            recorder.close()

    asyncio.run(scenario())


def test_duplicate_ros_timestamps_select_the_correct_raw_message(tmp_path, monkeypatch):
    pytest.importorskip("rosbag2_py")
    hub = Hub()
    hub.recording.configure(raw_settings())
    recorder = Recorder(tmp_path, controls=hub.recording)
    topic = raw_settings()["topics"]["camera"]
    clock = iter((100.0, 102.0))
    monkeypatch.setattr(
        "cleany_dev_monitor.recording.time.monotonic", lambda: next(clock)
    )
    for payload in (b"first frame", b"second frame"):
        hub.recording.offer(
            hub.recording.reserve(topic),
            topic,
            "sensor_msgs/msg/Image",
            payload,
            7.0,
            timestamp_ns=7_000_000_000,
        )
    recorder.append_raw()
    received = [
        r[0] for r in recorder.db.execute("SELECT received FROM raw_index ORDER BY id")
    ]
    assert (
        recorder.raw_message(recorder.session_id, "camera", received[0])[0]
        == b"first frame"
    )
    assert (
        recorder.raw_message(recorder.session_id, "camera", received[1])[0]
        == b"second frame"
    )
    recorder.close()


def test_raw_loss_totals_survive_setting_changes_without_double_counting(tmp_path):
    hub = Hub()
    hub.recording.configure(raw_settings())
    recorder = Recorder(tmp_path, controls=hub.recording)
    hub.recording.dropped("camera", 2)
    recorder.append_raw()
    with hub.recording.transition():
        recorder.append_raw()  # Sealing tickets must not re-count old losses.
        hub.recording.configure(raw_settings())
    hub.recording.dropped("camera", 1)
    recorder.append_raw()
    recorder.append_raw()
    loss = json.loads(
        recorder.db.execute(
            "SELECT value FROM metadata WHERE key='raw_gap'"
        ).fetchone()[0]
    )
    assert loss["camera"]["dropped"] == 3
    recorder.close()


def test_failed_bag_finalization_is_not_exported_as_complete(tmp_path):
    class BrokenFinalizer:
        def __init__(self, path):
            self.path = path
            path.mkdir()

        def write(self, message):
            pass

        def close(self):
            (self.path / "metadata.yaml").write_text("partial metadata")
            raise RuntimeError("disk failure while finalizing")

    async def scenario():
        hub = Hub()
        recorder = Recorder(
            tmp_path, controls=hub.recording, bag_factory=BrokenFinalizer
        )
        async with (
            TestServer(create_app(hub, {}, recorder, tmp_path)) as server,
            ClientSession() as client,
        ):
            settings = raw_settings()
            assert (
                await client.put(
                    server.make_url("/api/recording/settings"), json=settings
                )
            ).status == 200
            topic = settings["topics"]["camera"]
            hub.recording.offer(
                hub.recording.reserve(topic),
                topic,
                "sensor_msgs/msg/Image",
                b"raw",
                1.0,
            )
            recorder.append_raw()
            session = recorder.session_id
            assert (
                await client.put(
                    server.make_url("/api/recording/settings"),
                    json={**settings, "active": False},
                )
            ).status == 200
            assert "finalization failed" in recorder.reason
            assert not recorder.bag_available(session)
            assert recorder.sessions()[0]["metadata"]["bag_state"] == "incomplete"
            assert (
                await client.get(server.make_url(f"/api/recordings/{session}/bag"))
            ).status == 404
            assert (
                recorder.bag_path / "metadata.yaml"
            ).read_text() == "partial metadata"

    asyncio.run(scenario())
