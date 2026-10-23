import base64
import gzip
import io
import math
from types import SimpleNamespace as N

import pytest
from cleany_logger import Recorder
from cleany_dev_monitor.core import Hub
from cleany_logger.recording import defaults, validate
from cleany_dev_monitor.slam import SlamState, marker_geometry, texture_png
from cleany_dev_monitor.sources import SCHEMA, resolved, routes, source_roles
from PIL import Image


def pose(x=0.0, y=0.0, angle=0.0):
    return N(
        position=N(x=x, y=y, z=0.0),
        orientation=N(x=0.0, y=0.0, z=math.sin(angle / 2), w=math.cos(angle / 2)),
    )


def entry(index=0, version=1, x=0.0, angle=0.0):
    return N(
        trajectory_id=0,
        submap_index=index,
        submap_version=version,
        pose=pose(x=x, angle=angle),
        is_frozen=False,
    )


def texture(raw=bytes([0, 255, 128, 128, 0, 0, 64, 128]), width=2, height=2):
    return N(
        width=width,
        height=height,
        cells=gzip.compress(raw),
        resolution=0.1,
        slice_pose=pose(x=1.0, y=2.0, angle=math.pi / 2),
    )


def test_texture_pixels_alpha_and_pose_are_not_transposed():
    value = texture_png(texture(), 4)
    image = Image.open(io.BytesIO(base64.b64decode(value["image"].split(",")[1])))
    assert image.size == (2, 2)
    assert image.getpixel((0, 0)) == (0, 0, 0, 255)
    assert image.getpixel((1, 0)) == (0, 0, 0, 128)
    assert image.getpixel((0, 1)) == (0, 0, 0, 0)
    assert image.getpixel((1, 1)) == (0, 0, 0, 128)
    light = Image.open(io.BytesIO(base64.b64decode(value["light_image"].split(",")[1])))
    assert light.getpixel((1, 1)) == (255, 255, 255, 64)
    free = texture_png(texture(raw=bytes([127, 0]), width=1, height=1), 4)
    free_light = Image.open(
        io.BytesIO(base64.b64decode(free["light_image"].split(",")[1]))
    )
    assert free_light.getpixel((0, 0)) == (255, 255, 255, 127)
    assert value["slice_pose"] == pytest.approx([1.0, 2.0, math.pi / 2])


@pytest.mark.parametrize(
    "bad",
    [texture(width=3), texture(raw=b"a" * 100000), texture(raw=b"a"), texture(raw=b"")],
)
def test_texture_rejects_geometry_and_gzip_expansion_overflow(bad):
    with pytest.raises(ValueError):
        texture_png(bad, 4)


def test_pose_only_optimization_reuses_texture_but_restart_and_late_queries_do_not():
    state = SlamState(submap_limit=2)
    first, _ = state.update(N(submap=[entry()]), [10.0, 0.0, math.pi / 2])
    old_epoch = first["epoch"]
    image = texture_png(texture(), 4)
    assert state.accept("0/0", old_epoch, 1, image)
    data, events = state.update(N(submap=[entry(x=0.2)]), [10.0, 0.0, math.pi / 2])
    assert data["submaps"][0]["pose"] == pytest.approx([10.0, 0.2, math.pi / 2])
    assert events[0]["distance_m"] == pytest.approx(0.2)
    assert state.pending() == []
    state.update(N(submap=[entry(version=2)]), [0.0, 0.0, 0.0])
    assert state.accept("0/0", old_epoch, 1, image)["version"] == 1
    assert not state.accept("0/0", old_epoch, 3, image)
    assert len(state.pending()) == 1
    restarted, events = state.update(N(submap=[entry(version=0)]), [0.0, 0.0, 0.0])
    assert restarted["epoch"] != old_epoch and events == [{"kind": "slam_reset"}]
    assert not state.accept("0/0", old_epoch, 0, image)


def test_removed_maps_and_large_sessions_are_bounded_and_invalid_pose_is_atomic():
    state = SlamState(submap_limit=2)
    value, _ = state.update(N(submap=[entry(i) for i in range(10)]), [0.0, 0.0, 0.0])
    assert value["truncated"] and value["source_count"] == 10
    assert set(state.entries) == {"0/8", "0/9"}
    state.accept("0/8", state.epoch, 1, texture_png(texture(), 4))
    state.update(N(submap=[entry(9)]), [0.0, 0.0, 0.0])
    assert not state.textures
    bad = entry(10, x=float("nan"))
    with pytest.raises(ValueError):
        state.update(N(submap=[bad]), [0.0, 0.0, 0.0])
    assert set(state.entries) == {"0/9"}


def test_marker_pairs_transform_and_incremental_deletion_have_bounded_retention():
    def marker(identity=0, action=0):
        return N(
            ns="constraints",
            id=identity,
            action=action,
            type=5,
            pose=pose(angle=math.pi / 2),
            header=N(frame_id="local", stamp=None),
            color=N(r=1.0, g=0.2, b=0.1, a=1.0),
            points=[N(x=x, y=0.0, z=0.0) for x in range(5)],
        )

    data = marker_geometry(N(markers=[marker()]), lambda *_: [2.0, 3.0, 0.0], 3)
    assert data["truncated"]
    assert data["markers"][0]["points"][0] == [2.0, 3.0]
    assert data["markers"][0]["points"][1] == pytest.approx([2.0, 4.0])
    state = SlamState()
    assert len(state.markers("constraints", data, 4)["markers"]) == 1
    deletion = marker_geometry(N(markers=[marker(action=2)]), lambda *_: [], 3)
    assert not state.markers("constraints", deletion, 4)["markers"]
    for i in range(20):
        state.markers(
            "constraints",
            marker_geometry(N(markers=[marker(i)]), lambda *_: [0.0, 0.0, 0.0], 2),
            4,
        )
    assert len(state.marker_sets["constraints"]) == 2
    clear = marker_geometry(N(markers=[marker(action=3)]), lambda *_: [], 3)
    assert not state.markers("constraints", clear, 4)["markers"]


def test_historical_submap_images_and_poses_survive_rate_limit_and_live_mutation(
    tmp_path,
):
    hub = Hub()
    config, *_ = routes({})
    hub.source_roles = source_roles(config)
    hub.source_categories = {
        topic: SCHEMA[key]["group"]
        for key, topic in resolved(config).items()
        if isinstance(topic, str)
    }
    recorder = Recorder(tmp_path, controls=hub.recording)
    # Deliberately slow source setting must not drop texture/list coherence.
    settings = defaults()
    settings["source_hz"]["slam_submaps_topic"] = 0.1
    hub.recording.configure(settings)
    topic = config["slam_submaps_topic"]
    hub.put(
        "slam",
        "submaps",
        {
            "valid": True,
            "topic": topic,
            "epoch": "one",
            "submaps": [{"id": "0/0", "version": 1, "pose": [0, 0, 0]}],
        },
    )
    first_image = hub.put(
        "slam_texture",
        "0/0",
        {"topic": topic, "epoch": "one", "version": 1, "image": "old-pixels"},
    )
    recorder.append(hub.snapshot())
    hub.put(
        "slam",
        "submaps",
        {
            "valid": True,
            "topic": topic,
            "epoch": "one",
            "submaps": [{"id": "0/0", "version": 2, "pose": [1, 2, 0]}],
        },
    )
    second_image = hub.put(
        "slam_texture",
        "0/0",
        {"topic": topic, "epoch": "one", "version": 2, "image": "new-pixels"},
    )
    recorder.append(hub.snapshot(recorder.last))
    earlier = recorder.replay(recorder.session_id, first_image["received_at"])["rows"]
    later = recorder.replay(recorder.session_id, second_image["received_at"])["rows"]
    assert next(r["data"] for r in earlier if r["kind"] == "slam")["submaps"][0][
        "pose"
    ] == [0, 0, 0]
    assert (
        next(r["data"] for r in earlier if r["kind"] == "slam_texture")["image"]
        == "old-pixels"
    )
    assert (
        next(r["data"] for r in later if r["kind"] == "slam_texture")["image"]
        == "new-pixels"
    )
    recorder.close()


def test_pre_slam_recording_settings_migrate_without_overwriting_user_choices():
    old = defaults()
    del old["enabled"]["slam"]
    del old["hz"]["slam"]
    old["enabled"]["motion"] = False
    old["source_hz"]["odometry_topic"] = 1.3
    updated = validate(old)
    assert updated["enabled"]["slam"] and not updated["enabled"]["motion"]
    assert updated["source_hz"]["odometry_topic"] == 1.3


def test_texture_cache_budget_eviction_is_explicit_and_does_not_requery_same_version():
    state = SlamState(submap_limit=3, texture_bytes_limit=100)
    state.update(N(submap=[entry(i) for i in range(3)]), [0.0, 0.0, 0.0])
    pixels = {"image": "x" * 30, "light_image": "y" * 30}
    state.accept("0/0", state.epoch, 1, pixels)
    state.accept("0/1", state.epoch, 1, pixels)
    assert set(state.textures) == {"0/1"}
    assert state.omitted_textures == {"0/0": 1}
    assert [s["id"] for s in state.pending()] == ["0/2"]
    state.update(N(submap=[entry(0, version=2), entry(1), entry(2)]), [0.0, 0.0, 0.0])
    assert "0/0" not in state.omitted_textures
    assert [s["id"] for s in state.pending()] == ["0/0", "0/2"]
