import json
import math

import pytest

from planar_extrinsic import PlanarExtrinsic


def test_defaults_and_json_loading(tmp_path):
    extrinsic = PlanarExtrinsic()
    assert extrinsic.to_dict() == {
        "yaw_deg": -7.85, "lever_arm_x_m": 0.029, "lever_arm_y_m": 0.0
    }
    assert PlanarExtrinsic.from_json(None) == extrinsic
    path = tmp_path / "extrinsic.json"
    path.write_text(json.dumps({
        **extrinsic.to_dict(), "provenance": "ignored"
    }))
    assert PlanarExtrinsic.from_json(path) == extrinsic


def test_identity_and_pure_translation():
    extrinsic = PlanarExtrinsic()
    assert extrinsic.lidar_to_base(0, 0, 0) == (0, 0, 0)
    got = extrinsic.lidar_to_base(1, 0, 0)
    assert got == pytest.approx((math.cos(math.radians(-7.85)),
                                 math.sin(math.radians(-7.85)), 0))


def test_rotation_and_both_lever_arms():
    ext = PlanarExtrinsic(yaw_deg=0, lever_arm_x_m=2, lever_arm_y_m=3)
    assert ext.lidar_to_base(0, 0, math.pi / 2) == pytest.approx((5, 1, math.pi / 2))


def test_inverse_round_trip_se2():
    ext = PlanarExtrinsic(yaw_deg=31, lever_arm_x_m=-0.2, lever_arm_y_m=0.4)
    motion = (0.7, -1.3, 2.4)
    assert ext.base_to_lidar(*ext.lidar_to_base(*motion)) == pytest.approx(motion)
    assert ext.lidar_to_base(*ext.base_to_lidar(*motion)) == pytest.approx(motion)


@pytest.mark.parametrize("data", [
    {"yaw_deg": 0, "lever_arm_x_m": 0},
    {"yaw_deg": True, "lever_arm_x_m": 0, "lever_arm_y_m": 0},
    {"yaw_deg": "0", "lever_arm_x_m": 0, "lever_arm_y_m": 0},
    {"yaw_deg": float("nan"), "lever_arm_x_m": 0, "lever_arm_y_m": 0},
    {"yaw_deg": 0, "lever_arm_x_m": float("inf"), "lever_arm_y_m": 0},
])
def test_invalid_json_values_rejected(tmp_path, data):
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        PlanarExtrinsic.from_json(path)
