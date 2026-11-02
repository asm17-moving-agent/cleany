"""Validate capture failure detection without launching ROS or a simulator."""
import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest


TOOL = Path(__file__).resolve().parents[2] / 'tools/validation/capture_cameras.py'
spec = importlib.util.spec_from_file_location('capture_cameras', TOOL)
capture = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = capture
spec.loader.exec_module(capture)


def message(data: bytes, *, encoding='rgb8', step=8, bigendian=False):
    return SimpleNamespace(width=2, height=1, step=step, data=data,
                           encoding=encoding, is_bigendian=bigendian,
                           header=SimpleNamespace(frame_id='optical'))


def test_bgr_padding_is_removed_and_channels_are_rgb():
    image = message(bytes([3, 2, 1, 6, 5, 4, 99, 99]), encoding='bgr8')
    expected = capture.Camera('/rgb', 'optical', 2, 1)
    np.testing.assert_array_equal(capture.image_pixels(image, expected),
                                  [[[1, 2, 3], [4, 5, 6]]])


def test_big_endian_depth_with_padding():
    data = np.array([1.25, 2.5], dtype='>f4').tobytes() + b'padding!'
    image = message(data, encoding='32FC1', step=16, bigendian=True)
    np.testing.assert_allclose(capture.image_pixels(
        image, capture.Camera('/depth', 'optical', 2, 1, depth=True)), [[1.25, 2.5]])


@pytest.mark.parametrize('field,value', [
    ('width', 3), ('encoding', 'mono8'), ('step', 5), ('data', b''),
    ('header', SimpleNamespace(frame_id='wrong_frame')),
])
def test_invalid_image_contract_fails(field, value):
    image = message(bytes(8))
    setattr(image, field, value)
    with pytest.raises(ValueError):
        capture.image_pixels(image, capture.Camera('/rgb', 'optical', 2, 1))


def test_cli_rejects_invalid_timeout_before_creating_output(tmp_path):
    output = tmp_path / 'capture'
    result = subprocess.run([sys.executable, str(TOOL), '--backend', 'mujoco',
                             '--timeout', '0', '--output', str(output)], capture_output=True)
    assert result.returncode == 2
    assert not output.exists()


def test_owned_process_is_stopped():
    process = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                               start_new_session=True, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    try:
        capture.stop_process(process)
        assert process.poll() is not None
    finally:
        if process.poll() is None:
            capture.stop_process(process)
