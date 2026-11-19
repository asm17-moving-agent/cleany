"""Sampling and coordinate checks for recorded directional-slip reports."""
import importlib.util
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'slip_direction_report', Path(__file__).resolve().parents[2] / 'tools/validation/slip_direction_report.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def test_command_frame_uses_window_start_and_unwraps_yaw():
    commands = [(t, 1., 0.) for t in [0., .2, .4, .6, .8]]
    delta, coverage = report.command_displacement(commands, [(0., math.pi/2), (1., math.pi/2)], 0., 1.)
    assert coverage == pytest.approx(1.)
    assert delta == pytest.approx([1., 0.])
    delta, _ = report.command_displacement(commands, [(0., math.radians(179)), (1., math.radians(-179))], 0., 1.)
    assert report.direction(delta) == pytest.approx(1., abs=.01)


def test_recording_gap_is_not_filled_with_invented_command():
    delta, coverage = report.command_displacement([(0., 1., 1.)], [(0., 0.), (1., 0.)], 0., 1.)
    assert delta is None and coverage == pytest.approx(.3)


def test_near_stationary_direction_is_not_reported_as_forward():
    assert report.direction([0., 0.]) is None
    assert report.direction([.001, .001]) is None
    assert report.direction([float('nan'), 1.]) is None
    assert report.direction([1., 1.]) == pytest.approx(45.)
