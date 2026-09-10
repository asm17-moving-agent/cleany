"""MCU snapshot validation and count-to-angle conversion, without ROS."""

from collections import deque
from dataclasses import dataclass
from math import isfinite, pi
import re

from cleany_base_odometry.encoder_http import EncoderSample
from cleany_base_odometry.mecanum_odometry import WheelPositions


class RejectedEncoderSample(ValueError):
    """A sample cannot advance the wheel baseline."""


@dataclass(frozen=True)
class EncoderCalibration:
    """All arrays use PCB order M1 FL, M2 FR, M3 RR, M4 RL."""

    ticks_per_revolution: tuple[float, float, float, float]
    signs: tuple[int, int, int, int]

    def __post_init__(self) -> None:
        if len(self.ticks_per_revolution) != 4 or not all(
            isfinite(value) and value > 0 for value in self.ticks_per_revolution
        ):
            raise ValueError('Four positive finite encoder resolutions are required')
        if len(self.signs) != 4 or any(sign not in (-1, 1) for sign in self.signs):
            raise ValueError('Four encoder signs of -1 or +1 are required')


@dataclass(frozen=True)
class EncoderAngles:
    positions: WheelPositions
    stamp_ns: int
    baseline_reason: str = ''


class EncoderAngleTracker:
    def __init__(
        self, calibration: EncoderCalibration, max_gap_sec: float = 0.5,
        max_wheel_speed_rad_s: float = 50.0, max_clock_error_sec: float = 0.25,
    ) -> None:
        if not all(isfinite(v) and v > 0 for v in (
            max_gap_sec, max_wheel_speed_rad_s, max_clock_error_sec,
        )):
            raise ValueError('Gap, speed and clock limits must be positive and finite')
        self._calibration = calibration
        self._max_gap_sec = max_gap_sec
        self._max_wheel_speed = max_wheel_speed_rad_s
        self._max_clock_error_ns = int(max_clock_error_sec * 1e9)
        self._last: EncoderSample | None = None
        self._retired_boots: deque[str] = deque(maxlen=16)
        self._angles = [0.0] * 4
        self._anchor_mcu_us = 0
        self._anchor_ros_ns = 0
        self._last_receive_ns = 0

    def update(self, sample: EncoderSample, received_ns: int) -> EncoderAngles:
        if re.fullmatch(r'[0-9a-f]{16}', sample.boot_id) is None:
            raise RejectedEncoderSample('MCU boot ID and sample time are required')
        if (
            len(sample.ticks) != 4
            or any(type(v) is not int or not -(2**31) <= v < 2**31 for v in sample.ticks)
            or type(sample.sample_seq) is not int or not 0 <= sample.sample_seq < 2**32
            or type(sample.sample_time_us) is not int or not 0 <= sample.sample_time_us < 2**63
            or not isfinite(sample.round_trip_time_sec)
            or not 0 <= sample.round_trip_time_sec <= self._max_gap_sec
            or received_ns <= 0
        ):
            raise RejectedEncoderSample('Invalid encoder snapshot')
        if sample.boot_id in self._retired_boots:
            raise RejectedEncoderSample('Late packet from a retired MCU boot')

        previous = self._last
        baseline = ''
        # Sample time lies inside the request/response interval. The first RTT
        # midpoint only approximates its ROS epoch; MCU deltas provide timing
        # thereafter. This is not a synchronized MCU/Jetson clock.
        midpoint_ns = received_ns - round(sample.round_trip_time_sec * 0.5e9)
        if midpoint_ns <= 0:
            raise RejectedEncoderSample('Invalid response time interval')
        if previous is None:
            baseline = 'initial'
        elif sample.boot_id != previous.boot_id:
            self._retired_boots.append(previous.boot_id)
            baseline = 'boot_changed'
        else:
            sequence_delta = (sample.sample_seq - previous.sample_seq) % (2**32)
            if not 0 < sequence_delta < 2**31:
                raise RejectedEncoderSample('Duplicate or out-of-order sequence')
            dt_sec = (sample.sample_time_us - previous.sample_time_us) * 1e-6
            if dt_sec <= 0:
                raise RejectedEncoderSample('Non-increasing MCU timestamp')
            predicted_ns = self._anchor_ros_ns + (
                sample.sample_time_us - self._anchor_mcu_us
            ) * 1000
            delta_angles = [
                ((new - old + 2**31) % (2**32) - 2**31) * sign * 2 * pi / resolution
                for new, old, sign, resolution in zip(
                    sample.ticks, previous.ticks, self._calibration.signs,
                    self._calibration.ticks_per_revolution,
                )
            ]
            if dt_sec > self._max_gap_sec:
                baseline = 'sample_gap'
            elif received_ns <= self._last_receive_ns or abs(
                midpoint_ns - predicted_ns
            ) > self._max_clock_error_ns:
                baseline = 'clock_changed'
            elif any(abs(angle) / dt_sec > self._max_wheel_speed for angle in delta_angles):
                baseline = 'tick_jump'
            else:
                self._angles = [a + delta for a, delta in zip(self._angles, delta_angles)]

        if baseline:
            self._anchor_mcu_us = sample.sample_time_us
            self._anchor_ros_ns = midpoint_ns
        stamp_ns = self._anchor_ros_ns + (
            sample.sample_time_us - self._anchor_mcu_us
        ) * 1000
        self._last = sample
        self._last_receive_ns = received_ns
        fl, fr, rr, rl = self._angles
        return EncoderAngles(WheelPositions(fl, fr, rl, rr), stamp_ns, baseline)
