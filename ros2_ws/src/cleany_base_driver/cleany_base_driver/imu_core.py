"""Receipt-anchored timestamp mapping and IMU transport checks."""
from __future__ import annotations

from collections import deque
from math import isfinite


class ImuValidationError(ValueError):
    """An IMU sample violates the transport or sensor contract."""


class ImuAdapter:
    def __init__(self, expected_device_mac: str = '', max_transport_delay_s: float = 0.1,
                 max_clock_jump_s: float = 0.05) -> None:
        if any(not isfinite(value) or not 0 < value <= 60
               for value in (max_transport_delay_s, max_clock_jump_s)):
            raise ValueError("timing limits must be positive and finite")
        if not isinstance(expected_device_mac, str):
            raise ValueError("expected MAC must be a string")
        if expected_device_mac:
            parts = expected_device_mac.split(":")
            if len(parts) != 6 or any(
                    len(p) != 2 or any(c not in "0123456789abcdefABCDEF" for c in p)
                    for p in parts):
                raise ValueError("expected MAC must be six colon-separated hex bytes")
            self.expected = tuple(int(p, 16) for p in parts)
            if not any(self.expected) or self.expected[0] & 1:
                raise ValueError("expected MAC must be nonzero unicast")
        else:
            self.expected = None
        self.delay_ns = int(max_transport_delay_s * 1e9)
        self.jump_ns = int(max_clock_jump_s * 1e9)
        self.boot = self.seq = self.mcu_us = self.mac = self.anchor = None
        self.retired: deque[int] = deque(maxlen=32)

    def stamp(self, message, receipt_ros_ns: int, receipt_steady_ns: int) -> int:
        if (message.protocol_version != 2 or message.who_am_i != 0x68
                or message.fault_bits or not message.boot_id or not message.timestamp_us):
            raise ImuValidationError("invalid protocol, sensor identity, fault, or time")
        mac = tuple(message.device_mac)
        if not any(mac) or mac[0] & 1 or (self.expected and mac != self.expected):
            raise ImuValidationError("invalid or unexpected unicast device MAC")
        if self.mac is not None and mac != self.mac:
            raise ImuValidationError("device identity changed")
        reboot = self.boot is not None and message.boot_id != self.boot
        if reboot and message.boot_id in self.retired:
            raise ImuValidationError("retired boot ID")
        if self.boot is not None and not reboot:
            delta = (message.sequence - self.seq) & 0xffffffff
            if not 0 < delta < 0x80000000 or message.timestamp_us <= self.mcu_us:
                raise ImuValidationError("duplicate/out-of-order sample")
        if self.anchor is not None and not reboot:
            ros0, steady0, mcu0 = self.anchor
            elapsed = (message.timestamp_us - mcu0) * 1000
            steady_elapsed = receipt_steady_ns - steady0
            ros_elapsed = receipt_ros_ns - ros0
            if steady_elapsed < 0 or abs(ros_elapsed - steady_elapsed) > self.jump_ns:
                self.anchor = None
                raise ImuValidationError("ROS clock jump; timestamp anchor cleared")
            if steady_elapsed - elapsed > self.delay_ns:
                raise ImuValidationError("transport backlog exceeds limit")
            stamp = ros0 + elapsed
        else:
            stamp = receipt_ros_ns
        if stamp > receipt_ros_ns + self.delay_ns:
            raise ImuValidationError("MCU acquisition time is ahead of receipt")
        if stamp > 0x7fffffff * 1_000_000_000 + 999_999_999:
            raise ImuValidationError("mapped timestamp exceeds ROS signed seconds")
        if reboot:
            self.retired.append(self.boot)
        if self.anchor is None or reboot:
            self.anchor = (receipt_ros_ns, receipt_steady_ns, message.timestamp_us)
            stamp = receipt_ros_ns
        self.boot, self.seq, self.mcu_us, self.mac = (
            message.boot_id, message.sequence, message.timestamp_us, mac)
        return stamp
