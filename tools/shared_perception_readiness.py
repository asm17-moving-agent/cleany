#!/usr/bin/env python3
"""Check a shared perception provider before starting physical control."""
from __future__ import annotations

import argparse
import json
import math
import time

TOPICS = ('/camera/color/image_raw', '/camera/depth/image_raw')
ENDPOINTS = ('/perception/inspect_scene', '/perception/observe_object_reference',
             '/perception/observe_wrist_target')
PROVIDER_PARAMETERS = {'target_frame': 'base_link', 'enable_reference_observation': True,
                       'enable_wrist_observation': True}


def provider_issues(values: dict[str, object] | None) -> list[str]:
    """Require an actual RPC reply, including the frame used by grasp selection."""
    if values is None:
        return ['/perception_inspector/get_parameters (RPC)']
    return [f'perception_inspector.{name} (expected {expected!r})'
            for name, expected in PROVIDER_PARAMETERS.items() if values.get(name) != expected]


def pending_inputs(now: float, stamps: dict[str, float],
                   endpoints: dict[str, bool], maximum_age: float = 1.) -> list[str]:
    missing = [name for name in ENDPOINTS if not endpoints.get(name, False)]
    missing.extend(name for name in TOPICS
                   if name not in stamps or not math.isfinite(stamps[name])
                   or not 0 <= now-stamps[name] <= maximum_age)
    return missing


def main() -> int:
    import rclpy
    from cleany_interfaces.action import InspectScene
    from cleany_interfaces.srv import ObserveObjectReference, ObserveWristTarget
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=10.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('shared_perception_readiness')
    action = ActionClient(node, InspectScene, ENDPOINTS[0])
    reference = node.create_client(ObserveObjectReference, ENDPOINTS[1])
    wrist = node.create_client(ObserveWristTarget, ENDPOINTS[2])
    settings = node.create_client(GetParameters, '/perception_inspector/get_parameters')
    settings_future = None
    values = None
    stamps = {}

    def received(name, message):
        stamps[name] = message.header.stamp.sec + message.header.stamp.nanosec/1e9

    subscriptions = [node.create_subscription(Image, name,
        lambda message, topic=name: received(topic, message), qos_profile_sensor_data)
        for name in TOPICS]
    deadline = time.monotonic()+args.timeout
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=.05)
            if settings_future is None and settings.service_is_ready():
                settings_future = settings.call_async(
                    GetParameters.Request(names=list(PROVIDER_PARAMETERS)))
            if settings_future is not None and settings_future.done() and values is None:
                response = settings_future.result()
                values = {name: (value.string_value if value.type == 4 else value.bool_value)
                          for name, value in zip(PROVIDER_PARAMETERS, response.values)}
            available = {ENDPOINTS[0]: action.server_is_ready(),
                         ENDPOINTS[1]: reference.service_is_ready(),
                         ENDPOINTS[2]: wrist.service_is_ready()}
            pending = pending_inputs(node.get_clock().now().nanoseconds/1e9, stamps, available)
            pending.extend(provider_issues(values))
            if not pending or time.monotonic() >= deadline:
                print(json.dumps(dict(event='shared_perception_readiness',
                    ready=not pending, missing=pending, motor_commands_sent=0)), flush=True)
                return int(bool(pending))
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
