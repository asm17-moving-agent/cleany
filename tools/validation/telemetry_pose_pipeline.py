#!/usr/bin/env python3
"""Validate synthetic ROS -> observer -> authenticated Backend on an isolated domain.

Requires a built ROS overlay and a control-plane Python environment. Does not start
motion nodes, use the operational DB, or publish on the robot's ROS domain.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import tempfile
import time
from pathlib import Path
from urllib.request import Request, urlopen


def backend(args):
    import uvicorn
    from control_plane.server import ControlPlaneApplication, create_app
    from control_plane.settings import Settings

    application = ControlPlaneApplication(start_dispatcher=False, settings=Settings(
        database_path=':memory:', robot_mode='gateway', cookie_secure=False,
        pose_receive_timeout_seconds=0.5,
    ))
    accounts = application.accounts
    customer = accounts.create_customer('Synthetic pose validation')
    site = accounts.create_site(customer, 'Synthetic map', site_id='POSE_FIXTURE')
    _, temporary = accounts.provision(customer, 'pose-fixture', 'Pose fixture')
    session = accounts.login('pose-fixture', temporary, 'fixture')
    session = accounts.change_password(session, 'pose-fixture-password-2026')
    token = accounts.register_robot(customer, site)
    directory = Path(args.credentials)
    for name, value in [('device-token', token), ('session', session)]:
        path = directory / (name + '.new')
        path.write_text(value)
        path.chmod(0o600)
        path.replace(directory / name)
    uvicorn.run(create_app(application), host='127.0.0.1', port=args.port, log_level='warning')


def validate(args):
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Set a nonzero isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1.')
    import rclpy
    from cleany_control_bridge.pose_node import PoseBackendBridge
    from cleany_telemetry.pose_node import PoseObserver
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import TransformBroadcaster

    def params(**values):
        return [Parameter(name, value=value) for name, value in values.items()]

    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    origin = f'http://127.0.0.1:{port}'
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='cleany-pose-') as credentials:
        rclpy.init()
        observer = PoseObserver(parameter_overrides=params(source='tf', input_timeout=0.3, rate=20.0))
        bridge = PoseBackendBridge(parameter_overrides=params(
            url=f'ws://127.0.0.1:{port}/api/robots/cleany-01/pose/ws',
            device_token_file=str(Path(credentials) / 'device-token'), map_id='synthetic-map',
            input_timeout=0.3, reconnect_initial=0.1, reconnect_max=0.3, rate=20.0,
        ))
        source = rclpy.create_node('pose_pipeline_fixture')
        broadcaster = TransformBroadcaster(source)
        executor = SingleThreadedExecutor()
        for node in (source, observer, bridge):
            executor.add_node(node)
        observations = []
        source.create_subscription(PoseStamped, '/telemetry/pose', observations.append, qos_profile_sensor_data)
        transform = TransformStamped(child_frame_id='base_link')
        transform.header.frame_id = 'map'
        transform.transform.translation.x = 1.25
        transform.transform.translation.y = -2.5
        transform.transform.rotation.w = 1.0
        stamps = set()
        child = None
        log = (output / 'backend.log').open('w')

        def pump(seconds, fresh=True):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if fresh:
                    transform.header.stamp = source.get_clock().now().to_msg()
                    stamps.add((transform.header.stamp.sec, transform.header.stamp.nanosec))
                broadcaster.sendTransform(transform)
                executor.spin_once(timeout_sec=.01)

        def start_backend():
            env = dict(os.environ)
            env['PYTHONPATH'] = str(Path(args.backend_root) / 'apps/backend')
            return subprocess.Popen([
                args.backend_python, str(Path(__file__).resolve()), '--backend',
                '--credentials', credentials, '--port', str(port),
            ], env=env, stdout=log, stderr=log)

        def snapshot():
            session = (Path(credentials) / 'session').read_text()
            request = Request(origin + '/api/robots/cleany-01/pose?site_id=POSE_FIXTURE',
                              headers={'Cookie': 'cleany_session=' + session})
            with urlopen(request, timeout=.5) as response:
                return json.load(response)

        def wait_pose(x):
            deadline = time.monotonic() + 12
            last = None
            while time.monotonic() < deadline:
                pump(.05)
                if child.poll() is not None:
                    raise RuntimeError('Fixture Backend exited; see backend.log')
                try:
                    last = snapshot()
                except (OSError, ValueError):
                    continue
                if not last['stale'] and last['pose'] and last['pose']['x'] == x:
                    return last
            raise AssertionError(f'No fresh Backend pose: {last}')

        def stop_backend():
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)

        try:
            pump(1.5)
            assert len(observations) >= 3, 'Observer depends on Backend availability'
            offline_count = len(observations)
            child = start_backend()
            first = wait_pose(1.25)
            assert first['pose']['frame_id'] == 'map'
            assert first['pose']['map_id'] == 'synthetic-map'
            assert first['pose']['y'] == -2.5 and first['pose']['yaw'] == 0
            pump(1.2, fresh=False)
            count = len(observations)
            pump(.4, fresh=False)
            assert len(observations) == count, 'Frozen TF refreshed the observation'
            expired = snapshot()
            assert expired['stale'], 'Frozen TF remained fresh at Backend'
            stop_backend()
            before = len(observations)
            transform.transform.translation.x = 2.75
            pump(.8)
            assert len(observations) > before, 'Backend shutdown stopped observations'
            child = start_backend()  # New memory DB and new credentials, same token file.
            recovered = wait_pose(2.75)
            assert recovered['pose']['map_id'] == 'synthetic-map'
            assert all((p.header.stamp.sec, p.header.stamp.nanosec) in stamps for p in observations)
            report = {
                'evidence': 'synthetic ROS TF and isolated real Backend; no physical pose proof',
                'ros_domain': os.environ['ROS_DOMAIN_ID'], 'observations_without_backend': offline_count,
                'observations_total': len(observations), 'source_stamps_preserved': True,
                'first_pose': first, 'frozen_source': expired, 'recovered_pose': recovered,
                'credential_rotation_on_reconnect': True,
            }
            (output / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
        finally:
            bridge.destroy_node()
            observer.destroy_node()
            source.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            if child is not None and child.poll() is None:
                stop_backend()
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--credentials', help=argparse.SUPPRESS)
    parser.add_argument('--port', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--backend-root')
    parser.add_argument('--backend-python')
    parser.add_argument('--output')
    args = parser.parse_args()
    if args.backend:
        backend(args)
    else:
        if not all((args.backend_root, args.backend_python, args.output)):
            parser.error('--backend-root, --backend-python and --output are required')
        validate(args)


if __name__ == '__main__':
    main()
