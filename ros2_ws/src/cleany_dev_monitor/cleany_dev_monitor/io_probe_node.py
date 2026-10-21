"""Passive manipulation IO recorder. No publishers, clients, Goals or device access."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String
from cleany_interfaces.msg import ManipulationExecutionRecord

from .io_probe import EvidenceWriter, ProbeState, header_stamp_ns, load_config
from .request import read_request


def udp_counters() -> dict:
    lines = Path('/proc/net/snmp').read_text().splitlines()
    for keys, values in zip(lines[::2], lines[1::2]):
        if keys.startswith('Udp:'):
            return dict(zip(keys.split()[1:], map(int, values.split()[1:])))
    return {}


def json_file(path: Path, data: dict) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


class IOProbe(Node):
    def __init__(self, config: dict, output: Path, *, request_file: str = '', **kwargs):
        super().__init__('manipulation_io_probe', enable_rosout=False,
                         start_parameter_services=False, **kwargs)
        self.config, self.output = config, output
        self.started, self.wall_ns = time.monotonic(), time.time_ns()
        self.state = ProbeState(config)
        self.writer = EvidenceWriter(output / 'events.jsonl', config['writer_queue_size'],
                                     int(config['max_recording_mib']*2**20))
        self.subscriptions_owned = []
        self.last_motor_detail = -math.inf
        self.feedback_stamps = None
        self.previous_tick = 0.
        self.request_file = request_file
        self.stop_resources = threading.Event()
        self.telemetry_process = None
        self.telemetry_thread = None
        self.resource_thread = None
        self.last_execution_key = None
        for spec in config['streams']:
            message_type = get_message(spec['type'])
            self.subscriptions_owned.append(self.create_subscription(
                message_type, spec['topic'],
                lambda raw, s=spec, cls=message_type: self.receive(s, cls, raw),
                qos_profile_sensor_data, raw=True))
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_owned.append(self.create_subscription(
            ManipulationExecutionRecord, config['execution_topic'], self.execution, latched))
        self.subscriptions_owned.append(self.create_subscription(
            String, config['bt_topic'], lambda msg: self.writer.put(
                dict(kind='bt', t=self.elapsed(), data=msg.data)), latched))
        for arm in ('left', 'right'):
            self.subscriptions_owned.append(self.create_subscription(
                Bool, f'/{arm}_wrist_camera/enabled',
                lambda msg, a=arm: self.wrist(a, msg.data), latched))
        self.window_timer = self.create_timer(config['sample_interval_s'], self.tick)
        self.graph_timer = self.create_timer(5., self.graph)
        self.graph()

    def elapsed(self) -> float:
        return time.monotonic() - self.started

    def receive(self, spec: dict, message_type, payload: bytes) -> None:
        elapsed, ros_now = self.elapsed(), self.get_clock().now().nanoseconds
        row = self.state.receive(spec['topic'], elapsed, len(payload), header_stamp_ns(payload), ros_now)
        try:
            if spec['kind'] == 'joint':
                msg = deserialize_message(payload, message_type)
                row['joints'] = list(msg.name)
                row['positions_valid'] = len(msg.position) == len(msg.name) and all(math.isfinite(v) for v in msg.position)
            elif spec['kind'] == 'motor':
                msg = deserialize_message(payload, message_type)
                motors = {name: {k: float(v) if math.isfinite(v) else None
                                for k, v in zip(fields.interface_names, fields.values)}
                          for name, fields in zip(msg.joint_names, msg.interface_values)}
                stamps = tuple((name, fields.get('feedback_stamp_ns')) for name, fields in sorted(motors.items()))
                ages = [(ros_now - fields['feedback_stamp_ns'])/1e6 for fields in motors.values()
                        if fields.get('feedback_stamp_ns') is not None and fields['feedback_stamp_ns'] > 0]
                row.update(feedback_max_age_ms=max(ages) if ages else None,
                           feedback_stamp_count=len(ages), feedback_changed=stamps != self.feedback_stamps,
                           protocol_error_max=max((m.get('protocol_error_code') or 0 for m in motors.values()), default=0),
                           motor_status_max=max((m.get('motor_status') or 0 for m in motors.values()), default=0),
                           recovery_count_max=max((m.get('communication_recovery_count') or 0 for m in motors.values()), default=0))
                self.feedback_stamps = stamps
                if elapsed - self.last_motor_detail >= self.config['motor_detail_interval_s']:
                    self.writer.put(dict(kind='motors', t=elapsed, motors=motors))
                    self.last_motor_detail = elapsed
        except Exception as error:
            row['decode_error'] = str(error)
        self.writer.put(row)

    def wrist(self, arm: str, enabled: bool) -> None:
        elapsed = self.elapsed()
        self.state.enabled(arm, enabled, elapsed)
        self.writer.put(dict(kind='wrist', t=elapsed, arm=arm, enabled=enabled))

    def execution(self, msg) -> None:
        elapsed = self.elapsed()
        record = dict(message_to_ordereddict(msg))
        self.writer.put(self.state.observe_execution(record, elapsed, self.wall_ns))

    def tick(self) -> None:
        elapsed = self.elapsed()
        lag = max(0., (elapsed-self.previous_tick-self.config['sample_interval_s'])*1000)
        self.previous_tick = elapsed
        self.writer.put(self.state.sample(elapsed, lag))

    def graph(self) -> None:
        publishers = {}
        for spec in self.config['streams']:
            publishers[spec['topic']] = [dict(node=p.node_namespace.rstrip('/')+'/'+p.node_name,
                                              qos=str(p.qos_profile))
                                         for p in self.get_publishers_info_by_topic(spec['topic'])]
        self.writer.put(dict(kind='graph', t=self.elapsed(), publishers=publishers))

    def start_resources(self, tegrastats: bool) -> None:
        if tegrastats and shutil.which('tegrastats'):
            self.telemetry_process = subprocess.Popen(
                ['tegrastats', '--interval', str(int(self.config['resource_interval_s']*1000))],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            def gpu_reader():
                for line in self.telemetry_process.stdout:
                    match = re.search(r'GR3D_FREQ\s+(\d+)%', line)
                    self.writer.put(dict(kind='tegrastats', t=self.elapsed(), raw=line.strip(),
                                         gpu_percent=int(match[1]) if match else None))
            self.telemetry_thread = threading.Thread(target=gpu_reader, daemon=True)
            self.telemetry_thread.start()
        def sample_resources():
            process = psutil.Process()
            psutil.cpu_percent(); process.cpu_percent()
            previous_request = None
            while not self.stop_resources.wait(self.config['resource_interval_s']):
                try:
                    mem = psutil.virtual_memory()
                    row = dict(kind='resources', t=self.elapsed(), wall_ns=time.time_ns(),
                               cpu_percent=psutil.cpu_percent(), memory_used_bytes=mem.total-mem.available,
                               memory_available_bytes=mem.available, swap_used_bytes=psutil.swap_memory().used,
                               probe_cpu_percent_one_core=process.cpu_percent(), probe_rss_bytes=process.memory_info().rss,
                               udp=udp_counters(), writer_dropped=self.writer.dropped,
                               writer_queue=self.writer.queue.qsize())
                    self.writer.put(row)
                    json_file(self.output / 'latest.json', row)
                    if self.request_file:
                        request = read_request(self.request_file)
                        if request['updated_at_ns'] >= self.wall_ns or request['owner_alive']:
                            self.state.command = request
                            self.state.command_seen = self.elapsed()
                        identity = (request['execution_id'], request['updated_at_ns'], request['owner_alive'])
                        if identity != previous_request:
                            self.writer.put(dict(kind='command', t=self.elapsed(), record=request,
                                                 historical=request['updated_at_ns'] < self.wall_ns))
                            previous_request = identity
                except (OSError, ValueError, KeyError) as error:
                    self.writer.put(dict(kind='resource_note', t=self.elapsed(), message=str(error)))
        self.resource_thread = threading.Thread(target=sample_resources, daemon=True)
        self.resource_thread.start()

    def finish(self) -> dict:
        self.window_timer.cancel(); self.graph_timer.cancel()
        self.stop_resources.set()
        if self.resource_thread:
            self.resource_thread.join(timeout=3)
        if self.telemetry_process:
            self.telemetry_process.terminate()  # Only this recorder's child; never tegrastats --stop.
            try:
                self.telemetry_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.telemetry_process.kill(); self.telemetry_process.wait(timeout=3)
            self.telemetry_thread.join(timeout=3)
        self.tick()
        writer = self.writer.close()
        return dict(duration_s=self.elapsed(), writer=writer,
                    counts={t: s.count for t, s in self.state.streams.items()},
                    last_execution=self.state.execution)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--config', type=Path)
    result.add_argument('--output', required=True, type=Path)
    result.add_argument('--seconds', type=float, default=900.)
    result.add_argument('--label', default='supervised-manipulation')
    result.add_argument('--request-file', default=os.environ.get('CLEANY_MANIPULATION_PROGRESS_FILE',
                        '~/.local/state/cleany/manipulation/request.json'))
    result.add_argument('--repository-root', type=Path)
    result.add_argument('--no-tegrastats', action='store_true')
    result.add_argument('--synthetic', action='store_true', help='Mark test publishers; never evidence of real motion')
    return result


def main(args=None) -> None:
    options = parser().parse_args(args)
    if not math.isfinite(options.seconds) or options.seconds <= 0:
        raise SystemExit('--seconds must be finite and positive')
    if options.config is None:
        from ament_index_python.packages import get_package_share_directory
        options.config = Path(get_package_share_directory('cleany_dev_monitor'))/'config/manipulation_io_probe.json'
    config = load_config(options.config)
    options.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema=1, label=options.label, synthetic=options.synthetic,
                    start=datetime.now().astimezone().isoformat(), requested_seconds=options.seconds,
                    config=config, config_sha256=hashlib.sha256(options.config.read_bytes()).hexdigest(),
                    env={k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')},
                    rmem_max=int(Path('/proc/sys/net/core/rmem_max').read_text()), udp_before=udp_counters(),
                    observer='Additional Best Effort raw CDR subscriber; no image decoding, encoding or payload storage',
                    commands_sent=False, finished=False)
    if options.repository_root:
        captured = {}
        for relative in ('configs/robot/collection_hardware.json', 'configs/robot/hardware_motion_monitor.yaml',
                         'configs/robot/feetech_control.yaml', 'configs/robot/left_wrist_rgb.json',
                         'configs/robot/right_wrist_rgb.json'):
            path = options.repository_root/relative
            if path.is_file():
                content = path.read_text()
                captured[relative] = dict(sha256=hashlib.sha256(content.encode()).hexdigest(), text=content)
        manifest['runtime_configs'] = captured
    json_file(options.output/'manifest.json', manifest)
    rclpy.init(args=[])
    node = None
    try:
        node = IOProbe(config, options.output, request_file=options.request_file)
        manifest['observation_start_wall_ns'] = node.wall_ns
        node.start_resources(not options.no_tegrastats)
        print(f'Recording only: {options.output}; no robot commands; {options.seconds:g}s', flush=True)
        while rclpy.ok() and node.elapsed() < options.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.writer.error:
                raise OSError(node.writer.error)
        manifest['finished'] = True
    except KeyboardInterrupt:
        manifest['finished'] = True
        manifest['stopped_by_operator'] = True
    except Exception as error:
        manifest['error'] = str(error)
        raise
    finally:
        if node:
            manifest.update(node.finish())
            node.destroy_node()
        manifest['udp_after'] = udp_counters()
        manifest['end'] = datetime.now().astimezone().isoformat()
        json_file(options.output/'manifest.json', manifest)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
