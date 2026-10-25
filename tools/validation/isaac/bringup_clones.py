"""Start existing lifecycle managers after discovery, before offering Missions."""
from __future__ import annotations

import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from cleany_interfaces.msg import MissionStatus
from nav2_msgs.srv import ManageLifecycleNodes


def main() -> None:
    output = Path('/output')
    count = json.loads((output/'config/clones.json').read_text())['num_envs']
    rclpy.init(args=[])
    node = rclpy.create_node('cleany_clone_bringup')
    ready = {}
    clients = []
    record = {'passed': False, 'policy': 'One STARTUP per manager after discovery; no reset/retry or Mission offer',
              'started_wall': time.time(), 'managers': []}

    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise TimeoutError(f'Lifecycle bringup checkpoint timed out: {ready}')

    try:
        for i in range(count):
            node.create_subscription(MissionStatus, f'/env_{i}/mission/status',
                lambda msg, index=i: ready.update({index: {'ready': msg.ready, 'reason': msg.reason}}),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            for suffix in ('localization', 'navigation', 'safety'):
                name = f'/env_{i}/lifecycle_manager_{suffix}/manage_nodes'
                clients.append((name, node.create_client(ManageLifecycleNodes, name)))
        wait(lambda: all(client.service_is_ready() for _, client in clients), 120.)
        # Service availability can precede discovery of the response reader in
        # Humble FastDDS. Managers already own their child lifecycle clients.
        settled = time.monotonic()+2.
        wait(lambda: time.monotonic() >= settled, 5.)
        for name, client in clients:
            started = time.time()
            future = client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
            wait(future.done, 90.)
            result = future.result()
            record['managers'].append({'service': name, 'startup_calls': 1,
                                       'success': result.success, 'wall_s': time.time()-started})
            assert result.success, name
        wait(lambda: len(ready) == count and all(s['ready'] for s in ready.values()), 120.)
        record.update(passed=True, ready=ready, finished_wall=time.time())
    except BaseException as error:
        record.update(error=repr(error), ready=ready)
        raise
    finally:
        (output/'evaluations/bringup.json').write_text(json.dumps(record, indent=2)+'\n')
        for _, client in clients:
            node.destroy_client(client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
