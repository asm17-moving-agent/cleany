"""The real operator adapter can coexist with another command adapter."""
import json
import os
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_RUN_ROS_TESTS') != '1', reason='ROS opt-in')


def test_concurrent_operator_adapters_do_not_inhibit_collection_readiness():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    from cleany_dev_monitor.operator_ros import OperatorROS
    from cleany_skill_executor.runtime_readiness import graph_pending
    rclpy.init()
    server = rclpy.create_node('operator_state_test_server')
    def state(_request, response):
        response.success = True
        response.message = json.dumps(dict(schema=1, active_execution_id=''))
        return response
    server.create_service(Trigger, '/manipulation/control_state', state)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    adapters = []
    try:
        adapters = [OperatorROS(), OperatorROS()]
        assert adapters[0].node.get_name() != adapters[1].node.get_name()
        deadline = time.monotonic() + 5
        expected = {p.node.get_name() for p in adapters}
        while time.monotonic() < deadline:
            names = adapters[0].node.get_node_names()
            if expected <= set(names):
                break
            time.sleep(.05)
        assert expected <= set(names)
        # Keep the production duplicate-node protection enabled.
        graph_pending(['/' + name for name in names], [], {}, gui=False)
        assert all(p.snapshot()['schema'] == 1 for p in adapters)
    finally:
        for adapter in adapters:
            adapter.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        server.destroy_node()
        rclpy.try_shutdown()
