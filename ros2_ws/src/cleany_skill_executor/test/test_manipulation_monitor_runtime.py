"""A real ROS Action, durable event subscription and a ZMQ viewer client."""

import struct
import xml.etree.ElementTree as ET

import pytest
from rclpy.parameter import Parameter
import zmq

from cleany_skill_executor.manipulation_monitor_node import ManipulationMonitorNode
from test_manipulation_runtime import RosHarness, eventually, goal, response


@pytest.mark.parametrize('scenario, expected', [('success', 'SUCCESS'), ('grasp_failure', 'FAILED')])
def test_ros_execution_reaches_zmq_viewer(tmp_path, scenario, expected):
    # Let the OS allocate an available loopback port before constructing the node.
    import socket
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    harness = RosHarness(tmp_path, scenario=scenario, slow=True)
    monitor = ManipulationMonitorNode(context=harness.context, namespace=harness.namespace,
                                     parameter_overrides=[Parameter('monitor_port', value=port)])
    harness.executor.add_node(monitor)
    context = zmq.Context()
    client = context.socket(zmq.REQ)
    client.setsockopt(zmq.LINGER, 0)
    client.setsockopt(zmq.RCVTIMEO, 3000)
    client.connect(f'tcp://127.0.0.1:{port}')
    try:
        client.send(struct.pack('<BBI', 2, ord('T'), 1))
        header, xml = client.recv_multipart()
        assert len(header) == 22
        nodes = {node.tag: int(node.get('_uid')) for node in
                 list(ET.fromstring(xml).find('BehaviorTree').iter())[1:]}
        handle = harness.send(goal())
        eventually(lambda: monitor.projection.latest is not None and
                   monitor.projection.latest.substage == 'ApproachObject')
        client.send(struct.pack('<BBI', 2, ord('S'), 2))
        _, payload = client.recv_multipart()
        assert dict(struct.iter_unpack('<HB', payload))[nodes['ApproachObject']] == 1
        assert response(handle.get_result_async()).result.status == expected
        eventually(lambda: monitor.projection.latest.has_result)
        client.send(struct.pack('<BBI', 2, ord('S'), 3))
        _, payload = client.recv_multipart()
        statuses = dict(struct.iter_unpack('<HB', payload))
        if expected == 'SUCCESS':
            assert statuses[nodes['FinalizeSuccess']] == 2
        else:
            assert statuses[nodes['GraspObject']] == 2
            assert statuses[nodes['ConfirmGrasp']] == 3
            assert statuses[nodes['LiftObject']] == 0
            assert statuses[nodes['FinalizeFailure']] == 2
    finally:
        client.close()
        context.term()
        harness.executor.remove_node(monitor)
        monitor.destroy_node()
        harness.close()
