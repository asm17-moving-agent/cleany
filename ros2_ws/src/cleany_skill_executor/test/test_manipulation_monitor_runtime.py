"""실제 ROS Action과 유지되는 이벤트를 ZMQ 뷰어까지 전달하는지 검증합니다."""

import struct
import threading
import time
import xml.etree.ElementTree as ET

import pytest
from rclpy.parameter import Parameter
import zmq

from cleany_skill_executor.manipulation_monitor_node import ManipulationMonitorNode
from test_manipulation_runtime import RosHarness, eventually, goal, response


@pytest.mark.parametrize('scenario, expected', [('success', 'SUCCESS'), ('grasp_failure', 'FAILED')])
@pytest.mark.parametrize('viewer_delay', [0.0, 0.5])
def test_ros_execution_reaches_zmq_viewer(tmp_path, monkeypatch, scenario, expected, viewer_delay):
    # 노드를 만들기 전에 OS가 빈 loopback 포트를 선택하도록 합니다.
    import socket
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    harness = RosHarness(tmp_path, scenario=scenario, slow=True)
    release_approach = threading.Event()
    core = harness.server.core
    original_poll = core.port.poll

    def poll_after_viewer(now):
        # ROS 이벤트 확인과 ZMQ 조회 사이에도 실행 중 상태를 관측할 수 있어야 합니다.
        # 실제 전송 경로는 유지하고 mock 완료 시점만 뷰어의 확인과 동기화합니다.
        if core.record and core.record.substage == 'ApproachObject' and not release_approach.is_set():
            return None
        return original_poll(now)

    monkeypatch.setattr(core.port, 'poll', poll_after_viewer)
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
        # 느린 조회는 기존 mock 단계 시간(0.25초)을 넘겨도 같은 실행 상태를 검사합니다.
        time.sleep(viewer_delay)
        client.send(struct.pack('<BBI', 2, ord('S'), 2))
        _, payload = client.recv_multipart()
        assert dict(struct.iter_unpack('<HB', payload))[nodes['ApproachObject']] == 1
        release_approach.set()
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
        release_approach.set()
        client.close()
        context.term()
        harness.executor.remove_node(monitor)
        monitor.destroy_node()
        harness.close()
