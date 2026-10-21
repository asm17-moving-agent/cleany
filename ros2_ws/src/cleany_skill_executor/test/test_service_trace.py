from concurrent.futures import Future
import json

import pytest
from moveit_msgs.srv import GetPositionIK
from rclpy.serialization import deserialize_message

from cleany_skill_executor.service_trace import ServiceTrace


def test_trace_saves_request_and_result_without_replacing_future(tmp_path):
    trace = ServiceTrace(str(tmp_path), lambda message: pytest.fail(message))
    request, response = GetPositionIK.Request(), GetPositionIK.Response()
    request.ik_request.group_name = 'left_grasp_arm'
    response.error_code.val = -31
    future = Future()
    trace('/compute_ik', request, future)
    future.set_result(response)
    assert future.result() is response
    assert deserialize_message((trace.directory / '000001.request.cdr').read_bytes(),
                               GetPositionIK.Request) == request
    assert deserialize_message((trace.directory / '000001.response.cdr').read_bytes(),
                               GetPositionIK.Response) == response
    metadata = json.loads((trace.directory / '000001.json').read_text())
    assert metadata['service'] == '/compute_ik' and not metadata['cancelled']
    assert metadata['elapsed_sec'] >= 0


def test_trace_keeps_timed_out_request_and_no_response(tmp_path):
    trace = ServiceTrace(str(tmp_path), lambda message: pytest.fail(message))
    future = Future()
    trace('/compute_ik', GetPositionIK.Request(), future)
    future.cancel()
    assert json.loads((trace.directory / '000001.json').read_text())['cancelled']
    assert not (trace.directory / '000001.response.cdr').exists()


def test_trace_write_error_does_not_modify_service_future(tmp_path):
    warnings = []
    trace = ServiceTrace(str(tmp_path), warnings.append)
    future = Future()
    trace('/compute_ik', object(), future)
    assert warnings and not future.done()


def test_trace_rejects_relative_paths():
    with pytest.raises(ValueError, match='absolute'):
        ServiceTrace('relative', lambda _: None)
