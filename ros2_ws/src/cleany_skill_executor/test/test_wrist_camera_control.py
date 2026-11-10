"""Camera ownership and cleanup remain independent of aborted robot motion."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest
from std_srvs.srv import SetBool

from cleany_skill_executor.core.hardware_execution import ExecutionCanceled
from cleany_skill_executor.wrist_camera_control import WristCameraSession


def session(*, wait=None, failures=()):
    calls = []

    def client(_service, name):
        arm = name.split('_')[0].lstrip('/')

        def submit(request):
            calls.append((arm, request.data))
            future = Future()
            future.set_result(SetBool.Response(success=(arm, request.data) not in failures,
                                               message='USB unavailable'))
            return future

        return NS(wait_for_service=lambda **_: True, call_async=submit)

    control = WristCameraSession(NS(create_client=client), .05,
                                 wait or (lambda future, timeout: future.result()))
    return control, calls


def test_idle_and_arm_switch_keep_only_selected_capture_open():
    control, calls = session()
    control.close()
    assert calls == []
    control.activate('left')
    assert calls == [('right', False), ('left', True)]
    control.activate('left')
    assert len(calls) == 2
    control.activate('right')
    assert calls[-2:] == [('left', False), ('right', True)]
    control.close()
    assert calls[-1] == ('right', False) and control.active_arm is None


def test_opposite_camera_must_acknowledge_off_before_selected_camera_opens():
    control, calls = session(failures={('right', False)})
    with pytest.raises(RuntimeError, match='right wrist camera control failed'):
        control.activate('left')
    assert calls == [('right', False)]


def test_aborted_enable_reply_still_closes_usb_without_motion_guard():
    def guarded_wait(future, timeout):
        if len(calls) == 2:
            raise ExecutionCanceled('Motion canceled after submitting camera enable')
        return future.result()

    control, calls = session(wait=guarded_wait)
    with pytest.raises(ExecutionCanceled):
        control.activate('left')
    control.close()
    assert calls == [('right', False), ('left', True), ('left', False)]


def test_failed_enable_and_failed_disable_are_reported_and_cleanup_can_retry():
    failures = {('left', True)}
    control, calls = session(failures=failures)
    with pytest.raises(RuntimeError, match='left wrist camera control failed'):
        control.activate('left')
    failures.add(('left', False))
    with pytest.raises(RuntimeError, match='left wrist camera control failed'):
        control.close()
    failures.clear()
    control.close()
    assert calls[-1] == ('left', False)


def test_disable_wait_is_bounded_even_if_camera_never_replies():
    control, _ = session()
    control.activate('left')
    control.clients['left'].call_async = lambda _: Future()
    with pytest.raises(RuntimeError, match='Timed out disabling left wrist camera'):
        control.close()


def test_cancel_submits_disable_without_waiting_and_stop_reuses_acknowledgement():
    control, calls = session()
    control.activate('right')
    pending = Future()

    def disable(request):
        calls.append(('right', request.data))
        return pending

    control.clients['right'].call_async = disable
    control.request_close()
    control.request_close()
    assert calls.count(('right', False)) == 1 and not pending.done()
    pending.set_result(SetBool.Response(success=True))
    control.close()
    assert control.active_arm is None and calls.count(('right', False)) == 1


def test_old_disable_acknowledgement_cannot_close_a_new_capture_session():
    control, calls = session()
    control.activate('left')
    control.request_close()
    control.activate('right')
    control.close()
    control.activate('left')
    control.close()
    assert calls[-2:] == [('left', True), ('left', False)]


def test_same_arm_reactivation_after_pending_shutdown_opens_a_new_session():
    control, calls = session()
    control.activate('left')
    control.request_close()
    control.activate('left')
    control.close()
    assert calls[-2:] == [('left', True), ('left', False)]
