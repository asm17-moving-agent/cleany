from concurrent.futures import Future
from types import SimpleNamespace as N

import pytest

from cleany_mission_manager.core.sensor_desk import SensorDesk
from cleany_mission_manager.core.result import ResultStatus


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class Transport:
    def ready(self):
        return True

    def send(self, session):
        self.session = session
        self.future = Future()
        return self.future


def detection(**kwargs):
    data = dict(object_id=7, track_id='cup-track', tracking_state='TRACKED',
                sorting_category='trash', sorting_reason='Discarded disposable cup',
                confidence=.9, distance_valid=True, distance_m=.7)
    return N(**dict(data, **kwargs))


def setup():
    transport = Transport()
    desk = SensorDesk(transport, lambda: 100., lambda: 10_000_000_000, destination='trash_right')
    desk.begin()
    identifier = desk.start('mission')
    array = N(snapshot_id='snapshot-real', tracking_epoch='epoch',
              tracking_session_id=desk.session, header=N(stamp=N(sec=10, nanosec=0)),
              detections=[detection()], missing_objects=[])
    return desk, transport, identifier, array


def respond(transport, array, *, success=True, status=4):
    transport.future.set_result(N(accepted=True,
        get_result_async=lambda: completed(N(status=status,
            result=N(success=success, message='observation error', detections=array)))))


def test_real_ids_and_observations_are_not_removed_after_success():
    desk, transport, identifier, array = setup()
    respond(transport, array)
    result = desk.poll(identifier)
    obj = result.data.objects[0]
    assert obj.object_id == 'epoch/cup-track'
    assert obj.wire_object_id == 7
    assert result.data.source == 'isaac'
    desk.manipulation_succeeded(dict(snapshot_id='snapshot-real', object_id=7))
    assert len(result.data.objects) == 1


@pytest.mark.parametrize('override', [dict(sorting_category=''), dict(sorting_category='lost_item'),
    dict(tracking_state='AMBIGUOUS'), dict(distance_valid=False), dict(confidence=float('nan'))])
def test_uncertain_object_requires_review(override):
    desk, transport, identifier, array = setup()
    array.detections = [detection(**override)]
    respond(transport, array)
    assert desk.poll(identifier).data.objects[0].disposition == 'human_review'


@pytest.mark.parametrize('failure', ['failed', 'old', 'identity', 'duplicate'])
def test_bad_observation_never_means_empty_desk(failure):
    desk, transport, identifier, array = setup()
    if failure == 'old':
        array.header.stamp.sec = 1
    if failure == 'identity':
        array.tracking_session_id = 'another-desk'
    if failure == 'duplicate':
        array.detections *= 2
    respond(transport, array, success=failure != 'failed')
    result = desk.poll(identifier)
    assert result.status == ResultStatus.FAILED and result.data is None


def test_cancel_waits_for_late_acceptance_and_terminal_result():
    desk, transport, identifier, array = setup()
    terminal, cancellations = Future(), []
    desk.cancel(identifier)
    assert not desk.stopped()
    transport.future.set_result(N(accepted=True, get_result_async=lambda: terminal,
        cancel_goal_async=lambda: cancellations.append(True)))
    assert desk.poll(identifier) is None
    assert cancellations and not desk.stopped()
    terminal.set_result(N(status=5, result=None))
    assert desk.poll(identifier).status == ResultStatus.CANCELLED
    assert desk.stopped()


def test_unexplained_missing_object_requires_review():
    desk, transport, identifier, array = setup()
    array.detections = []
    array.missing_objects = [N(track_id='cup-track')]
    respond(transport, array)
    assert desk.poll(identifier).data.objects[0].disposition == 'human_review'
