"""Mission Runtime under System Supervisor authority, with a scripted lease port."""
import pytest

from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import ModuleResult
from cleany_mission_manager.core.runtime import MissionRuntime
from cleany_mission_manager.core.runtime_models import RuntimePolicy, RuntimeState
from cleany_mission_manager.core.supervision import Authorization
from cleany_mission_manager.mocks.desk import MockDesk
from test_runtime import Clock, drive, request


class Lease:
    """Scripted Supervisor view: availability, lease decisions and call record."""

    def __init__(self):
        self.system = (True, "")
        self.status = (Authorization.GRANTED, "")
        self.calls = []

    def available(self):
        return self.system

    def acquire(self, mission_id):
        self.calls.append(("acquire", mission_id))

    def authorization(self):
        return self.status

    def release(self):
        self.calls.append(("release", None))

    def abandon(self):
        self.calls.append(("abandon", None))

    def maintain(self):
        pass


def make(lease=None, nav_polls=3, policy=None):
    clock, lease = Clock(), lease or Lease()
    desk = MockDesk(clock)
    nav = DeferredPort(lambda _: ModuleResult.success(), polls=nav_polls)
    runtime = MissionRuntime(navigator=nav, perception=desk.perception, planner=desk.planner,
                             executor=desk.executor, clock=clock, supported_targets={"seat-12"},
                             policy=policy, on_accept=desk.begin, supervision=lease)
    return runtime, lease, nav, clock


def test_unavailable_system_rejects_admission_with_supervisor_reason():
    runtime, lease, nav, _ = make()
    lease.system = (False, "SYSTEM_FAULT_LOCKED")
    assert runtime.offer(request()).reason == "SYSTEM_FAULT_LOCKED"
    assert not lease.calls and not nav.commands


def test_granted_mission_completes_and_returns_the_lease_after_confirmed_stop():
    runtime, lease, nav, clock = make()
    assert runtime.offer(request()).accepted
    drive(runtime, clock)
    assert runtime.last_report.outcome == "SUCCESS"
    assert runtime.snapshot()["supervision"] == "required"
    assert lease.calls == [("acquire", "one"), ("release", None)]


def test_no_motion_before_grant_and_denial_ends_without_runtime_error():
    runtime, lease, nav, clock = make()
    lease.status = (Authorization.PENDING, "waiting")
    runtime.offer(request())
    for _ in range(5):
        runtime.tick()
        clock.now += .1
    assert not nav.commands and runtime.state == RuntimeState.NAVIGATE_TO_TARGET
    lease.status = (Authorization.DENIED, "required capability unavailable or unknown")
    drive(runtime, clock)
    report = runtime.last_report
    assert report.outcome == "BLOCKED" and report.failure_code == "PERMISSION_REVOKED"
    assert runtime.state == RuntimeState.IDLE  # A system fault is not a Runtime execution error.
    assert not nav.commands


def test_pending_lease_times_out_before_first_motion():
    runtime, lease, nav, clock = make(policy=RuntimePolicy(permission_timeout=.5))
    lease.status = (Authorization.PENDING, "waiting")
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.failure_code == "PERMISSION_REVOKED" and not nav.commands


def test_revocation_during_navigation_cancels_and_confirms_stop_before_finishing():
    runtime, lease, nav, clock = make(nav_polls=50)
    runtime.offer(request())
    for _ in range(3):
        runtime.tick()
        clock.now += .1
    assert nav.commands == ["seat-12"] and nav.pending
    lease.status = (Authorization.DENIED, "operation locked")
    runtime.tick()
    assert runtime.state == RuntimeState.CANCELLING and not nav.pending
    drive(runtime, clock)
    assert runtime.last_report.outcome == "BLOCKED"
    assert nav.commands == ["seat-12"]  # No automatic return home on lost authority.
    assert lease.calls[-1] == ("release", None)


def test_revocation_during_desk_work_requires_review():
    runtime, lease, _, clock = make()
    runtime.offer(request())
    drive(runtime, clock, until=RuntimeState.WORKING)
    lease.status = (Authorization.DENIED, "lease expired or Supervisor unavailable")
    drive(runtime, clock)
    assert runtime.last_report.needs_human_review


def test_supervisor_stop_request_and_confirmation_contract():
    runtime, lease, nav, clock = make(nav_polls=50)
    runtime.offer(request())
    runtime.tick()
    assert not runtime.supervisor_stop("other", "x").accepted
    assert runtime.supervisor_stop("one", "required capability lost").accepted
    assert not runtime.supervisor_stop_confirmed("one")
    drive(runtime, clock)
    assert runtime.supervisor_stop_confirmed("one")
    assert runtime.supervisor_stop("restored-lease-from-old-epoch", "").accepted


def test_unconfirmed_stop_abandons_the_lease_instead_of_releasing():
    class Stuck(DeferredPort):
        def cancel(self, operation_id):
            pass  # The operation never confirms its stop.
    clock, lease = Clock(), Lease()
    desk = MockDesk(clock)
    nav = Stuck(lambda _: ModuleResult.success(), polls=10_000)
    runtime = MissionRuntime(navigator=nav, perception=desk.perception, planner=desk.planner,
                             executor=desk.executor, clock=clock, supported_targets={"seat-12"},
                             on_accept=desk.begin, supervision=lease)
    runtime.offer(request())
    runtime.tick()
    lease.status = (Authorization.DENIED, "operation locked")
    drive(runtime, clock)
    assert runtime.state == RuntimeState.ERROR  # The Runtime's own stop-timeout rule.
    assert lease.calls[-1] == ("abandon", None)
    assert not runtime.supervisor_stop_confirmed("one")


def test_unsupervised_runtime_is_visible_in_the_snapshot():
    clock = Clock()
    desk = MockDesk(clock)
    runtime = MissionRuntime(navigator=DeferredPort(lambda _: ModuleResult.success()), perception=desk.perception,
                             planner=desk.planner, executor=desk.executor, clock=clock,
                             supported_targets={"seat-12"}, on_accept=desk.begin)
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.snapshot()["supervision"] == "disabled"


@pytest.mark.parametrize("timeout", [0., float("nan")])
def test_permission_timeout_must_be_bounded(timeout):
    with pytest.raises(ValueError):
        RuntimePolicy(permission_timeout=timeout)
