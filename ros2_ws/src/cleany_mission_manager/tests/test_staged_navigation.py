from cleany_mission_manager.core.staged_navigation import StagedNavigator
from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import ModuleResult, FailureCode


def test_desk_work_waits_for_approach_but_home_skips_it():
    nav = DeferredPort(lambda _: ModuleResult.success(), polls=0)
    approach = DeferredPort(lambda _: ModuleResult.success(), polls=1)
    port = StagedNavigator(nav, approach)
    op = port.start('seat-12')
    assert port.poll(op) is None
    assert not port.stopped()
    assert port.poll(op).ok
    assert port.stopped() and approach.commands == ['seat-12']
    op = port.start('home')
    assert port.poll(op).ok
    assert approach.commands == ['seat-12']


def test_failure_cancel_and_late_result_do_not_start_approach():
    nav = DeferredPort(lambda _: ModuleResult.failed(FailureCode.NAVIGATION_FAIL), polls=0)
    approach = DeferredPort(lambda _: ModuleResult.success(), polls=0)
    port = StagedNavigator(nav, approach)
    op = port.start('seat-12')
    assert not port.poll(op).ok and not approach.commands
    nav.handler = lambda _: ModuleResult.success()
    op = port.start('seat-12')
    port.cancel(op)
    assert not port.poll(op).ok and not approach.commands
    assert port.poll('old') is None


def test_cancel_after_staging_cancels_approach():
    nav = DeferredPort(lambda _: ModuleResult.success(), polls=0)
    approach = DeferredPort(lambda _: ModuleResult.success(), polls=10)
    port = StagedNavigator(nav, approach)
    op = port.start('seat-12')
    assert port.poll(op) is None
    port.cancel(op)
    assert not port.poll(op).ok and port.stopped()
