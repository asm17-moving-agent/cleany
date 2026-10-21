from dataclasses import replace

import pytest

from cleany_mission_manager.core.journal import MissionJournal
from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import FailureCode, ModuleResult
from cleany_mission_manager.core.runtime import MissionRuntime
from cleany_mission_manager.core.runtime_models import (
    RuntimePolicy, RuntimeRequest, RuntimeState, SceneObject, SceneSnapshot, TaskProposal,
)
from cleany_mission_manager.mocks.desk import MockDesk


class Clock:
    now = 10.0

    def __call__(self):
        return self.now


def make_runtime(*, clock=None, desk=None, navigator=None, journal=None, policy=None):
    clock = clock or Clock()
    desk = desk or MockDesk(clock)
    navigator = navigator or DeferredPort(lambda _: ModuleResult.success(), polls=3)
    runtime = MissionRuntime(
        navigator=navigator, perception=desk.perception, planner=desk.planner,
        executor=desk.executor, clock=clock, supported_targets={"seat-12", "seat-13"},
        journal=journal, policy=policy, on_accept=desk.begin,
    )
    return runtime, desk, navigator, clock


def request(mission_id="one", target="seat-12"):
    return RuntimeRequest(mission_id, target, "operator")


def drive(runtime, clock, until=None):
    for _ in range(1000):
        runtime.tick()
        clock.now += .1
        if until and runtime.state == until:
            return
        if not until and runtime.state in (RuntimeState.IDLE, RuntimeState.ERROR):
            return
    pytest.fail("runtime did not reach a bounded checkpoint")


def test_actual_checkpoint_loop_and_return_home():
    runtime, desk, nav, clock = make_runtime()
    assert runtime.offer(request()).accepted
    drive(runtime, clock)
    report = runtime.last_report
    assert report.outcome == "SUCCESS"
    assert report.completed_tasks == ["trash-1", "trash-2"]
    assert report.before_observation != report.after_observation
    assert len(desk.perception.commands) == 4  # before, each action, final done verification
    assert nav.commands == ["seat-12", "home"]
    assert report.cleaning_mode == "mock"
    assert report.return_result == "OK"


def test_repeated_ticks_do_not_start_a_running_operation_twice():
    nav = DeferredPort(lambda _: ModuleResult.success(), polls=20)
    runtime, _, _, clock = make_runtime(navigator=nav)
    runtime.offer(request())
    for _ in range(15):
        runtime.tick()
    assert nav.commands == ["seat-12"]
    assert runtime.state == RuntimeState.NAVIGATE_TO_TARGET
    drive(runtime, clock)


def test_admission_busy_unknown_target_and_durable_duplicate(tmp_path):
    journal = MissionJournal(str(tmp_path / "journal.db"))
    runtime, _, nav, clock = make_runtime(journal=journal)
    assert runtime.offer(request(target="unknown")).reason == "UNKNOWN_TARGET"
    assert runtime.offer(request()).accepted
    assert runtime.offer(request()).duplicate
    assert runtime.offer(request("two")).reason == "BUSY"
    assert runtime.offer(request(target="seat-13")).reason == "MISSION_ID_CONFLICT"
    drive(runtime, clock)
    assert runtime.offer(request()).report.outcome == "SUCCESS"
    assert nav.commands == ["seat-12", "home"]
    restarted, _, restarted_nav, _ = make_runtime(journal=journal)
    assert restarted.offer(request()).report.outcome == "SUCCESS"
    assert restarted_nav.commands == []
    with pytest.raises(RuntimeError, match="immutable"):
        journal.finish(runtime.last_report)


@pytest.mark.parametrize("objects,outcome", [
    ((), "SUCCESS"),
    ((SceneObject("uncertain", "human_review"),), "HUMAN_REVIEW_REQUIRED"),
    ((SceneObject("leave", "skip"),), "PARTIAL_SUCCESS"),
])
def test_empty_and_non_actionable_scenes(objects, outcome):
    clock = Clock()
    desk = MockDesk(clock, objects)
    runtime, _, _, _ = make_runtime(clock=clock, desk=desk)
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == outcome
    assert not desk.executor.commands


def test_failed_actions_reobserve_before_retry_or_termination():
    runtime, desk, _, clock = make_runtime()
    desk.executor.handler = lambda _: ModuleResult.failed(
        FailureCode.GRASP_FAIL, retryable=True
    )
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == "FAILED"
    assert len(desk.executor.commands) == 3
    assert len(desk.perception.commands) == 4
    assert runtime.last_report.failure_code == "GRASP_FAIL"


def test_final_observation_can_disprove_a_done_proposal():
    runtime, desk, _, clock = make_runtime()
    original = desk.observe
    count = 0

    def observe(command):
        nonlocal count
        count += 1
        if count == 4:
            desk.objects.append(SceneObject("new-object"))
        return original(command)

    desk.perception.handler = observe
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.completed_tasks == ["trash-1", "trash-2", "new-object"]
    assert runtime.last_report.outcome == "SUCCESS"


def test_stale_proposal_is_blocked_without_executing():
    runtime, desk, _, clock = make_runtime()
    desk.planner.handler = lambda _: ModuleResult.success(
        TaskProposal("collect_trash", "old-snapshot", "trash-1")
    )
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == "BLOCKED"
    assert not desk.executor.commands


def test_invalid_or_repeated_observation_is_blocked():
    runtime, desk, _, clock = make_runtime()
    desk.perception.handler = lambda _: ModuleResult.success(
        SceneSnapshot("same", clock(), (SceneObject("trash-1"),), "mock://same")
    )
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == "BLOCKED"
    assert runtime.last_report.failure_code == "PERCEPTION_FAIL"


@pytest.mark.parametrize("stage", [RuntimeState.NAVIGATE_TO_TARGET, RuntimeState.WORKING,
                                    RuntimeState.RETURN_HOME])
def test_cancel_each_phase_and_do_not_restart_a_cancelled_return(stage):
    runtime, _, nav, clock = make_runtime()
    runtime.offer(request())
    if stage != RuntimeState.NAVIGATE_TO_TARGET:
        drive(runtime, clock, stage)
    runtime.tick()
    assert runtime.cancel("one").accepted
    drive(runtime, clock)
    assert runtime.last_report.outcome == "CANCELLED"
    assert nav.commands.count("home") == 1
    assert runtime.cancel("one").duplicate


def test_wait_for_next_accepts_a_new_offer_without_returning():
    runtime, _, nav, clock = make_runtime(policy=RuntimePolicy(post_mission="wait_for_next"))
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.return_result == "NOT_REQUESTED"
    assert runtime.offer(request("two", "seat-13")).accepted
    drive(runtime, clock)
    assert nav.commands == ["seat-12", "seat-13"]


def test_return_failure_preserves_cleaning_success_without_reporting_overall_success():
    nav = DeferredPort(lambda target: ModuleResult.success() if target != "home" else
                       ModuleResult.failed(FailureCode.NAVIGATION_FAIL))
    runtime, _, _, clock = make_runtime(navigator=nav)
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == "FAILED"
    assert runtime.last_report.completed_tasks == ["trash-1", "trash-2"]
    assert runtime.last_report.return_result == "FAILED"


def test_navigation_timeout_waits_for_stop():
    nav = DeferredPort(lambda _: ModuleResult.success(), polls=10000)
    runtime, _, _, clock = make_runtime(
        navigator=nav, policy=RuntimePolicy(navigation_timeout=.2)
    )
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.failure_code == "TIMEOUT"
    assert nav.stopped()


def test_unacknowledged_cancel_latches_error_and_rejects_new_work():
    nav = DeferredPort(lambda _: ModuleResult.success(), polls=10000)
    nav.cancel = lambda _: None
    runtime, _, _, clock = make_runtime(navigator=nav, policy=RuntimePolicy(cancel_timeout=.2))
    runtime.offer(request())
    runtime.tick()
    runtime.cancel("one")
    drive(runtime, clock)
    assert runtime.state == RuntimeState.ERROR
    assert runtime.offer(request("two")).reason == "TIMEOUT"
    assert runtime.reset_error().reason == "NOT_STOPPED"


def test_restart_interrupts_active_work_and_requires_explicit_safe_reset(tmp_path):
    journal = MissionJournal(str(tmp_path / "journal.db"))
    first, _, _, _ = make_runtime(journal=journal)
    first.offer(request())
    second, _, nav, _ = make_runtime(journal=journal)
    assert second.state == RuntimeState.ERROR
    assert second.last_report.outcome == "INTERRUPTED"
    assert second.offer(request()).report.outcome == "INTERRUPTED"
    assert not nav.commands
    assert second.reset_error().accepted
    assert second.offer(request("two")).accepted


def test_disallowed_actions_and_action_budget_are_bounded():
    runtime, desk, _, clock = make_runtime(policy=RuntimePolicy(max_actions=1))
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.failure_code == "TIMEOUT"
    assert len(desk.executor.commands) == 1
    runtime, desk, _, clock = make_runtime()
    desk.planner.handler = lambda command: ModuleResult.success(TaskProposal(
        "drive_motors", command[0].snapshot_id, "trash-1"
    ))
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == "BLOCKED"
    assert not desk.executor.commands


def test_unknown_target_kind_and_conflicting_request_do_not_admit():
    runtime, _, _, _ = make_runtime()
    assert runtime.offer(replace(request(), target_kind="ZONE")).reason == "UNSUPPORTED_MISSION"
    assert runtime.offer(replace(request(), mission_id="")).reason == "INVALID_REQUEST"


def test_repeated_adapter_exceptions_do_not_extend_stop_deadline():
    runtime, _, nav, clock = make_runtime(policy=RuntimePolicy(cancel_timeout=.2))
    runtime.offer(request())
    runtime.tick()
    def fail(_):
        raise RuntimeError("disconnected")
    nav.poll = fail
    nav.cancel = fail
    drive(runtime, clock)
    assert runtime.state == RuntimeState.ERROR
    assert runtime.last_report.failure_code == "UNKNOWN_ERROR"
    assert clock() < 11


def test_independent_safety_release_requires_stopped_feedback_and_explicit_reset():
    runtime, _, nav, clock = make_runtime()
    runtime.offer(request())
    runtime.tick()
    runtime.safety_fault(FailureCode.E_STOP)
    drive(runtime, clock)
    assert runtime.state == RuntimeState.ERROR
    assert runtime.last_report.failure_code == "E_STOP"
    assert nav.commands == ["seat-12"]
    assert runtime.reset_error().reason == "HARDWARE_RELEASE_REQUIRED"
    assert runtime.safety_released(FailureCode.E_STOP).accepted
    assert runtime.state == RuntimeState.ERROR
    assert runtime.reset_error().accepted


@pytest.mark.parametrize("timeout", [float("nan"), float("inf"), 0.0])
def test_policy_requires_bounded_timeouts(timeout):
    with pytest.raises(ValueError):
        RuntimePolicy(navigation_timeout=timeout)


def test_restart_report_keeps_the_execution_profile_from_admission(tmp_path):
    journal = MissionJournal(str(tmp_path / "missions.db"))
    runtime, _, _, _ = make_runtime(journal=journal)
    runtime.execution_profile["navigation"] = "sim"
    runtime.offer(request())
    restarted, _, _, _ = make_runtime(journal=journal)
    assert restarted.last_report.execution_profile["navigation"] == "sim"
    assert restarted.execution_profile["navigation"] == "mock"


def test_debug_events_retain_short_lived_bt_checks_without_changing_mission():
    runtime, _, nav, clock = make_runtime()
    runtime.offer(request())
    drive(runtime, clock)
    events = list(runtime.debug_events)
    assert runtime.last_report.outcome == 'SUCCESS'
    assert nav.commands == ['seat-12', 'home']
    assert any(e['kind'] == 'bt_tick' and e['data'] == {
        'node_id': 'ValidateProposal', 'status': 'SUCCESS'} for e in events)
    assert any(e['kind'] == 'transition' and e['data']['to'] == 'POST_MISSION' for e in events)
    assert any(e['kind'] == 'action_start' and e['task_id'] and e['execution_id'] for e in events)
    assert [e['sequence'] for e in events] == list(range(1, len(events)+1))
    snapshot = runtime.debug_snapshot()
    assert snapshot['safe_to_drive']
    assert len(snapshot['bt_nodes']) == 9
    assert all(e['boot_id'] == snapshot['boot_id'] for e in events)
