"""Statistics must retain failures and isolate the measured execution interval."""
import pytest

from summarize_motion_benchmarks import (
    latency_statistics, summarize_trials, telemetry_for_trial, comparison_key,
)


def trial(wall, success, *, warm=True, **extra):
    return {'warm': warm, 'wall_sec': wall, 'successes': int(success),
            'items': [{'success': success, 'completion_sec': wall}], **extra}


def test_warmup_and_skipped_trials_do_not_bias_request_statistics():
    rows = [trial(100., True, is_warmup=True), trial(2., True),
            trial(3., False), trial(0., False, skipped='No common IK goal')]
    summary = summarize_trials(rows)
    assert summary['warmup_trials'] == summary['skipped_trials'] == 1
    assert summary['warm']['calls'] == 2
    assert summary['warm']['wall_time']['mean_sec'] == 2.5
    assert summary['warm']['call_success_rate'] == .5


def test_failed_calls_remain_in_denominator_but_have_no_first_valid_latency():
    summary = summarize_trials([trial(1., True), trial(100., False), trial(3., True)])['warm']
    assert summary['call_success_rate'] == 2/3
    assert summary['wall_time']['mean_sec'] == pytest.approx(104/3)
    assert summary['first_valid_time_conditional_on_success']['mean_sec'] == 2.
    assert summary['first_valid_time_conditional_on_success']['samples'] == 2


def test_percentile_requires_at_least_thirty_samples():
    assert latency_statistics([1., 2., 3.])['p95_sec'] is None
    assert latency_statistics(list(range(30)))['p95_sec'] == pytest.approx(27.55)


def test_inconsistent_success_counter_is_rejected():
    item = trial(1., True); item['successes'] = 0
    with pytest.raises(ValueError, match='disagrees'):
        summarize_trials([item])


def test_telemetry_excludes_startup_and_marks_partial_energy_coverage():
    events = [
        {'kind':'tegrastats', 'unix_sec':0., 'gpu_percent':99., 'power_mw':10000.},
        {'kind':'tegrastats', 'unix_sec':101., 'gpu_percent':50., 'power_mw':4000.},
        {'kind':'tegrastats', 'unix_sec':109., 'gpu_percent':60., 'power_mw':6000.},
        {'kind':'tegrastats', 'unix_sec':200., 'gpu_percent':99., 'power_mw':10000.},
    ]
    result = telemetry_for_trial({'started_unix_sec':100., 'ended_unix_sec':110.}, events)
    assert result['samples'] == 2
    assert result['gpu_percent']['mean'] == 55.
    assert result['observed_system_energy_j'] == 40.
    assert result['power_sample_coverage_fraction'] == .8


def test_legacy_or_mismatched_models_cannot_be_grouped_as_fair_comparisons():
    assert comparison_key({'backend':'moveit'}) is None
    data = {'scene_collision_model_verified':True, 'source_sha256':'source',
            'collision_model_sha256':'model', 'fixture_configuration_sha256':'fixture',
            'phase':'pregrasp','actual_candidate_count':8,'config':{key:1 for key in (
                'position_tolerance_m','approach_tolerance_degrees','closing_tolerance_degrees',
                'validation_step_rad','maximum_validation_states','approach_corridor_radius_m')}}
    assert comparison_key(data) != comparison_key({**data, 'collision_model_sha256':'other'})
