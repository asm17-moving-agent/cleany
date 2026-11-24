from cleany_dev_monitor.prepare_progress import prepare_progress


def test_separate_jobs_and_nested_gpu_time(tmp_path):
    log = tmp_path / 'job.log'
    assert prepare_progress(log, tmp_path) == {}
    support = tmp_path / 'runtime/collection-operator/one/logs/hardware-support.log'
    support.parent.mkdir(parents=True)
    support.write_text('warmup: phase=GPU_IK state=started arm=right\n')
    log.write_text(f'started hardware_support (PID 12, log {support})\n')
    progress = prepare_progress(log, tmp_path)
    assert progress['planning']['state'] == 'active'
    assert progress['gpu'] == {'state': 'active', 'phase': 'GPU_IK', 'arm': 'right'}
    with support.open('a') as stream:
        stream.write('{"event":"startup_ik_warmup","outcome":"ready","wall_sec":26.5}\n')
        stream.write('{"event":"planning_sequence_ready","status":"ready","wall_sec":57.3}\n')
    progress = prepare_progress(log, tmp_path)
    assert progress['gpu'] == {'state': 'done', 'seconds': 26.5}
    assert progress['planning'] == {'state': 'done', 'seconds': 57.3}
    assert 'executor' not in progress
    log.write_text('Collection runtime is already running; repeating the full readiness check.\n')
    assert prepare_progress(log, tmp_path) == {'reuse': {'state': 'done'}}
