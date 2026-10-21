"""Evidence semantics: missing data, intentional OFF, stale stamps and recorder loss."""
import json
from pathlib import Path
import struct

import pytest

from cleany_dev_monitor.io_probe import EvidenceWriter, ProbeState, header_stamp_ns, load_config
from cleany_dev_monitor.io_probe_report import read_recording, stage_segments

CONFIG = Path(__file__).parents[1]/'config/manipulation_io_probe.json'


def test_raw_header_accepts_cdr_endianness_and_rejects_invalid_or_unsupported():
    for identifier, fmt in ((b'\x00\x01', '<iI'), (b'\x00\x00', '>iI')):
        assert header_stamp_ns(identifier+b'\x00\x00'+struct.pack(fmt, 42, 123)) == 42_000_000_123
    assert header_stamp_ns(b'\x00\x03'+b'\x00'*100) is None
    assert header_stamp_ns(b'\x00\x01\x00\x00'+struct.pack('<iI', 42, 1_000_000_000)) is None
    assert header_stamp_ns(b'\x00'*12) is None
    assert header_stamp_ns(b'') is None


def test_window_boundaries_empty_windows_and_source_age_do_not_reset_on_republish():
    state = ProbeState(load_config(CONFIG)); topic = '/joint_states'
    for t in (0, .2, .9): state.receive(topic, t, 12, 1000, 1_001_000)
    stream = state.sample(1., 0)['streams'][0]
    assert stream['count'] == 2 and stream['hz'] == 2
    assert state.sample(2., 0)['streams'][0]['hz'] == 0
    assert state.sample(2., 0)['streams'][0]['last_receipt_age_ms'] == 1100
    repeated = state.receive(topic, 2.1, 12, 1000, 2_001_000)
    assert repeated['duplicate_stamp'] and repeated['stamp_age_ms'] == 2
    assert state.sample(2.2, 0)['streams'][0]['current_source_age_ms'] == pytest.approx(102)


def test_wrist_off_unknown_warmup_and_reenable_do_not_hide_stalls():
    state = ProbeState(load_config(CONFIG)); topic = '/left_wrist_camera/image_raw'
    spec = state.streams[topic].spec
    assert state.mode(spec, 0)[0] == 'UNKNOWN'
    state.enabled('left', False, 0)
    assert state.mode(spec, 1)[0] == 'OFF'
    assert state.mode(spec, 4)[0] == 'UNKNOWN'  # Dead publisher is not permanently OFF.
    state.enabled('left', True, 4)
    state.receive(topic, 4.1, 100, 1, 2)
    state.enabled('left', False, 4.2)
    state.enabled('left', True, 10)
    assert state.receive(topic, 10.1, 100, 1, 2)['active_gap_ms'] is None
    assert state.receive(topic, 10.2, 100, 1, 2)['active_gap_ms'] == pytest.approx(100)
    state.enabled('left', True, 12)
    row = next(r for r in state.sample(12.1, 0)['streams'] if r['topic'] == topic)
    assert row['status'] == 'OBSERVED' and row['hz'] == 0 and row['last_receipt_age_ms'] > 1800


def test_execution_heartbeat_staleness_and_terminal_are_distinct():
    state = ProbeState(load_config(CONFIG))
    assert state.stage(0)['stage'] == 'NO_EXECUTION'
    state.execution = dict(execution_id='e', record_state='ACTIVE', stage='GRASPING')
    state.execution_seen = 0
    assert state.stage(2)['stage'] == 'GRASPING'
    assert state.stage(4)['stage'] == 'STALE_EXECUTION'
    state.execution['has_result'] = True
    assert state.stage(4)['stage'] == 'TERMINAL'


def test_pre_action_inspection_and_new_request_override_old_terminal():
    state = ProbeState(load_config(CONFIG))
    state.command = dict(execution_id='new', phase='INSPECTING_SCENE', state='ACTIVE', owner_alive=True)
    state.command_seen = 1
    assert state.stage(2)['stage'] == 'REQUEST:INSPECTING_SCENE'
    state.execution = dict(execution_id='old', has_result=True)
    assert state.stage(2)['execution_id'] == 'new'
    assert state.stage(5)['stage'] == 'REQUEST:STALE_REQUEST'
    state.execution = dict(execution_id='new', record_state='ACTIVE', stage='GRASPING')
    state.execution_seen = 5
    assert state.stage(5)['stage'] == 'GRASPING'


def test_latched_old_result_is_history_but_active_heartbeat_establishes_liveness():
    state = ProbeState(load_config(CONFIG))
    row = dict(execution_id='e', accepted_at_ns=1, updated_at_ns=2, revision=1,
               record_state='ACTIVE', has_result=False, stage='GRASPING')
    assert state.observe_execution(row, 0, 100)['historical']
    assert state.observe_execution(row, .01, 100)['historical']
    assert not state.observe_execution(row, 1, 100)['historical']
    terminal = {**row, 'record_state': 'FINISHED', 'has_result': True, 'revision': 2}
    assert state.observe_execution(terminal, 2, 100)['historical']
    assert state.observe_execution(terminal, 4, 100)['historical']


def test_writer_flushes_all_evidence_and_size_limit_is_not_reported_complete(tmp_path):
    writer = EvidenceWriter(tmp_path/'ok.jsonl', 10, 100000)
    writer.put(dict(kind='arrival', t=1)); writer.put(dict(kind='window', t=2))
    assert writer.close()['complete']
    assert len((tmp_path/'ok.jsonl').read_text().splitlines()) == 2
    writer = EvidenceWriter(tmp_path/'full.jsonl', 10, 5)
    writer.put(dict(kind='arrival', t=1))
    outcome = writer.close()
    assert not outcome['complete'] and outcome['error'] and outcome['dropped'] == 1
    with pytest.raises(FileExistsError): EvidenceWriter(tmp_path/'ok.jsonl', 10, 1000)


def test_replayed_or_missing_run_is_not_labeled_physical_success(tmp_path):
    from cleany_dev_monitor.io_probe_report import render
    config = load_config(CONFIG)
    (tmp_path/'manifest.json').write_text(json.dumps(dict(config=config, synthetic=True,
        label='synthetic-test', duration_s=3, finished=True, writer={'complete': True})))
    rows = [ProbeState(config).sample(.5, 0), ProbeState(config).sample(3., 0)]
    (tmp_path/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    manifest, recorded = read_recording(tmp_path)
    assert stage_segments(recorded, 3)[0]['identity'][1] == 'NO_EXECUTION'
    summary = render(tmp_path)
    assert summary['status'] == 'SYNTHETIC_TEST' and not summary['execution_ids']
    assert (tmp_path/'report/stability-timeseries.pdf').stat().st_size > 1000


def test_invalid_config_rejects_excessive_sampling_and_duplicate_topics(tmp_path):
    config = load_config(CONFIG); path = tmp_path/'bad.json'
    config['streams'].append(config['streams'][0])
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError): load_config(path)
    config = load_config(CONFIG); config['sample_interval_s'] = .001
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError): load_config(path)
