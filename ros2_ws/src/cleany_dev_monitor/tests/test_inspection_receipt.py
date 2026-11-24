from types import SimpleNamespace
from uuid import uuid4

import pytest


def test_inspection_stores_numbered_frame_without_selecting_target(tmp_path):
    from cleany_dev_monitor.proposal_ros import ProposalROS
    from cleany_dev_monitor.proposals import ProposalStore
    frame = SimpleNamespace(header=object(), width=640, height=480)
    store = ProposalStore(tmp_path)
    port = SimpleNamespace(images=[frame], store=store, inspect_scene=lambda cancelled: frame,
                           _observation_record=lambda scene: {'snapshot_id':'capture-A','targets':[]},
                           capture_image=lambda scene, target, cancelled: b'numbered-jpeg')
    job_id = str(uuid4())
    result = ProposalROS.inspect_for_monitor(port, job_id, lambda: False)
    assert result['image_available'] and result['objects'] == []
    assert result['source_width'] == 640 and result['source_height'] == 480
    assert min(result['seconds'].values()) >= 0
    assert (store.directory / (job_id+'.jpg')).read_bytes() == b'numbered-jpeg'
    assert not store.pointer.exists()
    port.capture_image = lambda *args: None
    assert not ProposalROS.inspect_for_monitor(port, str(uuid4()), lambda: False)['image_available']
    with pytest.raises(InterruptedError):
        ProposalROS.inspect_for_monitor(port, str(uuid4()), lambda: True)


def test_prior_times_include_actions_outside_recent_window(tmp_path):
    import json
    from cleany_dev_monitor.operator_jobs import previous_action_times
    for i in range(24):
        (tmp_path/f'{i}.json').write_text(json.dumps(dict(action='run' if i == 0 else 'prepare',
            state='SUCCEEDED',created_at=100+i,finished_at=101+i)))
    latest = tmp_path/'new.json'
    times = previous_action_times(latest)
    assert times['run']['finished_at'] == 101
    assert times['prepare']['finished_at'] == 124
