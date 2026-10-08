import json
from pathlib import Path
import pytest
from after_smoke_recheck import require_completed_smoke


def fixture(directory: Path) -> list[dict]:
    jobs=[{'count':1,'name':'smoke','scenario':'blocked'}]
    (directory/'scenario_smoke_jobs.status.json').write_text(json.dumps(
        [{'job':jobs[0],'exit_code':0,'end':1}]))
    worker=directory/'smoke/worker-00';worker.mkdir(parents=True)
    (worker.parent/'batch.json').write_text(json.dumps({'exit_codes':[0]}))
    (worker/'shutdown.json').write_text(json.dumps({'sim':0,'nav':0,'mission':0}))
    return jobs


def test_stale_status_does_not_admit_new_build(tmp_path: Path):
    jobs=fixture(tmp_path);jobs[0]['name']='different-job'
    with pytest.raises(RuntimeError,match='queue is incomplete'):
        require_completed_smoke(tmp_path,jobs)


def test_missing_worker_or_failed_shutdown_prevents_new_stack(tmp_path: Path):
    jobs=fixture(tmp_path)
    shutdown=tmp_path/'smoke/worker-00/shutdown.json'
    shutdown.write_text(json.dumps({'sim':-9,'nav':0,'mission':0}))
    with pytest.raises(RuntimeError,match='shutdown is incomplete'):
        require_completed_smoke(tmp_path,jobs)
    shutdown.unlink()
    with pytest.raises(FileNotFoundError):require_completed_smoke(tmp_path,jobs)


def test_clean_negative_smoke_allows_independent_navigation_recheck(tmp_path: Path):
    jobs=fixture(tmp_path)
    (tmp_path/'smoke/worker-00/result.json').write_text(json.dumps({'outcome':'FAILED'}))
    require_completed_smoke(tmp_path,jobs)
