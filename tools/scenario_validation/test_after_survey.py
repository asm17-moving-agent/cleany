import json
from pathlib import Path
import pytest
from after_survey import survey_coverage


def fixture(directory: Path, seats: list[str]) -> None:
    folder=directory/'fixture';folder.mkdir()
    (folder/'seat_inventory.json').write_text(json.dumps({'seats':seats}))
    (directory/'survey_jobs.json').write_text(json.dumps([{'name':'survey-00-r1'}]))
    (directory/'survey_jobs.status.json').write_text(json.dumps([{'exit_code':0,'end':1}]))
    worker=directory/'survey-00-r1/worker-00';worker.mkdir(parents=True)
    result={'world_manifest':{'seat':'seat-43'},'scenario':'normal','expected_mission_count':1,'final_latest':{'cmd_vel':[0,0,0],'ground_truth':{'twist':[0,0,0]}},'missions':[{'seat':'seat-43','accepted':True,'mission_id':'a','final':{'last_result':{'request':{'mission_id':'a','target_id':'seat-43'},'outcome':'SUCCESS'}}}]}
    (worker/'result.json').write_text(json.dumps(result));(worker/'shutdown.json').write_text(json.dumps({'sim':0,'navigation':0,'mission':0}))


def test_coverage_requires_every_registered_seat(tmp_path: Path):
    fixture(tmp_path,['seat-43','seat-13'])
    with pytest.raises(RuntimeError,match='complete seat inventory'):
        survey_coverage(tmp_path)


def test_coverage_preserves_failed_normal_seat_as_failure(tmp_path: Path):
    fixture(tmp_path,['seat-43'])
    path=tmp_path/'survey-00-r1/worker-00/result.json'
    result=json.loads(path.read_text());result['missions'][0]['final']['last_result']['outcome']='FAILED';path.write_text(json.dumps(result))
    report=survey_coverage(tmp_path)
    assert report['seat_count']==1 and report['successful_seats']==0
    assert report['failed_seats']==['seat-43']
