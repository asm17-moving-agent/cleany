import json
from pathlib import Path
from survey import choose_count


def batch(directory: Path,count: int,seconds: float,success=True,repeat=1) -> None:
    folder=directory/f'capacity-{count}-long-r{repeat}';folder.mkdir()
    (folder/'batch.json').write_text(json.dumps({'elapsed_wall':seconds,'exit_codes':[0]*count}))
    for i in range(count):
        worker=folder/f'worker-{i:02d}';worker.mkdir()
        result={'scenario':'normal','expected_mission_count':1,'final_latest':{'cmd_vel':[0,0,0],'ground_truth':{'twist':[0,0,0]}},'missions':[{'seat':'seat-43','mission_id':'a','accepted':True,'final':{'last_result':{'request':{'mission_id':'a','target_id':'seat-43'},'outcome':'SUCCESS' if success else 'FAILED'}}}]}
        (worker/'result.json').write_text(json.dumps(result));(worker/'shutdown.json').write_text(json.dumps({'sim':0,'navigation':0,'mission':0}))


def test_selection_uses_completed_missions_per_hour_and_rejects_failed_fast_batch(tmp_path: Path):
    batch(tmp_path,1,100);batch(tmp_path,2,130);batch(tmp_path,3,120,False)
    count,candidates=choose_count(tmp_path)
    assert count==2
    assert {c['count'] for c in candidates}=={1,2}


def test_selection_uses_repeated_median_instead_of_one_fast_trial(tmp_path: Path):
    batch(tmp_path,1,200)
    for repeat,seconds in enumerate((100,400,400),1):batch(tmp_path,2,seconds,repeat=repeat)
    for repeat in (1,2,3):batch(tmp_path,3,300,repeat=repeat)
    count,candidates=choose_count(tmp_path)
    assert count==3
    assert next(c for c in candidates if c['count']==2)['missions_per_hour']==18
