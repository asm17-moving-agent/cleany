from audit import assess


def result(outcome='SUCCESS', mission='a', scenario='normal'):
    return {'scenario':scenario,'final_latest':{'cmd_vel':[0,0,0],'ground_truth':{'twist':[0,0,0]}},'missions':[{'seat':'seat-43','accepted':True,'mission_id':'a','final':{'last_result':{'request':{'mission_id':mission,'target_id':'seat-43'},'outcome':outcome}}}]}


def test_success_requires_matching_terminal_result_and_observed_stop():
    assert assess(result(),{'sim':0})['passed']
    assert not assess(result(mission='previous'),{'sim':0})['passed']
    r=result();r['final_latest']['ground_truth']['twist']=[.1,0,0]
    assert not assess(r,{'sim':0})['passed']


def test_expected_failure_is_not_normal_success():
    assert not assess(result(outcome='FAILED'),{'sim':0})['passed']
    assert assess(result(outcome='FAILED',scenario='blocked'),{'sim':0})['passed']


def test_sensor_loss_requires_stop_before_cleanup_cancel():
    r=result(outcome='CANCELLED',scenario='sensor_loss');m=r['missions'][0]
    m.update(guard_decisions_after_stimulus={'STALE_DEPTH':1},events=[{'wall':10}],stopped_after_stimulus={'wall_seconds':5},cleanup_cancel={'wall':14})
    assert not assess(r,{'sim':0})['passed']
    m['stopped_after_stimulus']['wall_seconds']=1
    assert assess(r,{'sim':0})['passed']


def test_partial_repeat_series_does_not_pass():
    r=result();r['expected_mission_count']=5
    assert not assess(r,{'sim':0})['passed']


def test_path_failure_requires_actual_planner_loss_stimulus():
    r=result(outcome='FAILED',scenario='path_failure')
    assert not assess(r,{'sim':0})['passed']
    r['missions'][0]['events']=[{'stopped_node':{'name':'planner_server','pid':1}}]
    assert assess(r,{'sim':0})['passed']


def test_terminal_target_must_match_the_worlds_requested_seat():
    r=result();r['missions'][0]['final']['last_result']['request']['target_id']='seat-13'
    assert not assess(r,{'sim':0})['passed']
