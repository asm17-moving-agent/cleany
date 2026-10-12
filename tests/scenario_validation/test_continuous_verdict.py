from copy import deepcopy
from continuous_verdict import SEATS, assess_campaign
from worlds import chair_name


def campaign(successes):
    sequence = SEATS * 10
    stop = {'cmd_vel': [0, 0, 0], 'ground_truth': {'twist': [0, 0, 0]}}
    chairs = [chair_name(s) for s in SEATS]
    result = {'scenario': 'normal', 'expected_mission_count': 60, 'final_latest': stop,
              'chair_fixture_sequence': sequence, 'chair_inventory': chairs, 'missions': [],
              'chair_scene_evidence': [{'seat': s, 'removed': chair_name(s),
                  'observed_chairs': [c for c in chairs if c != chair_name(s)]} for s in sequence]}
    for i, seat in enumerate(sequence):
        result['missions'].append({**deepcopy(stop), 'seat': seat, 'mission_id': str(i),
            'accepted': True, 'matched_terminal': True, 'final': {'last_result': {
                'request': {'mission_id': str(i), 'target_id': seat},
                'outcome': 'SUCCESS' if i < successes else 'FAILED', 'navigation_result': 'OK',
                'return_result': 'OK' if i < successes else 'FAILED'}}})
    return result


def test_threshold_preserves_all_ten_failed_results():
    verdict = assess_campaign(campaign(50), {'sim': 0})
    assert verdict['achieved'] and verdict['successes'] == 50
    assert len(verdict['failures']) == 10
    assert not assess_campaign(campaign(49), {'sim': 0})['achieved']


def test_fifty_successes_in_partial_run_do_not_satisfy_sixty_attempts():
    result = campaign(60)
    result['missions'] = result['missions'][:50]
    assert not assess_campaign(result, {'sim': 0})['achieved']


def test_stale_terminal_or_moving_success_does_not_count():
    result = campaign(50)
    result['missions'][0]['ground_truth']['twist'] = [.1, 0, 0]
    assert not assess_campaign(result, {'sim': 0})['achieved']
    result = campaign(50)
    result['missions'][-1]['final']['last_result']['request']['mission_id'] = 'stale'
    assert not assess_campaign(result, {'sim': 0})['achieved']
