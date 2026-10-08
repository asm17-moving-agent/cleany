from collections import Counter
from repeat_representatives import fresh_jobs


def test_every_selected_seat_gets_each_fresh_instance_repeat():
    seats=['seat-01','seat-13','seat-43','seat-a1-01','seat-m3-06','seat-48']
    jobs=fresh_jobs(seats,10,'/tested/setup.bash')
    assert len({j['name'] for j in jobs})==len(jobs)
    assert Counter(s for j in jobs for s in j['seats'].split(','))==Counter({s:10 for s in seats})
    assert all(len(j['seats'].split(','))==j['count']<=3 for j in jobs)
