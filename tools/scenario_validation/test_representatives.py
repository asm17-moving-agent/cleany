import math
import pytest
from representatives import cumulative_turn,select_cases


def test_turn_metric_does_not_count_angle_wrap_as_a_full_rotation():
    assert cumulative_turn([math.pi-.1,-math.pi+.1])==pytest.approx(.2)


def test_selection_keeps_unique_cases_and_does_not_hide_failed_survey_rows():
    rows=[{'seat':str(i),'passed':True,'distance_home_m':float(i),'minimum_map_center_clearance_m':float(i),
           'cumulative_yaw_rad':float(i),'target_gt_error_m':float(i),'guard_intervention_fraction':float(i)} for i in range(7)]
    rows.append({'seat':'failed','passed':False,'distance_home_m':1000})
    selected=select_cases(rows)
    assert len(selected)==len({r['seat'] for r in selected})==6
    assert selected[0]['seat']=='0' and selected[1]['seat']=='6'
    assert all(r['seat']!='failed' for r in selected)


def test_missing_measurements_do_not_generate_a_fabricated_representative():
    with pytest.raises(RuntimeError):select_cases([])
