from hybrid_target_matrix import (TargetCase, cartesian_offsets, distribution,
                                  joint_samples, orientation_offsets, summarize)
from cleany_skill_executor.core.physical_grasp_selection import PoseValue


def test_joint_sample_counts_reproducibility_and_limits():
    limits=[(-1.5,3.2)]*5
    samples=joint_samples(limits,seed=20261002)
    assert samples==joint_samples(limits,seed=20261002)
    assert len(samples['interior'])==32 and len(samples['boundary'])==24
    assert len(set(samples['boundary']))==24
    for group in samples.values():
        assert all(all(low<q<high for q,(low,high) in zip(values,limits)) for values in group)


def test_position_and_orientation_conditions_are_distinct():
    assert len(set(cartesian_offsets()))==32
    assert len(set(orientation_offsets()))==24


def test_distribution_handles_empty_samples_and_interpolates_percentiles():
    assert distribution([])['mean'] is None
    assert distribution([1.,2.,3.])==dict(count=3,mean=2.,p50=2.,p95=2.9,maximum=3.)


def test_summary_separates_unique_conditions_repeats_and_planning():
    cases=[TargetCase('a','left','interior',PoseValue((0.,0.,0.),(0.,0.,0.,1.)),
                      witness_valid=True).json(),
           TargetCase('b','right','unreachable',PoseValue((2.,0.,0.),(0.,0.,0.,1.)),
                      expected_rejection=True).json()]
    rows=[dict(case_id='a',repeat=0,ik_accepted=True,plan_attempted=True,
               plan_passed=True,plan_seconds=.1,position_error_m=.0001,failure=''),
          dict(case_id='a',repeat=1,ik_accepted=False,plan_attempted=False,
               failure='orientation_mismatch'),
          dict(case_id='b',repeat=0,ik_accepted=False,plan_attempted=False,
               failure='no_position_ik')]
    result=summarize(cases,rows,[])
    assert result['unique_conditions']==2 and result['evaluations']==3
    assert result['plan_attempts']==result['plans_passed']==1
    assert result['categories']['interior']['known_feasible_evaluations']==2
    assert result['categories']['interior']['known_feasible_accepted']==1
    assert result['expected_rejection_false_accepts']==0
