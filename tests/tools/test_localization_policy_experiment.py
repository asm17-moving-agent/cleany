"""Evidence windows must not turn gaps or ambiguous matches into recovery success."""
import importlib.util
from pathlib import Path
import sys

from cleany_navigation.localization_alignment import AlignmentScore

path=Path(__file__).resolve().parents[2]/'tools/validation/localization_policy_experiment.py'
spec=importlib.util.spec_from_file_location('localization_policy_experiment',path)
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module
spec.loader.exec_module(module)


def test_reverification_needs_continuous_new_evidence():
    good=AlignmentScore(120,.5,.12,.1,.8)
    bad=AlignmentScore(120,.2,.3,.3,.8)
    assert module.stable([good]*7,[i*.25 for i in range(7)],module.SETTINGS)==6
    assert module.stable([good]*7,[0,.25,.5,2,2.25,2.5,2.75],module.SETTINGS)is None
    assert module.stable([good]*7,[0]*7,module.SETTINGS)is None
    assert module.stable([good,good,good,bad,good,good,good],[i*.25 for i in range(7)],module.SETTINGS)is None


def test_lower_ratio_does_not_bypass_ambiguity_or_required_improvement():
    result={'validation':{'points':120,'inlier_ratio':.52,'mean_distance':.12,
                         'median_distance':.1,'known_ratio':.8},
            'baseline':{'mean_distance':.2},'alternative_gap':.01}
    assert module.candidate_gate(result,.5,True)==(False,'ambiguous_candidate')
    result['alternative_gap']=.04
    assert module.candidate_gate(result,.5,True)==(True,'accepted')
    assert module.candidate_gate(result,.65,True)==(False,'candidate_not_well_aligned')
    result['baseline']['mean_distance']=.13
    assert module.candidate_gate(result,.5,True)==(False,'insufficient_improvement')
