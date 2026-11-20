"""물체 경계 구와 수거함 입구의 접촉 경계를 검사합니다."""
import pytest
from cleany_skill_executor.core.sorting import (
    bin_release_region, validate_release_point, validate_payload_above_opening,
)


def test_planning_tolerance_must_fit_inside_release_region():
    lower, upper = bin_release_region((0., 0.), (.3, .3, .3), .01, .3, .05, .06, .21, .005)
    validate_release_point((0., 0., .45), lower, upper)
    with pytest.raises(ValueError):
        validate_release_point((lower[0], 0., .45), lower, upper)
    with pytest.raises(ValueError):
        validate_release_point((0., 0., float('nan')), lower, upper)


@pytest.mark.parametrize('center,radius', [((0., 0., .35), .05), ((.09, 0., .4), .05),
    ((0., 0., .4), 0.), ((0., 0., .4), float('nan')), ((0., float('inf'), .4), .05)])
def test_rim_contact_or_invalid_payload_is_rejected(center, radius):
    with pytest.raises(ValueError):
        validate_payload_above_opening(center, radius, (0., 0.), (.3, .3, .3), .01, .3)


def test_payload_strictly_above_and_inside_opening_is_allowed():
    validate_payload_above_opening((0., 0., .4), .05, (0., 0.), (.3, .3, .3), .01, .3)
