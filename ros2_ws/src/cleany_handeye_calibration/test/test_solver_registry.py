from types import SimpleNamespace

import pytest

from cleany_handeye_calibration.solver import (
    HAND_EYE_METHOD_REGISTRY,
    HandEyeMethod,
    RegistryCompletenessError,
    validate_hand_eye_registry,
)


EXPECTED_SYMBOLS = {
    'CALIB_HAND_EYE_TSAI',
    'CALIB_HAND_EYE_PARK',
    'CALIB_HAND_EYE_HORAUD',
    'CALIB_HAND_EYE_ANDREFF',
    'CALIB_HAND_EYE_DANIILIDIS',
}


def _fake_cv_module():
    return SimpleNamespace(
        CALIB_HAND_EYE_TSAI=0,
        CALIB_HAND_EYE_PARK=1,
        CALIB_HAND_EYE_HORAUD=2,
        CALIB_HAND_EYE_ANDREFF=3,
        CALIB_HAND_EYE_DANIILIDIS=4,
    )


def test_installed_opencv_registry_contains_exactly_five_methods():
    resolved = validate_hand_eye_registry()

    assert tuple(resolved) == tuple(HandEyeMethod)
    assert {
        method_spec.opencv_symbol
        for method_spec in HAND_EYE_METHOD_REGISTRY
    } == EXPECTED_SYMBOLS
    assert len(resolved) == 5


def test_registry_preflight_rejects_missing_method():
    fake_cv = _fake_cv_module()
    del fake_cv.CALIB_HAND_EYE_HORAUD

    with pytest.raises(RegistryCompletenessError, match='missing=.*HORAUD'):
        validate_hand_eye_registry(fake_cv)


def test_registry_preflight_rejects_new_unregistered_method():
    fake_cv = _fake_cv_module()
    fake_cv.CALIB_HAND_EYE_FUTURE = 5

    with pytest.raises(RegistryCompletenessError, match='extra=.*FUTURE'):
        validate_hand_eye_registry(fake_cv)
