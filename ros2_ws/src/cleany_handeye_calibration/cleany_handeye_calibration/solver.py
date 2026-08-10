"""OpenCV eye-in-hand solver registry and failure-isolated adapter."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Integral
import time
from typing import Any, Sequence

import cv2
import numpy as np

from cleany_handeye_calibration.models import (
    CalibrationSample,
    SampleSplit,
)
from cleany_handeye_calibration.transforms import RigidTransform


MINIMUM_HAND_EYE_SAMPLE_COUNT = 3


class HandEyeMethod(str, Enum):
    TSAI = 'tsai'
    PARK = 'park'
    HORAUD = 'horaud'
    ANDREFF = 'andreff'
    DANIILIDIS = 'daniilidis'


class HandEyeMethodCategory(str, Enum):
    SEPARABLE = 'rotation_translation_separable'
    SIMULTANEOUS = 'rotation_translation_simultaneous'


@dataclass(frozen=True, slots=True)
class HandEyeMethodSpec:
    method: HandEyeMethod
    opencv_symbol: str
    category: HandEyeMethodCategory


HAND_EYE_METHOD_REGISTRY = (
    HandEyeMethodSpec(
        HandEyeMethod.TSAI,
        'CALIB_HAND_EYE_TSAI',
        HandEyeMethodCategory.SEPARABLE,
    ),
    HandEyeMethodSpec(
        HandEyeMethod.PARK,
        'CALIB_HAND_EYE_PARK',
        HandEyeMethodCategory.SEPARABLE,
    ),
    HandEyeMethodSpec(
        HandEyeMethod.HORAUD,
        'CALIB_HAND_EYE_HORAUD',
        HandEyeMethodCategory.SEPARABLE,
    ),
    HandEyeMethodSpec(
        HandEyeMethod.ANDREFF,
        'CALIB_HAND_EYE_ANDREFF',
        HandEyeMethodCategory.SIMULTANEOUS,
    ),
    HandEyeMethodSpec(
        HandEyeMethod.DANIILIDIS,
        'CALIB_HAND_EYE_DANIILIDIS',
        HandEyeMethodCategory.SIMULTANEOUS,
    ),
)


class RegistryCompletenessError(RuntimeError):
    """Installed OpenCV hand-eye methods differ from the declared registry."""


class InvalidHandEyeDataset(ValueError):
    """Samples do not satisfy the eye-in-hand input frame contract."""


class HandEyeFailure(str, Enum):
    METHOD_EXCEPTION = 'method_exception'
    MALFORMED_OUTPUT = 'malformed_output'
    INVALID_TRANSFORM = 'invalid_transform'


@dataclass(frozen=True, slots=True)
class HandEyeFrameConvention:
    base_frame: str
    gripper_frame: str
    camera_frame: str
    target_frame: str

    def __post_init__(self) -> None:
        values = (
            self.base_frame,
            self.gripper_frame,
            self.camera_frame,
            self.target_frame,
        )
        if any(
            not isinstance(value, str)
            or not value
            or value != value.strip()
            for value in values
        ):
            raise ValueError('hand-eye frame names must be non-empty strings')
        if len(set(values)) != len(values):
            raise ValueError('hand-eye frame names must be distinct')


@dataclass(frozen=True, slots=True)
class HandEyeTransformValidityPolicy:
    """Robot-specific physical bounds for accepting a solver transform."""

    max_translation_norm_m: float

    def __post_init__(self) -> None:
        if isinstance(self.max_translation_norm_m, bool):
            raise ValueError(
                'max_translation_norm_m must be a positive finite value'
            )
        try:
            maximum = float(self.max_translation_norm_m)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'max_translation_norm_m must be a positive finite value'
            ) from error
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError(
                'max_translation_norm_m must be a positive finite value'
            )
        object.__setattr__(self, 'max_translation_norm_m', maximum)


DEFAULT_HAND_EYE_FRAMES = HandEyeFrameConvention(
    base_frame='base_link',
    gripper_frame='left_gripper_frame',
    camera_frame='left_wrist_rgb_optical_frame',
    target_frame='charuco_target',
)


@dataclass(frozen=True, slots=True)
class HandEyeResult:
    method: HandEyeMethod
    opencv_symbol: str
    valid: bool
    gripper_T_camera: RigidTransform | None
    failure_reason: HandEyeFailure | None
    failure_detail: str | None
    runtime_ms: float


@dataclass(frozen=True, slots=True)
class _OpenCvHandEyeInputs:
    rotations_gripper_to_base: tuple[np.ndarray, ...]
    translations_gripper_to_base: tuple[np.ndarray, ...]
    rotations_target_to_camera: tuple[np.ndarray, ...]
    translations_target_to_camera: tuple[np.ndarray, ...]

    def copied_lists(self) -> tuple[list[np.ndarray], ...]:
        return (
            [
                np.array(value, copy=True)
                for value in self.rotations_gripper_to_base
            ],
            [
                np.array(value, copy=True)
                for value in self.translations_gripper_to_base
            ],
            [
                np.array(value, copy=True)
                for value in self.rotations_target_to_camera
            ],
            [
                np.array(value, copy=True)
                for value in self.translations_target_to_camera
            ],
        )


def validate_hand_eye_registry(
    cv_module: Any = cv2,
) -> dict[HandEyeMethod, int]:
    """Resolve all declared methods and reject missing or newly added ones."""

    declared_symbols = tuple(
        method_spec.opencv_symbol
        for method_spec in HAND_EYE_METHOD_REGISTRY
    )
    declared_methods = tuple(
        method_spec.method for method_spec in HAND_EYE_METHOD_REGISTRY
    )
    if len(set(declared_symbols)) != len(declared_symbols) or len(
        set(declared_methods)
    ) != len(declared_methods):
        raise RegistryCompletenessError(
            'hand-eye registry contains duplicate methods or symbols'
        )

    expected = set(declared_symbols)
    installed = {
        name
        for name in dir(cv_module)
        if name.startswith('CALIB_HAND_EYE_')
    }
    missing = sorted(expected - installed)
    extra = sorted(installed - expected)
    if missing or extra:
        raise RegistryCompletenessError(
            f'OpenCV hand-eye registry mismatch: missing={missing}, '
            f'extra={extra}'
        )

    resolved: dict[HandEyeMethod, int] = {}
    for method_spec in HAND_EYE_METHOD_REGISTRY:
        value = getattr(cv_module, method_spec.opencv_symbol)
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise RegistryCompletenessError(
                f'{method_spec.opencv_symbol} must be an integer constant'
            )
        resolved[method_spec.method] = int(value)
    return resolved


def _validate_sample_frames(
    sample: CalibrationSample,
    frames: HandEyeFrameConvention,
) -> None:
    if sample.base_T_gripper.parent_frame != frames.base_frame:
        raise InvalidHandEyeDataset(
            f'{sample.sample_id} base_T_gripper parent must be '
            f'{frames.base_frame}'
        )
    if sample.base_T_gripper.child_frame != frames.gripper_frame:
        raise InvalidHandEyeDataset(
            f'{sample.sample_id} base_T_gripper child must be '
            f'{frames.gripper_frame}'
        )
    if sample.camera_T_target.parent_frame != frames.camera_frame:
        raise InvalidHandEyeDataset(
            f'{sample.sample_id} camera_T_target parent must be '
            f'{frames.camera_frame}'
        )
    if sample.camera_T_target.child_frame != frames.target_frame:
        raise InvalidHandEyeDataset(
            f'{sample.sample_id} camera_T_target child must be '
            f'{frames.target_frame}'
        )


def _prepare_opencv_inputs(
    samples: Sequence[CalibrationSample],
    frames: HandEyeFrameConvention,
) -> _OpenCvHandEyeInputs:
    sample_tuple = tuple(samples)
    if len(sample_tuple) < MINIMUM_HAND_EYE_SAMPLE_COUNT:
        raise InvalidHandEyeDataset(
            f'hand-eye calibration requires at least '
            f'{MINIMUM_HAND_EYE_SAMPLE_COUNT} samples'
        )
    for sample in sample_tuple:
        if not isinstance(sample, CalibrationSample):
            raise InvalidHandEyeDataset(
                'hand-eye inputs must be CalibrationSample instances'
            )
        if sample.split is not SampleSplit.CALIBRATION:
            raise InvalidHandEyeDataset(
                f'{sample.sample_id} is not in the calibration split'
            )
        _validate_sample_frames(sample, frames)

    return _OpenCvHandEyeInputs(
        rotations_gripper_to_base=tuple(
            np.array(sample.base_T_gripper.rotation_array(), copy=True)
            for sample in sample_tuple
        ),
        translations_gripper_to_base=tuple(
            np.array(
                sample.base_T_gripper.translation_array().reshape(3, 1),
                copy=True,
            )
            for sample in sample_tuple
        ),
        rotations_target_to_camera=tuple(
            np.array(sample.camera_T_target.rotation_array(), copy=True)
            for sample in sample_tuple
        ),
        translations_target_to_camera=tuple(
            np.array(
                sample.camera_T_target.translation_array().reshape(3, 1),
                copy=True,
            )
            for sample in sample_tuple
        ),
    )


def _failure_result(
    method_spec: HandEyeMethodSpec,
    reason: HandEyeFailure,
    detail: str,
    runtime_ms: float,
) -> HandEyeResult:
    return HandEyeResult(
        method=method_spec.method,
        opencv_symbol=method_spec.opencv_symbol,
        valid=False,
        gripper_T_camera=None,
        failure_reason=reason,
        failure_detail=detail,
        runtime_ms=runtime_ms,
    )


def _run_method(
    method_spec: HandEyeMethodSpec,
    method_constant: int,
    inputs: _OpenCvHandEyeInputs,
    frames: HandEyeFrameConvention,
    validity_policy: HandEyeTransformValidityPolicy,
    cv_module: Any,
) -> HandEyeResult:
    method_inputs = inputs.copied_lists()
    start_ns = time.perf_counter_ns()
    try:
        output = cv_module.calibrateHandEye(
            method_inputs[0],
            method_inputs[1],
            method_inputs[2],
            method_inputs[3],
            method=method_constant,
        )
    except Exception as error:
        runtime_ms = (time.perf_counter_ns() - start_ns) / 1.0e6
        return _failure_result(
            method_spec,
            HandEyeFailure.METHOD_EXCEPTION,
            f'{type(error).__name__}: {error}',
            runtime_ms,
        )
    runtime_ms = (time.perf_counter_ns() - start_ns) / 1.0e6

    if not isinstance(output, tuple) or len(output) != 2:
        return _failure_result(
            method_spec,
            HandEyeFailure.MALFORMED_OUTPUT,
            'calibrateHandEye must return a two-element tuple',
            runtime_ms,
        )
    try:
        rotation = np.asarray(output[0], dtype=np.float64)
        translation = np.asarray(output[1], dtype=np.float64)
    except (TypeError, ValueError) as error:
        return _failure_result(
            method_spec,
            HandEyeFailure.MALFORMED_OUTPUT,
            str(error),
            runtime_ms,
        )
    if rotation.shape != (3, 3) or translation.size != 3:
        return _failure_result(
            method_spec,
            HandEyeFailure.MALFORMED_OUTPUT,
            f'unexpected shapes R={rotation.shape}, t={translation.shape}',
            runtime_ms,
        )
    if not np.all(np.isfinite(rotation)) or not np.all(
        np.isfinite(translation)
    ):
        return _failure_result(
            method_spec,
            HandEyeFailure.INVALID_TRANSFORM,
            'calibrateHandEye returned non-finite values',
            runtime_ms,
        )
    translation_norm_m = math.hypot(
        *(float(value) for value in translation.reshape(3))
    )
    if translation_norm_m > validity_policy.max_translation_norm_m:
        return _failure_result(
            method_spec,
            HandEyeFailure.INVALID_TRANSFORM,
            'calibrateHandEye translation norm exceeds the configured '
            f'physical bound: {translation_norm_m:.9g} m > '
            f'{validity_policy.max_translation_norm_m:.9g} m',
            runtime_ms,
        )
    try:
        estimate = RigidTransform(
            parent_frame=frames.gripper_frame,
            child_frame=frames.camera_frame,
            rotation_matrix=rotation,
            translation_m=translation.reshape(3),
        )
    except ValueError as error:
        return _failure_result(
            method_spec,
            HandEyeFailure.INVALID_TRANSFORM,
            str(error),
            runtime_ms,
        )
    return HandEyeResult(
        method=method_spec.method,
        opencv_symbol=method_spec.opencv_symbol,
        valid=True,
        gripper_T_camera=estimate,
        failure_reason=None,
        failure_detail=None,
        runtime_ms=runtime_ms,
    )


def solve_all_hand_eye_methods(
    samples: Sequence[CalibrationSample],
    *,
    validity_policy: HandEyeTransformValidityPolicy,
    frame_convention: HandEyeFrameConvention = DEFAULT_HAND_EYE_FRAMES,
    cv_module: Any = cv2,
) -> tuple[HandEyeResult, ...]:
    """Run all five methods on the same validated calibration pose pairs."""

    inputs = _prepare_opencv_inputs(samples, frame_convention)
    constants = validate_hand_eye_registry(cv_module)
    return tuple(
        _run_method(
            method_spec,
            constants[method_spec.method],
            inputs,
            frame_convention,
            validity_policy,
            cv_module,
        )
        for method_spec in HAND_EYE_METHOD_REGISTRY
    )
