from dataclasses import replace

import numpy as np
import pytest

from cleany_perception.core.models import FailureKind, InspectionFailure
from cleany_perception.core.reference_observation import ReferenceObservationConfig, observe_surface


def test_free_surface_does_not_extend_held_object_to_table(synthetic_scene):
    scene = synthetic_scene
    result = observe_surface(scene['snapshot'], scene['mask'], scene['transform'],
                             ReferenceObservationConfig())
    # Observed surface at z=.2, NOT supported-box center z=.1.
    assert result.center[2] == pytest.approx(.2)
    assert result.extent[2] == pytest.approx(0.)
    assert result.mask_pixels == len(result.points) == 2000
    assert result.valid_depth_fraction == 1.


@pytest.mark.parametrize('kind', ['empty', 'whole_image', 'border', 'shape', 'nonbool'])
def test_invalid_reference_mask_fails_closed(synthetic_scene, kind):
    s = synthetic_scene
    mask = s['mask'].copy()
    if kind == 'empty':
        mask[:] = False
    elif kind == 'whole_image':
        mask[:] = True
    elif kind == 'border':
        mask[0, 20] = True
    elif kind == 'shape':
        mask = mask[:4]
    elif kind == 'nonbool':
        mask = mask.astype(float)
    with pytest.raises(InspectionFailure) as failure:
        observe_surface(s['snapshot'], mask, s['transform'], ReferenceObservationConfig())
    assert failure.value.kind == FailureKind.MASK


@pytest.mark.parametrize('invalid', [np.nan, np.inf, 0., 4.])
def test_current_invalid_depth_cannot_use_reference_depth(synthetic_scene, invalid):
    s = synthetic_scene
    depth = s['snapshot'].depth_m.copy()
    depth[s['mask']] = invalid
    with pytest.raises(InspectionFailure) as failure:
        observe_surface(replace(s['snapshot'], depth_m=depth), s['mask'], s['transform'],
                        ReferenceObservationConfig())
    assert failure.value.kind == FailureKind.DEPTH


@pytest.mark.parametrize('kwargs', [dict(minimum_points=0), dict(border_margin_px=-1),
    dict(maximum_mask_fraction=float('nan')), dict(minimum_valid_depth_fraction=0),
    dict(minimum_depth_m=3.), dict(trim_fraction=.5)])
def test_reference_configuration_rejects_invalid_limits(kwargs):
    with pytest.raises(ValueError):
        ReferenceObservationConfig(**kwargs)
