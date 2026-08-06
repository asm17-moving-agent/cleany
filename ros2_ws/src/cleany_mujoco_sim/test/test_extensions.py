from dataclasses import FrozenInstanceError

import pytest

from cleany_mujoco_sim.extensions import MujocoSimulationContext


def test_simulation_context_exposes_native_handles_as_frozen_fields(
    model_data,
):
    model, data = model_data
    context = MujocoSimulationContext(model=model, data=data)

    assert context.model is model
    assert context.data is data
    with pytest.raises(FrozenInstanceError):
        context.model = model
