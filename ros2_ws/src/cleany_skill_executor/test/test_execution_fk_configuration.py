"""Create execution clients only; no controller or motor commands are sent."""
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline


@pytest.mark.parametrize('enabled', [False, True])
def test_pipeline_wires_verified_fk_and_timing_parameters(enabled):
    context = Context()
    rclpy.init(context=context)
    node = None
    try:
        node = PregraspPipeline(mode='hardware', context=context, parameter_overrides=[
            Parameter('execution_use_verified_local_fk', value=enabled),
            Parameter('execution_timing_enabled', value=enabled)])
        assert node.motion._config.use_verified_local_fk is enabled
        assert node._execution_timing_enabled is enabled
        assert node.motion._config.validate_motion_paths
        assert not node.motion._verified_fk_arms
    finally:
        if node is not None:
            node.scene_monitor.close()
            node.destroy_node()
        rclpy.shutdown(context=context)
