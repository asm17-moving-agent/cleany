"""Validate shared-provider freshness and observation endpoint requirements."""
import math
import pytest
from shared_perception_readiness import (ENDPOINTS, TOPICS, PROVIDER_PARAMETERS,
                                         pending_inputs, provider_issues)


def test_all_readonly_services_and_current_rgbd_are_required():
    stamps = dict.fromkeys(TOPICS, 99.5)
    services = dict.fromkeys(ENDPOINTS, True)
    assert pending_inputs(100., stamps, services) == []
    services[ENDPOINTS[2]] = False
    assert pending_inputs(100., stamps, services) == [ENDPOINTS[2]]


@pytest.mark.parametrize('stamp', [98., 101., math.nan, math.inf])
def test_stale_future_or_nonfinite_camera_frames_cannot_pass(stamp):
    stamps = {TOPICS[0]: stamp, TOPICS[1]: 99.5}
    assert pending_inputs(100., stamps, dict.fromkeys(ENDPOINTS, True)) == [TOPICS[0]]


def test_missing_camera_topics_are_reported():
    assert pending_inputs(100., {}, dict.fromkeys(ENDPOINTS, True)) == list(TOPICS)


def test_discovery_without_parameter_rpc_cannot_pass():
    assert provider_issues(None) == ['/perception_inspector/get_parameters (RPC)']


def test_shared_provider_requires_grasp_frame_and_observation_services():
    assert provider_issues(dict(PROVIDER_PARAMETERS)) == []
    assert len(provider_issues({})) == 3
    for name in PROVIDER_PARAMETERS:
        values = dict(PROVIDER_PARAMETERS)
        values[name] = 'camera_link' if name == 'target_frame' else False
        assert len(provider_issues(values)) == 1
