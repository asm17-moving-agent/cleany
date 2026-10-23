"""Semantic Perception readiness reported to the System Supervisor."""

AVAILABLE, UNAVAILABLE, UNKNOWN = 'AVAILABLE', 'UNAVAILABLE', 'UNKNOWN'


def perception_readiness(models_ready: bool, latest_stamp_ns: int | None, now_ns: int,
                         maximum_age_s: float) -> tuple[str, str]:
    """Camera frames alone do not prove readiness; loaded models and fresh RGB-D both do.

    Remote classifier reachability is not probed here; it is reported per request.
    """
    if not models_ready:
        return UNKNOWN, 'local models not loaded (enable preload_models for readiness reporting)'
    if latest_stamp_ns is None:
        return UNKNOWN, 'no synchronized RGB-D snapshot received yet'
    age = (now_ns - latest_stamp_ns) / 1e9
    if age < 0 or age > maximum_age_s:
        return UNAVAILABLE, f'synchronized RGB-D snapshot age {age:.2f}s outside 0..{maximum_age_s:.2f}s'
    return AVAILABLE, 'models loaded and fresh synchronized RGB-D input'
