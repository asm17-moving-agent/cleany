"""System Supervisor에 보고하는 의미상 인식 준비 상태입니다."""

AVAILABLE, UNAVAILABLE, UNKNOWN = 'AVAILABLE', 'UNAVAILABLE', 'UNKNOWN'


def perception_readiness(models_ready: bool, latest_stamp_ns: int | None, now_ns: int,
                         maximum_age_s: float) -> tuple[str, str]:
    """카메라 프레임만으로는 준비 완료가 아니며 모델 로드와 최신 RGB-D가 모두 필요합니다.

    원격 분류기 연결은 여기서 검사하지 않고 요청별로 보고합니다.
    """
    if not models_ready:
        return UNKNOWN, 'local models not loaded (enable preload_models for readiness reporting)'
    if latest_stamp_ns is None:
        return UNKNOWN, 'no synchronized RGB-D snapshot received yet'
    age = (now_ns - latest_stamp_ns) / 1e9
    if age < 0 or age > maximum_age_s:
        return UNAVAILABLE, f'synchronized RGB-D snapshot age {age:.2f}s outside 0..{maximum_age_s:.2f}s'
    return AVAILABLE, 'models loaded and fresh synchronized RGB-D input'
