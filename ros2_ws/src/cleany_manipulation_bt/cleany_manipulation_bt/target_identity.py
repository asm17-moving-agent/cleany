"""기존 시뮬레이션 결과 평가기를 위한 명시적 관측 label 별칭입니다.

별칭은 같은 지원 물체 유형을 가리킬 뿐 처분 권한을 부여하지 않습니다.
모델 분류, 유일성과 기하 연속성은 별도로 충족해야 합니다.
"""

VERIFICATION_LABELS = {
    'cup': 'cup',
    'paper cup': 'cup',
    'disposable paper cup': 'cup',
    'crumpled tissue': 'crumpled tissue',
    'mouse': 'mouse',
    'computer mouse': 'mouse',
    'wireless mouse': 'mouse',
    'lego brick': 'lego brick',
}

TRASH_LABELS = ('cup', 'paper cup', 'disposable paper cup', 'crumpled tissue')
LOST_ITEM_LABELS = ('mouse', 'computer mouse', 'wireless mouse', 'lego brick')


def label_key(label: str) -> str:
    normalized = ' '.join(label.strip().lower().split())
    return VERIFICATION_LABELS.get(normalized, normalized)
