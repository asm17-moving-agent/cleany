"""Observable operations within the existing execution stages."""

from dataclasses import dataclass

from .models import Stage


@dataclass(frozen=True)
class Step:
    node: str
    label: str


STAGE_STEPS: dict[Stage, tuple[Step, ...]] = {
    Stage.VALIDATING: (Step('ValidateGoal', '모델과 실행 준비 확인'),),
    Stage.PREPARING_TARGET: (
        Step('PrepareTarget', '대상 관측 확인'),
        Step('ReconstructTarget', '선택 물체 3D 복원'),
        Step('GenerateGrasp', '파지 후보 생성'),
        Step('SelectArmAndPath', '팔과 경로 결정'),
    ),
    Stage.APPROACHING: (
        Step('MoveToPregrasp', '잡기 전 위치 이동'), Step('ApproachObject', '물체 접근'),
    ),
    Stage.GRASPING: (
        Step('GraspObject', '그리퍼 닫기'), Step('ConfirmGrasp', '파지 확인'),
    ),
    Stage.LIFTING: (
        Step('LiftObject', '물체 들어 올리기'), Step('ConfirmHeld', '보유 상태 확인'),
    ),
    Stage.TRANSPORTING: (Step('CarryObject', '수거함 이동'),),
    Stage.PLACING: (
        Step('CheckPlacementTarget', '놓을 위치 확인'),
        Step('OpenGripperAtDestination', '그리퍼 열기'), Step('ConfirmRelease', '물체 이탈 확인'),
    ),
    Stage.RETURNING_ARM: (Step('ReturnArm', '팔 복귀'),),
    Stage.VERIFYING_PLACEMENT: (Step('VerifyPlacedObject', '수거함 내부 확인'),),
}

STEP_BY_ID = {step.node: step for steps in STAGE_STEPS.values() for step in steps}
