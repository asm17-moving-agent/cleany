# 저장소 품질 점검 — 2026-09-14

## 판정과 범위

**개발을 이어갈 기반은 갖췄지만 통합 완료 판정은 보류한다.** 순수 core·모델 정합성·
실패 차단 검사는 폭넓게 통과한다. 새 CAD의 합성 파지·Hand-eye 실행 회귀는 복구했고,
바퀴 순간 속도 추종에는 실패가 남아 있다. 단위 테스트 수를 물리 수거 성공률로 해석하지 않는다.

점검 기준은 `bac2a16` 위의 미커밋 작업 전체다. 작업 시작 시 69개 tracked 파일 변경과
추가 CAD·성능 프로필·발표 도구·실행 기록이 있었다. 변경 코드·설정·manifest·README,
13개 ROS 패키지의 빌드·테스트, Python 정적 검사, 로컬 문서 링크·구문을 확인했다.
Mesh·이미지·발표 출력물과 KB submodule은 재생성하거나 수정하지 않았다.

| 영역 | 평가 | 근거와 한계 |
| --- | --- | --- |
| Core·계약 | 양호 | 순수 Python core와 ROS adapter 분리, 명시적 메시지·서비스, 접촉·충돌·시간 검사 회귀 |
| 검증 재현성 | 개선 | 외부 pytest 플러그인 차단, 프레임워크 명시, 누락된 테스트·CMake 재등록. 전체 실행 실패는 계속 보고 |
| 모델 통합 | 부분 개선 | URDF/MJCF 정합성, grasp·대체 후보 pregrasp·Hand-eye 실제 controller 회귀 통과. 바퀴 순간 속도 오차는 기준 초과 |
| 문서 | 개선 | 루트 구현 현황, 관찰/자동검증 구분, 최신 수거함·물체·실행 명령 반영. 과거 기록은 날짜별 이력으로 분리 |
| 유지보수 | 보완 필요 | 중복·미사용 코드와 launch 보정값 복제 제거, 작은 테스트 통합·중복 검사 축소. 조작 coordinator는 여전히 크고 ROS 결합이 강함 |
| 전체 수거·실물 | 검증 미완료 | 기록상 개별 배치 성공은 있으나 연속 전체 정리 성공을 입증하지 못함. Jetson·모터 인수검사 별도 |

## 이번 정리에서 수정한 문제

1. **검사가 실행되지 않아도 통과처럼 보이는 경로.** `tests_require`가 무시되며 일부
   Python 패키지가 `Ran 0 tests`로 종료됐고 사용자 `anyio` 플러그인은 system pytest와
   충돌했다. test extra를 정리하고 Make에서 pytest를 명시 선택하며 외부 플러그인 자동
   로드를 차단했다. `make test`는 CMake 재설정도 포함한다. MuJoCo 전체 검사,
   그리퍼 안정화 검사, study-cafe collision CTest도 검증 경로에 포함했다.
2. **고정 결합부의 자기충돌.** 일반 MoveIt에서 `base_link`–`top_base_link`가 충돌해
   정상 계획도 거부됐다. 실제 URDF의 fixed parent/child 한 쌍에 `Adjacent` 예외를
   추가했다. 기둥–팔 충돌의 포괄 제외와 구분하며 해당 경계 회귀 검사도 추가했다.
3. **잘못된 놓기 위치.** `NaN`은 비교식만으로 거부되지 않는다. release 직전 물체 중심과
   반경의 유한성·양수 조건을 명시하고 비정상 입력에서는 개방 전 실패하도록 검사했다.
4. **오류 원인 유실.** 파지 실패와 depth boost 해제 실패가 겹치면 정리 오류가 최초
   동작 오류를 가렸다. 두 오류를 기록하되 원래 동작 실패를 유지한다.
5. **불필요한 코드.** 중복 simulator 메서드/import, 사용하지 않는 의자 생성기·
   변수를 제거했다. 파지 깊이의 selector/executor launch mapping을 공유했다. 벤치마크 RTF는
   실제 모델 timestep을 사용한다. runtime 테스트의 실패 로그 표기도 현재 node에 맞췄다.
6. **문서·산출물 혼재.** 미구현 scaffold를 구현 완료처럼 소개하던 설명, 지갑/기존 배치,
   오래된 속도·초기 개방·완료 조건을 정정했다. 발표 원본·출력은 로컬에 보존하되
   `tools/presentation/`, `output/`, 발표 초안 문서를 모두 Git에서 제외했다.
   공통 테스트 명령과 구현 README도 제외된 발표 파일에 의존하지 않도록 정리했다.
7. **CAD 이관에서 누락된 합성 목표 좌표.** 왼쪽 어깨가
   `(+0.0263, +0.029917, +0.036297) m` 이동했지만 reach 데모와 최근접 후보 시험은
   예전 목표를 사용했다. 후보와 렌더링 box를 함께 옮겨 기존 팔 상대 자세를 유지했다.
   FK 위치 일치, 방향을 포함한 IK, MoveIt 충돌·계획, MuJoCo 실행 검사를 통과했다.
   상태 marker도 설정된 목표를 따라가도록 수정했다.
8. **유효하지 않은 Hand-eye 근거.** 기존 관절각은 현재 장면에서 fixture clearance가
   0이고 최소 카메라 깊이도 0.168 m로 0.18 m 기준에 못 미쳤다. 현재 모델에서
   clearance 0.14179 m, 보드 전체 가시성, 24개 ChArUco corner와 유효 PnP를
   측정한 자세로 교체했다. 저장된 자세·FK·clearance의 정합성 검사도 추가했다.
   기존 0.10 m 여유와 관절 일치·settling·영상 시각의 feedback 조건은 유지했다.

## 테스트 축소

발표를 제외한 Python 테스트 파일은 **144 → 127개**, 소스는 **29,350 → 28,877줄**,
`test_` 함수는 **945 → 922개**로 줄였다. 매개변수별 실행 건수와 함수 수는 다르다.
작은 파일 15개는 관련 기능의 suite로 통합하고, 소스 문자열·기본값 중심 파일 2개와
중복 검사를 제거했다. 분류 loop의 반복 준비 코드는 공통 harness로 모았다.
launch 설정 전달은 실제 launch 인자를 평가해 selector·coordinator 간 정합성을 검사한다.
접촉·충돌·실패 복구·실제 controller 실행 검사는 유지했다.

`origin/main`과 현재 브랜치의 공통 조상 기준으로 현재 작업을 모두 커밋한다고 가정하면
변경 경로는 **348 → 309개**, 그중 테스트 코드는 **87 → 70개**다.
후자는 삭제되는 기존 테스트 파일 1개도 포함한다. 발표 원본 23개가 변경 대상에서 빠졌다.
실제 index에 staging하거나 커밋하지 않고 임시 index로 계산했다.

## 검증 결과

- `DISPLAY=:0 make test`: **13패키지 빌드 성공**, colcon 집계
  **1,435개 중 오류 0, 실패 1, skip 3**. 전체 명령은 아래 바퀴 회귀 때문에
  실패 종료했다. 일반 MoveIt mock 계획·실행, 보정한 파지·Hand-eye는 전체 실행에서도 통과했다.
- `make test-grasp-pregrasp`: **Python 912개 + C++/CTest 집계 45개 통과**, 9패키지 빌드 성공.
- `python3 -m pytest -q tools/test_gazebo_profile.py containers/vision/test`
  (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`): **23개 통과, 2개 환경 조건부 skip**.
  전체 Make 명령이 ROS 실패에서 중단되므로 이 도구 검사는 별도로 실행했다.
- `python3 -m flake8 ros2_ws/src tools containers/vision --exclude tools/presentation --select F,E9`: 통과.
- 이관 후 집중 실행: Hand-eye fixture 정합성·실제 샘플 기록,
  grasp 실행·최근접 실패 후 대체 후보 실행 **4개 통과** (67.83초).
- 변경에 가까운 최소 검사부터 실행한 뒤 전체 workspace와 영향받는 실제 motion 경로를 확인했다.
- 최초 품질 점검에서 Markdown 링크·heading과 Python·XML·xacro·SRDF·YAML 구문을 확인했다.
  테스트 축소 후 Python 구문·로컬 Markdown 링크와 `git diff --check`를 다시 확인했다.

최종 workspace·이관 실행·주행 측정·정적 검사 로그는 로컬 Git 제외 경로
`artifacts/quality_review_20260914/`에 보관했다. 테스트 축소 후 전체·집중 검사 로그와
변경 수 집계는 `artifacts/test_reduction_20260914/`, 축소 전 테스트 사본은
`artifacts/test_reduction_before_20260914/`에 있다. colcon 수치는 CTest 결과도 포함하는
집계이며 집중 검사와 중복되므로 두 실행의 건수를 더하지 않는다.

처음 디스플레이 없이 실행한 카메라 timeout은 유효한 `DISPLAY=:0`를 전달한 검사에서
해소됐다. vendor 카메라는 headless에서도 X display가 필요하다. 단위 검사 성공과
환경 문제, 모델·fixture 회귀 실패를 구분한다.

## 남은 우선순위

| 우선순위 | 항목 | 확인한 현상·다음 검증 |
| --- | --- | --- |
| 높음 | 바퀴 추종 회귀 | `test_drive_tracks_forward_targets_and_stops`: 2초 시점 목표 4.0 rad/s, 후륜 약 3.756/3.757 rad/s로 ±0.2 기준 실패. 접촉에 따른 속도 진동과 제어 응답을 확인하고 기존 순간 오차 기준을 만족하는 조정 필요 |
| 높음 | 전체 분류 완료 근거 | 기본 `sorting_verify_placement=false`는 작업자 관찰 모드. `mission_complete_unverified`와 자동 검증 성공을 분리하고 검증을 켠 연속 4물체 시험 필요 |
| 중간 | 시뮬레이션 충돌 가정 | `robot_top_bins.yaml`의 `simulation_ignore_mast_collision=true`는 기둥 물리 접촉과 로봇 링크 간 충돌 제외. 실물 운용을 위한 모델·경로 검증과 구분 필요 |
| 중간 | 유지보수와 객체 수량 검사 | 큰 coordinator를 관측·운반·완료 상태 책임으로 나눌 여지가 있음. 완료 수량 비교는 identity 추적이 아니므로 재검출·오분류·가림 시험 필요 |
| 중간 | 과거 종료 crash | 수거 실행 기록의 MoveIt/ros2_control 종료 crash는 별도 재현 필요. 이번 합성 회귀 검사가 해결을 입증하지 않음 |

바퀴 응답을 별도 측정한 결과, 1.5–2.0초 후륜 평균은 약 4.010 rad/s지만
순간 범위는 약 3.738–4.190 rad/s였다. 2초 이동 거리는 0.498 m,
최대 전압은 8.267 V로 10.8 V 한계 이내였다. 같은 시점에서 정지 명령을 주고
1초 후 측정한 최대 속도는 0.0187 rad/s로 기존 0.05 기준을 만족했다.
따라서 관찰 결과는 단순한 지속 저속이나 전압 포화보다 속도 진동을 가리킨다.
정확한 원인과 gain 변경은 추가 동역학 검증이 필요하며 평균값으로 기존 순간 오차
시험을 대체하지 않았다.

실패를 없애기 위해 관절·접촉·clearance 허용치를 낮추거나 기존 시험을 skip하지 않았다.
이 점검은 실물 모터 구동, Jetson GPU/AnyGrasp 라이선스 인수검사, 외부 Gemini API를
사용한 새 수거 실행을 포함하지 않는다. 발표 자료도 다시 렌더링하지 않았다.

## 참고

- [현재 구현과 개발 진입점](../README.md)
- [Workspace 테스트 명령](../ros2_ws/README.md)
- [날짜별 수거 실행 기록](SORTING_PIPELINE_TRIALS_20260909.md)
