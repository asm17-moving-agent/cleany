# MuJoCo 분리 수거 성능 작업 기록 (2026-09-28)

범위는 앞선 성능 점검의 Task 5–12다. Task 13 이후는 변경하지 않았다.

| Task | 적용 내용 | 상태 |
| --- | --- | --- |
| 5 | sorting 기본 실행을 headless, RViz/image viewer 비활성화로 변경 | 완료 |
| 6 | 검출 중 head 10 Hz, 검출 뒤 head 2 Hz 전환; wrist 선택과 grasp depth boost 유지 | 완료 |
| 7 | depth 값을 행 단위로 환산 | 완료 |
| 8 | 카메라별 ROS 메시지와 640픽셀 depth 임시 행 버퍼 재사용 | 완료 |
| 9 | viewer 비활성 시 debug overlay 생성·재발행 생략 | 완료 |
| 10 | cloud 구독자가 없으면 depth 투영 생략 | 일부 완료 |
| 11 | packed RGB/32FC1 snapshot은 메시지 저장소를 읽기 전용으로 참조하고 PIN 시 재복사 생략 | 완료 |
| 12 | wrist HANDOFF 뒤 선택하지 않은 팔의 frame cache 제거 및 새 frame 무시 | 완료 |

Task 10의 작업 단계별 중지는 보류했다. sorting에서는 MoveIt OctoMap이 cloud를
계속 구독하고, coordinator가 최신 cloud와 지도 반영 시각을 2초 이내로 검증한다.
단순히 동작 단계에서 생성을 끄면 충돌 지도 최신성 조건을 깨뜨린다. 다음 구현에서는
요청형 갱신과 지도 반영 확인을 함께 설계해야 한다.

검증: observer 20개 테스트 통과, perception/skill executor 745개 통과·1개 skip.
이후 cloud 구독자 조건 추가에 대한 19개 테스트 통과. `DISPLAY=:0`에서 MuJoCo GUI
창과 head RGB-D, MoveIt depth integration을 확인했다. head 유휴 목표 2 Hz에서
ROS header simulation timestamp 기준 8프레임 평균 1.98 Hz, wall time 기준
1.10 Hz였다. 현재 VM의 simulation speed가 wall time보다 낮아 두 수치는 다르다.

640×480 균일 depth를 200프레임 반복한 별도 C++ `-O2` microbenchmark에서 기존
픽셀별 변환·복사는 프레임당 중앙값 0.220 ms, 행 변환·복사는 0.093 ms였다
(7회 프로세스 실행). 이는 depth 환산 코드만의 합성 측정이며 전체 ROS/렌더링
속도 향상을 뜻하지 않는다. packed RGB와 depth의 불필요한 복사 최대 약 2.05 MiB를
snapshot 생성마다 줄이고, PIN 시 같은 크기의 추가 복사를 줄였다.

GUI sorting은 YOLOE로 `lego brick`을 검출했지만 분류 category가 없어 `review`에서
종료됐다. 이번 실행에는 Gemini API key가 없어 분류 입력을 제공하지 못했다.
따라서 집기·놓기 전체 성공이나 전후 전체 파이프라인 속도 개선은 검증하지 못했다.
45초 진단 실행의 종료 신호가 남긴 ROS `KeyboardInterrupt` 로그는 시간 제한 종료다.
