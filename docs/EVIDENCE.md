# 검증 범위와 결과

**작업 요청부터 현장 좌석 이동, 책상 정리, 복귀, 결과 보고까지 이어지는 전체 실물 미션은 검증이 남아 있습니다.** 아래 표는 각 환경에서 관찰한 결과와 그 결과가 뒷받침하는 범위를 요약합니다.

| 기능 | 시험 환경과 관찰 결과 | 검증 범위와 근거 |
|---|---|---|
| 센서와 로봇 모델 | 실물 Jetson에서 RGB-D 입력을 수신했습니다. 관절 상태의 시각에 맞춰 헤드·손목 카메라와 파지 프레임의 TF를 조회했습니다. | 모델과 센서 연결을 확인했습니다. [RViz·TF 측정 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L7-L19) |
| 장애물 지도 | Jetson에서 기록한 점군과 TF를 재생했습니다. OctoMap 갱신 최적화 전후에 모든 재생 단계의 지도 해시가 일치했습니다. | 같은 입력에 대한 지도 보존을 확인했습니다. 현장 주행 중 장애물 회피는 별도 시험입니다. [측정 조건과 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L280-L298) |
| 거리 센서 | 단독 ESP32 보드에서 ToF 센서의 8×8 거리 프레임을 120초간 수신했습니다. | 보드의 출력과 전송 연속성을 확인했습니다. 로봇 장착 상태의 감지 범위와 정지 거리는 현장에서 측정해야 합니다. [펌웨어](../esp32/tof_vl53l7ch/README.md#실물-기록과-재생) |
| 물체 인식 | 운영 환경에서 헤드 RGB-D 한 프레임을 입력해 객체 6개가 포함된 응답을 받았습니다. | 입력부터 응답까지 연결한 시험입니다. 검출률과 3D 위치 정확도는 별도 평가가 필요합니다. [인식 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L324-L336) |
| 단일 물체 수거 | 고정 베이스 MuJoCo에서 컵 하나의 수거, 놓기, 팔 복귀와 정지를 확인했습니다. 파지 중 취소와 제어기 중단 시의 결과도 검사했습니다. | 단일 물체 시뮬레이션의 결과입니다. 여러 물체의 수거 성공률과 실물 팔 성능은 별도 평가 대상입니다. [시뮬레이션 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/ros2_ws/src/cleany_manipulation_bt/README.md?plain=1#L231-L306) |
| 미션 연동 | Gazebo/Nav2 주행과 Mock 조작 서버로 두 물체의 실행 ID, 재관찰과 미션 완료 보고를 확인했습니다. | 인식·계획·팔 동작에 모의 입력을 사용했습니다. [통합 시험 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L1022-L1044) · [재현 절차](implementation/how-to/mission.md#gazebo-주행과-mock-조작) |

현장 인수검사에서는 좌석별 접근 좌표와 장착 보정값을 확인한 뒤, 한 작업 요청의 이동·수거·복귀·결과 보고를 같은 미션 ID로 추적합니다. [보정 가이드](implementation/calibration.md) · [실행 가이드](implementation/running.md)

패키지별 검사 명령과 입력은 [검증 가이드](implementation/testing.md)에서 확인할 수 있습니다. 표에 인용한 실험의 커밋, 환경, 입력과 원본 위치는 [전체 실행 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md)에 보존되어 있습니다. 변경별 자동 검사 결과는 [GitHub Actions](https://github.com/asm17-moving-agent/cleany/actions/workflows/ci.yml)에서 확인할 수 있습니다.

## Jetson 수거 검증 기록

아래 기록은 Jetson Orin의 ROS 2 Humble에서 수집했다. 운영 측정은 `f45801bc`의 현장 변경을,
왼팔 연속 복귀 검사는 그 작업본에 새 경유점과 사전 계산 궤적을 적용한 상태를 사용했다.
원본 경로는 Jetson의 `/home/cleany/cleany-data/runs/` 아래 상대 경로다.

| 항목 | 입력과 관찰 결과 | 원본 |
|---|---|---|
| 순차 준비 | 모의 준비 검사 50개와 가상 모터 검사 4개가 통과했다. 계획과 GPU 예열이 대기하거나 실패한 동안 토크 OFF가 유지됐다. | `torque-after-warmup-H8Bhb1LV/` |
| 손목 카메라와 기본 자세 | 손목 관측을 켠 구성과 끈 구성을 모의 시험했다. 웹 준비는 명령 제어기를 준비하고 복귀 자세를 확인했으며 팔 이동을 요청하지 않았다. | `disable-wrist-cameras-hjZwFow8/`, `web-prepare-current-pose-lsNRyTZv/` |
| 피드백 전송과 보호 | UDP 적용 후 읽기 전용 118.4초에서 5,846개 모터 표본의 최고 나이는 76.0 ms였다. 앞선 반복 시험에서는 ID 5 시리얼 읽기 시간이 초과됐다. 전송 전 실패만 재시도하는 조건은 PTY에서 확인했다. | `feedback-transport-0kcSsCC8/`, `feedback-prepare-20261118-0856/` |
| 그리퍼 가속도 | 양측 그리퍼가 raw 52를 50으로 제한했다. 설정을 50으로 맞춘 뒤 가상 시리얼 검사와 실물 준비 1회가 통과했다. 준비는 75.4초가 걸렸다. | `prepare-gripper-accel-20261120.Oi08YNnn/` |
| CAD 충돌 모델 | 같은 1 cm OctoMap과 관절 표본 138개에서 이전 모델과 간소화 모델의 유효 36개, 충돌 102개 및 접촉 링크 쌍이 일치했다. 측정된 시간 차이는 실행별 편차 범위에 있었다. | `cad-arm-shells-20261119.q4YWq6h0/`, `cad-share-deploy-20261120.u51DL5te/` |
| 파지 후보와 폭 | 기록된 PlanGrasp 요청 22건에서 후보가 생성된 경우는 수정 전후 각각 18건이었다. 안경 케이스의 중앙 윗면 폭은 새 방식에서 59.3 mm, 사람이 표시한 두 선의 역투영 폭은 63.5 mm였다. 배치 IK와 후보 제한은 저장된 OctoMap으로 격리 재생했다. | `grasp-top-width-20261120/`, `grasp-center-correction-20261120/`, `grasp-ik-shortlist-20261119.n5dfbr52/` |
| 원본 접근 궤적 | 시작점이 맞을 때 계획한 위치, 시간, 속도와 가속도를 사용했다. 시리얼 에뮬레이터는 보호 정지했고, 격리 GenericSystem의 JointTrajectoryController는 원본 86점 궤적을 받아 완료했다. | `native-rrt-live-deploy-20261123.UPDk1pQ8/`, `selected-trajectory-execution-20261123.0EXHpHyM/` |
| 왼팔 수거함 복귀 | 복귀 664표본과 마지막 구간 사전 검사 86표본에서 관절 한계, CAD와 저장된 깊이 지도 충돌 검사를 통과했다. 손목 roll 1틱 차이에서도 설치본이 궤적을 재타이밍했다. | `rear-bin-left-continuous-20261123.fi55_0ek/` |

이 기록의 가상 제어기와 저장 장면 재생은 해당 입력에서의 결과다. 새 복귀 궤적의 실물 실행 검증은 남아 있다.
