# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

## 2026-10-25 GPU PhysX와 CPU의 4개, 8개 Facility 18 환경 비교

기존 단일 Isaac Lab 복제 실행기에 GPU dynamics와 GPU broadphase 선택을
추가했다. 완료 비교군은 `matrix-03/cpu-04`, `matrix-03/gpu-04`,
`matrix-04/cpu-08`, `matrix-04/gpu-08`이다. 4환경은 seat-12, seat-03,
seat-27, seat-46이고, 8환경은 같은 순서를 두 번 배치한 독립 월드다.
동일 월드에서 로봇 8대가 상호 회피하는 시험은 아니다.

실제 Mission Manager, Nav2, ros2_control로 24개 Mission이 SUCCESS를
보고했고 48개 NavigateToPose goal의 feedback recovery는 모두 0이었다.
planner 실패, 복구 행동, costmap clear도 모두 0회다. 최종 검증은 23/24다.
CPU 4환경의 seat-46 한 건은 실제 복귀 오차 9.04cm로 기존 8cm 기준을
넘어 실패했다. GPU 4환경과 CPU/GPU 8환경은 각각 4/4, 8/8, 8/8 통과했다.
23개 통과에는 최종 정지, IDLE/ready 복원과 Mission/작업 DB 대조가 포함된다.
24개 환경 모두 별도의 실제 팔 trajectory/cancel 검사를 통과했다.

| 지표 | CPU 4환경 | GPU 4환경 | CPU 8환경 | GPU 8환경 |
|---|---:|---:|---:|---:|
| Mission SUCCESS | 4/4 | 4/4 | 8/8 | 8/8 |
| 위치/정지/기록 포함 최종 검증 | 3/4 | 4/4 | 8/8 | 8/8 |
| Nav2 feedback recovery 합계 | 0 | 0 | 0 | 0 |
| 평균 주행 RTF | 0.504 | 0.212 | 0.211 | 0.137 |
| 공유 physics step 평균 | 2.053ms | 7.429ms | 4.988ms | 10.060ms |
| Mission batch wall time | 366.50초 | 756.78초 | 771.97초 | 1150.30초 |
| 초기화/종료 포함 전체 시간 | 497.48초 | 913.79초 | 1007.69초 | 1412.08초 |
| 완료 Mission/분 | 0.655 | 0.317 | 0.622 | 0.417 |
| 엄격 검증 통과/분 | 0.491 | 0.317 | 0.622 | 0.417 |
| 전체 CPU 시간 가중 평균 | 399.3% | 300.7% | 525.2% | 475.6% |
| Mission당 누적 CPU core-second | 358.32 | 547.19 | 484.69 | 648.61 |
| 프로세스 RSS 평균 합계 | 12,433.7MiB | 12,504.3MiB | 17,824.9MiB | 17,790.1MiB |
| 소유 Isaac GPU framebuffer peak | 913MiB | 1,293MiB | 913MiB | 1,357MiB |

Mission batch는 첫 주행 시작부터 마지막 IDLE까지이고 mock 작업 시간도
포함한다. Mission SUCCESS와 엄격 검증 통과를 구분해 처리량을 계산했다.
성능은 물리 위치 검사에 실패한 CPU 4환경의 완주 궤적도 포함한다.
주행 RTF는 각 환경의 NAVIGATE_TO_TARGET/RETURN_HOME 창에서 구한 뒤 평균했다.
공유 step 비용은 한 환경의 비용이 아니라 모든 복제 환경을 함께 진행하는 비용이다.

| step 내부 구간 | CPU 4 | GPU 4 | CPU 8 | GPU 8 |
|---|---:|---:|---:|---:|
| 환경별 명령/상태 처리 | 1.122ms | 1.178ms | 2.606ms | 2.661ms |
| 공유 물리 step | 2.053ms | 7.429ms | 4.988ms | 10.060ms |
| 공통 clock 발행 | 0.122ms | 0.136ms | 0.278ms | 0.299ms |
| 환경별 센서/상태 발행 | 0.546ms | 0.556ms | 1.222ms | 1.225ms |

GPU의 Mission batch는 CPU보다 4환경에서 2.06배, 8환경에서 1.49배 길었다.
공유 physics step은 각각 3.62배, 2.02배 길었고, 환경별 명령/상태와 센서
비용은 같은 환경 수에서 비슷했다. GPU의 순간 CPU 사용률은 낮았지만,
Mission당 누적 CPU 시간은 각각 약 52.7%, 33.8% 늘었다. 이는 현재 native
ROS 제어와 host readback 경로의 측정 결과이며 GPU 물리 일반의 성능 결론은 아니다.
CPU 8환경의 처리량은 CPU 4환경보다 약 5.0% 낮았다. GPU 8환경은 GPU
4환경보다 약 31.6% 높은 처리량이었지만 CPU 비교군보다 낮았다.

CPU 100%는 한 코어이며 자원 평균은 모든 Mission이 활성인 공통 구간의
시간 가중 평균이다. 누적 CPU는 전체 Mission batch 동안 적분했다. RSS 합은
공유 페이지를 중복 포함하며 실제 물리 RAM 사용량이 아니다. GPU 메모리는
소유 Isaac PID의 framebuffer다. 호스트의 다른 작업은 중단하지 않았고,
해당 공통 구간의 다른 프로세스 CPU 평균은 약 26.2–35.0%였다.
각 조합의 완료 측정 batch는 한 번이며 확률적인 localization/navigation의
통계적 신뢰도나 장기 안정성을 보장하지 않는다.

복귀 오차 원본은 다음과 같다. 8환경의 뒤 네 값은 같은 네 좌석의 두 번째 복제본이다.

- `cpu-04` (`env_0`부터): 5.22, 4.29, 3.20, 9.04cm.
- `gpu-04` (`env_0`부터): 4.86, 7.29, 4.57, 4.67cm.
- `cpu-08` (`env_0`부터): 6.08, 5.00, 6.24, 6.39, 3.84, 7.43, 4.12, 6.77cm.
- `gpu-08` (`env_0`부터): 5.67, 3.80, 3.68, 5.25, 4.62, 6.35, 5.78, 6.64cm.

CPU 4환경의 실패 후 정지 상태에서 직접 수집한 map→odom TF와 PhysX
odometry를 대조했다. AMCL 기반 추정 위치의 목표 오차는 1.02cm로 Nav2
3cm goal checker 안이지만, 실제 물리 위치 오차는 9.04cm였다.
초기 `matrix-01/cpu-04`에서도 다른 seat-27의 실제 오차 8.88cm에 대해
추정 오차는 1.55cm였다. GPU 문제로 단정하지 않으며, 위치 추정과 물리
복귀 정확도의 차이는 남아 있는 문제다. 비교 중 Nav2 설정, 속도, 허용 오차를
변경하거나 실패 환경을 골라 재시도하지 않았다.

GPU mode는 `SimulationManager.setup_simulation(device='cpu')`의 host
buffer 계약을 유지하면서 PhysX scene의 GPU dynamics와 GPU broadphase를
활성화한다. 설치된 native ros2_control은 CPU tensor buffer를 사용하므로
완전한 CUDA resident tensor pipeline에서는 device 0 대 CPU -1 불일치가
발생했다. 새 제어기를 구현하거나 vendor backend를 교체하지 않았다.
`manager_device=cpu`만으로 GPU 물리가 꺼졌다고 판단하지 않는다.
실행 중 backend flag와 실제 PhysX GPU solver/articulation heap을 함께
검사했다. GPU 4환경의 articulation 관련 heap은 1,071,056 bytes,
GPU 8환경은 1,214,816 bytes였고 CPU 비교군은 0이었다. PyTorch CUDA trace는
기존 CUPTI subscriber 충돌로 kernel을 수집하지 못했다. 따라서 위 물리 step
시간을 순수 GPU kernel 시간이나 CPU↔GPU 복사 시간으로 해석하지 않는다.

8환경을 처음 띄웠을 때 기본 Fast DDS 포트 할당 한도 때문에 일부 참가자가
수신 포트를 확보하지 못했다. 112개 독립 ROS Context와 늦게 시작한 관측자를
이용한 재현에서 기본 설정은 0/112 토픽 수신, initial peer 범위만 확대한
설정도 0/112였고, `mutation_tries=256`만 적용한 설정은 112/112였다.
[Fast DDS 2.6의 설정 문서](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/domainparticipant.html)와
[수신 포트 할당 설명](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html)에
대응하는 한도다. loopback UDP와 discovery 방식은 유지했다. 4환경은 기존
기본값 100, 8환경은 256을 사용했으며 각 환경 수 안의 CPU/GPU profile은
동일하다. 비교 분석에서 두 profile의 유일한 의미상 차이가 이 한도임을 확인했다.
현재 실행 profile에는 256을 반영했다.

다중 시작 시 lifecycle 응답 유실과 센서 publisher discovery 선후 관계도
수정했다. 각 기존 localization/navigation/safety manager에 STARTUP을
한 번씩 요청하고 모든 Mission ready를 확인한 뒤 검증기를 시작한다.
검증기는 센서 실제 수신과 publisher discovery를 확인한 뒤 미션을 할당한다.
미션 중 recovery를 삽입하거나 안전 경로를 우회하지 않았다. 느린 RTF에서
정지 확인은 최근 0.8 simulation 초의 10개 이상 표본과 최소 0.5 simulation 초를
검사하고, 최신 수신의 wall freshness 0.8초를 유지한다. 관련 단위 검사 7개와
실제 팔/미션 정지 검사를 통과했다.

완료 비교군 모두 공통 clock 1개, 환경별 센서와 최종 cmd_vel publisher 1개,
collision monitor 소유권과 root topic 미발행을 확인했다. 초기 수동 graph
관측은 CPU 8환경에서 4초 안에 cmd_vel 하나를 놓쳤고, GPU 8환경에서는
20초 안에 native joint-state publisher들을 놓쳤다. 초기 결과를 보존했으며,
CPU는 이후 20초 관측, GPU는 직접 joint-state 구독을 포함한 35초 관측에서
모두 확인했다. GPU에서 기다린 시간과 구독 추가의 효과를 분리하지 않았으므로
관측 지연의 정확한 원인은 단정하지 않는다. 각 Mission 검증기는 별도로
필수 스트림 수신과 publisher 수를 확인한 후 출발했다.

초기 실패와 중단한 시도도 보존했다.

- `gpu-probe-01`: GPU resident tensor와 native ros2_control의 device 불일치.
- `gpu-probe-02`: 진단 코드의 제거된 tensor wrapper 참조 오류. 직접 view 호출로 수정.
- `gpu-probe-03`: 두 팔과 실제 Nav2 이동 호환성 확인 후 의도적으로 중단. 완주 표본에서 제외.
- `matrix-01`: CPU 4환경 Mission 4/4, 엄격 검증 3/4. GPU 4환경의 한 safety lifecycle 응답 유실로 준비 실패, 나머지 미션은 중단.
- `matrix-02`: 팔/기동은 통과했지만 두 검증기가 초기 joint-state discovery 전에 검사해 실패. 다른 두 미션은 중단.
- `matrix-03/cpu-08`: 팔 8/8과 lifecycle 24회 활성화는 성공했지만 ready 상태 수신 누락으로 미션 할당 전 실패. GPU 8환경은 이 설정으로 시작하지 않음.
- `matrix-04`: 포트 할당 한도를 수정한 뒤 CPU 8환경과 GPU 8환경을 각각 한 batch 실행. 둘 다 8/8 통과.

환경은 Ryzen 5 9600X, RTX 5070 12GB, NVIDIA driver 615.78.08,
ROS Humble, Isaac Sim 6.1, Isaac Lab v3.0.0-EA다. Lab commit은
`ae37b028ea415c91ea2bc32609efcd759ed2b974`다. 작업 HEAD는
`eb2c56a190ad9fbb52cb104ff55282e955a56fcb`, Navigation 입력은
`codex/nav2-tuning-recovered-20261024`의 `69b25020b4506912f2ebd8881212407d3c2740be`다.
ROS source digest는 모든 비교군에서
`2df3ec15d4854079dec4e8054b0427ebf8d375fd01c0205dc48176e97f58a475`이며
runtime Python hash도 같다. ROS image ID는
`210bda156c496a3705c96ece2cc2e41ccc9de4ce86bbcd788a31c9c3178d977e`,
Isaac image ID는 `d6c64782555db147cfcaaaa101d2cfe6b815daf5e57b010339bcb8fde14a9265`다.
모든 실행은 물리 500Hz, native controller manager 100Hz, Nav2 20Hz,
clock 100Hz, odometry/TF 50Hz, scan 10Hz/361 rays, Fabric, 실시간 pacing,
Isaac CPU thread 제한 4를 유지했다. URDF, motor/controller, Facility geometry와
namespace를 제거한 Navigation 입력 hash, 발행 개수와 step 수를 대조했다.
실제 베이스는 기존 이상적인 root velocity 구동이며 인식/계획/조작 작업은
mock이다. 물리적인 책상 정리나 RGB-D 인식의 완성을 주장하지 않는다.

원본과 재현 경로는 다음과 같다.

- [비교 JSON](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/comparison.json), [재집계 스크립트](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/analyze_scale.py), [하드웨어](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/hardware.json).
- [4환경과 초기 8환경 campaign](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/matrix-03/campaign.json), [수정 후 8환경 campaign](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/matrix-04/campaign.json).
- [모든 진단 시도](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/diagnostics.json), [DDS 재현 검사](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/dds-probe/audit.json), [DDS 재현 코드](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/dds-probe/many-run.py).
- [CPU 4환경 실제/추정 위치 대조](../../cleany-data/runs/isaac-facility18-gpu-scale-20261025/matrix-03/cpu-04/env_3/evaluations/return-localization.json).
- 각 run의 `provenance/`, `evaluations/`, `env_N/evaluations/`, `logs/`, Mission/작업 SQLite DB에 입력 snapshot과 원시 결과를 보존했다. 초기 관측 누락 파일도 `runtime-isolation-initial-*.json`으로 남겼다.

새 실행은 [확장 실행기](RUNNING.md#isaac-lab-단일-프로세스-환경-복제)를 사용한다.
기존 결과 재집계는 다음 명령으로 수행한다. runtime source가 바뀌었으면
기록 당시 ROS source와 runtime snapshot을 준비한 별도 checkout을
`--source /absolute/checkout`으로 지정한다.

```bash
python ../cleany-data/runs/isaac-facility18-gpu-scale-20261025/analyze_scale.py
```

## 2026-10-25 Isaac Lab 복제 환경과 독립 프로세스 비교

Isaac Lab `v3.0.0-EA`의 공개 `usd_replicate` API로 전체 Facility 18과
Cleany를 두 벌 복제하고 하나의 Isaac Sim 프로세스에서 실행했다. Lab commit은
`ae37b028ea415c91ea2bc32609efcd759ed2b974`다. 각 환경의 2,117개 Facility
collider와 82개 의자, 로봇 관절 연결을 유지했다. 두 원점은 100m 떨어져 있고,
센서/odometry는 각 환경의 로컬 좌표를 사용한다. CPU PhysX의 공유 scene과
500Hz step을 사용했으며 GPU physics replication이나 RL training은 아니다.

복제 실행 `probe-04`에서 seat-12와 seat-03 모두 실제 Mission Manager를 통한
왕복이 SUCCESS로 끝났다. 두 Nav2 Action의 feedback recovery, planner 실패,
복구 행동과 costmap clear는 환경마다 모두 0회였다. 실제 팔 trajectory/cancel,
최종 정지와 IDLE/ready 복원, Mission/작업 DB 대조를 통과했다. 복귀 오차는
각각 3.83cm와 6.39cm였다. 기존 collision monitor가 각 환경의 최종 cmd_vel
발행자이며 공통 clock은 1개, 센서와 TF, Action과 Mission 토픽은 환경별로
격리됨을 실행 중 확인했다. 주기 수신 및 전체 step/시뮬레이션 시간/발행 개수의
대조도 통과했다. 물리적 정리 대신 기존 인식/계획/조작 mock을 사용하고,
베이스는 기존 이상적인 root velocity 구동을 유지했다.

동일한 최종 runtime 코드와 ROS build로 복제 실행을 마친 뒤 2개 독립
프로세스 비교군을 새로 실행했다. 비교군도 두 좌석 모두 복구 없이 통과했다.
URDF, controller/motor, 월드 geometry, namespace를 벗긴 Navigation 설정과
physics/clock/odom/scan 주기가 동일하며 실행 전후 runtime hash가 같음을
`comparison.json` 생성 시 검증했다.

| 지표 | Lab 1 process / 2 env | Isaac 2 processes / 2 env |
|---|---:|---:|
| 최종 정상 왕복 | 2/2, 복구 0회 | 2/2, 복구 0회 |
| seat-12 주행 RTF | 0.847 | 0.894 |
| seat-03 주행 RTF | 0.849 | 0.900 |
| seat-12 실제 주행 wall time | 139.61초 | 128.39초 |
| seat-03 실제 주행 wall time | 166.59초 | 160.48초 |
| 두 미션 batch wall time | 250.20초 | 244.25초 |
| 초기화/종료 포함 전체 시간 | 341.12초 | 323.74초 |
| Isaac CPU 평균 합계 | 267.7% | 482.2% |
| ROS CPU 평균 합계 | 73.4% | 81.5% |
| 검증기 포함 전체 CPU 평균 합계 | 360.7% | 586.1% |
| 프로세스 RSS 평균 합계 | 10,389.9MiB | 18,865.2MiB |
| Isaac별 GPU framebuffer peak 합계 | 913MiB | 1,826MiB |

CPU의 100%는 논리 코어 1개다. 자원 값은 두 미션이 모두 활성인 공통 구간의
시간 가중 평균이고 mock 작업 대기도 포함한다. RSS 합은 공유 페이지가
중복되므로 실제 물리 RAM 사용량이 아니다. GPU 값은 소유 Isaac PID에 귀속한
framebuffer이며 데스크탑 전체 GPU 메모리나 GPU compute 활용률이 아니다.

주행 RTF의 두 좌석 평균은 0.848 대 0.897로 복제 방식이 약 5.5% 낮았다.
CPU 합계는 38.5%, RSS 합계는 44.9%, Isaac framebuffer는 50% 줄었다.
Mission batch는 첫 주행 시작부터 마지막 IDLE까지이며 mock 작업 시간을
포함한다. 복제 방식은 batch 2.4%, 전체 실행 5.4% 더 오래 걸렸다.
전체 시간에는 시작/종료와 polling이 포함되고 복제 실행에는 추가 3초 graph
검사도 있다. 두 환경이 실제로 동시에 주행한 시간은 복제 126.06초,
독립 프로세스 112.91초였다.

복제의 공유 physics step 평균은 주행 창에 따라 1.076–1.081ms였으며 두 환경을
함께 진행하는 비용이다. 독립 프로세스의 환경별 step은 0.760–0.794ms였다.
공유 step을 단일 환경 step으로 해석하거나 독립 프로세스의 wall time을
단순히 더해 처리량으로 해석하지 않는다. 이번 조건에서는 복제가 주행 속도를
높이지 않았으며 자원 절약 효과가 컸다.

각 방식에서 완료한 비교 표본은 두 좌석 1쌍이다. 두 방식 모두 CPU PhysX,
Fabric, split 발행과 실시간 pacing을 유지했다. Isaac 프로세스당 thread 제한은
4로 같지만 전체 예산은 1×4 대 2×4다. 동일 총 thread 예산이나 pacing 없는
최대 처리량 비교가 아니다. 호스트의 다른 작업을 중단하지 않은 순차 A/B이며
장기 안정성이나 환경 수 증가에 따른 성능을 보장하지 않는다. 복제 환경은 시각과
process를 공유하므로 환경별 독립 pause/reset 및 simulator 장애 격리가 없다.

단독 실행과 복제 실행의 월드 생성은 `scene_assets.py`를 공유한다. ROS 패키지
코드는 이번 비교에서 변경하지 않았다. 기존 launch를 감싸 namespace/remap과
parameter root만 적용했다. 실제 Humble ROS 테스트에서 센서 remap, 공통 clock,
독립 Action endpoint와 remap된 publisher 조회를 검증했다.

초기 통합 시도 3개도 삭제하지 않았다. `probe-01`은 Lab import 의존성
`lazy_loader` 누락으로 초기화 실패했다. 별도 의존성 디렉터리로 보완했다.
`probe-02`는 팔 검증기의 절대 Action 경로가 namespace를 따르지 않아 실패했고,
상대 경로로 수정했다. `probe-03`은 검증기의 `count_publishers` 호출이 remap을
적용하지 않는 문제로 Mission 할당 전에 실패했다. `resolve_topic_name`을 거쳐
조회하도록 수정하고 실제 Humble 회귀 테스트를 추가했다. 이 시도들은 정상
왕복 성공 표본에 포함하지 않으며, 실패한 통합 시도를 포함해 보존한다.

원본은 데이터 루트의 `runs/isaac-facility18-cloned-20261025/`다.
`probe-04-command.json`과 각 실행의 `provenance/run.json`에 정확한 명령,
소스 snapshot/hash, Lab commit과 이미지 ID가 있다. `probe-04/evaluations/`에는
공유 물리/자원 profile, clone과 초기 controller graph 검사가 있고,
`probe-04/env_0/evaluations/runtime-isolation.json`에는 실행 중 ROS 격리 검사가
있다. 각 `env_N`에는 Mission/팔 검증, DB, 자세 기록과 로그를 남겼다.
`fresh-two-process/`는 같은 최종 코드로 재실행한 domain 175/176 비교군이다.
`analyze_comparison.py`는 입력/소스 동일성과 DB를 검증하고 성능을 집계한다.

main `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`, Navigation 도입
`69b25020b4506912f2ebd8881212407d3c2740be`와 앞선 미커밋 수정,
ROS source digest `2df3ec15d4854079dec4e8054b0427ebf8d375fd01c0205dc48176e97f58a475`,
Ryzen 5 9600X / RTX 5070 12GB / Isaac Sim 6.1.0 / Humble을 사용했다.
Isaac Lab source/dependency는 전용 경로에서 읽기 전용으로 mount했으며
기존 Isaac Lab 설치와 다른 작업 환경은 변경하지 않았다.

## 2026-10-25 Facility 18 독립 환경 2개 병렬 실행

기존 `run.py`를 domain 175와 176으로 동시에 실행해 seat-12와 seat-03의
실제 Mission 왕복을 검증했다. **두 실행 모두 SUCCESS, 복구 없이 통과**했다.
Isaac Sim 프로세스와 전체 Facility 월드, Mission/Nav2/검증기를 각각 실행했다.
Isaac Lab의 단일 프로세스 벡터 환경이나 공유 월드의 다중 로봇 검증은 아니다.
이번 검사에서는 runtime 코드나 ROS 소스를 변경하지 않았다.

| 환경 | 이전 단독 RTF (2회 평균) | 병렬 주행 RTF | 복귀 오차 |
|---|---:|---:|---:|
| domain 175 / seat-12 | 0.909 | 0.885 | 3.81cm |
| domain 176 / seat-03 | 0.913 | 0.894 | 5.52cm |

두 미션이 동시에 주행 상태였던 시간은 116.12초다. 두 환경의 Isaac/ROS/검증기
6개 컨테이너가 함께 관측된 구간은 212.50초였다. 전체 2개 실행의 완료 시간은
339.77초이며, 이전 동일 좌석의 단독 실행 2회씩 평균을 합한 605.43초와 비교하면
처리량은 약 1.78배다. 이 시간에는 초기화, mock 작업 대기, 종료와 최대 5초의
집계 polling이 포함된다. 이전 순차 표본을 이용한 참고 비교이며, 호스트 부하를
통제한 동시기 A/B나 장기 병렬 안정성 검증으로 해석하지 않는다.

서로 다른 DDS domain, 컨테이너 이름, 출력/DB 경로와 복사된 전용 캐시를 사용했다.
빌드는 동일한 읽기 전용 경로를 재사용했다. 각 domain에서 clock/odom/scan/
joint state publisher는 각각 1개였고, 수신 상태와 DB는 해당 실행의 서로 다른
Mission ID 및 목표 좌석과 일치했다. 한쪽 종료 후에도 다른 환경의 복귀가 계속됐다.
Nav2 복구, 경로 생성 실패와 costmap 초기화는 양쪽 모두 0회였다.
IDLE/ready 복원, 실제 정지, 관절 제어, Mission/작업 DB 대조를 통과했다.

500Hz physics와 기존 controller_manager 100Hz, split 발행(clock 100Hz,
odom/TF 50Hz, scan 10Hz), Fabric 설정을 유지했다. 두 실행의 실제 수신 주기와
물리 step/시뮬레이션 시간/발행 개수 대조도 통과했다. GPU 여유 최솟값은
6,853MiB, 호스트 가용 메모리 최솟값은 15,108MiB였다. 메모리 guard는 유지했다.

원본은 데이터 루트의 `runs/isaac-facility18-parallel-20261025/`다.
`parallel.json`에 정확한 두 명령과 모든 종료 결과, `verification.json`에
성능, 실제 주행 시간 중첩과 격리 검사, `concurrent-resources.jsonl`에 동시
컨테이너와 자원 표본을 보존했다. `run_parallel.py`, `analyze_parallel.py`는
실행과 집계의 원본이며, 각 실행 아래에 기존 provenance, Mission/Nav2 로그,
DB, 자세 기록과 profile을 보존했다. 재현 절차는 [테스트](TESTING.md)를 따른다.

main `eb2c56a`, Navigation 도입 `69b2502`와 앞선 미커밋 수정,
ROS source digest `2df3ec15d4854079dec4e8054b0427ebf8d375fd01c0205dc48176e97f58a475`,
Ryzen 5 9600X / RTX 5070 12GB / Isaac 6.1.0 / Humble을 사용했다.
이전 단독 실행과 로봇, controller/motor, Navigation, 월드 geometry 입력 hash가
동일하고 실행 전후 runtime 도구 hash도 동일함을 확인했다.
실제 Mission/Nav2와 이상적인 베이스 actuator를 사용했으며,
인식, 작업 계획과 조작은 기존 mock이다. 검증 범위는 동시 실행 1쌍이다.

## 2026-10-25 Facility 18 물리 내부 계측과 발행 주기 최적화

원본은 데이터 루트의 `runs/isaac-facility18-optimize-20261025/`이다.
CPU PhysX의 timestep 2ms(500Hz), controller_manager 100Hz, Nav2 controller
20Hz, LiDAR 10Hz×361 ray와 기존 속도/도착 기준을 유지했다. 이전 프로파일
설명의 “500Hz 물리/관절 제어” 중 실제 controller_manager 설정은 100Hz다.
단일 환경을 순차 실행했으며 병렬 환경이나 GPU 물리는 사용하지 않았다.

Carbonite CPU native trace를 첫 실제 이동 3초 동안 수집했다. main thread의
중첩 zone은 exclusive 시간으로 합산하고 worker 시간은 별도 집계했다.
`baseline-native-02` 874 step과 `optimized-native-01` 1,359 step의 결과다.

| native 구간 (inclusive, ms/call) | 기존 USD | Fabric + 발행 분리 |
|---|---:|---:|
| 전체 물리 step | 2.239 | 0.750 |
| updateRenderTransforms | 1.711 | 0.324 |
| 그 내부 USDWrite | 0.329 | 0.001 |
| waitForCompletion | 0.404 | 0.325 |
| worker PhysX simulate | 0.300 | 0.247 |
| worker Dynamics:solveIsland | 0.114 | 0.105 |
| worker Sim.narrowPhase | 0.021 | 0.017 |
| post-step subscription 전체 | 0.075 | 0.061 |

중첩 구간과 병렬 worker 값은 서로 더하지 않는다. 기존 transform 동기화는
전체 step의 약 76%를 차지했다. USD 쓰기뿐 아니라 attribute 변경 처리,
extents, notice callback 등이 포함된다. post-step subscription에는 기존
제어 callback이 포함되지만 vendor callback 안의 ros2_control 비용만 따로
분리하지는 못했다. native 계측 자체의 비용이 있어 성능 비교는 아래 별도
비-native 실행을 사용했다. 상세 원본은 각 실행의 `evaluations/native-trace.json.gz`,
`native-profile.json`, `native-summary.json`이다.

`SimulationManager.enable_fabric(True)`로 매 step physics 결과의 USD 쓰기를
끄고 Fabric 경로를 사용했다. 센서·odom·영상 기록은 실제 PhysX 자세를 읽는다.
현재 headless 경로와 기록 기반 오프라인 렌더러에 적용한 최적화이며, 이후
live USD 기반 카메라/소비자를 추가하면 동기화를 다시 검증해야 한다.
물리 solver iteration, collision 형상, 환경의 329개 모델/2,117개 collider와
82개 의자는 줄이지 않았다.

발행은 physics tick의 정수 배수로 나눴다. clock 500→100Hz,
odom/odom→base TF 500→50Hz, scan 10Hz 유지다. odom 속도는 마지막
2ms 변화가 아니라 실제 발행 간격 20ms 전체의 위치·각도 변화로 계산한다.
검증기는 수신 timestamp의 단조 증가와 실제 주기를 검사한다. 전체 physics
step 수와 SimulationManager 시간, 발행 개수의 정수 배수 관계도 대조한다.

### 동일 seat-12 성능 비교

`baseline-control-01`과 최종 행렬의 seat-12 두 실행은 같은 최종 도구 코드로
실행했다. 비교기는 ROS source digest, 로봇 URDF, controller, motor,
Navigation, world geometry hash와 물리 주기/좌석이 같음을 확인한다.
RTF와 물리 평균은 상태 전이 창을 제외한 주행 구간이며 실제 주행 시간은
기록된 이동 구간의 합이다. mock 작업 대기 시간과 시작/종료 시간은 제외한다.

| 지표 | 기존 | 개선 후 (2회 평균) | 변화 |
|---|---:|---:|---:|
| 주행 RTF | 0.636 | 0.909 | 42.8% 증가 |
| 물리 step 평균 | 2.038ms | 0.693ms | 66.0% 감소 |
| 실제 주행 시간 | 182.267초 | 130.748초 | 28.3% 감소 |
| Isaac CPU (100%=1 논리 코어) | 328.9% | 240.7% | 26.8% 감소 |
| ROS CPU (100%=1 논리 코어) | 80.2% | 42.5% | 47.0% 감소 |

`publication-only-01`(split + USD)의 RTF는 0.784, 주행 시간은 146.89초였다.
발행 분리만으로도 ROS 부하가 줄고, Fabric 추가가 물리 step 비용을 줄였다.
호스트의 다른 작업을 중지하지 않았고 그 CPU도 기록했다. 독립 closed-loop
주행의 궤적·시간은 조금씩 다르며 기준 1회/개선 2회 비교이므로 통계적으로
일반화된 속도 보장은 아니다. 개선 후 RTF 약 0.91의 나머지 차이에는 기존
실시간 pacing과 scan tick 비용이 포함된다.

### 최종 반복·실패 검증

최초 8회 행렬은 **7회 왕복 성공, 1회 미션 시작 전 초기화 실패**였다.
실패를 보존하고 같은 코드·설정으로 seat-46을 명시적으로 한 번 보충했다.
따라서 **전체 9번 시도 중 실제 미션이 시작된 8번은 모두 복구 없이 왕복 완료**다.
전체 실행 성공률을 8/8로 표기하지 않는다. 최초 `campaign.json`의
`all_passed_without_recovery=false`와 `final-suite.json`의 failed를 유지했다.

| 좌석 | 전체 시도 | 복구 없는 왕복 | 복귀 오차 (cm) | 주행 RTF |
|---|---:|---:|---:|---:|
| seat-12 | 2 | 2 | 4.16–4.27 | 0.908–0.909 |
| seat-03 | 2 | 2 | 4.95–5.78 | 0.910–0.915 |
| seat-27 | 2 | 2 | 4.68–5.21 | 0.904–0.909 |
| seat-46 | 3 | 2 | 4.53–6.24 | 0.908–0.912 |

8회 모두 SUCCESS, RETURN_HOME 후 IDLE/ready와 정지를 확인했다.
16개 NavigateToPose Action의 feedback 복구 횟수와 로그의 경로 생성 실패,
복구 행동, costmap 초기화가 모두 0회다. 모든 복귀 오차는 기존 8cm 기준 이내다.
각 Mission report와 독립 SQLite journal 대조를 통과했고 활성 실행 기록은 0건이다.
물리 실제 시간/step 수와 정수 배수 발행 개수 검사도 8회 모두 통과했다.
수신 주기는 clock 100Hz, odom/TF 50Hz, scan 10Hz, joint states 약 100Hz였다.

초기화 실패 `optimized-matrix/08-seat-46-r2`에는
`controller_server/change_state` 응답 timeout이 기록됐고 Mission이
IDLE/ready=false, NAV2_NOT_READY에 머물렀다. Mission/작업 DB는 0건이다.
`startup-failure-audit.json`에 근거를 남겼다. 응답 timeout의 근본 원인은
확정하지 않았으며, 보충 실행 성공을 초기화 문제 해결로 해석하지 않는다.
보충 원본은 `supplemental-seat46/`, 명령/사유는 `supplemental.json`,
검사 결과는 `supplemental-summary.json`이다. 최초 보충 명령은 반복 도구의
최소 2회 조건에 거부되어 시뮬레이터를 시작하지 않았다. 이후 단일 실행 도구를
사용했으며 거부 기록도 `supplemental-command-rejected.json`에 보존했다.

같은 split + Fabric 설정의 실패/취소 회귀 3개도 모두 통과했다.

| 시나리오 | 결과 | 종료 상태 | 복귀 / 사람 확인 |
|---|---|---|---|
| 주행 timeout | FAILED / TIMEOUT | IDLE, ready=true | 미시작 / 불필요 |
| mock 조작 실패 | FAILED / GRASP_FAIL | ERROR, ready=false | 미시작 / 필요 |
| 이동 중 취소 | CANCELLED | RETURN_HOME 후 IDLE, ready=true | OK / 불필요 |

세 실행 모두 결과 이후 fresh odometry의 정지, DB 무결성과 journal 대조,
활성 작업 0건을 확인했다. `failure-journal-audit.json`에 상태 전이와 결과가 있다.
의도적으로 실패를 유도한 이 세 실행은 정상 왕복의 무복구 집계와 구분한다.

재현 방법은 [실행 옵션](RUNNING.md), [반복 및 프로파일 검사](TESTING.md)를 따른다.
`final-suite.json`, `optimized-matrix/campaign.json`, `failure-regression.json`에
정확한 실행 명령과 모든 시도를 보존했다. `performance-comparison.json`은
전후 비교, `verification.json`은 주기·반복·코드 불변성의 최종 집계다.
각 실행에는 `provenance/source/`, `provenance/run.json`, 원시 profile,
Mission/Nav2 로그, DB, 실제 자세 기록을 남겼다. 도구 hash를 고정한 최종
행렬 중 runtime 도구와 ROS 소스를 바꾸지 않았다.

main `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`, 도입 Navigation
`69b25020b4506912f2ebd8881212407d3c2740be`와 앞선 미커밋 수정이 기준이다.
ROS source digest는 `2df3ec15d4854079dec4e8054b0427ebf8d375fd01c0205dc48176e97f58a475`로
이번 최적화에서 바뀌지 않았다. Ryzen 5 9600X, RTX 5070 12GB,
Isaac 6.1.0 / ROS Humble, domain 174와 기존 전용 build/cache를 사용했다.
정확한 image ID는 각 실행 provenance에 있다.

계측기 초기 시도 `baseline-native-01`은 direct CPU backend 설정에 mux
interface를 요청해 미션 시작 전에 실패했다. interface 선택을 수정했으며
그 실패 원본도 보존했다. cadence 5개, profile 분석 3개, 외부 Facility fixture
검사 5개가 통과했다. fixture 검사는 numpy가 있는 ROS 컨테이너에서 수행했다.

검증 범위는 실제 Mission Manager/Nav2/AMCL/관절 제어, 이상적인 베이스 actuator,
mock 인식·작업 계획·조작이다. 책상 위 물체를 물리적으로 정리하는 전체 구현의
성공으로 해석하지 않는다. 매회 새 Isaac/ROS/DB로 시작한 D-HUB 네 좌석의 독립
미션이며, 모든 Facility 좌석이나 같은 프로세스의 장기 연속 운용 검증은 아니다.

## 2026-10-25 Facility 18 계산 부하 프로파일

동일한 seat-12 왕복, 물리 500Hz, LiDAR 10Hz×361 ray, 자세 기록 10Hz와
Navigation 설정을 유지하고 선택형 `--profile` 계측을 실행했다. 원본은
데이터 루트의 `runs/isaac-facility18-profile-20261025/profile-01/`이다.
실제 Mission SUCCESS, 복구/경로 실패/costmap 초기화 각각 0회, 복귀 오차
6.77cm, IDLE/ready와 두 DB 대조를 통과했다. 전체 실행은 342.66초였다.
실제 자세 기록상 주행은 simulation 115.70초 / wall 181.89초였다.

환경은 Ryzen 5 9600X 6-core/12-thread, RTX 5070 12GB, EndeavourOS,
Isaac Sim 6.1.0 CPU PhysX와 ROS Humble이다. 기본 main `eb2c56a`, 도입 Navigation
`69b2502`와 미커밋 수정, ROS source digest는 위 반복 검증과 같다.
새 ROS 변경 없이 adapter 계측과 호스트 읽기 전용 sampler를 추가했다.
정확한 image ID, 소스 hash와 실행 명령은 `provenance/run.json`,
실행 코드 사본은 `provenance/source/`에 남겼다. 다른 사용자의 프로세스는
계속 실행 중이었으며 그 부하를 별도로 측정했다.

아래 비율의 분모는 상태 전이 경계를 제외한 주행 5초 창 35개, wall 175.05초다.
해당 창의 simulation 진행은 111.308초, RTF는 0.636, 실제 처리율은
317.9 physics step/wall-second였다. 55,654개 루프 모두 계산 시간이 목표
2ms를 넘었고 양의 sleep 요청은 0이었다. 따라서 실시간 pacing 제한이
이 실행의 지연 원인은 아니다.

| 루프 구간 | 주행 wall 시간 비중 | 호출당 평균 |
|---|---:|---:|
| 물리 step / native simulation 콜백 | 65.14% | 2.049ms |
| 상태 읽기, 명령 수신/적용 | 12.01% | 0.378ms |
| Odometry / TF 구성 및 발행 | 10.59% | 0.333ms |
| Clock 발행 | 6.54% | 0.206ms |
| LiDAR raycast 361개 | 3.54% | 5.571ms/scan |
| Pacing 호출 | 0.41% | 0.013ms |
| 영상용 link 자세 기록 | 0.12% | 0.184ms/frame |
| Scan 발행 / base 기록 | 0.09% | 0.139ms |
| 루프 바깥 검사와 계측 등 미분류 | 1.58% | — |

물리 step의 main-thread CPU 평균은 1.766ms이며 worker CPU는 이 값에
포함되지 않는다. 정지한 WORKING 구간에서도 물리 step 평균 1.976ms로
주행 2.049ms와 비슷했다. `SimulationManager.step()`의
`_physics_sim_interface.simulate()` native 호출을 포함한 측정이다.
PhysX solver, contact 처리와 ros2_control 동기 콜백 내부 비용은 아직
분해하지 않았으므로 65.14% 전체를 solver 단독 비용으로 해석하지 않는다.
원본 구현 발췌는 `provenance/simulation-step-source.txt`에 보존했다.

프로세스 CPU는 100%=논리 코어 하나 기준으로 Isaac Python 평균 327.4%,
ROS 전체 79.4%, 검증기 14.4%였다. Controller는 16.3%, planner 7.3%,
AMCL 4.0%, Mission runtime 27.2%였다. 주행 중 관측한 Isaac main thread의
runqueue 대기는 176.89초 중 5.00초(약 2.8%)였다. 세 TBB worker도 CPU를
사용했다. 다른 프로세스의 총 CPU는 평균 150.3%였다. 공유 GPU 전체 사용률은
평균 10.9%, 최대 78%이며 다른 프로그램의 사용량을 포함한다. 이번 Isaac의
pmon utilization `-`는 수치 0으로 대체하지 않았다. 현재 시뮬레이터는 CPU
PhysX와 수동 headless step으로 실행하므로 GPU 포화가 주원인이라는 근거는 없다.

계측의 exclusive 시간 합산, CPU/wall 구분과 상태 전이 창 제외 검사 2개가
통과했다. 호스트 Python의 50,000-loop 미세 검사는 계측/최종 직렬화 추가 비용
약 2.17µs/loop(2ms 예산의 0.11%)였으며 Isaac Python의 비용을 보장하지는 않는다.
이전 비계측 실행과의 비교는 다른 호스트 부하와 궤적이 섞여 있어 통제된 A/B가
아니다. 별도 CPU native profiler로 내부 step 비용을 더 분해하고,
500Hz 물리/관절 제어와 odom/TF 발행 주기를 분리하는 실험이 다음 최적화 후보다.
물리 timestep이나 기존 관절/센서 설정을 바꾼 성능 결과는 이번 측정에 없다.

`evaluations/profile-summary.json`, `loop-profile.jsonl`, `system-profile.jsonl`과
`journal-audit.json`이 분석/원본이다. 실행 루트 상위의 `host-cpu.json`,
`instrumentation-overhead.json`, `navigation-comparison.json`,
`analysis-provenance.json`, `analysis-source/`에 호스트, 계측 비용과 분석 코드의
근거를 보존했다. 재현과 해석 기준은 [프로파일 검사](TESTING.md#isaac-mission-시나리오)를 따른다.

## 2026-10-25 Facility 18 복수 좌석 반복 검증

속도와 근접 재계획 수정 후 같은 설정으로 D-HUB의 seat-12/03/27/46을
각각 두 번 왕복했다. 총 **9번 실행 시도 중 미션이 시작된 8번은 모두 SUCCESS**였고,
1번은 localization 초기화 실패로 미션을 시작하지 못했다. 실패를 제외한 8/8을
전체 실행 성공률로 해석하지 않는다. 원본은 데이터 루트의
`runs/isaac-facility18-repeat-20261025/`다.

| 좌석 | 전체 시도 | 복구 없는 왕복 완료 | 복귀 위치 오차 (cm) |
|---|---:|---:|---:|
| seat-12 | 2 | 2/2 | 4.82–5.26 |
| seat-03 | 2 | 2/2 | 4.35–5.03 |
| seat-27 | 3 | 2/2 | 4.19–4.21 |
| seat-46 | 2 | 2/2 | 4.40–5.64 |

8회 모두 `IDLE → NAVIGATE_TO_TARGET → WORKING → POST_MISSION → RETURN_HOME → IDLE`,
최종 ready와 정지를 확인했다. 총 16개 NavigateToPose Action feedback의 복구 횟수는
모두 0이며, 로그의 경로 생성 실패, 복구 행동, costmap 초기화도 각각 0회다.
8개 Mission report를 SQLite journal과 대조했고 mock 작업 16건은 모두 FINISHED,
활성 작업 0건이었다. 기존 8cm 복귀 도착 기준을 모두 통과했다.

`07-seat-27-r2`는 `/map_server/get_state` 응답 timeout과 lifecycle 전이 실패로
AMCL이 준비되지 않았다. ready=false, reason=NAV2_NOT_READY였으며 미션과 작업 DB는
모두 0건이다. 서비스 timeout의 근본 원인은 확정하지 않았다. 이 실패를 원본
`campaign.json`에 남기고, 코드/설정을 변경하지 않은 명시적 보충 실행
`09-seat-27-r2-retry`로 두 번째 주행 표본을 확보했다. 최초 8개 행렬의 결과는
7/8이며 `all_passed_without_recovery=false`를 유지한다. 4번째 실행 완료 후
집계 프로세스가 exit 143으로 종료된 별도 사건은 원인이 불명확하며,
이미 통과한 실행 결과와 DB를 대조해 중복 실행 없이 집계를 재개했다.

기준 main은 `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`, Navigation 도입 기준은
`69b25020b4506912f2ebd8881212407d3c2740be`이며 미커밋 근접 재계획 수정과
Facility override를 포함한다. ROS source digest는
`2df3ec15d4854079dec4e8054b0427ebf8d375fd01c0205dc48176e97f58a475`다.
Isaac 6.1.0 / Humble / RTX 5070, domain 174와 기존 전용 build/cache를 사용했다.
시작부터 마지막 실행까지 ROS 소스와 도구 hash가 동일함을 다시 확인했다.
`campaign.json`의 build manifest, tool hashes와 각 실행의 `provenance/`에
image ID, 실제 명령과 소스 사본을 보존한다.

`campaign-combined.json`은 9번의 모든 시도와 좌석별 집계,
`supplemental.json`은 추가 실행 사유와 명령, `startup-failure-audit.json`은 초기화
실패 근거, `navigation-metrics.json`은 실제 자세 기록 기반 시간/거리다.
각 실행의 `evaluations/mission.json`, `navigation.json`, `journal-audit.json`,
`logs/mission.log`, DB와 `media/motion.jsonl`을 함께 보존했다.
재현 명령은 [복수 좌석 검사](TESTING.md#isaac-mission-시나리오)를 따른다.

매회 Isaac/ROS/DB를 새로 시작한 독립 미션 검증이다. 전체 Facility의 모든 좌석이나
한 프로세스에서 연속 미션을 누적하는 장기 운용을 검증한 것은 아니다.
실제 Mission Manager/Nav2와 이상적인 베이스 actuator를 사용했고,
인식/작업 계획/조작은 기존 mock이다. 물리적인 책상 정리 성공으로 해석하지 않는다.

## 2026-10-24–25 Isaac Mission 통합

기준 코드는 원격 main `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`이며,
`codex/isaac-mission-e2e`의 미커밋 adapter/실행 도구를 사용했다.
실행마다 `provenance/source/`에 실제 실행한 도구를 복사하고
`provenance/run.json`에 소스 SHA-256, Git 상태, image ID, 명령을 보존했다.
ROS 소스 digest는 `a3e9d7ec585bf68e06fa77b8a47f256873a322bab4bce3cd65cd96e4e004f0f3`이다.
기존 main의 미커밋 파일과 isaac-training 저장소는 변경하지 않았다.

환경은 EndeavourOS, RTX 5070 12GB, Isaac Sim 6.1.0, 컨테이너 ROS Humble이다.
Isaac image ID는 `d6c64782555db147cfcaaaa101d2cfe6b815daf5e57b010339bcb8fde14a9265`,
ROS image ID는 `210bda156c496a3705c96ece2cc2e41ccc9de4ce86bbcd788a31c9c3178d977e`다.
독립 domain 174, loopback DDS, 별도 build/cache/DB를 사용했다.
원본은 데이터 루트의 `runs/isaac-mission-e2e-20261024/`에 있다.
빌드 manifest와 로그는 `builds/isaac-mission-e2e/`에 있다.

### 실행 범위

실제 Mission Manager, AMCL/Nav2, LiDAR collision monitor, PhysX 로봇과
기존 ros2_control controller를 실행했다. Mission의 인식/계획/조작은
기존 MockDesk와 mock Action이며 결과에도 `sim/mock/mock/mock`으로 기록된다.
물리 payload는 방에 존재하지만 수거하지 않는다. 단순 방/책상 fixture이며
전체 카페 재현, RGB-D, depth 안전, MoveIt planning, 실제 파지는 미검증이다.
베이스는 고정 바퀴/마찰 0 바닥의 이상적 root 속도 actuator로 메카넘 동역학을
검증하지 않는다. `/wheel/odom`은 PhysX pose 변화량이며 encoder 추정이 아니다.

### 검증 결과

| 검사 / 원본 | 결과 |
|---|---|
| `ros-build-dependencies.log`, build manifest | 기존 의존 패키지 12개 빌드 통과 |
| `mission-tests.log` | 기존 Mission core 검사 82 passed, 15 skipped; 조건부 ROS 검사는 별도 실행 범위 |
| `fixture-tests-final.log` | target schema와 지도상의 home/desk footprint 여유 1 passed |
| `attempt-08/` | 정상 Mission SUCCESS, mock 물체 2건 완료, 실제 Nav2 왕복 OK, 최종 IDLE/ready |
| `navigation-failure-01/` | 베이스 차단, FAILED/TIMEOUT, 조작 미시작, CANCELLING 후 IDLE/ready |
| `manipulation-failure-02/` | FAILED/GRASP_FAIL, 정지는 확인되나 팔 복구 미확인, CANCELLING 후 ERROR/not ready, human review 필요 |
| `manipulation-failure-03/` | ERROR/TERMINAL, ready=false, human review와 복귀 미시작을 자동 판정에 추가한 재실행 통과 |
| `cancel-01/` | 실제 속도 0.0384m/s에서 취소, Nav2 CANCELLED, RETURN_HOME/OK 후 CANCELLED 및 IDLE/ready |

정상 실행 상태는 `IDLE → NAVIGATE_TO_TARGET → WORKING → POST_MISSION →
RETURN_HOME → IDLE`이다. 최종 base 위치는 (0.0253, 0.0274)m로 home 오차
3.73cm이며 기존 8cm arrival 조건을 통과했다. 정상 실행의 오른팔 궤적 최대
관절 오차는 0.00100rad였다. Action 취소 응답 뒤 fresh 관절 속도가 0.02rad/s
미만으로 0.5초 이상 유지되고 원위치 궤적도 성공했다. 이는 독립 관절 제어 검사다.
각 Mission 결과 후에도 fresh base 속도/정지 및 두 DB의 integrity를 확인했다.
`audit_results.py`로 위 다섯 실행의 저장 결과를 다시 대조했다.
모든 Mission report가 journal과 일치했고, 조작 execution ID도 보고와 일치하며
모든 조작 기록이 FINISHED였다. 최종 ready/ERROR 정책과 base 시각의 단조 증가도
통과했다. 집계는 같은 실행 루트의 `suite-summary.json`에 있다.

`evaluations/mission.json`은 상태 전이, 최종 보고/odom, mock 범위와 판정을,
`arm.json`과 `arm-samples.json`은 실제 관절 검사를 보존한다.
`base.jsonl`, `logs/`, `missions.db`, `executions.db`가 원본 근거다.
실행 방법은 [RUNNING](RUNNING.md#isaac-sim-통합-검증),
시나리오는 [TESTING](TESTING.md#isaac-mission-시나리오),
미연결 계약과 후속 순서는 [통합 분석](ISAAC_INTEGRATION.md)에 있다.

### 실패와 중단 근거

초기 adapter 탐색과 `attempt-02`–`attempt-07`도 보존했다. viewport 기반
step의 낮은 진행률로 중단한 실행, controller 초기화 timeout,
Mission의 `seat-*` target 계약 위반, arm 취소 후 정지 불충족이 있었다.
최종 adapter는 수동 500Hz physics step, 원본 MJCF armature와 유효 target을 사용한다.
`attempt-07`은 고정 바퀴와 바닥 마찰 때문에 주행하지 못해 실제 Mission이
TIMEOUT으로 종료했다. 정상 시험은 이를 실패로 판정했고, 마찰 0의 명시적
이상 구동 조건으로 수정한 `attempt-08`에서 통과했다.
이 실패들을 삭제하거나 성공 실행에 합산하지 않았다.
`manipulation-failure-01`은 실패 후 물체를 유지한 외부 fixture를 시도했으나,
기존 Mission의 정상 수거용 snapshot 목록 검증에 거부되어 시작하지 못했다.
원본 fixture를 유지하고 `max_skill_retries=0`인 단일 실패 정책으로 다음 시도를
구성했다. 실제 결과는 object UNKNOWN, arm_recovered=false로 재관찰 전에
ERROR로 종료했고 복귀/재시도를 시작하지 않았다. 실패 후 재시도 snapshot
계약은 미검증으로 남긴다.

### 2026-10-25 성공 주행 다각도 GIF

`runs/isaac-mission-gif-20261025-02/`에서 기록 옵션으로 정상 Mission을 다시
실행해 SUCCESS, 왕복 OK, 최종 home 오차 3.51cm를 확인했다. PhysX link pose
1,375개를 기록하고 접근/복귀 구간 149개를 표본화했다.
`media/rendered-v2/`에 전방 사선, 측면, 상단 GIF와 세 시점 통합 GIF를 보존한다.
각 GIF는 149프레임, 10fps, 14.9초이며 명목 5배속이다.
`three-views-compact.gif`는 공유용으로 다시 압축한 통합본이다.

이 영상은 실제 성공 주행의 link pose를 Isaac RTX에서 다시 렌더링한 관찰
영상이다. 물리/ROS 재실행이나 로봇 센서 영상으로 해석하지 않는다.
기록된 link 위치와 렌더링 위치의 최대 차이는 0m였다. 원본 USD, pose JSONL,
PNG 프레임, 카메라 설정, 소스/명령은 해당 실행 폴더에 있다.
초기 기록 실행의 중첩 link 경로 오류와 첫 렌더링의 XformOp API 오류도
각 원본 로그에 보존했다. 최종 프레임의 구도와 GIF 프레임 수/길이를 확인했다.

## 2026-10-25 Facility 18F와 튜닝된 주행 코드

Facility 실행 원본은 데이터 루트의 `runs/isaac-facility18-20261025/`에 있다.
main 기준은 위 실행과 동일하며, 사용자 요청으로 주행 튜닝 변경을
`codex/nav2-tuning-recovered-20261024`의
`69b25020b4506912f2ebd8881212407d3c2740be`에서 가져왔다.
MPPI 구현/설정 등 86개 파일이 원본 브랜치 hash와 일치한다. 차이는 main의
GTest 헤더 순서 수정과 기존 DWB 헤더의 주석이며 `navigation-import-check.json`에 기록했다.
다른 시점의 Mission/문서 삭제는 가져오지 않았다.
별도 `builds/isaac-facility18-nav2-tuned/`에서 12개 의존 패키지를 빌드했고,
`make test-navigation`은 colcon 집계 288 tests, 0 errors/failures/skipped였다.
`fixture-tests-tuned-final.log`는 시설/mesh/park/설정 합성 검사 4 passed다.

원본 Facility 생성기의 정적 모델 329개, collision 2,117개를 변환했다.
D-HUB ROLY 의자 48개와 다른 방의 Fuel 의자 34개, 벽/문/천장을 유지했다.
지도는 외부 `fixtures/refactor-local-materials/facility_integration.yaml/.pgm`이다.
현재 생성기는 이 과거 fixture에서 빠졌던 의자 13/43도 포함한다. 지도 검사는
벽 위치와 선택한 home/seat-12의 footprint를 대상으로 하며 전체 가구/좌석
접근성의 완전한 검증이 아니다. 실제 LiDAR는 현재 월드의 충돌 형상을 관측한다.

팔은 기존 park 프로파일을 원본 관절 한계 안으로 제한한 초기 자세이며,
전체 visual/collision bounds는 base 기준 X 약 ±0.242m, Y 약 ±0.328m다.
원본 팔을 제거하거나 관절 한계를 넓히지 않았다. 접힘 전환과 self-collision은
미검증이다. Mission의 인식/계획/정리 Action은 여전히 mock이고, Facility의
가구나 물리 물체가 mock 작업에 맞춰 이동하지 않는다.

초기 진단 시도도 보존했다. `attempt-01/02`는 기존 경로 계획기와 안전 costmap
설정의 누락을 수정하기 위해 중단했다. `attempt-03`은 기본 팔 자세에서
오른쪽 어깨 yaw가 목표 0에서 약 -0.683rad로 벗어나고 주행 progress 실패가
발생했다. `attempt-04`는 접힌 자세로 진행 중 사용자 요청에 따라 주행 브랜치를
교체하면서 중단했다. `nav2-tuned-success-01`은 Isaac launch의 MPPI safety
가중치/예측 구간 누락을 수정하려고 중단했다. 모든 중단 이유는 해당 시도의
`evaluations/operator-stop.json`에 있으며 성공 실행으로 합산하지 않는다.

`nav2-tuned-success-02`는 seat-12 접근 중 desk-20 모서리의 scan 점 3개가
StopZone에 들어가 정지했고 progress 실패가 발생했다. 당시 실제 base 위치는
약 (-1.475, 1.159)m였다. `evaluations/navigation-snapshot.json`에 관측을 남겼다.
복구된 benchmark 프로파일의 출력 궤적 검증과 경로 샘플을 연결하기 위해
중단했으며, 안전 영역과 도착 허용 오차는 완화하지 않았다.
새 설정의 합성 검사는 `fixture-tests-recovered-profile.log`에서 4 passed다.

`nav2-tuned-success-03`은 seat-12 미션을 `SUCCESS`로 완료했다.
실제 Mission 상태는 `IDLE → NAVIGATE_TO_TARGET → WORKING → POST_MISSION →
RETURN_HOME → IDLE`였으며 복귀 오차는 0.01736m다. 독립 팔 Action 검사 최대
오차는 0.00151rad, 주행 중 park 최대 오차는 0.00462rad였다.
전체 실행은 초기화/팔 검사/정리 대역을 포함해 537.51초였다.
복귀 목표 근처에서 Smac 재계획 실패와 Nav2 회전/후진 복구가 발생한 뒤
성공했다. 재시도 없는 주행이나 전체 좌석 접근 성공을 뜻하지 않는다.
`suite-summary.json`은 Mission 결과와 SQLite journal 일치, 종료 상태,
남은 실행 0개와 시각 단조성을 검사한다. 물리 정리는 검증하지 않았다.

동일한 월드/seat-12/주행 설정에서 주요 실패 검사도 통과했다. 아래 경로는
`runs/isaac-facility18-20261025/` 기준이다. 각 실행은 팔 Action 검사를 포함하며,
종료 후 fresh odometry로 정지, DB 무결성과 남은 실행 0개를 확인했다.

| 실행 폴더 | 주입과 결과 | 최종 상태 |
|---|---|---|
| `nav2-tuned-navigation_failure-01` | base 구동 차단, `FAILED/TIMEOUT`, 작업 미시작 | `CANCELLING → IDLE`, ready |
| `nav2-tuned-manipulation_failure-01` | 도착 후 mock grasp 실패, `FAILED/GRASP_FAIL`, 복귀 미시작 | `CANCELLING → ERROR`, ready=false, 사람 확인 필요 |
| `nav2-tuned-cancel-01` | 실제 속도 0.03m/s 초과 후 취소, `CANCELLED`, 복귀 오차 0.01843m | `CANCELLING → RETURN_HOME → IDLE`, ready |

취소 후 복귀에서도 근접 재계획 실패와 회전 복구가 관측됐다. 네 시나리오의
교차 검증은 `suite-summary.json`과 `audit.log`에 보존한다.

성공 실행의 `media/facility-views-v3/three-views-compact.gif`는 전방 사선,
높은 측면, 전체 층 상단의 3분할 영상이다. 1440×430, 122프레임, 12.2초,
명목 20배속이며 `front.gif`, `side.gif`, `top.gif`도 제공한다. 실제 주행의
PhysX 링크 자세를 재생했고 위치 오차는 0m였다. 관찰을 위해 재생에서만
천장 visual을 숨겼으며 경로를 `render.json`에 기록했다. frame 0/30/90을
검수했고 `visual-review.json`에 GIF hash와 ffprobe 결과를 남겼다.
첫 `facility-views`는 천장에 가려졌고 v2는 측면 카메라가 벽에 가려져,
최종 제공본은 v3다. 이 영상은 실제 센서 영상이나 물리 정리의 증거가 아니다.

## 2026-10-25 Facility 속도와 근접 재계획 수정

원본은 `runs/isaac-facility18-fast-20261025/`다. 설치된
`ros-humble-nav2-smac-planner`는 `1.1.20-1jammy.20260915.232226`이다.
해당 버전의 [Smac 구현](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_lattice.cpp)은
시작/목표가 같은 XY cell과 heading bin일 때 A*의 같은 노드를 가리킨다.
[NodeLattice 역추적](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/node_lattice.cpp)은
parent가 없는 이 경우 false를 반환해 빈 경로를 만든다. 설치 라이브러리에
같은 칸의 두 실제 좌표/방향을 입력해 기존 실패를 재현했다.
최신 Humble upstream의 same-cell 처리와 같은 경계 사례를 보완하되,
Cleany의 기존 footprint/swept 검사를 재사용하고 정확한 목표 pose를 유지한다.
나머지 경로는 같은 Smac과 원래 144개 primitive로 계산한다.

`smac-terminal-test.log`는 기존 빈 경로 재현, 수정된 정확한 목표 연결,
정지점/다른 cell 경로, lethal/unknown/out-of-map 거부 검사가 통과한 기록이다.
`navigation-tests.log`는 colcon 집계 290 tests, errors/failures/skipped 0,
`fixture-tests.log`는 5 passed다. 초기 빌드는 upstream의 non-const angular-bin
API 호출 시 const 참조 때문에 실패했으며 참조 수정 후 12개 package 빌드가 통과했다.

Facility 전용 병진 상한은 0.15→0.25m/s, 회전 상한은 0.3→0.5rad/s다.
회전 샘플 표준편차는 0.07→0.2, TwirlingCritic은 10→2로 조정했다.
StopZone, SlowZone의 0.35 감속 비율, wheel projection, 목표 0.03m/0.1rad와
Mission 도착 gate는 그대로다. Isaac base actuator의 기존 상한도 바꾸지 않았다.
이는 시뮬레이션 프로파일이며 실물 속도 설정을 승인하는 근거가 아니다.

`success-01`은 실제 Mission의 seat-12 접근/모의 정리/복귀를 `SUCCESS`로
완료했고, `--require-no-recovery` 판정도 통과했다. 두 NavigateToPose feedback의
복구 횟수는 각각 0, 로그의 경로 생성 실패/복구 행동/costmap 초기화도 모두 0이다.
실제 복귀 오차는 0.04691m로 Mission의 0.08m 기준을 통과했다. Nav2의
0.03m 도착 기준은 AMCL 좌표 기준이며 물리 ground-truth 오차와 구분한다.

| 실제 자세 기록의 주행 구간 | 이전 `nav2-tuned-success-03` | 수정 `success-01` |
|---|---:|---:|
| 책상 접근 simulation 시간 | 63.8초 | 53.9초 |
| 복귀 simulation 시간 | 179.7초 | 65.6초 |
| 합계 simulation 시간 | 243.5초 | 119.5초 |
| 합계 wall 시간 | 372.16초 | 224.56초 |
| 경로 생성 실패 / 복구 행동 / costmap 초기화 | 9 / 3 / 9 | 0 / 0 / 0 |

`comparison.json`은 mock 작업 대기와 초기화를 제외한 주행 구간만 비교한다.
Simulation 시간은 50.9% 줄었고 wall 시간은 동시 GPU 작업과 실행 속도의 영향도
받는다. 수정 후 완주 검증은 1회이며 모든 좌석이나 반복 성공률의 검증은 아니다.
결과/DB 교차 검증은 `suite-summary.json`, `audit.log`에 있다.
새 영상은 `success-01/media/facility-fast/three-views-compact.gif`다.
3개 시점, 1440×430, 120프레임/12초, 명목 10배속이며 기록 자세와 렌더 위치
차이는 0m다. frame 25/115를 검수했고 `visual-review.json`에 hash와 영상 정보를
남겼다. 이전 GIF의 20배속과 재생 배율이 다르므로 실제 속도 비교는 위 수치를 사용한다.

## Jetson 실물 마운트와 카메라 보정

2026-10-21에 `jetson-orin:/home/cleany/cleany`의
`52e44abe89b7103bd832f3cbf806a121c37cb82c`와 미커밋 작업 내용을 읽어 선별했다.
프로파일은 [jetson_20261009](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/)
아래에 보존한다. [provenance.json](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/provenance.json)은
보정 원본, 사용한 내부 보정, 과거 검증 보고서의 경로와 SHA-256을 기록한다.
검증 지표와 실패 또는 미검증 조건은 각 extrinsics YAML과 [캘리브레이션 가이드](CALIBRATION.md)에 있다.
보고서 원본과 이미지 데이터셋은 Jetson의 기록된 위치에 남아 있다.

[mount.png](assets/physical-calibration/mount.png)는 사용자가 제공한 형상 렌더다.
`/Users/hyeon/Downloads/Jetson/mount.png`에서 원본 그대로 복사해 이전 PR 이미지를
교체했다. SHA-256은 `c36c8aff0047bbe3dd34ec6904da879269c4221d2dd68f64d0363cce8d5d9d8b`다.
실물 제어와 calibration 재수행의 증거로 사용하지 않는다.

### 2026-10-21 적용 검증

검증 코드는 `19f3882918f660fe580fe7f7fac0d8869d0a6331`이다.
Jetson Ubuntu 22.04, ROS 2 Humble, Python 3.10.12, MuJoCo 3.11.0의 별도
`/tmp/cleany-calibration-pr-20261021` workspace에서 실행했다.
`cleany_description`의 Git 추적 파일 100개가 해당 commit과 동일한지 hash로 확인했다.
빌드 대상은 description과 인접 MoveIt config 두 패키지이며 전체 workspace 검증은 아니다.

| 검사 | 결과 |
|---|---|
| 루트 `make build`, 별도 `CLEANY_BUILD_ROOT` | 두 패키지 빌드 통과 |
| `colcon test`: description 모델 정합성 | 46개 통과 |
| `colcon test`: 카메라 보정 프로파일 | 10개 통과 |
| MoveIt 설정과 collision 계약 | 18개 통과 |
| 기본 description launch | 명목 헤드 TF, 손목 보정 TF 미발행 확인 |
| `jetson_20261009` description launch | 보정된 헤드와 양손목 TF, RGB alias 5개 확인 |
| MoveIt mock 실행 | Pilz LIN plan-only 결과가 `ABORTED`여서 1개 실패 |

마지막 실패는 수정 전 `main`의 `d5a640a556d473b2a0af59932974b3b3b0aa4732`를
별도 workspace로 빌드해 동일 환경에서 재현했다.
`test_per_arm_position_ik_plan_and_execute`의 `_assert_pilz_lin_is_cartesian`에서
`status == 6`으로 실패하며 기대값은 `STATUS_SUCCEEDED == 4`이다.
이번 변경에서 실행기를 수정하거나 해당 검사를 생략하지 않았다.
Description launch 검사는 localhost 전용 ROS domain 212와 213에서 수행했고
실물 장치 명령은 전송하지 않았다.

원본 로그와 JUnit은 데이터 루트의
`reports/physical-calibration-20261021/`에 보관한다.
Jetson 원본 출력은 `/tmp/cleany-calibration-pr-20261021-output/`,
비교 기준 출력은 `/tmp/cleany-calibration-main-baseline-output/`이다.

### 2026-10-21 시뮬레이터 정합성 재검증

검증 코드는 `e5b26da2e077007003999d1abedb3b67447f6cba`다.
같은 Jetson 환경에서 `make build`로 두 패키지를 다시 빌드하고,
description의 모델 정합성 46개와 보정 프로파일 10개를 통과했다.
검사한 description 추적 파일 100개는 로컬 코드와 SHA-256이 일치한다.
두 URDF 모두 Jetson 모델의 마스트 간격 `0.05`를 유지하며,
`0.08`을 요청하면 오류로 거부하는 회귀 검사를 포함한다.

Gazebo의 `test_cad_frame.py` 6개도 통과했다. SDFormat 12.9.0과 스키마는
공식 ROS APT 패키지를 별도 `/tmp` 경로에 풀어 사용했다.
실제 URDF→SDF 변환 결과에서 RGB와 depth 센서의 위치 및 방향이
변경된 헤드 장착 형상과 TF metadata에 일치하는지 확인했다.
이 검증은 모델 변환 검사이며 Gazebo 동적 실행이나 실물 구동 검증은 아니다.
원본 로그와 JUnit은 데이터 루트의 `reports/pr58-fix-20261021/`에 보관한다.

### 2026-10-21 v8 손목 마운트 반영 검증

검증 코드는 `b6d52578f8e18f9a082cbb43cb2e56f6d8ebd902`다. Jetson의 같은
HEAD와 최신 미커밋 작업에서 v8 마운트만 선별했다. 메시 16개, 형상 JSON,
독립 MJCF와 xacro는 원본 바이트와 일치한다. 실제 보정 YAML 두 개도 이전
수집본과 동일하며, 이번에 hand-eye 보정을 다시 수행하지 않았다.

Ubuntu 22.04 / ROS 2 Humble / Python 3.10.12 / MuJoCo 3.11.0에서 별도
`/tmp/cleany-pr58-wrist-20261021` 사본과 `/tmp/cleany-pr58-wrist-output` 빌드
경로를 사용했다. Description 파일 121개가 로컬 코드와 SHA-256으로 일치한다.
루트 `make build`로 description과 MoveIt config 빌드를 통과했고,
`colcon test`에서 모델 정합성 46개, 보정 프로파일 10개, 마운트 자산 검사 1개,
MoveIt 설정 및 collision 계약 18개가 통과했다. 검사에는 좌우 30개 충돌
primitive의 배치와 크기, m/mm 단위, 실측 기준 치수와 원본 hash를 포함한다.

MoveIt mock 실행 1개는 앞서 `main`에서도 재현한 동일 Pilz LIN 단계에서
`ABORTED`로 실패했다. 이번 검사의 전체 colcon 결과는 이 실패를 포함한다.
새 마운트의 실물 간섭 여유, 카메라 정확도와 로봇 동작은 검증하지 않았다.
원본 로그, JUnit과 파일 manifest는 데이터 루트의
`reports/pr58-wrist-20261021/`에 보관한다.

### PR 58 카메라 수정 후속 검증 (2026-10-22)

기준은 PR 58의 `f8ffd75f741b8da9087de87b01d4ad5ea968ea37`과
`fix/pr58-wrist-camera-pose`의 미커밋 변경이다. MuJoCo의 렌더링 카메라와
optical body/site, sorting 카메라 설정을 v8 마운트 등록 위치·방향에 맞췄다.
Gazebo 레거시 및 팔 포함 CAD 모델도 같은 등록을 사용한다. 손목 마운트의
시야 가림과 좌표계 회귀 검사를 패키지 테스트로 등록했다.

CI 최적화는 PR 61에서 관리하며, 이 변경은 workflow, Makefile과 CI 인프라를
수정하지 않는다. 기존 모델 검사의 xacro 캐시와 Base 검사 등록 변경도 포함하지
않는다. 재촬영은 `tools/validation/capture_cameras.py`를 명시 실행한다.

Ubuntu 22.04 arm64, ROS Humble에서 별도 `builds/pr58-camera-only/` 경로에
관련 6개 패키지를 빌드했다. 최초 선택 빌드는 설치되지 않은 Navigation의
package 환경 파일 때문에 실패했다. Navigation C++ 의존성은 기존과 같이
충족되지 않아 선택 빌드에서 제외하고 촬영에 필요한 Python 소스만 PYTHONPATH로
제공했다. 최초 pytest도 description overlay를 source하지 않아 실패했으며,
새 빌드 overlay를 적용한 최종 description 65개, Gazebo 손목 2개와 도구 9개가
통과했다. 전체 workspace 빌드와 주행 검증을 뜻하지 않는다.

Xvfb/GLX와 software rendering에서 MuJoCo 및 팔 포함 Gazebo CAD의 실제
head/양손목 카메라 촬영을 통과했다. Xvfb는 기존 `pr58-test-cleanup/xvfb-bin`
링크로 추출된 실행 파일을 사용했다. 이 수정은 최신 main `254ffc4`와 함께
임시 worktree에서 충돌 없이 적용됐으며, 원격 CI는 실행하지 않았다.
원본 실패·성공 로그, JUnit, 카메라 PNG와 JSON, 변경 manifest는 데이터 루트의
`reviews/pr58-camera-only/`에 있다. 앞선 전체 backend 검증 원본은
`reviews/pr58-all-backends/`, CI 통합 비교 원본은 `reviews/pr58-pr61-preserved/`에
별도로 보존한다.

| 데이터 루트 아래 경로 | 내용 |
|---|---|
| `fixtures/cad-fortress-command-hold/` | 과거 시나리오 입력 |
| `fixtures/cad-fortress-command-hold-portable/` | local mesh/texture를 고정한 별도 입력 |
| `runs/scenarios/<run-name>/` | 과거 시나리오 결과, journal, 로그 |
| `runs/historical/20261010/artifacts/` | 이전 실행 원본 |
| `reports/legacy-20261010/docs/` | 과거 검증, navigation 분석, ChatGPT 인수인계 사본 |
| `experiments/legacy-20261010/` | 과거 캠페인, SLAM 코드 사본 |
| `experiments/legacy-runtime-cleanup-b3ca816/manifest.json` | 레거시 코드의 commit/hash |
| `migrations/20261010-repository-cleanup/` | 이관 대응표, hash, 검증 로그 |
| `archives/legacy-20261010/scrum-426-scenarios-20261010/worktree.tar.zst` | 이전 scenario worktree |
| `presentation/legacy-20261010/` | 발표 참조, 원본 자료 |
| `builds/<worktree-name>/` | 해당 worktree의 colcon 출력 |
| `builds/ci-optimization-evidence/` | CI 실행 metadata와 원본 로그, 테스트 수집 비교, 종료 경쟁 재현 |
| `reports/documentation-reorganization-20261013/before/` | 이번 문서 개편 전 Markdown 사본 |
| `reports/documentation-reorganization-20261013/manifest.json` | 사본 source commit, dirty 상태, 각 파일 SHA-256 |

Logger 분리와 CLI의 초기 검증은 [SCRUM-486 결과](../reports/SCRUM-486/validation.json)를 따른다.
`a401f6f`의 Ubuntu 22.04 ARM64/Humble 검증에서는 합성 ROS와 웹 fixture로
Logger 55개, Monitor 80개, Mock 통합 225개(Gazebo 1개 제외), 웹 13개와 CLI 19건을 확인했다.
초기 rosdep 선언과 launch 기본값 누락 오류는 수정 후 재검증했다. 원본은
`ubuntu-vm:/home/ubuntu/Documents/cleany-data/runs/` 아래
`pr62-conflicts-20261023/`와 `pr62-launch-local-review-20261023/`에 보존한다.

`f5e26ab` 기준 Ubuntu 22.04 ARM64/Humble에서 관측 콜백을 300ms 늦춰 Navigation 검사의 기존 실패를 재현했다. 네 속도 토픽을 함께 기다리는 미커밋 수정 후 기본 조건과 지연 조건을 각각 3회 통과했고, 패키지 검사 188개도 오류와 실패 없이 통과했다.
원본과 소스 SHA-256은 `ubuntu-vm:/home/ubuntu/Documents/cleany-data/runs/pr62-navigation-20261024/verification.json`과 같은 경로의 로그에 보존한다.

## 재현, 보존

문서 개편 전 사본의 source commit은 `fbe5854`입니다. Commit, dirty 상태, SHA-256은 보관 manifest, 과거 경로 대응은 `relocation-manifest.json`에서 확인합니다. 원본의 절대 경로와 hash는 보존하고, fixture의 현재 위치와 profile-manifest의 과거 provenance를 구분합니다.

재현할 때는 기록된 commit의 별도 checkout과 해당 환경, 입력을 사용합니다. 다른 브랜치의 자료는 해당 버전의 근거로 취급하며, 제품 결정은 KB 검토, Decision 절차를 따릅니다.

## Mission Mock 조작 통합

| 코드와 환경 | 입력과 결과 범위 | 원본 (`cleany-data/runs/` 아래) |
|---|---|---|
| `171ac80`, Ubuntu 22.04 ARM64, Humble/Fortress | 40초 공유 fixture, study-cafe seat-12, wait_for_next. Mission/Mock/Bridge ROS 208, Monitor 41, 웹 27 검사 통과. Gazebo 139.97초, 두 execution ID와 추가 빈 관측. 깨끗한 Nav2 재빌드는 삭제된 임시 dependency export 경로로 실패했으며 실행은 기존 설치 사용 | `code-review-20261020/verification.json`, `execution-evidence.json`, 로그와 DB |
| 위 코드의 문서 정리 | 링크, 명령과 소스 불변 검사. 최신 #54 병합 전 근거 | `docs-review-20261020/` |
| 이전 40초 fixture | 초기 시간 조정과 ExecuteOne 관측 자료 | `manipulation-timing-20261015-XwwCUn/`, `code-verification-20261015-NS9Nqz/` |

이 수치는 서로 겹치는 검사 묶음이며 합산하지 않는다. 현재 코드의 근거와 과거 실행을 구분한다.

### 최신 PR #54 병합 검증

`171ac80`과 PR #54 `65fdf06`의 병합 작업트리를 Ubuntu 22.04 ARM64, Humble/Fortress에서 검사했다. 원본은 `cleany-data/runs/merge-pr54-20261020/`에 보관한다. 최종 commit과 세부 결과는 해당 폴더의 `verification.json`을 기준으로 확인한다.

| 검사 | 입력과 결과 | 원본 |
|---|---|---|
| Mission과 Mock Action | 두 물체 fixture, ROS domain 193. 208 passed, Gazebo 1개는 별도 실행 | `ros-mock.log` |
| Monitor | ROS domain 194. Cartographer ROS 1개를 제외한 99 passed. 전체 실행의 해당 1개는 `cartographer_ros_msgs` 미설치로 import 실패 | `monitor-ros.log`, `monitor-ros-available.log` |
| 웹 fixture | 33 passed, 실제 Monitor가 필요한 3개는 별도 실행. TypeScript와 production build 통과 | `web-all.log`, `web-build.log` |
| Gazebo와 Mock 조작 | 새 통합 install, study-cafe seat-12, domain 195, wait_for_next. 138.51초에 SUCCESS. 서로 다른 execution ID 2개, 각 17개 세부 동작과 추가 빈 관측 확인. 두 DB 무결성 통과 | `gazebo.log`, `gazebo-evidence.json` |
| 실제 Monitor | FSM, 토픽 선택, 기록 재생, host resource와 Gazebo 지도 검사 통과. 실제 조작 화면과 ExecuteOne 상세도 확인 | `web-live.log`, `web-map.log` |
| 새 통합 build | 생성물 정리 후 16개 package와 Monitor 설치 build 통과 | `build-sim-clean.log`, `build-monitor-clean.log` |
| 계약과 문서 | 최신 #54 FSM, 청소 BT, journal과 보고 model 동일. 로컬 문서 링크 146개 확인 | `mission-core-invariants.json`, `docs-check.json` |

로컬 Nav2 cache의 CMake export가 예전 `/tmp` 추출 경로와 누락된 include 경로를 참조했다. 검증에서는 원래 prefix를 현재 cache에 연결하고 `CPLUS_INCLUDE_PATH`로 설치된 header root를 제공했다. 생성된 export 파일은 수정하지 않았다. 이 환경 보정은 새 VM의 의존성 설치를 대체하지 않으며 재부팅 후 임시 prefix를 다시 준비해야 할 수 있다. 상세 경로와 include 목록은 `nav2-environment.json`, `nav2-build-includes.json`에 남겼다.

### 병합 후 코드 검토

`a7317be` 이후 작업트리를 Ubuntu 22.04 ARM64, Humble/Fortress에서 검토했다. 원본은 `cleany-data/runs/quality-review-pr54-20261020-08o9nj4k/`에 보관하며 최종 소스 hash와 집계는 `verification.json`을 따른다.

| 검토 항목 | 반영 내용 |
|---|---|
| Action remap | Humble의 client와 server에서 이름을 명시적으로 해결하고 내부 CancelGoal 경로도 맞춤. 기본 주소와 사용자 주소에서 두 물체 실행 및 취소 검사 |
| 종료 중 DB 쓰기 | callback과 같은 잠금으로 DB를 닫고 종료 후 tick을 차단. 진행 기록을 ACTIVE로 보존하며 종료를 정지 증거로 만들지 않음 |
| 요청 검증 | 문자열이 아닌 ID와 bool 또는 소수 object ID를 core 수락 전에 거절 |
| 중복과 테스트 | 사용되지 않는 저장소 복구 함수와 미사용 import 제거. 복구 정책은 기존 ExecutionJournal에서 수행. 합성 검증 timeout 대기를 5초에서 2초로 축소 |
| 웹 검사 격리 | UI fixture의 기록 API를 명시하여 실제 로컬 Monitor에 의존하지 않도록 수정 |
| 계약과 문서 | 서버 wall 시각과 steady timeout 구분, 조회와 DB 저장 성공의 차이를 명시. FSM, 청소 BT, 보고 model과 ROS 타입 23종 유지. 웹과 서버의 정상 9단계, 17동작 일치 |

초기 `remap-before.log`와 `ids-before.log`에서 기존 실패를 확인했다. 첫 전체 ROS 실행은 종료 중 SQLite 쓰기와 닫기가 겹쳐 segfault로 중단됐고 `ros.log`에 보존했다. 종료 동기화 수정 후 해당 회귀 검사와 서버 검사 34개가 통과했다. 최종 전체 ROS 검사는 **216 passed, 1 skipped**이며 Gazebo 1개는 별도로 통과했다. 원본은 `ros-final.log`다. 문서 링크 146개와 Python 소스 31개의 syntax, Pyflakes를 확인했다.

Monitor는 99개, 웹 fixture는 33개가 통과했으며 변경한 UI fixture 6개도 다시 통과했다. Cartographer ROS 1개는 이전과 같이 `cartographer_ros_msgs`가 없어 제외했다. Gazebo는 142.52초에 두 execution ID의 수거와 각각 17동작, 추가 빈 관측을 확인했고 Mission DB와 서버 DB의 integrity_check가 모두 통과했다. 마지막 종료 잠금 수정은 별도 동시성 회귀 검사로 검증했다. 원본은 `monitor.log`, `web.log`, `ui-final.log`, `gazebo.log`, `gazebo-evidence.json`, `stage-contract.json`, `static-final.log`에 있다.

## CI 성능 검증 (2026-10-20)

기준은 Ubuntu 22.04 GitHub runner, ROS Humble의 [실행 39417986298](https://github.com/asm17-moving-agent/cleany/actions/runs/39417986298), commit `11d1ba8`입니다. 전체 job은 804초, workspace 의존성 설치 288초, 빌드와 테스트 279초, 별도 Navigation 빌드와 runtime 123초였습니다. 입력은 해당 commit의 전체 workspace 검사와 기존 motor, Navigation, Gazebo runtime 검사입니다.

| 실행 | Commit | Job 전체 | 결과 |
|---|---|---|---|
| 기존 기준 | `11d1ba8` | 13분 24초 | 통과 |
| [이미지 생성, 게시와 compiler cache 최초 준비](https://github.com/asm17-moving-agent/cleany/actions/runs/39426732345) | `d44b065` | 15분 5초 | 통과 |
| [이미지와 compiler cache 재사용](https://github.com/asm17-moving-agent/cleany/actions/runs/39429236181) | `45df20f` | 4분 37초 | 통과 |

재사용 실행은 기준보다 65.5% 짧았습니다. 이미지 pull 84초, cache 복원 2초, 전체 workspace 빌드와 테스트 90초, motor core 1초, Navigation runtime 30초, Gazebo runtime 46초가 포함됩니다. C++ 입력이 같은 상태에서 compiler cache는 264건 모두 적중했고 전체 빌드는 42.7초였습니다. Workspace 검사 집계는 1,882개, 실패 0개, 기존과 같은 skip 24개이며 CI 도구 9개, Navigation runtime 3개와 Gazebo runtime 1개도 통과했습니다. 베이스 검사를 직접 보고하도록 바꾸어 기존 중첩 래퍼가 집계하지 않던 검사도 XML에 포함됩니다.

5분 이내는 위 cache 적중 실행의 실측값입니다. 의존성 변경이나 주차 갱신으로 이미지를 다시 만들거나, C++ 변경으로 cache miss가 생기면 더 오래 걸릴 수 있습니다. 캐시가 없는 최초 실행도 전체 검사를 통과했습니다.

`1ab6d8a`의 [최초 실행](https://github.com/asm17-moving-agent/cleany/actions/runs/39424741384)은 기존 command guard SIGINT 종료 검사에서 메시지 변환 중 RuntimeError로 실패했습니다. `d44b065`에서 콜백 이후에 종료하도록 수정했습니다. 동일한 Humble 의존성 이미지의 Linux arm64 로컬 재현은 수정 전 10회 중 3회 실패, 수정 후 기존 검사 30회 연속 통과였습니다. 베이스 드라이버의 기능 검사 수집 ID 72개는 변경 전후 동일하며, 기존 모델 검사 함수 24개와 runtime 검사 파일을 유지했습니다.

`d5852af`의 [환경 확인 실패](https://github.com/asm17-moving-agent/cleany/actions/runs/39428868069)는 런타임 `ROS_HOME`이 이미지의 rosdep metadata 위치와 달라 발생했습니다. `45df20f`는 의존성 확인 명령에 이미지 생성 시의 `ROS_HOME`을 지정하며, 재설치 없이 실제 manifest의 의존성 충족 여부를 검사합니다.

원본은 데이터 루트의 `builds/ci-optimization-evidence/`에 보존합니다. `timing-comparison.json`, `baseline-39417986298.json`, `cold-*.json`, `cold-*.log`, `warm-*.json`, `warm-*.log`, `coverage-audit.json`, `base-*-collection.txt`, `shutdown-before.log`, `shutdown-after.log`에서 환경, 실패와 검사 범위를 확인합니다. `cold-artifacts/`, `warm-artifacts/`와 CI artifact는 test XML, apt/pip 버전과 compiler cache 통계를 보존합니다. Simulator와 합성 backend 검사이며 실물 로봇 검증을 뜻하지 않습니다.

## Isaac 센서 기반 정리 통합 중간 근거 (2026-10-25)

기준 commit은 `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`이며
`codex/isaac-mission-e2e`의 미커밋 구현으로 실행했다. 참고한 조작 브랜치는
`feat/jetson-manipulation-integration`의 `575e95bbba3574cdf33d08c986515e70edc857a9`다.
튜닝된 Nav2를 유지하고 Mission의 센서 관측, 실제 BT Action, Isaac RTX RGB-D,
MoveIt와 독립 안착 평가 경계를 연결했다. 현재 기록은 물리 수거 성공 근거가 아니다.

원본은 `/home/donglee/workspace/cleany/cleany-data/runs/isaac-cleanup-integration-20261025/`다.
변경 전 상태는 `before.patch`, `before-untracked.json`에 보존했다.
모델은 외부 `cleany-data/fixtures/isaac-cleanup-models`에 저장하며 YOLOE-26s,
MobileCLIP2와 Qwen3-VL-2B-Instruct를 공식 저장소에서 받았다.
Qwen revision은 `89644892e4d85e24eaac8bacfd4f463576704203`이다.
CUDA ROS image ID는 `56ed915ac1e43a0f9ad8e3d3edbf7826b93b5d5290fdf88ca17598679630566d`다.
Isaac 6.1, ROS Humble, RTX 5070 12GB, domain 178을 사용했다.

| 근거 | 결과와 범위 |
|---|---|
| `bt-tests.log` | 기존 native BT와 테스트 backend 검사 321개 통과. 물리 파지 증거가 아님 |
| `adapter-tests-02.log` | Mission의 센서 snapshot/취소/결과 프로필/주행 허가 검사 39개 통과 |
| `core-tests-02.log` | Mission 검사 96개 통과, 15개 생략, 4개 실패. 실패는 기존 주행 plugin을 GoalDirectedController로 기대하는 검사와 보존한 MPPIController 설정의 불일치 |
| `camera-03`, `camera-04` | 실내 조도 부족과 인식 깊이 토픽 불일치를 발견. 카메라와 인식 결과를 성공으로 대체하지 않음 |
| `camera-05` | RGB-D 컵 검출 성공. 정면 접근 목표의 경로 거부와 복구 시도가 발생해 취소 후 종료. 작업 실행 없음 |
| `camera-06/evaluations/inspection.json` | 실제 컵 snapshot과 3D 복원 성공. 관측 OBB 약 62×61×86mm. 모델은 재질/폐기 여부 불명확으로 review 판정 |
| `camera-06/evaluations/review.json` | 책상 측면에서 시작한 실제 Mission이 HUMAN_REVIEW_REQUIRED로 종료. 조작 실행 0회, 재관측 후 복귀 OK, home 오차 0.0627m, 최종 정지와 IDLE 확인. 로그의 planner 실패/복구/costmap clear 모두 0 |
| `camera-07/evaluations/initial-state-validity.json` | head 인접 링크 정책 보완과 tilt 0.15rad에서 MoveIt 시작 자세 유효, 충돌 0 |
| `camera-07/evaluations/inspection.json` | 기존 휴지 형상이 영상 하단에 잘려 검출 0개. 물리 수거 fixture 검사는 실패하며 빈 책상 성공으로 인정하지 않음 |

`camera-06/evaluations/mission.json`은 SUCCESS를 요구한 검증기에서 실패했다.
별도의 `validate_review.py`가 실제 HUMAN_REVIEW_REQUIRED 결과, 실행 journal의
조작 0건, 복귀 위치와 새 odometry의 정지를 확인했다. 초기 at-desk 시험이므로
대기 위치에서 책상까지의 전체 이동과 물리 수거 E2E 성공을 뜻하지 않는다.
Qwen의 첫 GPU 추론은 17.88초, 다음 동일 장면 추론은 1.95초였다. 이후 첫 관측이
오래되어 거부되지 않도록 모델 준비 단계에서 CUDA 연산을 예열한다.

`camera-08`은 960×720, focal 16mm에서 휴지 전체가 보이는 RGB-D 시험이다.
기본 UDP 수신 설정의 8초 측정은 RGB 31개, depth 3개, 동기 쌍 3개였다.
`fastdds-rgbd.xml`의 4MiB 송수신 버퍼를 적용한 수신기는 각각 32개와
동기 쌍 32개를 받았다. 호스트의 기존 rmem/wmem 상한 4MiB를 사용했으며
커널 설정은 바꾸지 않았다. 정리 실행에만 이 transport 설정을 사용한다.
수신 수정 후 실제 InspectScene 호출은 성공했으나 휴지 검출이 0개여서
물리 fixture 검사는 실패했다. YOLOE-26l 비교도 최초 프레임에서만 cup으로
검출하고 안정화된 후속 영상에서는 놓쳤다. 이를 빈 책상 성공으로 인정하지 않았다.
원본은 `rgbd-transport*.json`, `inspection-small.json`, `yolo-*-probe.json`과
`logs/inspection*.log`다. 공식 대형 모델의 SHA256은
`a612d2d505f24e14d87ec82d688b823b6cb600646664f16125ce6c84ce360da9`다.

`camera-09`는 기존 MuJoCo 종이컵 형상으로 실행했다. YOLOE-26l 검출 신뢰도는
약 0.86이고 실제 OBB는 60.5×59.3×68.4mm였다. 2B의 일회용/재사용 분류 모순은
`inspection-2b.json`에 남겼다. 공식 Qwen3-VL-4B-Instruct revision
`ebb281ec70b05090aa6165b016eac8ec08e71b17`을 NF4 이중 양자화/BF16으로 실행한
실시간 관측은 trash로 분류했으며 추론 2.24초였다. 초기 스키마 형태 응답과
형식 지시 보완 후 응답을 각각 `qwen4-probe.json`, `semantics/`에 보존했다.
bitsandbytes 0.48.1을 포함하는 image ID는
`39e37ef01eb43bb5d1303b3f6b17463a214b74f58270ed303d4ccdebde43b51d`다.

실제 Mission `isaac-e62fcc019e5b49c095c22e64e800cb24`는 조작 후보를 선택하지
못해 BLOCKED/TARGET_UNAVAILABLE로 종료했다. 물체 NOT_TOUCHED, stop_confirmed,
최종 ERROR/ready=false와 복귀 미시작을 기록했다. `blocked-boundary.json`은
이 기록에 대한 실패 종료 검사이며 수거 성공 근거가 아니다.
접근 각도를 넓힌 계획 전용 비교에서 일부 후보가 IK/충돌 검사를 통과했으나
시야 검사가 거부했다. `visibility-stowed*.json`의 실제 서비스 비교에서
팔을 접은 상태도 기본 시야 검사에 실패했고, 센서 원점을 실제 하우징인
head_tilt_link로 표현하자 통과했다. 물리 충돌 검사나 가림 검사를 끄지 않았다.
이 변경과 launch 검사 41개는 `visibility-launch-tests.log`에서 통과했다.

`camera-10`은 컵 중심을 책상 가장자리 5cm 안쪽에 두고 주행 목표를 유지했다.
컵 전체가 상판에 놓임을 fixture 검사에서 확인했다. 실제 Mission은 오른팔
준비 자세 이동에 성공했으나, 재관측 시 self-filtered 점군 경과 시간 4.03초로
기존 2초 유효성 검사를 통과하지 못해 FAILED/MOTION_FAILED로 종료했다.
`failure-stop.json`은 새 관절 피드백의 정지, 준비 자세 밖 팔의 주행 금지,
Mission ERROR와 복귀 미시작을 검증한다. 물체를 잡거나 투입한 근거는 아니다.

정지된 같은 장면에서 `probe_scene_latency.py`로 측정한 depth 경과 시간은
중앙값 0초, 원시 점군은 0.16초였다. MoveIt 출력 점군의 중앙값은 3.39초로
처리 지연이 누적됐다. 장애물 지도용 pixel_stride를 2에서 4로 바꾼 후
안정 상태 중앙값 0.40초, p95 0.47초였다. 전환 직후 과도 구간도 별도 보존한다.
원본은 `scene-latency-stride2.json`, `scene-latency-stride4.json`,
`scene-latency-stride4-steady.json`이다. 인식 RGB-D와 물체 점군은 원래 해상도를
유지하며 유효성 검사 시간을 늘리지 않았다. 기존 파지와 MoveIt 어댑터 검사
103개는 `grasp-core-tests.log`에서 통과했다.

`camera-11`은 점군 stride 4에서 접근과 새 RGB-D 재인식에 성공했지만,
열린 손가락이 시야 원뿔을 가려 FAILED/MOTION_FAILED로 종료했다.
`visibility-trace.json`과 `visibility-jaw-replay.json`은 같은 MoveIt 요청을
그리퍼 0, 1.2, 1.4rad로 비교한 읽기 전용 검사다. 선택된 접근 자세가
닫힌 상태에서는 통과하고 실제 열림 1.2rad에서는 거부됨을 재현했다.
이후 선택기는 설정된 열림 위치로 시야를 검증하도록 수정했다.
팔 궤적 형식과 시야 허용 범위는 유지했다. 수거 성공 근거는 아니다.

`camera-12`은 열린 그리퍼 시야 검사 수정 후 팔 이동 전에 도달 가능한 후보를
찾지 못했다. 추가 계획 전용 비교는 열림 1.4rad와 고정 손가락 여유 6mm를
시험했으며 물리 충돌 또는 IK 때문에 실패했다. `collision-rejections.json`은
고정 손가락과 대상 OBB의 약 1.3mm 겹침을 기록한다. `camera-13`의 3cm 가까운
접근 시험도 시야 검사를 통과하지 못했다. 이 시험용 접근/여유 설정은 기본값으로
채택하지 않았다. 각 입력과 실행 명령은 `provenance/component.json`에 보존한다.

이후 기존 손목 영상 handoff 경로에 nominal CAD mount의 RTX RGB를 연결했다.
인식 노드의 명시적 wrist detector 주입과 simulation-only TF launch 검사는
`wrist-adapter-tests-isolated.log`에서 37개 통과했다. 최초 같은 ROS domain의
검사는 기존 실행의 카메라 구독과 충돌해 1개 실패했으며, 별도 domain 197로
분리한 결과를 채택한다. 이 검사는 물리 파지 성공 증거가 아니다.

`camera-14`은 head 재관측 후 손목 RTX RGB handoff와 실제 선형 접근까지
통과했다. `014_wrist_response.cdr`과 `media/right-wrist-*.png`에 근거를 남겼다.
그러나 그리퍼가 -0.30rad까지 닫혀 접촉 유지 조건을 충족하지 못했고 실제 Mission
`isaac-76203aa9bbe04062a3dc97855f406112`는 GRASP_FAIL로 종료했다.
object_state=UNKNOWN, stop_confirmed=true, arm_recovered=false, 복귀 미시작이다.
최초 perception 예열의 초기화 오류는 수정 후 재시작했고
`provenance/perception-restart.json`에 사용 코드와 경로를 보존했다.
예열 포함 관련 검사 19개는 `wrist-preload-tests.log`에서 통과했다.

`camera-15`는 접촉력 진단 API에 필요한 contact-report 설정이 없어 시뮬레이터
시작 후 실패했다. 물리 미션 근거로 사용하지 않는다. 이를 추가한 `camera-16`은
손목 handoff에서 뒤집힌 컵을 일반 YOLOE가 놓쳐 접근 전에 종료했다.
`wrist-rotation-probe.json`의 동일 원본 비교는 0/90/270도에서 검출이 없고
180도에서 cup 신뢰도 0.559를 기록했다. 원본은 `wrist-failures/`에 보존한다.
한 번의 회전 재추론과 원본 bbox/mask 복원 검사 2개가 통과했다.
PhysX 진단은 파일로만 저장하며 perception/계획/성공 판정 입력으로 사용하지 않는다.

`camera-17`은 원본 또는 180도 재추론을 사용하는 손목 handoff와 실제 접근을
통과했으나, 닫힘 중 컵이 약 32.1mm 밀려 GRASP_FAILED로 종료했다.
`grasp-physics-summary.json`의 손가락 최대 접촉력은 약 35.3/36.8N이고
`grasp-physics.jsonl`은 전체 위치/접촉 기록이다. 접촉이 발생했다는 사실을
파지 성공으로 간주하지 않았으며 들어올리기/투입은 실행하지 않았다.
이 시험의 삽입 offset 0.026m는 `provenance/component.json`에 기록했다.

`camera-18`의 접근 거리 0.52m, 삽입 offset 0.050m 시험은 IK 또는 대상/상판
충돌 검사에서 거부돼 실행하지 못했다. 원본 서비스 충돌 응답은
`collision-rejections.json`에 있다. 정지 상태에서 offset만 0.036m로 바꾼
계획 전용 검사는 `planning.json`에서 통과했다. 이 비교는 물리 성공이 아니다.

`camera-19`은 0.52m 접근과 0.036m 삽입에서 준비 자세 이동과 head 재관측을
통과했지만 손목 top view에서 컵을 검출하지 못해 접근 전에 종료했다.
`wrist-rotation-probe.json`과 `wrist-prompt-probe.json`은 같은 실패 영상의
회전 및 문구 비교다. 검출 실패를 통과로 바꾸지 않았다. 이후 기존 후보 생성의
기울기를 수직 기준 64도로 제한한 계획 전용 검사는 통과했으며 `planning.json`에
기록했다. 이는 일반 공개 모델의 top-view 한계를 피하는 시험 설정이다.

`camera-20`은 0.52m 접근, 0.036m 삽입, 64도 기울기에서 실제 손목 handoff와 접근, 접촉 유지 판정을 통과했다. 그러나 들어올리기 전 역방향 retreat 계획이 최대 5.84393도 자세 오차로 거부됐다. Mission은 LIFTING에서 STOP_UNCONFIRMED로 끝났고 복귀하지 않았다. 물리 수거 성공 근거가 아니다. 같은 요청의 계획 전용 재생에서 회전 잔차 가중치 0.002는 오차를 0.06278도로 낮췄고 기존 1mm/5도 및 MoveIt 충돌 검사를 통과했다. 원본은 `camera-20/evaluations/cartesian-replay.json`, 비교 로그는 `camera-20/logs/cartesian-replay-radius1mm.log`다. 관련 회귀 검사 92개는 `orientation-tests.log`에서 통과했다.

`camera-21`은 회전 잔차 보간 수정 후 접촉 유지 실패로 GRASP_FAILED, stop_confirmed=true로 종료했다. 닫힘 중 양쪽 손가락에 약 44N의 접촉력이 기록됐고 컵이 밀려났다. `camera-22`의 6초 그리퍼 동작 비교는 준비 자세 이후 OBB가 89mm 이동한 것으로 복원돼 STALE_TARGET으로 종료했으므로 닫힘 속도 효과는 미검증이다. 실제 컵은 움직이지 않았다. 원본 RGB-D 재생(`camera-22/evaluations/depth-mask-reconstruction.json`)에서 3294개 mask 픽셀 중 3195개 깊이 연결 성분을 남기자 OBB가 188.8×60.1×191.8mm에서 60.5×60.1×68.4mm로 복원됐다. 소수 전경/배경 조각을 물체 외곽으로 포함한 오류였으며, 이동 허용치를 완화하지 않았다.

`camera-23`은 깊이 mask 보정과 6초 gripper 동작에서 준비 자세 이후 재관측, 접근과 파지 유지까지 통과했다. 컵은 실제 역방향 retreat 중 올라갔으나 접촉 유지 속도 검사에서 LIFTING/STOP_UNCONFIRMED로 종료했다. `contact-stability.json`의 물리 높이는 최대 0.82869m(초기 0.75420m)이며 이것만으로 Lift 확인/안착 성공으로 간주하지 않는다. 종료 후 3초 동안 그리퍼 위치 범위는 약 0.000013rad였지만 보고 속도는 0.094~0.097rad/s로 정지 기준을 넘었다. `joint-velocity-audit.jsonl`과 `grasp-physics.jsonl`에 두 관측을 보존했다. PhysX split-impulse 계산은 위치 차분과 보고 속도가 일치하지 않을 수 있다([공식 설명](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html)). 접촉/정지 허용치를 유지하고 solver 설정을 비교한다.

`camera-24`의 TGS 반복 수 128 position/1 velocity 비교에서는 GRASP_LOST 이후 stop_confirmed=true로 종료했다. 컵은 파지 판정 후 들어올리기 시작 시 빠졌으며 안착/복귀는 미실행이다. 종전 32/8과 다른 시험 설정은 provenance에 보존했다. 컵 바닥의 물리 재질 누락과 마찰 결합 평균/최대 차이를 확인해 후속 비교에서 수정한다. 원래 MuJoCo 자산의 sliding friction 1.5는 유지하며 torsional/rolling friction은 아직 동일하게 이식되지 않았음을 구분한다.

`camera-25`은 컵 바닥에 동일 물리 재질을 바인딩하고 마찰 결합을 max로 복원했으나 닫힘 중 컵이 튀어 GRASP_FAILED로 종료했다. stop_confirmed=true, 팔 복귀/안착은 미확인이다. `camera-26`은 같은 조건에서 simulation 그리퍼의 operating torque cap만 2.6477955Nm에서 0.2Nm로 낮춘 비교다. 이는 실물 calibration 값으로 주장하지 않는다.

`camera-26`도 GRASP_FAILED, stop_confirmed=true로 종료했다. 컵은 그리퍼가 1.20rad로 열린 접근 중부터 10mm 이상 이동해, 낮춘 닫힘 토크만으로는 초기 고정 손가락 접촉을 해결할 수 없었다. 컵이 바닥으로 떨어진 물리 위치도 원본 기록에 남았다.

`camera-27`은 0.026m 삽입, 0.008m 고정 손가락 여유, 0.016m opening margin과 기존 direct vertical lift 옵션의 조합이다. 실제 physics-settings.json에서 solver 128/1과 양쪽 gripper 0.2Nm cap을 확인했다. 접근은 통과했으나 닫힘에서 컵이 기울어 GRASP_FAILED, stop_confirmed=true로 종료했다. 수직 리프트는 실행되지 않았다.

`camera-28`은 기존 geometric predictor의 상단 접촉 깊이 0.015m와 삽입 0.040m를 적용한 비교다. 실제 재인식/접근 이후 GRASP_FAILED, stop_confirmed=true로 종료했다. `evaluations/contact-geometry.png`는 기록된 PhysX body pose와 원본 URDF collision mesh를 겹친 진단이며, 닫힘 중 컵이 손가락 끝 방향으로 빠지는 모습이 보인다. 이 투영만으로 접촉을 판정하지 않으며 원본 `grasp-physics.jsonl`의 힘과 함께 사용한다. 물체는 물리적으로 바닥으로 떨어졌고 리프트/수거함 안착은 실행하지 못했다.

`camera-29`의 0.42m 접근 목표는 실제 Nav2가 현재 위치에서 같은 목표로 가는 계획조차 거부했다. costmap clear가 발생해 진단 컨테이너를 종료했다. 접근/파지 시험으로 진행하지 못했으므로 48도 기울기와 0.045m 삽입의 효과는 미검증이다. `diagnostic-termination.json`에 강제 종료와 미실행 범위를 기록했다. 이 목표를 운용 설정으로 채택하지 않는다.

`camera-30`은 camera-27 대비 cup collider의 minTorsionalPatchRadius만 0.08/1.5m로 바꾼 비교다. PhysX single-anchor torsion friction의 근사이며 원래 MuJoCo rolling friction은 이식하지 않았다. 기준 소스와 해시는 `provenance/torsional-friction-source.json`에 있다. 실제 접근 이후 GRASP_FAILED, stop_confirmed=true로 끝났으며 이 근사값을 운용 기본으로 채택하지 않았다. 이 폴더의 camera27-contact-geometry.png는 **camera-27 기록을 재생**한 진단이며 사용 JSON과 스크립트도 provenance에 보존한다.

`camera-31`은 cup 중심을 책상 끝에서 0.035m 안쪽으로 옮기고 삽입을 0.050m로 늘린 비교다. 크기와 질량은 유지했다. 오른팔은 열린 gripper와 관측 대상/환경의 충돌 또는 IK 실패로, 왼팔도 IK/경로 검사로 후보가 거부됐다. 팔을 움직이지 않고 TARGET_UNAVAILABLE로 종료했으며 깊은 파지의 물리 효과는 검증하지 못했다.

`camera-32`은 camera-27의 삽입만 0.026→0.036m로 늘린 비교다. 실제 접근과 양쪽 접촉은 있었으나 닫힘 중 컵이 튀어 GRASP_FAILED, stop_confirmed=true로 종료했다. 물리 최대 높이 0.84818m는 이 비의도적 움직임이며 리프트 성공이 아니다. `mesh-gap-diagnostic.json`은 camera-27의 고정 jaw와 cup 외벽 간 초기 약 5.5mm 간격, camera-23의 양쪽 접촉 위치를 원본 mesh와 기록 pose로 비교한 샘플링 진단이다. 해당 입력 프레임을 별도 보존했으며 실제 접촉 힘 기록을 대체하지 않는다.

`camera-33`은 camera-32 대비 simulation 그리퍼 토크 상한만 0.2→0.02Nm로 낮췄다. 실제 Tensor API max_force는 0.01999999955Nm였다. 접촉 감지 후 유지 조건이 깨져 ConfirmGrasp/STOP_UNCONFIRMED로 종료했다. 컵이 물리적으로 이탈했고 리프트/안착/복귀는 실행하지 않았다. 작은 토크에서도 moving jaw가 크게 되튀어, 추가 토크 조정 전에 실제 runtime armature와 drive 값을 읽는 plant audit를 별도로 수행한다.

`plant-audit-01/evaluations/physics-settings.json`은 모델 추론/미션 없이 controller 활성화 직후 읽은 물리 설정이다. 모든 관절 armature는 0.028000000864kg·m²로 원본 설정과 일치했다. stiffness 1200Nm/rad, damping 약 60Nm·s/rad와 gripper cap 약 0.02Nm도 확인됐다. 따라서 모터 관성 누락 가설은 이 환경에서 기각했다. camera-33의 원본 mesh/pose 재생 `table-gap.json`에서 moving jaw와 해당 desk 상면 사이 최소 샘플 간격은 약 26.1mm였으며, 상판 침투가 관측된 것으로 주장하지 않는다.

`camera-34`은 jaw net-force 진단에 필요한 ContactReportAPI가 없어 startup 후 종료됐다. 미션/파지 근거로 사용하지 않는다. 읽기 전용 진단을 위해 해당 API를 physics 초기화 전에 적용하도록 수정했다.

`camera-35`은 camera-32의 물리 조건에 읽기 전용 net contact force 및 렌더 전후 상태 진단을 추가한 재실행이다. ConfirmGrasp에서 접촉 유지 실패, GRASP_FAILED/stop_confirmed=true로 종료했다. `integration-audit-summary.json`과 원본 jsonl에 렌더 전후 joint position/velocity/target, root, payload 변화량 및 cup 접촉력을 뺀 jaw net force를 기록했다. 이 표본에서는 렌더링에 의한 물리 상태 변경이나 cup 이외 접촉력이 관측되지 않았다. 후속 직접 joint-target 물리 재생은 원인 분리용이며 ROS 제어/Mission E2E 성공 근거로 사용하지 않는다.

`contact-replay-01/02`는 camera-32 접근 중 기록한 관절/베이스/컵 상태에서 시작한 physics-only 비교다. ROS controller, 카메라 추론, Mission을 실행하지 않았으며 각 독립 trial 사이에 fixture 상태를 초기화했다. 500Hz, 128/1 반복 수와 동일한 6초 닫힘을 유지하고 TGS와 PGS를 비교했다. TGS에서는 6개 torque cap 중 0.5Nm만 마지막 표본에 양쪽 접촉이 남았으나 컵 이동과 큰 팔 추종 오차가 있었다. PGS에서는 0.05/0.1/0.2/0.5/2.6477955Nm의 마지막 표본에 양쪽 접촉이 남았고 0.2Nm의 최종 gripper 속도는 -0.00152rad/s였다. 마지막 접촉 표본만으로 안정 파지나 리프트 성공을 주장하지 않는다. 재현 script, 초기 상태, 이미지 명령, 소스 해시는 각 `provenance/`, 원본은 `evaluations/cap-*.json`과 `summary.json`에 보존했다.

`camera-36`은 camera-35의 solver만 PGS로 바꾼 실제 Mission component 실행이다. GRASPING 접촉 유지와 종료 시 stop_confirmed=true를 통과했지만 수직 lift 계획이 OctoMap/right_gripper_frame 충돌로 거부돼 LIFTING/MOTION_FAILED로 종료했다. 리프트, 투입, 복귀는 실행하지 못했다. `/move_group depth_cloud.sensor_plugin` 조회는 표준 PointCloudOctomapUpdater였다. 기존 shared launch가 start_simulator=true일 때만 KnownGeometry updater를 고르는 조건 때문에 external Isaac 실행에서 기존의 관측 형상 내부 잔상 정리가 선택되지 않았다. Isaac cleanup launch에서 기존 plugin을 명시해 후속 비교한다. 충돌 무시나 OctoMap 전체 삭제는 적용하지 않는다.

`camera-37`에서 실행 중 parameter 조회로 KnownGeometry updater 선택을 확인했다. 파지 유지와 종료 시 stop_confirmed=true는 통과했으나 수직 lift 계획은 25/73 샘플에서 OctoMap/right_gripper_frame 충돌로 거부됐다(기존 표준 updater는 10/73). 해당 표본의 개선만으로 잔상 문제가 해결됐다고 주장하지 않는다. 컵은 책상 위에서 약 6mm 이동했으며 리프트/투입/복귀는 미실행이다. 후속 비교는 기존 reverse approach retreat 경로를 사용한다.

`camera-38`은 기존 역방향 접근 retreat로 LiftObject 동작을 완료했고 실제 cup 중심은 0.75420→최대 0.83529m로 이동했다. 하지만 ConfirmHeld의 손목 RGB projection이 화면 밖으로 판정돼 GRASP_LOST, stop_confirmed=true로 종료했다. 따라서 리프트 확인/안착 성공은 아니다. 당시 전체 MoveIt 장면은 `lift-scene.cdr`, 근접 손목 영상은 `media/right-wrist-000300.png`에 보존한다. `wrist-projection.json`의 기록 OBB와 정지 후 FK 비교에서 화면 내 예상 bbox 비율은 0.264로 기존 0.6 기준 미달이다. 삽입을 15mm 줄이는 가상 투영에서는 0.789, 20mm 줄이면 1.0이었다. 이는 미실행 기하 비교이며 다음 실제 시험에서 파지와 시야를 함께 검증한다.

`camera-39`은 삽입을 0.021m로 줄인 비교다. Grasping 판정 후 lift 시작 시 실제 컵을 놓쳐 GRASP_LOST, stop_confirmed=true로 종료했다. 손목 시야 개선을 물리 파지 개선으로 간주하지 않으며 얕은 삽입을 운용 설정으로 채택하지 않는다. 다음 비교는 camera-38의 물리 조건에서 기존 head-only RGB-D 검증과 1회 재관측 경로를 사용한다.

`camera-40`의 기존 head-only 실행은 준비 자세에서 카메라-대상 영역을 팔이 가려 TARGET_UNAVAILABLE로 끝났고 물체를 건드리지 않았다. 이를 허용하도록 visibility gate를 완화하지 않았다. 접근 HANDOFF와 lift 관측 수단을 분리하는 선택 옵션 `sorting_lift_observation_source=head_rgbd`를 추가해 기존 head 재검출/높이/물체 연관 검증을 재사용한다. 기본 auto 동작은 유지하며 깊이 재관측 실패, 위치 불일치 및 기존 손목/launch 검사 65개가 `head-lift-tests.log`에서 통과했다. 실제 물리 검증은 후속 실행으로 구분한다.

`camera-41`에서 새 head lift 옵션과 빌드 13개 패키지는 로드됐지만, LiftObject 중 gripper 속도 변동으로 기존 retention guard가 실패해 head 확인 전에 STOP_UNCONFIRMED로 종료했다. 물리 cup은 약 0.835m까지 올라갔으며 이후에도 접촉은 남았지만 실패를 통과로 바꾸지 않았다. `joint-velocity-audit.jsonl`에 lift 중 약 ±0.15rad/s 변동을 보존했다. 후속 비교는 sorting 가속 설정 0.30m/s, 0.80m/s² 대신 기존 일반 Cartesian 값 0.10m/s, 0.20m/s²를 사용하며 접촉/정지 기준을 유지한다.

`camera-42`은 Cartesian 속도/가속도를 기존 일반값으로 낮췄으나 LiftObject의 retention guard 실패와 STOP_UNCONFIRMED가 반복됐다. 저속화만으로 해결됐다고 주장하지 않는다. 원본 `motors.json`에는 gripper 수동 damping 0.611437854Nm·s/rad, frictionloss 0.052Nm가 있지만 기존 runtime friction_properties는 모두 0이었다. 후속 camera-43은 이 원본 값의 이식을 비교한다. [PhysX 관절 마찰 문서](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html)에 따라 static/dynamic effort와 viscous coefficient를 별도로 설정하며 USD 각도 단위 변환을 provenance에 기록한다. 로봇 실물 보정값이나 완전히 같은 접촉 해석기로 주장하지 않는다.

`camera-43`의 `motor-mapping.json`에서 14개 관절 모두 원본 frictionloss/damping/armature와 runtime 값이 일치했다. LiftObject 동안 접촉을 유지했고 ConfirmHeld의 head 재검출 이후 재관측 자세를 찾지 못해 GRASP_LOST, stop_confirmed=true로 종료했다. 파지/리프트 안정성 개선과 시각 확인 성공을 구분한다. 원본 수동 관절 파라미터 이식은 cleanup 어댑터에 반영했다. 기록된 contact joint state와 선택 OBB로 계산한 `wrist-projection.json`은 삽입 12mm 감소 시 화면 비율 0.658을 예측하며, 후속 실제 비교에서 기존 wrist 검사를 사용한다.

`passive-friction-comparison.json`은 최초 파지 감지 로그부터 Mission 취소 시작까지의 기록된 gripper 속도를 비교한다. 최대 절댓값은 camera-41 0.1550, camera-42 2.7319, camera-43 0.01462rad/s이며 기존 0.05rad/s를 넘은 표본 비율은 각각 7.4%, 20.7%, 0%다. 실행 길이와 종료 원인이 다르므로 성공률 비교로 해석하지 않는다. camera-43은 여전히 시각 리프트 확인을 완료하지 못했다.

`camera-44`의 0.024m 삽입은 실제 lift 중 컵이 빠져 STOP_UNCONFIRMED로 종료했다. 깊은 파지의 안정성과 손목 시야를 동시에 확보한 근거가 아니다. `head-pose-probe.json`의 정지 상태 계획 검사에서 tilt 0.15/0.20rad는 유효했지만 0.25rad 이상은 top_base_link/head_tilt_link 충돌이었다. 관절을 실제로 움직이거나 충돌을 무시하지 않았다.

`camera-44/evaluations/reobservation-probe.json`은 camera-43의 인식 OBB와 계획 lift endpoint를 사용한 기하 재생이다. 기존 중앙 행 ray는 이동 제한 0.2m 안에서 후보가 없었고, 추가 화면 행에는 높이와 전체 sphere 시야 조건을 만족하는 후보가 2개 있었다. 기존 경로에 후보가 없을 때만 화면 위/아래 행을 보완했으며 모든 IK/충돌/관측 판정은 유지한다. 기록 calibration 기반 회귀 검사와 기존 wrist/sorting 검사 190개가 `reobservation-tests.log`에서 통과했다. 실제 자세 이동과 재인식은 다음 실행으로 검증한다.

`camera-45`는 head 재관측의 off-center 후보 생성 보완 후, 실제 물리 리프트와 재관측 MoveIt 이동을 통과했다. 그러나 컵 대부분이 그리퍼 뒤로 가려져 head RGB-D 재검출에 실패했고 ConfirmHeld/GRASP_LOST, stop_confirmed=true로 종료했다. 수거함 안착과 복귀는 실행하지 않았다. 같은 head 원본과 컵 주변 확대 영역 모두 공개 YOLOE 검출 0건이었다(`evaluations/held-detection-probe.json`). 이전 wrist 원본을 재생한 `partial-projection-probe.json`도 0건이며 OBB convex hull의 부분 가시성 0.412는 기존 0.6 문턱에 못 미쳤다. 가시성/검출 문턱을 낮추지 않았다.

`camera-45/evaluations/roll-probe.json`, `pitch-probe.json`, `view-probe.json`은 제어 명령 없는 IK/FK 검사다. 현재 컵 중심에서 wrist pitch -0.9rad, 기존 roll 기준을 유지하는 해는 충돌 검사를 통과했고 camera 시선과 approach 축 내적 절댓값이 0.919에서 0.790으로 줄었다. 이 결과는 시각 검출이나 실제 파지 유지의 근거가 아니다. 선택적 재관측 pitch 설정의 carry 제한 유지, 도달 불가/비유한 값 거부와 기존 관측 경로 회귀 검사는 `reobserve-pitch-tests.log`에서 59개 통과했고 `reobserve-pitch-build.log` 빌드가 통과했다. 후속 물리 비교는 `camera-46`에 기록한다.

`camera-46`은 fixed wrist pitch 옵션을 활성화했지만 파지 직후 안정화 검사에서 GRASP_FAILED, stop_confirmed=true로 끝났다. 재관측 옵션은 실행되지 않아 그 효과는 미검증이다. `grasp-comparison.json`의 센서 후보와 물리 접촉 비교에서 43/45/46 모두 첫 접촉은 moving jaw였고, 46에서는 양쪽 접촉이 생기기 전 컵이 약 54mm 이동했다. 이 실패를 재관측 코드의 결과나 성공했던 파지의 재현성 근거로 해석하지 않는다. 동일 설정 반복은 `camera-47`이다.

`camera-46/evaluations/visual-prompt-probe.json`은 [YOLOE 공식 참조 이미지 모드](https://docs.ultralytics.com/models/yoloe/)의 오프라인 비교다. camera-45 파지 전 RGB에서 검출한 cup(0.8699)의 bbox만 참조했고 ground truth 영역은 사용하지 않았다. 가려진 head 영상과 기존 wrist 영상 모두 confidence 0.25에서 검출 0건이었다. 이 대안을 runtime 성공 경로로 추가하지 않았다.

`camera-47`의 동일 조건 반복은 물리 파지, 리프트와 pitch -0.9rad 재관측 이동까지 통과했다. 컵은 물리적으로 유지됐지만 head 영상에서 wrist camera bracket에 가려져 ConfirmHeld/GRASP_LOST, stop_confirmed=true로 종료했다. `media/head-000700.png`, `evaluations/grasp-physics.jsonl`이 원본이다. `roll-wide-probe.json`의 ±45~90도 비교는 제어 명령 없는 FK/IK이며 실제 시각 검출 근거가 아니다.

`camera-48`은 재관측 pitch -0.9rad에 contact roll 기준 -60도 회전을 추가한 비교다. 시험 YAML에서 carry roll 범위를 ±60도로 명시 설정했으며 기존 기본값은 바꾸지 않았다. 이 범위 변화는 실제 자세 동작의 변경이며 기존 ±2도 운반과 동일한 조건이라고 주장하지 않는다. 접촉/시각/높이/충돌 문턱은 유지한다. 재관측 offset이 설정 carry 범위를 자동 확장하지 않는 회귀 검사와 기존 경로 검사 65개가 `reobserve-roll-tests.log`에서 통과했고 build도 통과했다.

`camera-48`도 실제 파지, 리프트와 재관측 이동 후 ConfirmHeld/GRASP_LOST, stop_confirmed=true로 종료했다. `current-detection-probe.json`에서 새 head 영상과 확대 영역을 0/90/180/270도로 비교했으나 YOLOE-26l와 공개 YOLO11l-seg 모두 confidence 0.25에서 검출 0건이었다. YOLO11l-seg는 공식 assets v8.4.0에서 받았으며 `coco-probe.json`에 SHA-256을 기록했다. runtime detector로 채택하지 않았다.

`camera-48/evaluations/raised-view-probe.json`은 관측 물체 중심을 5~12cm 더 올린 후보의 계획 전용 검사다. 여러 높이/위치에서 pitch -1.6rad, 추가 roll 없이도 유효한 IK를 얻었다. 현재 높이 유지/하강 후보만으로는 도달하지 못한 관측 자세였다. `sorting_reobserve_max_raising_m` 기본값 0으로 기존 동작을 보존하고, 양수 설정 때만 상승 후보를 추가한다. 전체 이동 0.20m와 전체 sphere 화면 포함의 독립 투영 검사 등을 포함한 69개 검사가 `reobserve-raised-tests.log`에서 통과했다. `camera-49`는 최대 상승 0.10m, pitch -1.6rad, 원래 carry roll 단계 [2,5,10,20,30] 설정의 물리 비교다.

`camera-49`는 접근과 접촉 안정화 이후 리프트 도중 컵이 실제로 빠졌다. 재관측 자세를 실행하기 전에 LIFTING/STOP_UNCONFIRMED, stop_confirmed=false로 종료했으며, 바닥에 떨어진 payload pose가 물리 기록에 남았다. 최대 상승 후보의 효과는 이 시험으로 판정할 수 없다. 동일 동작 반복 `camera-50`에는 읽기 전용 collider contact/rest offset 기록을 추가했다. 컵의 접촉 offset은 기존 1mm이며, 가져온 그리퍼 collider의 실제 runtime 값은 별도 확인한다.

`camera-50/evaluations/physics-settings.json`의 tensor runtime shape offset은 fixed gripper frame 0.078~0.656mm, moving jaw 0.295~0.360mm, cup 1.000mm였고 rest offset은 모두 0이었다. 큰 기본 contact margin이 적용됐다는 가설은 이 값으로 배제하며, offset을 변경하지 않는다. 동일 값의 authored/default 구분은 `gripper-colliders.json`에 남겼다.

`camera-50`도 리프트 도중 실제 컵 낙하와 LIFTING/STOP_UNCONFIRMED로 끝나 상승 재관측은 미실행이다. `camera-51`은 시뮬레이션 gripper operating torque를 0.2→0.1Nm로 낮추고, 원래 passive joint friction/damping을 유지한 상태에서 개폐 시간을 20초로 늘린 비교다. 성공/접촉/정지 문턱은 변경하지 않으며 아직 채택 설정이 아니다.

`camera-51`은 0.1Nm 상한에서 pregrasp의 동시 gripper 개방 목표를 추종하지 못했다(목표 오차 약 1.1666rad). 접근 단계 MOTION_FAILED, stop_confirmed=true, NOT_TOUCHED로 종료했다. 이 토크 상한은 채택하지 않는다. `camera-52`는 camera-50의 0.2Nm/6초 조건으로 되돌리고 PGS velocity iteration만 1→8로 늘린 비교다.

49/50의 외부 joint audit에서는 취소 후 2~5초 이내 모든 관절 속도가 0.02rad/s 미만이었지만, BT의 정지 확인은 실패했다. 이 외부 관측만으로 종료 결과를 바꾸지 않았다. 원인 구분을 위해 StopAndAssess timeout에 Action 종료/대기 응답, 피드백 유효성, 정지 샘플, 움직이는 관절의 진단 로그를 추가했다. 관련 ROS operations 검사 59개가 `stop-diagnostic-tests.log`에서 통과했다.

`camera-52`는 PGS 128/8에서 물리 리프트를 통과하고 +5cm 상승 재관측 이동을 시작했다. 이동 중 MoveIt error -3(MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)으로 중단됐고 stop_confirmed=true였다. 컵은 물리적으로 유지됐으며 `head-000600.png`에 중단 자세가 보인다. 현재 정지 자세의 GetStateValidity는 유효했다(`head-pose-probe.json`의 0.15rad 항목). 어느 미래 경로 구간이 무효화됐는지는 아직 확인하지 못했으므로 충돌 마스킹 오류나 실제 장애물로 단정하지 않는다. 이 차이를 확인하기 위해 후속 시험부터 joint MoveGroup Goal/Result CDR도 저장한다.

Joint motion tracing과 재관측 옵션을 포함한 관련 검사 264개가 `joint-trace-tests.log`에서 통과했다. 기존 테스트의 수동 parameter fixture에도 새 옵션의 기본값을 추가했다. `camera-53`은 camera-52와 같은 물리/제어 설정으로 joint Goal/Result 원본을 확보하는 반복이다.

`camera-53`의 같은 PGS128/8 반복에서는 물리 리프트와 +5cm/pitch -1.6rad 재관측 이동이 모두 성공했다. 이후 head RGB-D 재검출 실패로 ConfirmHeld/GRASP_LOST, stop_confirmed=true였다. `023_joint_motion_request.cdr`, `024_joint_motion_result.cdr`가 실제 재관측 경로의 원본이며 `scene-ring/`은 종료 직전 장면 8개다. `media/head-000600.png`는 중간 저장 영상이므로 최종 자세라고 간주하지 않는다. 종료 후 실제 RGB topic을 다시 수신한 `evaluations/final-head.png`와 stamp JSON을 최종 자세 진단에 사용했다. 같은 영상의 전체/확대 영역 모두 기존 class 집합과 cup/paper cup/bowl/container/glass 비교에서 confidence 0.25 검출 0건이었다. 각 원본 결과는 `final-detection-probe.json`, `container-label-probe.json`이며 runtime label 설정은 바꾸지 않았다.

`camera-54`는 원래 68.4mm 컵과 분리한 높이 95mm/지름 61.2mm hollow cup 시험이다. 형상 생성기는 동일하고 질량은 기존 가정 8g을 유지했으며 `room.json.cleanup.fixture_variant`에 차이를 기록했다. 원래 컵의 실패를 수정한 것으로 간주하지 않는다. 물리 리프트 이후 재관측 MoveIt error -3으로 종료했고 stop_confirmed=true, 안착/복귀는 미실행이다. `failed-path-validity.json`은 종료 후 장면에서 저장한 80개 경로점 중 6개가 OctoMap과 attached target 또는 moving jaw 충돌임을 보여 준다. 정확한 원래 중단 시각의 장면 재생은 아니다.

`camera-54/evaluations/held-geometry-audit.json`의 읽기 전용 물리 oracle 비교에서 리프트 후 관측 기반 attachment 중심과 실제 cup 중심 차이는 약 13.4mm였다. `hull-coverage.json`은 실제 컵 표면이 관측 mesh face 바깥으로 최대 약 13mm 나오는 상태를 기록했다. 이 물리 좌표는 planner/인식 입력으로 사용하지 않는다. 기존 15mm padding에서 radial mesh self-mask와 face-normal 내부 leaf 정리가 서로 다른 영역을 사용해, 실제 self return이 있는 경계 voxel을 점유로 남기는 경우를 `MasksFacePaddedReturnEvenWhenItsVoxelStraddlesTheBoundary`로 재현했다. 수정 전 같은 입력 3프레임 모두 실패했다(`face-mask-before.log`). 외부로 판정된 촬영 점에 같은 face-normal padding을 적용하고 기존 model endpoint 처리로 연결한다. padding 값, 전역 지도 삭제, 충돌 허용 목록은 바꾸지 않는다. 관측되지 않은 경계 voxel은 기존 8-corner 규칙으로 보존한다. 이 회귀 재현 자체가 camera-54 중단의 유일한 원인이나 수거 성공을 입증하지는 않는다.

Face-normal point masking의 수정 후 현재 shared library를 사용한 `face-mask-current-library-tests.log`에서 32개 검사(CTest 4개 묶음)가 모두 통과했다. 첫 직접 CMake 시험은 install 경로의 이전 shared library를 로드했으므로 채택하지 않으며 `face-mask-after.log`에 그 오류를 보존했다. `face-mask-build.log`의 13개 ROS 패키지 빌드가 완료됐고, 같은 95mm fixture/물리/동작/인식 설정의 `camera-55`에서 변경된 updater만 비교한다.

`camera-55`도 재관측 MoveIt error -3, stop_confirmed=true로 끝났다. 종료 후 경로 62점 중 4점이 OctoMap과 충돌했다. 이번 물리 oracle 대조에서는 attachment 중심 오차 약 10.8mm, 관측 hull face를 벗어나는 컵 표면 약 16.5mm를 확인했다. 따라서 15mm 점 필터 불일치 수정만으로 통합 문제가 해결됐다고 주장하지 않는다. 원본은 해당 `failed-path-validity.json`, `held-geometry-audit.json`, `hull-coverage.json`이다.

`camera-56`은 같은 95mm 컵/동작/물리 설정에서 `attachment_uncertainty_padding_m=0.01`을 명시한 비교다. 접촉 후 기존 관측 mesh를 보존하면서 관측 OBB에 면당 10mm 여유를 더한 box를 MoveIt attached geometry에 추가한다. 계획 충돌 검사와 self-mask가 같은 형상을 사용하며, 지도 제외 범위만 늘리지 않는다. 이 여유는 실물 보정값이나 모든 물체의 오차 상한이 아니다. 기본값 0은 기존 동작을 유지한다. attachment 이전 형상 유지, 원래 mesh 보존, 허용 접촉 링크 유지, 복구 정리와 잘못된 설정 거부를 포함한 32개 검사가 `attachment-envelope-tests.log`에서 통과했고 13개 ROS 패키지 빌드가 완료됐다.

`camera-56`은 리프트 도중 retention guard가 실패했고, 재관측 이전에 STOP_UNCONFIRMED로 끝났다. 새 진단에서 `pending_results=1`, 유효한 feedback과 함께 팔의 0.05–0.11rad/s 움직임이 확인됐다. 따라서 이번 실행으로 attachment envelope의 재관측 충돌 개선 여부는 판정하지 않는다. 실제 저장 장면에는 원래 mesh 1개와 추가 OBB envelope(약 80×77×115mm)가 함께 존재했다.

설치된 `ros-humble-moveit-ros-move-group`은 2.5.10이다. [동일 버전 ExecuteTrajectory 소스](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)는 같은 mutually-exclusive callback에서 실행을 동기 대기하며 cancel callback에서 TEM 정지를 호출하지 않는다. [MoveGroupInterface::stop](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp#L929)은 별도 `trajectory_execution_event`에 `stop`을 발행한다. 실제 domain 178에서도 해당 MoveIt subscriber를 확인했다.

`camera-56/provenance/stop_event_probe.py`는 Mission 실패 후 별도로 실행한 제어 진단이다. 11개 관절 경로 표본을 MoveIt 충돌 검사한 뒤, 1초/0.02rad 손목 동작 두 개에서 Action cancel만 보낸 경우와 TEM stop을 함께 보낸 경우를 비교했다. 전자는 취소 요청 후 1.6925초에 SUCCEEDED/code1로 완료돼 목표까지 이동했다. 후자는 0.01995초에 ABORTED/code-7로 종료됐고 요청 목표 0.02rad 대신 약 0.00043rad 이동했다. 이 실행은 수거 성공, 재관측 성공 또는 BT 자체 취소 성공으로 세지 않는다. 원본 관절 표본과 결과는 `evaluations/stop-event-probe.json`에 있다.

BT 정지 평가에 같은 TEM 이벤트를 연결했다. 미종료/늦은 Action을 위해 0.2초 간격으로 기한 내 반복하되, 기존 terminal 결과·fresh joint feedback·정지 속도/유지 시간 기준은 그대로 요구한다. `cancel=false` 평가에는 발행하지 않는다. Action cancel을 무시하는 대역과 늦은 수락, 이동/피드백 단절 등 112개 검사가 `moveit-stop-tests.log`에서 통과했다. 실제 BT에서의 후속 실패/취소 반복은 별도로 검증한다.

`camera-57`은 물리 리프트와 재관측 이동을 모두 완료한 뒤 head RGB-D 물체 연관 실패로 종료했다(stop_confirmed=true). 추가 envelope와 self-mask 수정이 있는 이 실행에서는 재관측 경로 무효화가 없었으나 반복 안정성이나 수거 성공을 입증하지 않는다. 실패 전 motion이 이미 완료됐으므로 이 실행의 정지 결과만으로 진행 중 Action 취소 보완을 검증했다고 주장하지 않는다. `final-head.png`는 종료 후 실제 topic 영상이다.

`camera-57/evaluations/bracket-view-probe.json`은 관측 OBB, 접촉 FK와 nominal camera mount만 사용하는 읽기 전용 IK/FK 비교다. 첫 후보에서 pitch -1.6rad/추가 roll0의 카메라–브래킷 방향 각도는 5.9도, pitch0/추가 roll0은 56.5도였고 둘 다 MoveIt 상태 검사를 통과했다. 이는 mesh 가시성이나 검출 성공의 근거가 아닌 자세 선택 휴리스틱이다. `camera-58`은 pitch만0으로 바꾸며 원래 carry roll 단계 [2,5,10,20,30]을 유지한다. 중단 후 늦게 수락된 Action callback에서도 TEM stop을 발행하는 검사를 추가해 `moveit-stop-late-tests.log`의 114개 검사가 통과했고 build도 완료했다.


`camera-58`은 pitch0 자세에서 실제 리프트와 재관측 이동을 완료했지만 ConfirmHeld의 RGB-D 재검출에 실패했다(stop_confirmed=true). 종료 후 `evaluations/final-head.png`에서 카메라 브래킷 대신 그리퍼 본체가 컵 중간을 가린다. 브래킷 방향 각도만으로 충분한 가시성을 판단할 수 없었으며 투입/복귀는 실행하지 않았다.

`camera-58/evaluations/occlusion-probe.json`은 camera-57의 관측 mesh와 URDF collision hull을 사용한 제어 없는 ray 비교다. 센서 하우징 head_tilt_link만 기존 가시성 계약처럼 제외하고, cylinder는 외접 32각형, sphere는 외접 box로 보수적으로 근사했다. 노출 면적은 실제 렌더/검출 결과가 아닌 후보 순위 휴리스틱이다. 상승 후보에서 pitch-1.6rad/roll-90도의 예상 노출 면적은95.1%였다. `camera-59`는 이 자세를 별도로 시험하며 carry roll 범위를90도로 명시한다. 기존 기본값이나 충돌·접촉·시각 확인 문턱을 바꾸지 않는다.


`camera-59`는 90도 roll 재관측 이동과 접촉 유지를 완료했지만 공개 YOLOE 원본 프레임 검출이 비어 ConfirmHeld/GRASP_LOST, stop_confirmed=true로 끝났다. `evaluations/final-head.png`에는 옆으로 누운 컵이 노출되어 있다. 원본 영상의 네 사분면 규칙 중 오른쪽 아래 구역을270도 회전한 오프라인 추론에서는 cup0.6153, mask4517pixels를 얻었다. `head-fallback-probe.json`은 물체 위치 입력 없이 전체 네 사분면/네 방향 재시도 후 같은 검출을 복원한 결과이며 최초 모델 준비를 포함해8.523초였다. 실제 RGB-D 연관이나 Mission 성공 근거는 아니다.

원본 frame이 비었을 때만 위 변환을 재시도하는 head adapter를 추가했다. 타일 가장자리와 접하는 mask는 거부하며 원본 좌표 복원 후 mask IoU 중복을 제거한다. 신뢰도는 올리지 않는다. 홀수 영상 크기의 네 방향 bbox/mask 복원, 원본 결과 보존, 잘린 mask 거부와 중복 제거 및 기존 손목 fallback 검사9개가 `head-fallback-tests.log`에서 통과했다. `camera-60`에서 실제 센서 pipeline과 물리 실행을 검증한다.


`camera-60`은 Mission 시작 전 MoveIt SIGSEGV로 launch 전체가 종료돼 새 head adapter의 물리 효과를 검증하지 못했다. `logs/ros.log`의 stack은 GetPlanningScene service → getOctomapMsg → OctoMap serialization이며 host core1705642 위치는 `provenance/coredump-info.txt`에 보존했다. [MoveIt2.5.10 service callback](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp#L570)은 scene mutex만 잠그고, [OctoMap serialization](https://github.com/moveit/moveit2/blob/2.5.10/moveit_core/planning_scene/src/planning_scene.cpp#L686)은 별도 tree lock을 얻지 않는다. 동시 sensor updater와의 경합으로 추정하며 allocator/use-after-free까지 core로 확정하지는 않았다. 자체 updater의 tree read/write RAII lock은 확인했다.

`camera-61`은 동일한 물리/인식 설정으로 재실행하며 추가 진단용 scene-ring/lift-scene 서비스 요청에서 OCTOMAP bit32를 제외한다(components991). 기존 실행 코드의 GetPlanningScene 요청은 ACM만 사용한다. 실제 planning OctoMap, sensor updater, 상태/경로 충돌 검사는 유지하며 upstream serialization 자체를 수정했다고 주장하지 않는다. 진단 CDR에는 OctoMap이 포함되지 않는다.


물리 실험 재현을 위해 `run.py/prepare.py --cleanup-profile slender-cup-95mm`를 추가했다. payload 높이95mm, 접근0.52m, PGS128/8과 camera-61의 launch/coordinator 설정을 명시적으로 기록하며 기본 canonical 설정은 유지한다. 두 프로필의 기존 chair/map geometry 보존, cup 바닥 높이 보존, 잘못된 object 조합 거부 및 head/wrist fallback 검사12개가 `cleanup-profile-tests.log`에서 통과했다. 이 프로필로 시작부터 끝까지 실행한 검증은 별도 결과가 필요하다.


`camera-61`은 리프트와 재관측 이동 후 RGB-D 연관 실패로 종료했다(stop_confirmed=true). 정지 후 같은 실제 perception Action을 읽기 전용으로 호출한 `evaluations/postfailure-inspection.json`에서는 cup0.3603과 유효한 depth가 나왔다. 이는 종료된 Mission을 재개하거나 성공으로 바꾼 결과가 아니다. 최초 실행 때 원본 인식 입력을 저장하지 않아 프레임 차이를 확정할 수 없으며, 다음 실행부터 head detector의 입력 PNG와 결과 JSON을 저장한다.

같은 phase2 결과의 desk-supported OBB는 높이0.25557m, 중심z0.46836m로 책상까지 공극을 포함한다. 실제 mask point433개의 bounds 중심은 [0.50129,-0.14445,0.55959]m이며 재관측 예상 중심 [0.50469,-0.15001,0.56755]m와 약1.03cm 차이다. 지원면까지 늘린 OBB를 공중 물체 중심으로 사용하는 버그를 수정해 head lift 연관/높이 검증에는 기존 selected target_cloud의 bounds 중심을 사용한다. frame/stamp 일치와 finite point를 요구하고 기존 연관·높이 기준은 유지한다. 책상 위 grasp OBB와 원본 perception message는 바꾸지 않는다. unit/regression 결과는 `lift-cloud-tests.log`에 보존한다.


`camera-62`에서는 실제 재관측 입력에 cup0.3328/mask4781pixels가 검출됐지만 phase2가 support plane397inliers<432로 거부돼 ConfirmHeld/GRASP_LOST, stop_confirmed=true로 종료했다. `evaluations/head-detections/1792952753188547868.{png,json}`와 `logs/ros.log`에 검출과 거부 사유를 보존했다. 첫 lift 원본 검출은0건이었다. 따라서 이번 실패는 새로운 cloud center 연관 검증 전에 발생했다.

기존 `InspectScene` phase2에 기본 false인 `free_space` 옵션을 추가한다. 공중 물체 확인 때만 실제 mask point bounds를 복원하고 지지 평면 추정/테이블까지 extrusion을 사용하지 않는다. 기본 tabletop 복원과 최소 mask/depth point/TF 검사는 유지한다. 평면이 없는 영상에서 기본 모드는 거부하고 free_space 모드는 관측 표면만 복원하며, 부족한 깊이는 여전히 거부하는 core검사14개가 `free-space-core-tests.log`에서 통과했다. ROS action 상호운용과 실제 물리 실행은 전체 재빌드 뒤 검증한다.

`free-space-build.log`에서13개 패키지 재빌드가 통과했다. DDS197에서 실제 InspectScene Action의 기본/공중 모드 요청·결과를 포함한 pipeline/node/sorting/wrist/head 회귀검사211개가 `free-space-integration-tests.log`에서 통과했다. 공중 확인의 기존 높이 미달 거부 검사도 포함한다. 물리 비교는 `camera-63`이다.

`profile-preparation/comparison.json`은 원래 seat-12 주행 목표에서 새 프로필로 준비한 접근/시작/payload 위치가 component 설정과2.23e-16m 이내로 일치하고 coordinator YAML도 동일함을 확인한다. 이는 설정 재현 검사이며 Mission 실행 성공이 아니다.


`camera-63`은 실제 head RGB-D lift 확인에 처음 성공했다(cup0.6384, center_z0.566m, association_error0.012m). `025_lift_inspection.cdr`에 free_space 결과를 기록했다. 이어 CarryObject가 재관측 후 고정 wrist roll로 수거함 위치에 도달하지 못해 TRANSPORTING/MOTION_FAILED, stop_confirmed=true로 종료했다. 투입/복귀 및 전체 Mission 성공은 아니다. 첫 lift의 공개 모델은 카메라/그리퍼 부근을 lego brick0.2904로 검출했지만 이 후보는 연관되지 않았고, 재관측 후 cup을 사용했다.

`camera-63/evaluations/bin-ik-probe.json`은 관측 형상, 실제 joint state와 MoveIt 장면을 사용하는 제어 없는 IK 검사다. 재관측의 roll-90도는 검사한 높이에서 거부됐고, 원래 roll에서 bin 위z0.60/0.65m는 유효했다(기존0.56m는 거부). 경로 실행이나 물리 투입 성공을 뜻하지 않는다. 시각 확인이 성공한 경우에만 측정해 둔 원래 arm pose로 돌아가는 선택 옵션을 추가하고, 실험 프로필의 trash release 높이는0.60m로 명시한다. 기본 운반 설정은 유지한다. 후속 물리 비교는 `camera-64`이다.

`restore-view-tests.log`의186개 검사와 `restore-view-build.log`의13개 패키지 빌드가 통과했다. 시각 확인 실패 시 복귀 이동을 실행하지 않는 회귀 검사를 포함한다. `camera-64`의 coordinator에는 `sorting_reobserve_restore_pose=true`, `sorting_fixed_release_trash_right_m=[-0.075,-0.105,0.60]`를 명시했다.

`camera-64`는 시각 확인 이전 LIFTING/GRASP_LOST로 종료했다. 마지막 grasp contact 직후 팔이 거의 정지한 상태에서 컵이 이탈했고, 실제 payload가 바닥으로 떨어졌다. `evaluations/grasp-physics.jsonl`의 simulation51.38→51.48s에서 컵 중심 x는 -1.14474→-1.10877m로 이동했고 양쪽 접촉력이 사라졌다. 따라서 시각 확인 후 원래 자세 복귀와 bin release 변경은 이번 실행에서 평가되지 않았다. Mission은 stop_confirmed=true, arm_recovered=false, placementNOT_CHECKED, returnNOT_STARTED로 종료했다. `camera-65`는 같은 설정에서 실제 LiftObject 이동 중 IMMEDIATE 취소를 보내는 별도 실패 처리 시험이다.

`camera-65`는 IMMEDIATE 취소를 수락하고0.6867초 후 CANCELED/stop_confirmed=true/arm_recovered=false로 종료했다. 그러나 취소 검사기는 LiftObject의 계획 단계에서 물리적인 관절 흔들림을 경로 실행으로 오인했다. 실제 ExecuteTrajectory 시작 전이므로 TEM stop 이벤트가 없었고 전용 검사 결과 passed=false다. 성공으로 재분류하지 않는다. 후속 검사기는 선택 팔의 속도와 `/execute_trajectory/_action/status`의 EXECUTING을 동시에 요구한다.

`camera-66`은 camera-64 대비 physics_hz만500→1000으로 변경한 비교다. 동일한95mm fixture, 토크 상한, 제어 속도, ROS 발행 주기와 성공 판정 조건을 유지하며, 물리 시간 간격을 줄였을 때 접촉 유지 여부를 확인한다. 아직 채택된 프로필이나 성공 근거는 아니다.

`camera-66`은 리프트, head RGB-D 확인(cup0.4802, center_z0.563m, association_error0.008m), 원래 팔 자세 복귀, 수거함 운반과 gripper 개방을 실행했다. ConfirmRelease에서 VERIFICATION_TIMEOUT/stop_confirmed=true로 종료했으며 팔/주행 복귀는 미실행이다. `placement-probe.json`의 실제 oracle10Hz 표본에는0.399999999초의5개 표본 구간이 존재한다. 기존 verifier는 항상 마지막5개만 검사해 안정 시간0.4초를 충족하지 못할 수 있었다. 최소 표본 수와 전체 안정 시간을 모두 충족하도록 후행 구간을 선택한다.

독립 oracle의 기존 회전 외접 box는 기울어진 tapered cup의 실제 표면보다 경계가 컸다. 저장된 물리 pose와 같은 시각의 base를 기존 tabletop collision vertex 및 바닥 cylinder로 재계산한 `placement-bounds-replay.json`에서는97~106sim초90개 표본 모두 수거함 내부였고, 기존 box는90개 모두 외부였다. 마지막 표본의 실제 형상 최저z0.22799m는 바닥0.228m와 맞지만 기존box 최저z는0.20929m였다. 이 읽기 전용 분석으로 종료한 Mission을 성공 처리하지 않는다. oracle 경계와 시간 구간 수정의24개 검사가 `placement-adapter-tests.log`에서 통과했다. 후속 실험 프로필은physics_hz1000을 명시하며 반복 안정성 및 전체 Mission 검증은 별도 필요하다.

`placement-profile-tests.log`의 추가8개 검사와 `placement-adapter-build.log`의13개 패키지 빌드가 통과했다. `camera-67`은 수정한 oracle과 현재 저장소 tools/profile을 사용하는 새 component 실행이며500Hz와 구별되는1,000Hz 설정을 기록한다. adapter-ready 메타데이터는 기능 활성화(`physical_cleanup_enabled`)와 최종 검증(`physical_cleanup_verified=false` at startup)을 구분한다.

`camera-67`은 실제 ConfirmRelease에서LEFT_GRIPPER/CONFIRMED를 얻었다. 이후 ReturnArm의 경로 시작5/113점이 무효로 판정돼 RETURNING_ARM/MOTION_FAILED, stop_confirmed=true/arm_recovered=false로 끝났다. `031_joint_motion_result.cdr`와 `failed-path-validity.json`의 정지 후 재검사에서 right_moving_jaw_link와top_base_link의 최대약9.78mm 자기 충돌을 확인했다. OctoMap 충돌은 아니며 진단 scene 요청은OCTOMAP를 제외한991이다.

`bin-opening-ik-probe.json`의 제어 없는 검사는z0.60~0.75m에서 닫힌 손가락은 유효하지만1.2rad 개방은 모두 자기 충돌을 보였다. z0.60m의0.6/0.7rad는 유효하고0.8rad부터 충돌했다. 기존 gripper_sweep_is_valid를 투입 IK와 실제 개방 직전에 재사용하고, 실험 프로필에는 투입 개방0.65rad를 명시한다. 접근용 개방1.2rad와 모든 충돌 허용 목록은 유지한다. 이 프로필에서 컵의 실제 이탈과 안착은 새 실행으로 확인해야 한다.

`release-sweep-tests.log`에서 sorting/wrist/BT ROS operations251개 검사가 통과했고, `release-sweep-build.log`의13개 패키지 빌드가 완료됐다. 새로운/캐시된 투입 IK의 개방 충돌 거부와 실제 개방 직전 재검사, 실패 시 controller 명령 금지를 포함한다. `camera-68`은 ExecuteTrajectory EXECUTING 및 선택 팔의 속도를 함께 확인한 뒤 IMMEDIATE 취소를 보내는 별도 시험이다.

`camera-68/evaluations/manipulation-cancel.json`의 실제 이동 중 취소 검사가passed=true다. LiftObject의 ExecuteTrajectory STATUS_EXECUTING에서 right_elbow_pitch_joint 속도0.06029rad/s를 확인하고 IMMEDIATE 요청을 보냈다.0.9673초 후CANCELED/stop_confirmed=true/arm_recovered=false가 되었고TEM stop 이벤트, 새 정지 관절 표본, ready_to_drive=false 및 MissionERROR/TERMINAL을 확인했다. 이 의도적 취소에서 정상 성공용 validator가 실패하는 것은 예상 결과이며 취소 전용 판정과 구분한다.

`e2e-69`는 책상 앞 초기화가 아닌 Facility18 대기 위치에서 시작하는 `run.py --cleanup --cleanup-profile slender-cup-95mm --target-id seat-12 --require-no-recovery` 실행이다. 원래 arm preflight부터 실제 Mission 할당, 주행, 수거, 복귀 및 복귀 후 안착 유지 확인까지 요구한다. 프로필에는1,000Hz/95mm fixture/0.65rad 투입 개방을 명시했다.

`e2e-69`는 Mission 전에 팔 preflight가30초 동안 joint feedback/Action 준비를 얻지 못해 종료했다. base.jsonl과arm-samples.json은 비어 있었다. adapter-ready를 controller 초기화 직후에 기록하던 문제를 수정해 clock/odom/scan 및cleanup의 실제RGB-D 발행 이후에만 준비 완료를 알린다. 시작 기한180초와 첫 프레임 대기60초 traceback을 추가했다. cleanup launcher의OMP_NUM_THREADS 강제값은 검증했던 component와 동일하게 생략하고 SimulationApp의4-thread 설정을 유지한다. 원인이OMP인지 첫 렌더 지연인지 단일 원인으로 확정하지 않으며, 새 실행은 `e2e-70`이다.

### 2026-10-26 Handoff 검토 시점

`e2e-70`은 첫 센서 발행 이후 준비 완료와 팔 사전 제어 검사를 통과했다(693개 joint 표본, 최대 목표 오차0.001915rad, controller cancel 및 정지 확인). 그러나 작업 접근 목표[-1.72,2.62,0]에 도착하기 전에 controller patience exceeded와 spin 복구가 발생했다. 복구 없는 성공 조건을 이미 위반했으므로 CancelMission을 요청했고, 취소 후 RETURN_HOME에서도 controller patience exceeded가 반복돼 시험을 종료했다. 원본은 `e2e-70/evaluations/recovery-abort.json`, `mission.json`, `base.jsonl`, `logs/mission.log`이다. validator SIGINT 종료 때 ROS context 종료 후 publish의 RCLError가 추가로 기록됐으며, 이 오류는 주행 실패 원인과 구별한다. 전체 물리 Mission 성공은 아직 없다.

마지막 변경은 실험 프로필에만 PathAlign/PathFollow threshold를1.0→0.10m, GoalAngle threshold를0.5→1.50m로 지정한 것이다. 기존 코드에서 두 path critic은 threshold 안에서 비활성화되고 GoalAngle은 threshold 안에서만 활성화된다. 가까운 책상 접근에서 회전이 늦어지는 현상에 대한 설정 가설이며, 수정 후 물리 주행은 아직 실행하지 않았다. 속도, footprint, 안전 영역, 충돌 검사 및 기존 Facility 기본 프로필은 바꾸지 않는다. 이 override의 적용 경로는 prepare_cleanup→room.cleanup.navigation_parameters→mission.launch.py다.

`e2e-71`은 위 threshold 변경을 실제 Mission에서 시험했지만 책상 앞에서 경로 진행이 멈춰 복구가 발생했다. 미션 취소를 수락한 뒤 실패 실행을 종료했으며, 수거는 시작하지 않았다. `evaluations/navigation-stream.jsonl.gz`에 글로벌 경로, AMCL/odom, 양쪽 속도 명령과 코스트맵을 보존했다. `pruning-analysis.json`은 복구 전32개 표본에서 XY 최근접 경로점이 회전 후 구간을 선택하는 반면 방향을 포함한 비교는 아직 수행할 회전 구간을 보존함을 보여 준다. 이것만으로 모든 주행 실패의 원인이 해결됐다고 보지는 않는다.

저장 경로의 상대 배치를 사용하는 `PreserveLatticeTurnBeforeNearbyFinalApproach`는 기존 실제 PathHandler에서 회전 후5번 점을 골라 실패했다(`pruning-before.log`). 선택적 `path_pose_yaw_weight`를 추가해 XY 거리와 최단 각도 차이를 함께 비교한다. 기본0은 기존 선택을 유지하며 실험 프로필에만0.5m/rad를 적용한다. 회전 전 구간 보존, 회전 후 진행 및 ±π 경계의 회귀 검사를 포함한 Navigation CTest23개 묶음과 fixture3개 검사가 `pruning-after.log`에서 통과했다. 13개 패키지를 재빌드했고 `e2e-72`에서 전체 Mission을 다시 검증한다. validator 종료 중 ROS context가 이미 종료된 경우에는 publisher를 호출하지 않아 원래 실패 기록을 RCLError로 덮지 않는다.

`e2e-72`도 주행 중 글로벌 플래너의 `Starting point in lethal space`로 실패해 수거 전에 취소했다. `navigation-stream.jsonl.gz`의 실제 transformed path와 로컬 raw costmap을 추가로 기록했다. `nav-costmap-comparison.{json,png}`의 실패 직전 샘플에서 글로벌 지도 footprint 경계는 inscribed cost253을 포함하지만 로컬 경계의 최대 cost는0이었다. 두 지도 표본은 같은 시각이 아니며 이 비교 자체를 정확한 실패 순간의 충돌 재생으로 취급하지 않는다. 실행 설정의 로컬 플러그인은 obstacle/inflation뿐이어서 글로벌 시설 지도와 다른 장애물 영역을 사용했다.

cleanup launch에 기존 글로벌 설정과 같은 Nav2 StaticLayer를 로컬 obstacle/inflation 앞에 연결했다. 시설 지도나 footprint를 줄이지 않고 글로벌/로컬 판단에 같은 알려진 장애물을 사용한다. room/Facility18 mock launch 보존과 physical launch의 StaticLayer 연결 검사를 포함해 `static-map-tests.log`의8개 검사가 통과했다(기존 선택적 환경 검사1개 skip). 후속 전체 실행은 `e2e-73`이다.

`e2e-73`은 StaticLayer를 기동했으나 책상 앞에서 진행 제한 시간을 초과해 실패했고 수거 전에 취소했다. 저장된 실제 raw map/pose/path를 기존 MPPI로 재생한 `evaluations/replay/variants.json`은4/8/12초 horizon 및 가중치 변경을 비교한다. 모두 정체했고 baseline의128개 경로 seed 전부 충돌 비용을 받았다. 실제 실행이나 성능 비교가 아닌 고정 자세/초기 속도0의 오프라인 진단이다.

같은 transformed path를 마지막 목표의 정확한 대응으로 map 좌표계에 돌린 `replay-global-samepath/path-check.log`에서도 중간 자세[-1.78856,2.36942,1.09231]의 native footprint cost가254였다. `smac-replay/summary.json`은 native planner의 홈/근접 위치 및 회전/비용/추가 padding 변형10개가 경로 점 검사를 통과해도 전체 연속 footprint 검사는 통과하지 못했음을 기록한다. [Nav2 1.1.20 NodeLattice](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/src/node_lattice.cpp)의 거리 기준 중간점 검사와 [Smac Lattice](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/src/smac_planner_lattice.cpp)의72방향 근사가 있으며, 이 실행에서 둘 중 하나만의 영향으로 분리하지는 않는다.

선택적 `validate_swept_path`로 기존 연속 footprint 검사를 적용하고, 거부된 native 경로는 이미 구현된 `checkedBodyHeadingGridPath`로 제한 시간 내 보정한다. `smac-replay/certified-{home,close}.{csv,log}`는 각각160/22개 자세, 잘못된 경로점0개, 전체 연속 충돌 검사 true를 기록한다. `replay-certified/critics.csv`의 첫 표본은128개 seed 중112개가 충돌 비용0이었다. 일반 미션에서는 기존 Nav2 action과 MPPI를 사용하고 계획 보정 횟수는 Nav2 복구 횟수와 별도로 기록한다.

MPPI의 경로 arc 인덱스도 끝 범위를 넘어갈 수 있었다. 길이3의 arc에서0.20m 요청이 인덱스3을 반환해 `path-index-before.log`에서 회귀 검사가 실패했다. 마지막 유효 점에 제한하고 검색 시작 인덱스를 보존하도록 수정했다. 계획된 회전의 SE(2) 진행 평가, 잘못된 회전/정지 거부, ±π 경계와 책상 코너 연속 경로 보정을 포함한 `continuous-navigation-tests.log`의 CTest24개 묶음과 Python8개 검사가 통과했다(선택적 환경1개 skip). 13개 패키지를 다시 빌드했다. 첫 전체 header 재빌드는 colcon의 Make 병렬 작업이12GiB 제한을 넘어서 종료됐으며 `continuous-build-oom.log`에 보존했다. `MAKEFLAGS=-j2 -l2`를 명시한 재빌드는 통과했다.

physical cleanup의 local costmap은 같은 시설 map 좌표계와 정렬된5cm rolling grid를 사용해 odom 변환 시 재샘플링 차이도 제거한다. 새 전체 물리 실행은 `e2e-74`이며 이전 실패를 성공으로 재분류하지 않는다.

`e2e-74`는 기존 정체 위치보다 앞에서 여유를 확보해 회전했지만, 마지막 접근 중 native 격자 시작점이 lethal로 판정되어 실패했다. 첫 주행 실패 시 자동으로 Mission 취소와 validator 종료를 요청한 근거는 `evaluations/fail-fast.json`, `recovery-abort.json`이다. 수거는 시작하지 않았다. 실패 직전 실제 자세[-1.793305,2.409410,0.393898]를 raw map과 함께 재생한 `replay-global/before.log`에서도 같은 오류와 빈 경로를 재현했다.

연속 자세를 native 중심/각도 bin에 맞춘 결과가 충돌할 때에는 native 검색 전 기존 연속 footprint 검색을 사용하도록 보완했다. 실제 시작/목표가 무효이면 보정도 거부한다. 같은 저장 입력의 `replay-global/after.log`는9개 자세, 잘못된 경로점0개, 전체 연속 충돌 검사 true를 기록했다. 추가 코너/격자 시작점 회귀 검사를 포함한 `test_smac_terminal`이 통과했다. 다음 전체 실행은 `e2e-75`다.

`e2e-75`는 실제 Mission의 outbound navigation_result=OK와 복구0회로 작업 접근을 완료했다. 이어 MoveToPregrasp의 결합 팔/그리퍼 궤적에서 right_gripper_controller의 PATH_TOLERANCE_VIOLATED로 실패했다. object_state=NOT_TOUCHED, stop_confirmed=true, arm_recovered=false이며 복귀와 컵 투입은 실행하지 않았다. `gripper-tracking-analysis.json`은 계획 최대 개방 속도0.24973rad/s, 실제 구간 최대0.24583rad/s, 최대 jaw 위치1.09949rad(개방 목표1.2rad), moving jaw 접촉력0N과 구동력 상한0.2Nm를 기록한다. 컨트롤러 tracking error는0.1035rad였다. 실험 damping/friction에서 추종 여유가 부족한 결합 개방 궤적이며, 실험 프로필의 기존 `velocity_scaling`만0.08→0.04로 낮춘다. 관절 tolerance, 파지 구동력, Nav2 속도는 유지한다. 후속 전체 실행은 `e2e-76`이다.

`e2e-76`도 주행 복구0회로 도착했지만 파지 후보가 없어 NOT_TOUCHED/TARGET_UNAVAILABLE로 종료했다. 새 개방 속도는 아직 실행되지 않았다. `approach-observation-comparison.json`의 실제 RGB-D 관측은75/76에서 컵 중심x0.57556/0.58992m, 신뢰도0.849/0.843와 정상적인95mm 높이를 기록한다. 주행의3cm 허용 오차 안에서도 작업 도달성이 달라졌으므로 실험 프로필의 XY 허용 오차를8mm로 줄인다. 물체나 접근 목표는 옮기지 않는다. native 경로의 실제 시작/정확한 요청 목표 연결도 기존 연속 검사에 포함한다. 다음 전체 실행은 `e2e-77`이다.

Handoff 검증은 DDS197/198의 별도 컨테이너에서 패키지별로 실행했다. BT326, Mission91(15skip), Perception178, Skill Executor693, MoveIt22, MuJoCo211, Isaac tools30(1skip)로 합계1551개 검사가 통과했다. 기존 Mission 구성 검사4개의 GoalDirectedController 기대값은 요청된MPPIController로 갱신한 뒤 재실행했다. Navigation23개와 Scene Mapping4개 CTest 묶음도 모두 통과했다. 원본은 `handoff-*.log`와 `handoff-test-summary.json`이다. 초기 일괄 수집의 Python import/compiled module 설정 오류와 소스 mount가 없던 첫 CTest 호출은 결과로 채택하지 않았으며, 올바른 설치 환경과 패키지별 실행으로 확인했다.

이전 주행 작업의93개 Nav2 파일이 물리 정리 시작 시 저장한 before.patch/before-untracked.json과 일치함을 확인했다. 해당 기반 및 MPPI 구성 검사 기대값은 별도 commit `77c2f53e`로 보존한다. 물리 정리 변경과 실행 도구는 후속 commit으로 분리한다. 원본 모델과 실행 데이터는 Git 밖에 유지한다.


`e2e-77`은 작업 접근 오차7.84mm, 주행 복구0회로 도착했고 낮춘 빈손 이동 속도로 MoveToPregrasp도 통과했다. 이후 fresh grasp의 gripper closure가 컵 주변 OctoMap과 충돌해 APPROACHING/MOTION_FAILED, NOT_TOUCHED, stop_confirmed=true로 종료했다. `collision-rejections.json`에는 moving jaw의 컵 근처 접촉점[0.56446,-0.12998,0.40245]와 깊이0.05mm가 기록돼 있다. 투입/팔 복귀/홈 복귀 성공은 아니다.

동일 실행에서 팔 이동 중 베이스 yaw가0.00274→-0.01384rad로 바뀌었으며, 기존 known geometry clear의200,000 leaf 예산 소진도 반복됐다. 둘의 영향은 아직 분리하지 않았다. 실험 프로필에만 기존 ideal root velocity 구동의 정지 제동을 추가해 마지막 정지 자세에 대한 제한된 속도 보정으로 팔 반작용 누적을 억제한다. 비영 주행 명령은 그대로 전달하며 pose reset/고정 joint/물체 부착은 없다. 기존 `depth_cloud.known_geometry_clear_max_leaves`를2,000,000으로 전달하고 알려진 형상 내부만 지우는 판정은 유지한다. 베이스 제동의 전달/포화/각도 경계/누적 외란 회귀 검사3개는 통과했다. `component-78`은 책상0.1m 앞에서 시작하는 부분 실행으로, 전체 outbound 주행 근거와 구분한다. 정상 전체 Mission 성공은 여전히 별도 새 실행이 필요하다.


`component-78`은 scoped GroupAction 안에 shared launch를 넣은 뒤 지연 action의 use_image_view 설정이 사라져 기동 단계에서 종료했다. `component-79`는 unscoped 수정 후 기동하고 실제 MoveIt 예산2,000,000을 확인했지만 depth_scene_node가 YAML override의 aligned depth/stride4 대신 기본 depth 토픽/stride2를 사용해 cloud_age/map_receipt_age=infinity로 ValidateGoal에서 차단됐다. 컵은 미접촉이며 둘 다 정상 동작 근거가 아니다. 전역 SetParameter 사용을 제거하고 기존 두 launch 사이에 기본값200,000의 단일 `known_geometry_clear_max_leaves` 인자를 전달하도록 바꿨다. `component-80`에서 실제 카메라 토픽과 지도 수신을 함께 재확인한다.


`component-80`은 올바른 aligned depth/stride4 및 실제 MoveIt clear 예산2,000,000을 확인했다(`live-depth-settings.txt`). 실제 지도는 약210,147 leaf를 검사했고 cloud 입력 나이는0.16초, 출력0.20초였다. 사전 파지, 재관측, 접근과 gripper close가 진행됐고 초기 접촉은11개 표본으로 확인했다. 리프트 시작 시 strict stationary-contact guard가 GRASP_LOST를 보고해 stop_confirmed=true, arm_recovered=false, MissionFAILED/GRASP_FAIL로 종료했다. 투입/왕복 성공은 아니다.

`lift-contact-analysis.json`은 실제 PhysX 기록에서57.349→57.549초의 jaw 각도가0.40239→0.38551rad로 추가 닫히고 한 표본에서 접촉력이0이 됐다가 양쪽4.26/4.97N으로 돌아온 것을 보여 준다. 컵은 책상 위에 남았다. 이는 완전한 파지 이탈 성공/실패 판정을 대신하지 않는다. `/save_map`으로 native 잠금 경로를 통해 저장한 pregrasp/approach-map.bt와 scene_ring991도 보존한다. 사용자가 파지 성공보다 E2E 흐름 검증을 우선하고 파지 실패를 허용했으므로 추가 파지 튜닝은 중단하며 실제 실패 기록을 보존한다.


사용자는 추가로 "잡기에 실패해도 돼, 그냥 e to e 검증만"이라고 범위를 조정했다. 파지 안정성 튜닝을 중단하고 같은 물리 fixture/작업 접근점을 사용하는 명시적 `--cleanup-backend mock` 검증 모드를 추가했다. 실제 Mission FSM/BT와 Nav2 왕복은 그대로이며 인식/작업 계획/정리 Action은 기존 mock fixture다. 이 결과를 물리 수거 성공으로 분류하지 않는다. 준비/launch 모드 구분 및 기본 physical 경로를 포함한14개 검사(선택 환경1skip)가 `mock-cleanup-mode-tests.log`에서 통과했다. `e2e-81`은 대기 위치부터 시작해 작업 접근과 홈 복귀 모두 Nav2 복구0회를 요구하는 전체 실행이다.


`e2e-81`의 mock 정리 전체 실행은 작업 접근 중 `MPPI: no collision-free emitted rollout`과 local costmap clear가 발생해 복구 없는 조건을 위반했다. watcher가 실제 Mission 취소를 수락한 뒤 실패 실행을 종료했고 정리 Action은 미실행이다. 원본은 `fail-fast.json`, `recovery-abort.json`, `navigation-stream.jsonl.gz`와 `logs/mission.log`다. 정리 mock 모드도 접근 실패를 성공으로 처리하지 않는다.

`evaluations/replay/seed-comparison.json`은 실패 직전 저장 map/pose/path 및 측정 속도에서 실제 C++ pathSeed를 재생하고 native footprintCostAtPose로 검사한 읽기 전용 비교다. 기존12mm/25mrad 구간 전환에서는128개 중78개 seed가 최소1개 잘못된 자세를 포함했고,0.1mm/0.1mrad에서는34개였다. 표본 간 연속 충돌 검사나 optimizer warm start 재생까지의 근거는 아니다. waypoint를 먼저 통과하도록 바꾸고, 모든 이동 출력이 거부돼도 동일한 연속 출력 검사(측정 초기 속도 포함)를 통과하는 정지 sequence만 선택적으로 허용한다. 실험 프로필에만 `validated_stop_fallback=true`를 지정하며 진행 제한12초는 유지한다. 정지 횟수는 navigation.json에 별도 집계한다. 잘못된 발자국과 초기 운동량을 정지로 덮지 못하는 회귀 검사 및 waypoint 단축 방지 검사를 포함한 optimizer 검사와 fixture11개가 `validated-output-tests.log`에서 통과했다. 새 전체 실행은 `e2e-82`다.


`e2e-82`는 동일한 접근 구간에서 모든 이동/정지 출력이 충돌 검사에서 거부돼 실패했고, `e2e-83`은 planner padding만0.02→0.04m로 늘린 뒤 global refinement가 실패했다. 둘 모두 Nav2 복구 요청 후 취소해 실패로 보존했으며 작업 Action은 미실행이다. `e2e-81/evaluations/clearance/variants.json`의 저장 global 지도 오프라인 비교는 원래 목표[-1.72,2.62,0]를 유지한 채0.04m padding에서도159점/무효0점/전체 연속 검사 true인 경로가 있음을 보여 준다. 지도 시각과 실제 제어가 다른 오프라인 근거다.

출력 검사 구현에서 sequence shifting을 반영하지 않는 불일치를 추가로 확인했다. 실제 `getControlFromSequenceAsTwist`는 shift 시1번 제어를 방출하지만 `sequenceCollisionFree`는0번부터 미래 이동을 검사했다. 버리는0번의 감속 후1번 전진이면 검사만 통과할 수 있었다. 실제 방출 index를 사용하도록 맞추고, 벽 앞에서0번 후진이 있을 때만 안전한 경로의 회귀 검사를 추가했다. 계획/제어 지도 padding은 모두0.04m로 맞춘다. 실제 robot geometry, 목표와 footprint 기준을 줄이지 않는다. 후속 전체 실행은 `e2e-84`다.


사용자가 선택 질문에 "정리 단계는 명시적 mock으로 두고, 실제 Mission·주행·왕복 전체 흐름 검증"을 확정했다. 이후 물리 파지 튜닝과 추가 physical 실행을 하지 않는다. 검증 도구에 준비하던 파지 실패 허용 옵션도 제거해 성공 판정을 넓히지 않았으며 실제 Mission SUCCESS/왕복 OK/최종 IDLE와 주행 복구0회를 요구한다. `e2e-84`는 확정한 mock 범위로 수행한다. 기동 시 adapter-ready도 cleanup_fixture_enabled와 실제 cleanup_backend를 구분한다.


`e2e-84`는 회전 코너를 지나 마지막 접근까지 진행했으나 optimizer 전체 후보 실패 후 local clear가 발생해 실패로 종료했다. `evaluations/goal-check/{local_raw,global_raw}/boundary.log`의 실제 저장 지도 native footprint 비교에서 요청 목표x=-1.720m는cost253이지만x=-1.719m는cost254였다(y=2.620m,yaw0). 4cm padding과52cm standoff에서 요청 목표가 장애물 격자 경계에 놓였음을 보여 준다. 저장된 지도 시각과 실제 실패 시각 차이가 있어 유일 원인으로 단정하지 않는다.

실험 접근 거리를 기존 canonical과 같은55cm로 되돌려 목표를[-1.75,2.62,0]으로3cm 물렸다. cup/desk world pose는 그대로이며 도착 허용 오차8mm와 확대된 global/local padding4cm를 유지한다. 물리 파지 도달성은 이번 mock E2E의 검증 범위가 아니다. fixture와 launch14개 검사가 `approach-margin-tests.log`에서 통과했다. `e2e-85`는 최종 mock 범위의 새 전체 실행이다.


`e2e-85`는 새 접근점까지 navigation_result=OK/복구0으로 도착하고 mock 정리2건을 수행했다. 컵 world pose 차이는2.22e-16m로 부동소수점 수준이며 `approach-adjustment.json`에 남겼다. 그러나 복귀 초반 Path progress timeout으로 실패해 전체 성공으로 채택하지 않았다. watcher 취소 후 Mission이 종료한 결과와 정상 성공을 구분한다.

`return-replay/variants.json`은 실제 정체 pose/path/local map, 초기 속도0의 고정 입력에서 GoalAngle threshold1.5/0.5/0.3m를 비교한다. 1.5m의 steady body vy=-0.0182m/s,0.5/0.3m는-0.0794m/s였으며 검사한 optimized 경로 충돌점은 모두0이다. 실제 폐루프 성공이나 성능 비교를 대신하지 않는다. truncated local path의 마지막 방향을 먼 위치에서 요구하지 않도록 기존0.5m로 복원했다. `return-angle-tests.log`의11개 검사(선택 환경1skip)를 통과했고 `e2e-86`에서 전체 왕복을 재시험한다.


`e2e-86`는 Facility18 seat-12 전체 Mission을SUCCESS로 완료했다. 실제 상태는IDLE→NAVIGATE_TO_TARGET→WORKING→POST_MISSION→RETURN_HOME→IDLE/ready=true였고 outbound/return 모두OK다. 두 Nav2 action의 recovery0, planner failure0, behavior0, costmap clear0, 연속 경로 보정 실패0이다. planning refinement113회와 native 검사로 통과한 정지 출력1주기를 별도로 기록했다(정지로 진행 제한을 초기화하지 않음). 전체415.71wall초, 최종 실제 odom 기준home 오차0.05923m(<0.08m), 팔 사전 제어 최대오차0.001581rad, 전체 park 최대오차0.002890rad다.

인식/계획/정리 Action은 사용자 확정에 따른mock이며 물리 수거 검증은false다. completed_tasks는mock trash-1/trash-2다. `evaluations/journal-audit.json`은 public Mission 결과와 missions.db의 결과 일치, executions.db의 모든 실행FINISHED/남은 실행0, base timestamp 단조성과 최종 상태를 독립 대조해passed=true다. 원본은run/mission/navigation/arm.json과두SQLite journal이다. 이 성공을 실제 컵 투입이나 물리 파지 성공으로 분류하지 않는다.

첫 offline GIF 시도(`provenance/render-rendered.json`)는 기본 Isaac Franka 예제가 자산 서버를 확인하면서network=none에서 지연돼 중단했다. 실제 주행은 다시 실행하거나 바꾸지 않았다. renderer만 설치된base Kit experience에서 사용하지 않는robot.policy.examples 의존을 제외한 임시 experience를 만들어 동일 RTX 렌더링을 재실행한다(`rendered-offline`). 원래 실패 로그와 provenance는 보존한다.

후속 렌더링은 저장 stage의 이전 프로세스 전용 Render graph와 physics joint를 복제본에서 제거하고, 별도 USD camera와 RGB annotator, asyncRendering 및 고정 render clock을 사용했다. 기록된 PhysX link pose를 매 프레임 적용하며 새 물리는 계산하지 않는다. `e2e-86/provenance/render-rendered-views.json`의 최종 렌더링이81.59초에 통과했다. `media/rendered-views/render.json`은315프레임,5배속, 기록 대비 최대 link 위치 차이0m를 기록한다. 정면/측면/상단 GIF와 합성 preview.gif, 축소본 및 ffmpeg 명령을 보존했다. 측면은 시작/종료 위치에서 원래 가구에 일부 가려진다. 화면에는 cleanup mock 범위를 표시한다. 중간 렌더러 실패들은 주행 실패와 구분해 원본 로그를 유지한다.

`e2e-87`의 seat-28 시도는 렌더러와 동시에 기동한 환경에서 센서 준비가180초 기동 제한을 넘어 Mission을 시작하지 못했다. 동시 부하의 단독 영향으로 확정하지 않으며 정상 주행 시험 수에 포함하지 않는다. 이후 시도는 순차 실행했다. `e2e-88`은 기존 지도 중심 여유만 검사한 접근점[-0.625,-1.32,π/2]가 원본office_chair_28과 겹쳐 outbound에서 실패했다. 정리 Action은 미실행이며 실패 원본을 보존한다.

fixture 준비에 원본 SDF 의자 collision bounds와 회전된 전체 footprint/padding의 SAT 겹침 검사를 추가했다. 가구 형상이나 배치는 바꾸지 않았고 실제 Mission/인식/계획에 정답 관측을 공급하지 않는다. `chair-approach-preflight.json`의 최초 준비 검사는82개 의자를 확인해 seat-12[-1.75,2.62,0]를 유지하고 seat-28의 의자와 겹친 후보만 제외해[-1.75,-0.55,0]를 선택했다. 저장 지도에는 의자가 없어도 전체 footprint 겹침을 거부하는 회귀 검사를 포함해 `chair-approach-tests.log`의17개 검사가 통과했다(선택 환경1skip). ROS 코드는 이전 성공과 같은 빌드를 사용한다. 새 전체 Mission은`e2e-89`이며 같은 원본 경로 아래 source patch와 untracked source snapshot을 보존한다.

`e2e-89`는 새 seat-28 접근점에 복구0회로 도착해 mock 정리2건을 완료했으나, 홈 직전 회전에서 Path progress timeout으로 실패했다. `evaluations/progress-replay.json`의 실제 navigation feedback을 최초 복귀 경로에 투영하면151→163sim초 동안 yaw 오차2.1978→0.3058rad로 줄어도 진행 좌표는4.9525→4.9605m에 머문다. 최초 경로의 회전 분기와 현재 재계획된 마지막 회전이 달라진 경우를 기존 고정 기준 경로가 반영하지 못했다. 최종 실제 Mission 취소/실패와 성공을 구분한다.

진행 감시 기준을 실제 controller에 전달된 최신 경로로 갱신하며 재계획 자체에는 시간이나 진행량을 주지 않는다. 기존 PathProgressGate.replan으로 arc 기준만 다시 잡고 누적 진행량/12초 제한은 유지한다. 재계획으로 arc 원점이 변해도 정지 시간을 연장하지 못하는 검사와, 갱신된 회전 경로에서 실제 진행만 인정하는 검사를 추가했다. 별도로 float0.008이 double 목표 허용 오차0.008보다 커져 마지막 회전 모드가 꺼지는 정밀도 불일치를 확인해 반경 파라미터도 double로 유지한다. 같은 값의 경계와 실제 초과값 거부를 검사한다.13개 패키지를 다시 빌드했으며 원본은 `progress-rebase-build.log`와 build manifest다. 이후 같은 최종 빌드로 seat-28/seat-12를 각각 새 실행한다.

`progress-rebase-tests.log`의 Navigation24개 검사 묶음은 모두 통과했다. `e2e-90`은 변경된 빌드의 첫 seat-28 실행으로, outbound 책상 옆 회전에서 optimizer 전체 후보 거부와 연속 계획 실패 후 복구 요청이 발생해 실패로 보존한다. 홈 직전 진행 판정 수정의 성공 근거는 아니다. 후속 seat-12 실행은 실패한 선행 시험 때문에 시작하지 않았다.

정리 mock 검증에는 팔의 물리 도달 거리 제약이 없으므로 mock 전용 접근 거리를0.75m로 분리했다. 이는 padded footprint의 회전 반경 약0.681m보다 넓은 여유이며 physical0.55m 설정은 유지한다. 의자 후보 검사에도 mock 전용0.10m 추가 여유를 둬 의자 앞의 새 근접 후보를 제외한다. 가구 배치와 Nav2의4cm padding,8mm 도착 오차,12초 진행 제한은 유지한다. `mock-approach-margin-tests.log`의17개 검사(선택 환경1skip)가 통과했다. 중간 추가 의자 여유 전 준비 결과는 `mock-margin-preflight-seat-{12,28}`이며 이 중간 설정의 실제 실행은 `e2e-91/config/room.json`으로 구분한다. seat-28의 당시 목표는[-1.95,-0.55,0]이며 후속 seat-12[-1.95,2.62,0]는 선행 시험 실패로 실행하지 않았다.

`e2e-91`도 outbound의 optimizer 실패 후 재시도와 최종 경로 정체로 실패했다.0.75m mock 접근 설정은 최종 채택하지 않았고 후속 seat-12는 시작하지 않았다. 이는 물리 파지용 책상 옆 근접 접근의 안정성이 아직 해결되지 않았다는 근거다.

최종 mock 모드는 Facility18의 기존 좌석 navigation target을 그대로 사용한다. 사용자 확정 범위는 실제 Mission·주행·왕복과 명시적 mock 정리이므로 물리 파지 접근점으로 목표를 바꾸는 동작을 분리했다. room.json의 cleanup.physical_approach_target은 미실행 물리 접근 후보이며 navigation_target_source는 'existing seat target'이다. 임시 mock 거리/의자 추가 여유 파라미터는 제거했다. 실제 사용 목표는 seat-28[-0.625,-1.625,1.570796], seat-12[-0.625,1.575,1.570796]이며 물리 파지용 근접 목표 성공을 주장하지 않는다. 앞 단락의0.75m 준비 자료와 실행은 역사적 실패 근거로만 남는다. 기존 목표 보존, physical/mock 분리와 의자 검사 회귀를 포함한 `mock-existing-target-tests.log`의17개 검사가 통과했다(선택 환경1skip). 새 순차 전체 시험은`e2e-92`와`e2e-93`이다.

`e2e-92`는 기존 좌석 목표를 유지했지만 physical Nav2 override와 local StaticLayer가 여전히 mock에 적용된 상태였으며, outbound optimizer 오류와 local clear 후 실패했다. 최종 구성에서는 해당 override와 StaticLayer를 physical backend에만 적용한다. mock의 Nav2 YAML 전체가 cleanup 없는 기존 Facility18 주행 구성과 같은지 launch 구성 검사에서 대조한다. 같은 recovered branch MPPI/lattice,3cm 목표 허용 오차, 기존 footprint 및 LiDAR safety를 재사용하며 모델/가구를 바꾸지 않는다. physical 설정과 분리한 새 전체 실행은`e2e-93`과`e2e-94`다.

전체 Nav2 YAML 동일성 검사를 포함해 `mock-existing-navigation-tests.log`의17개 검사가 통과했다(선택 환경1skip). `e2e-93`의 mock 기동 중 사용하지 않는 RGB-D 첫 capture에서 렌더링 지연도 관찰했다. 후속 mock 실행은 RGB-D camera/control/grasp audit를 만들지 않으며 실제 PhysX, LiDAR와 odometry는 유지한다. adapter-ready는 카메라 비활성화와 perception mock을 명시한다. physical camera 경로는 유지하며 주행 영상은 기록된 자세를 별도로 렌더링한다. 두 실행의 성능 비교 근거로 사용하지 않는다.

`e2e-93`은 기존 seat-28 목표와 기존 Facility18 Nav2 구성으로 실제 Mission SUCCESS를 완료했다. 왕복의 Nav2 recovery0회, planner failure0, behavior0와 costmap clear0이며 최종IDLE/ready=true, 실제 odom 홈 오차0.05310m다. 전체379.63wall초에는 RGB-D 초기 렌더링 지연이 포함된다. mock 정리2건과 물리 수거 미검증을 구분하며 SQLite journal 대조는 `evaluations/journal-audit.json`으로 보존한다. 이어 seat-12의`e2e-94`를 같은 ROS 빌드와 주행 설정에서 실행한다.

`e2e-94`는 기존 seat-12 목표에서 같은 실제 Mission 흐름을 SUCCESS로 완료했다. 전체293.72wall초, 홈 odom 오차0.05444m이며 두 Nav2 goal의 recovery0회, planner failure0, behavior0, costmap clear0이다. 검증된 정지 대체 출력과 연속 경로 보정도0회다. 최종IDLE/ready=true이고 상태는IDLE→NAVIGATE_TO_TARGET→WORKING→POST_MISSION→RETURN_HOME→IDLE였다. 실제 팔은 주행 자세를 유지했으며 mock Action의 arm_recovered=true를 물리 정리 후 팔 복귀 근거로 해석하지 않는다. `evaluations/journal-audit.json`은 Mission 결과와 SQLite report 일치, 모든 실행FINISHED/잔여0을 확인했다. adapter-ready는 실제 PhysX/LiDAR와 RGB-D 비활성화를 명시한다.93/94의 주행 설정은 같으며 RGB-D 기동 차이가 있으므로 wall 시간으로 성능 우열을 비교하지 않는다.

`mission-flow-final-audit.json`과 재현용 `final_scope_audit.py`는93/94의 실제 Mission/왕복/최종 ready, 각2개 navigation goal의 recovery0, 원본 로그의 optimizer failure/patience exceeded/path progress timeout0, controller/planner/costmap/collision monitor 설정 동일성, 빌드 source digest 일치를 확인했다. `mission-flow-journal-audit.json`은 두SQLite 기록을 독립 대조했다. 두 좌석의 정상 mock 정리 E2E2/2를 검증한 것이며, 실패한 물리 파지용 근접 접근이나 실제 컵 투입/팔 복귀를 성공으로 재분류하지 않는다.

최종 영상은 성공한`e2e-94/media/rendered-final/`에 정면/측면/상단 GIF와 preview.gif로 남겼다. `provenance/render-rendered-final.json`은passed, `render.json`은234프레임,5배속과 기록된 link 위치 대비 최대 차이0m를 기록한다. 새 물리 계산 없이 같은 PhysX 주행 기록을 RTX로 재생했으며 cleanup mocked를 영상에 표시했다. 축소본은front-small.gif/side-small.gif/top-small.gif이고 정확한 인코딩 명령과 hash는compression.json에 있다. 일부 측면 시야의 가구 가림을 유지한다. 최종 원본 루트는 `/home/donglee/workspace/cleany/cleany-data/runs/isaac-cleanup-integration-20261025`이며 ROS WIP는 base commit `d3f55473` 위의 실행별 source patch/hash로 식별한다.

2026-10-29 Handoff 검토에서 no-recovery 집계의 누락을 보완했다. Humble의 실제 `Planning algorithm ... failed to generate a valid path`, optimizer/controller 실패와 path progress timeout도 거부하며 정상 경로 보정/검증된 정지 출력은 별도로 유지한다. 새 집계기로 저장 원본93/94는 통과하고89/90/91/92는 거부됨을 재확인했다. 이는 실행 후의 판정 도구 보완이며 시뮬레이션을 새로 실행한 결과가 아니다. Isaac 도구/공통 launch/MoveIt Python82개 검사(선택 환경1skip)와 Navigation CTest24개 묶음이 통과했다. ROS source digest는13개 패키지의 기존 성공 빌드와 일치한다. 최초 CTest 호출은 읽기 전용 ament 출력 경로 때문에 테스트 본체를 시작하지 못했으며, 로그/결과 경로만 임시 쓰기 mount로 분리해 재실행했다. 원본은 위 데이터 루트의`handoff-review-20261029-diqgpwtq/`에 보존했다. 실제 수거 및 물리 파지용 근접 접근의 미완료 판정은 유지한다.
