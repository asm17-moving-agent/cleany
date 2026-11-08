# 실행 원본 색인

## 2026-11-08 D435 헤드 pan/tilt 실물 벽 비교

기준 commit `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경 환경의
Humble Distrobox에서 D435 261822075013, LiDAR, 새 WCH 5B14033435 포트를 사용했다.
ID 13/14 모델 777, 위치 모드와 토크 OFF를 실제 조회했다. 과거 영점 2009/2067과
방향 -1/+1을 적용하고 Cleany-final.FCStd의 horn 축을 이용한 동적 TF를 발행했다.
초기 고정 TF는 pan/tilt 장착 확인 후 중단했다. 카메라 공칭 지면 높이 약 1.117m는
CAD 바닥 추정값이며 실제 높이 검증은 아니다. LiDAR Z=0 평면 관례는 유지했다.

차체 이동 없이 현재 헤드 자세 부근 ±5°를 축별 시험했다. 첫 실행은 tilt 아래에서
복귀할 때 목표 차이 4틱으로 3틱 조건을 만족하지 못해 중단되고 토크 OFF를 확인했다.
실패 기준을 유지하고 아직 측정하지 않은 tilt 위쪽과 복귀를 별도 실행했다.
최종 엔코더 1997/2134와 두 모터 토크 OFF를 확인했다. RAM 속도 80tick/s,
가속도 5와 토크 한도 300은 남고 EEPROM은 변경하지 않았다.

7개 자세 창의 유효 비교 60개에서 자세별 전방 벽 차이 중앙값은 -0.80~-0.92cm,
수평 법선 차이 -1.17~-1.48°, 수직 법선 잔차 -1.34~-1.60°였다. 한 벽의 동일성과
수직성을 전제하며 높이, 벽에 평행한 위치, 장착 영점의 완전한 보정 근거가 아니다.
관련 합성 기하, 시간 유효성, 직렬 패킷과 쓰기 제한 검사 12개 통과.
`cleany-d435-head-tf.service`는 읽기 전용 실제 관절 TF 관측으로 실행했다.
costmap과 Collision Monitor의 깊이 입력은 추가하지 않았다.
원본, 실패 자료와 HTML은 `runs/navigation/integration/2026-11-08_100401_d435-low-obstacle/`
아래에 있으며 `manifest.json`에 분석 입력과 소스 해시를 남긴다.

## 2026-11-08 D435 연결 및 깊이 스트림 확인

기준 commit `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경 환경에서
Fedora USB와 ROS Humble Distrobox의 실제 D435(261822075013)를 확인했다.
USB 2.1, 640×480 Z16, 요청 15fps로 깊이 스트림만 실행했다. 읽기 전용
`tools/validation/probe_depth_camera.py`의 60프레임 표본은 수신 14.07Hz,
유효 깊이 픽셀 중앙값 85.79%였다. 광학 깊이 영상과 CameraInfo 수신 근거이며
8cm 높이 장애물 검출, 차체 좌표 보정 또는 충돌 회피 검증이 아니다.
phone 주행의 LiDAR 전용 obstacle layer와 Collision Monitor는 변경하지 않았다.
카메라는 별도 관측 프로세스로 실행했으며 자동 시작 설정은 추가하지 않았다.
원본은 `runs/navigation/integration/2026-11-08_100401_d435-low-obstacle/`의
`depth-probe.json`, `depth_m.npy`, `depth-preview.png`에 있다.

## 2026-11-08 복구 후보 하한 50% 실행 적용

사용자 요청으로 phone_localization.yaml의 recovery_inlier_ratio를 0.65에서
0.50으로 변경했다. 정상 기준 0.30/0.18m, 개선량 0.04m와 모호성 차이 0.025는
유지했다. 관련 프로필 및 core 검사 21개 통과. 설정은 초기화 시 읽으므로 로봇
정지 확인 후 주행 service를 재기동했다. 컨테이너 내부 runtime 종료가 지연되어
식별한 기존 runtime owner에 SIGINT로 정상 종료를 요청했다. 이전 위치를 복원했고
새 노드 1개 및 GetParameters 응답 0.5를 확인했다. API는 ready, available,
출력 disabled와 STOP 확인 상태다. 이동 목표는 보내지 않았다. 앞서 검증한
짧은 정상 관측의 개선 시간 갱신 수정도 새 프로세스에 반영됐다.
원본은 physical-validation 실행의 recovery-floor-50/에 보존했다.

이 적용 중 기존 recover 함수의 already_aligned 분기를 확인했다. 아래 실험의
기존/A 모델에는 이 분기가 빠져 있으므로 정상 참조의 재지정 수치로 실제
실행 코드 대비 B의 우위를 주장한 결론을 철회한다. 결과는 명시된 검색 분기
모델의 수치로만 해석하며, 전체 실행 정책 비교는 기존 분기를 포함해 다시 해야 한다.
해당 HTML에도 정정 안내를 추가했다.

## 2026-11-08 복구 하한 완화와 정상 재검증 흐름의 다중 bag 비교

`phone-nav2-demo`의 기준 commit `4eaf919145e03ffe248732c950dab1267ded17d2`와
미커밋 변경에서 ROS Humble Distrobox의 오프라인 비교를 수행했다. 완료된 bag 5개와
실행 중 bag의 SQLite 스냅샷 1개를 사용하고 최신 입력은 09:27:30까지 제한했다.
총 2701.68초에서 정지 관측 창 28개를 선택하고 원래 위치와 X/Y ±0.4m,
yaw ±0.35rad를 주입한 196개 조건을 비교했다. 앞 3개와 별도 3개 bag의 결과를
분리했고 결과를 보고 조건을 수정하지 않았다.

기존 65%, 하한만 50%로 완화한 A, 정상 점수 1.5초 유지 시 위치 재지정 없이
복귀하고 그 외에는 개선·모호성·후속 관측으로 후보를 검사하는 B를 비교했다.
정상 기준은 30%/18cm로 동일하다. B의 새 후보 하한도 30%이며 후속 안정성은
1.5초다. 정합 산식 자체는 바꾸지 않았다.

정상 점수가 안정된 참조 창 18개 중 기존/A/B의 복귀 허용은 1/17/18개,
위치 재지정은 1/17/0개였다. 해당 참조의 주입 오차 시험 108개에서는 허용 결과가
참조에서 20cm 또는 0.15rad를 넘게 떨어진 경우가 1/7/9개였다. B의 9개 중
4개는 주입된 위치를 그대로 유지한 판정이다. 별도 3개 bag의 54개 시험에서는
A/B의 허용·참조 잔차 이내 결과가 44/53개, 큰 잔차 허용은 모두 0개였다.
따라서 B의 불필요한 재지정 감소는 확인했으나 모든 bag에서 A보다 낫다고
판단할 수 없다. 어떤 후보도 실물 실행 설정에 적용하지 않았다.

2057개 관측의 기록 점수 대조에서 최대 차이는 일치율 약 0.42%p, 평균 잔차
약 3.3mm였다. 증거 시간 단절과 중복 관측, 모호한 후보와 개선 부족을 확인하는
도구 검사 2개가 통과했다. 기록 AMCL은 실측 정답이 아니며 주입 오차 시험은
참조 상대 결과다. 후보 적용 뒤 AMCL 재수렴, 실제 정지와 물리 성공률은 미검증이다.
원래 불안정한 참조 10개와 파생 시험은 정답 평가에서 제외하고 원본에 남겼다.

입력 스냅샷, 사전 조건, 창별 점군, 전체 후보·판정, HTML과 그래프는
`CLEANY_DATA_ROOT/runs/navigation/traction/2026-11-08_094147_recovery-policy-comparison/`
에 있다. `protocol.json`, `inputs.json`, `results.json`, `comparison.csv`,
`summary.json`, `index.html`로 재현 조건과 결과를 확인한다.

## 2026-11-08 기존 bag의 위치 정합 감속 정책 비교

`phone-nav2-demo`의 기준 commit `4eaf919145e03ffe248732c950dab1267ded17d2`와
미커밋 변경을 Fedora aarch64의 Ubuntu 22.04 ROS Humble 컨테이너에서 검증했다.
완료된 bag `1d3af8ee-85f8-4db3-8b46-d7969de3db62`와
`b826aa59-0d7f-426b-94db-4283b6cff214`의 합계 999초를 5Hz로 평가했다.
원시 스캔, 수신 시각까지의 TF, AMCL pose와 각 진단 bag의 map을 사용했다.
8회의 AMCL 초기 위치 수신 로그에서는 세 정책 모두 신뢰를 재검증하도록 초기화했다.
직접 기록되지 않은 initialpose의 수동/복구 소유자는 확정하지 않았다.

정상 기준을 모두 30%/18cm로 맞춘 정책 비교에서 차단 판정 누적은 기존 202.0초,
수정 전 감속 정책 202.4초, 수정 후 179.2초였다. 기존 차단이 감속으로 바뀐 구간은
25.2초이고 반대로 기존 허용이 새 차단으로 바뀐 구간은 2.4초였다. 짧은 정상 관측이
개선 시간을 갱신하지 않던 문제를 수정하되 연속 정상 1초와 총 대기 8초 제한은 유지했다.
원본 점수와 대조 가능한 819개 스캔에서 최대 차이는 일치율 약 0.42%p와 평균 잔차
약 3.3mm 이하였다. 입력 DB 4개의 전후 SHA-256은 동일했다.

실제 LocalizationGuard에 선정한 기록 구간을 DDS domain 196으로 재생하여
available → degraded → available 전이와 SpeedLimit의 100/30/0% 발행을 확인했다.
Core 12개, 실제 Guard/Supervisor에 합성 입력을 연결한 ROS 9개 검사가 통과했다.
초기 분석에서 reset 이력을 빠뜨린 결과는 initial/에 보존하고 최종 판단에는
corrected/ 결과를 사용했다. 입력은 고정된 과거 AMCL 결과이므로 변경된 속도에서의
폐루프 위치 정확도, 감속 비율의 최적성, 실제 주행 성공률을 입증하지 않는다.
실물 이동 명령이나 서버 재시작은 수행하지 않았으며 수정 코드는 다음 프로세스부터 적용된다.

원본 비교, 설정, 재현 인자, 테스트 로그와 소스 스냅샷은
`CLEANY_DATA_ROOT/runs/navigation/traction/2026-11-08_091549_recorded-reconvergence-validation/`
에 있다. 최종 HTML은 `corrected/index.html`, 입력과 소스 해시는
`corrected/manifest.json`, 최초 분석은 `initial/`에 보존했다.

## 2026-11-08 위치 정합 회복 대기와 단계별 감속

`phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경에서
검증된 위치의 경미한 지도 불일치를 degraded로 구분했다. 기존 NAVIGATION lease를
유지하면서 `navigation/localization_speed_limit`으로 정상 상한의 30%를 요청한다.
MPPI와 기존 Collision Monitor가 같은 제한을 적용하며, Monitor는 초당 30%p로
상한을 낮춘다. 0% 또는 0.6초의 제한 메시지 단절은 즉시 0 출력이다.

2cm의 평균 오차 감소 또는 5%p의 일치율 증가를 개선으로 관측한다. 4초간 개선이
없거나 불일치 시간이 총 8초가 되면 정지한다. 일치율 10% 미만과 평균 오차 35cm
초과가 동시에 0.8초 지속돼도 정지한다. 첫 위치 미검증, 센서/TF 단절, 잘못된
공분산은 유예하지 않는다. 정상 정합 1초 후 감속을 해제하며, 이미 취소한 목표는
자동 재개하지 않는다. 정지 후 후보 검증 기준 65%와 슬립 BT 비활성은 유지했다.

Core 검사 10개, 실제 Guard/Supervisor와 합성 LiDAR/pose를 연결한 ROS 검사 및
Collision Monitor 검사 11개, 감속·메시지 단절의 최종 Monitor 검사 2개가 통과했다.
Navigation CTest 25개 그룹도 통과했고 navigation/teleop 및 UI 빌드가 완료됐다.
ROS lease 검사는 정규 갱신을 지속해 만료로 인한 정지와 구분했다. 각 묶음은
중복을 포함하므로 합산하지 않는다. 심한 오류/단절 정지와 초기 미검증 차단은
그대로 확인했다.

실물은 STOP ACK와 출력 0을 확인한 뒤 재기동했다. 기존 추정 pose를 시드로 전달했고
AMCL을 바꾸거나 새 이동 목표를 보내지 않았다. 실제 graph에서 단일 Guard 발행자와
MPPI/Monitor 두 소비자를 확인했다. 초기 available 이후 degraded 30%와
reconvergence_stalled 0% 전이를 관측했다. 마지막 평균 오차는 약 19.8cm로 정상
기준 18cm를 초과해 자동주행이 제한됐다. 이는 상태 전이/전달 검증이며 실물 감속
거리나 AMCL의 위치 정확도, 모든 지도 누락에 대한 강인성 검증은 아니다.

원본은 데이터 루트 `runs/navigation/traction/2026-11-08_080053_physical-validation/`
아래 `reconvergence-*.json`, `reconvergence-*.log`,
`reconvergence-source-snapshot.tar.gz`에 있다. 실행 연결은
`reconvergence-live-wiring.json`, 실제 전이는 `reconvergence-live-observation.json`,
마지막 차단 사유는 `reconvergence-current-state.json`에 보존했다.

## 2026-11-08 지도 정합 차단 기준 30% 실험

사용자 요청으로 휴대폰 `min_inlier_ratio`를 0.55에서 0.30으로 낮췄다.
복구 후보 0.65, 절사 평균 거리 0.18m, 불일치 0.8초와 정상 복귀 1초는 유지했다.
STOP을 확인한 뒤 런타임을 재기동하고 이전 추정 pose를 복원했다. 실행 파라미터를
재조회해 변경을 확인했으며, 51~56% 관측에서 정합 available과 navigation ready를
확인했다. 위치 정확도를 실측하거나 새 이동 목표를 보내지는 않았다.
원본은 데이터 루트의 `runs/navigation/traction/2026-11-08_080053_physical-validation/`
아래 `alignment-30-before.json`, `alignment-30-after.json`,
`alignment-30-parameters.json`, `alignment-30-build.log`에 있다.

## 2026-11-08 병진 합성 속도와 회전 속도 구간별 제한

`phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경에서
사용자가 요청한 Omni 속도를 적용했다. 빈 공간은 병진 0.5m/s와 회전 0.7rad/s,
주황 영역은 0.16m/s와 0.3rad/s, 목표점 20cm 안의 목표 상한은 0.06m/s와 0.15rad/s다.
병진은 `hypot(vx, vy)`로 제한한다. 기존 운용 휠 상한 4/1.5rad/s를 제거하고 MPPI,
명령 bridge와 Collision Monitor에는 드라이버와 같은 최종 상한 10rad/s를 유지했다.
혼합 동작은 최종 휠 상한에 따라 요청보다 느려질 수 있다.

MPPI는 목표 속도 제한을 후보 평가와 필터 후 출력에 적용한다. 축/휠 가감속 제한과
0.3초 반응 시간을 포함한 사전 감속 곡선을 사용한다. 설정상 0.5m/s에서 주황 영역
속도로 줄이는 시작 거리는 주황 경계 약 0.98m 전이며 실측 제동 거리 보장은 아니다.
새 장애물, 입력 단절과 정지 영역은 기존 즉시 정지 경로를 유지한다. 별도 종방향
재시도 프로필은 기존 속도 범위를 보존했고 슬립 BT 복구는 계속 꺼져 있다.

관련 설정/실제 Collision Monitor ROS 검사 52개, 조종 경로 검사 50개와 navigation
CTest 25개 그룹이 통과했다. C++ 검사는 합성 병진과 회전, 최종 휠 한도, 장애물
사전 감속, 도착 접근, 외부 속도 제한과 가감속 연속성을 확인했다. Navigation과
teleop을 빌드했다. 최초 회귀의 기존 설정값 예상 실패는 새 사용자 설정과
분리한 종방향 프로필 유지 조건으로 갱신했다. 각 검사 묶음은 중복이 있어 합산하지 않는다.

실물은 STOP ACK, MCU 비활성과 출력 0을 확인하고 재기동했다. 이전 추정 pose를
시드로 복원한 후 정합 available, 자동/수동 ready, 모든 관련 lifecycle active를
확인했다. MPPI, bridge와 Collision Monitor의 실제 파라미터를 재조회했고 편집
지도를 유지했다. 새 이동 목표를 보내지 않았으며 0.5m/s 실측 주행, 제동 거리와
해당 속도의 바닥 슬립은 검증하지 않았다.

원본은 데이터 루트 `runs/navigation/traction/2026-11-08_080053_physical-validation/`
아래 `body-speed-*.log`, `body-speed-*.json`, `body-speed-source-snapshot.tar.gz`에 있다.
실행값은 `body-speed-live-parameters.json`, 정지와 준비 상태는
`body-speed-live-observation.json`에 보존했다.

## 2026-11-08 편집 지도 적용과 슬립 감지만 사용하는 검증 구성

`phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경에서
사용자의 `map_adj_1.pgm`을 편집 작업 폴더의 `map.pgm`으로 반영했다. 기존 지도와
해상도, 원점 및 573×782 크기는 같으며 296개 픽셀이 다르다. 원본 기록의 지도는
유지했다. map_server LoadMap 응답과 실제 `/map` 데이터가 새 이미지의 점유 변환과
일치함을 확인하고, 현재 서비스의 다음 시작 경로도 편집 지도의 YAML로 변경했다.

사용자 선택에 따라 `--slip-recovery false`를 추가하고 launch까지 전달했다.
Omni의 `slip_monitor`와 로거 기록은 유지하며 `SlipRecovery` BT, 별도 종방향
planner와 `traction_brake`는 실행하지 않는다. 충돌 감시와 위치 정합 복구는 유지한다.
양쪽 launch 구성을 포함한 관련 검사 48개와 navigation/bringup 빌드가 통과했다.

STOP ACK와 출력 0을 확인하고 재기동했다. 실제 런타임에서 편집 지도 셀 hash,
방향 critic 활성 및 가중치 1/20, 휠 상한 4/1.5rad/s, `slip_monitor` 메시지 수신과
Logger 구독을 확인했다. BT는 기본 Nav2 트리이며 `SlipRecovery`가 없고 모든 관련
lifecycle은 active였다. 자동 명령은 `motion/cmd_vel → collision_monitor →
navigation/safe_cmd_vel`을 지나 기존 단일 base 명령 소유자로 전달된다. 이전 추정
pose를 시드로 전달한 뒤 정합 available과 navigation ready를 확인했다. 새 이동
목표는 보내지 않았으므로 변경 후의 실물 주행과 슬립 복구 성능 검증은 아니다.

원본은 데이터 루트 `runs/navigation/traction/2026-11-08_080053_physical-validation/`
아래 `edited-map-*.json`, `edited-map-slip-*.log`,
`edited-map-monitor-only-source.tar.gz`에 있다. 현재 서비스 설정은 사용자 systemd
drop-in `cleany-phone-nav2.service.d/20-edited-map.conf`에 남겼다.

## 2026-11-08 지도 정합 감지와 정지 상태 위치 복구

기준은 `phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋
변경이며 Fedora aarch64의 `ros2-humble`에서 검증했다. `localization_guard`가
scan 시각의 TF와 기존 지도를 대조하고, phone Supervisor의 NAVIGATION 조건에
정합 health를 추가했다. 복구 owner는 정지 피드백을 확인한 뒤 제한된 pose 후보를
검색하고 독립 검증 빔, 새 scan과 AMCL 결과를 확인한다. 속도 명령과 지도 수정은
하지 않으며 이전 목표도 자동 재개하지 않는다. 일반 안전 프로필은 변경하지 않았다.

최종 Guard/core와 Supervisor 검사 84개가 통과했다. 합성 ROS 입력은 별도 domain에서
작은 공분산의 잘못된 pose, 높은 공분산, 수치 오차 수준의 음수 분산, 불충분한 scan,
모호한 통로, 미확인 정지, 동시 요청과 수동 initialpose 선점을 확인했다. 실제
Guard와 Supervisor를 사용했으나 LiDAR 장면과 AMCL 응답, STOP port는 합성이다.
앞선 profile/runtime/teleop/logger-policy 회귀 162개, base/Supervisor/logger 회귀
238개와 opt-in logger ROS 검사 3개도 통과했다. 해당 묶음은 중복을 포함하므로
합산하지 않는다. navigation CTest 25개 그룹, 5개 관련 패키지 최종 빌드와 UI 빌드가
통과했다. 초기 logger preset 누락은 회귀 검사에서 검출해 수정했다.

실물에서 최초 후보는 독립 빔 일치율 63.3%로 복구 기준 65%에 못 미쳐 적용을
거부했다. 이후 실제 AMCL에서 `-1.4016e-11`의 분산이 관측되어 기존 `1e-12`
roundoff 허용치가 정상 데이터를 거부하는 원인도 확인했다. 두 검사에 `1e-9`
수치 오차 허용을 맞추고, `-1e-8`, NaN, 과도한 분산은 여전히 거부하는 검사를
추가했다. 높은 분산은 주행을 막으면서 유효한 scan/TF를 복구 탐색에는 제공한다.

STOP ACK, MCU disabled와 출력 0을 확인한 뒤 런타임을 재기동하고 이전 추정 pose를
시드로 한 번 전달했다. 마지막 20초 관측에서 초기 안정화 뒤 정합 available과
navigation ready가 유지됐다. 대표 일치율은 63.3%, 절사 평균 잔차는 7.5cm였으며
Supervisor는 OPERATIONAL, 활성 lease와 복구 job은 없었다. 새 이동 목표나 실제
슬립은 유발하지 않았다. 이 결과는 물리 절대 위치 정확도나 실물 슬립 후 복구 성공을
입증하지 않는다. `/localization/status`의 bag 저장, 단일 명령 소유자, 허가된
health publisher 및 지도 YAML/PGM 체크섬 불변을 확인했다.

원본은 데이터 루트의 `runs/navigation/traction/2026-11-08_080053_physical-validation/`
아래 `localization-*.json`, `localization-*.log`, `localization-source-snapshot.tar.gz`,
`runtime/localization/`에 있다. 변경 소스 hash는 `localization-source-manifest.json`,
마지막 관측은 `localization-final-live-observation.json`과
`localization-final-live-probe.json`, 실패 후보는 `runtime/localization/d03a8cd2e245e81b.json`
에 보존했다. 실제 지도 편집과 별도 슬립 경로 복구 알고리즘 변경은 수행하지 않았다.

## 2026-11-08 방향 비용 1/20 적용과 실물 슬립 방향 비교

`phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경에서
sin 기반 `DirectionalMotionCritic`을 횡 1, 대각 20 cost/m로 변경했다. 정지와 MCU
STOP ACK를 확인한 뒤 controller를 deactivate/cleanup/configure/activate했고,
실행 중 `enabled=true`, 가중치 1/20, lifecycle active를 재조회했다. 새 주행 목표는
보내지 않았다. 관련 프로필/runtime/slip 문맥 검사 48개, 보고서 좌표/누락 처리 검사
3개와 navigation 빌드가 통과했다.

실제 bag `9d2c867c-73e4-489a-9e07-c2ee12c64aaf`의 08:20:35.35–08:20:49 KST를
분석했다. 이는 이전 0.1/0.45 비용으로 주행한 기록이며 새 비용의 효과 검증은 아니다.
첫 확정 창의 적분 명령 방향은 28.49도, wheel 방향 29.45도, LiDAR 추정 8.76도였다.
측방 이동은 wheel 5.62cm와 LiDAR 0.80cm, 전체 이동 불일치는 6.75cm/59.1%였다.
약 1초 창이 겹치며, LiDAR 정합 실패와 1.5cm 미만 이동의 방향은 표시하지 않았다.

원본, 적용 증거, PNG/SVG 그래프와 HTML은 데이터 루트의
`runs/navigation/traction/2026-11-08_080053_physical-validation/analysis/directional20-slip/`
및 같은 실행의 `directional20-activation.json`에 있다. 복구 중단 원인은 종방향
플래너의 `no_valid_lattice_path`였으며, 실패 입력 보존과 짧은 안전 복구 자세를 먼저
탐색하는 개선안은 HTML의 계획으로 남겼다. 해당 복구 알고리즘은 이번에 변경하지 않았다.

## 2026-11-08 재기동 잠금과 STOP 응답 복구

기준은 `phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2` 및 미커밋
변경이다. Fedora aarch64의 `ros2-humble`에서 검증했다. 이전 USB 단절의 base latch가
정상 기동 후 남은 사례와 STOP 단발 유실, 최초 MCU 관측 전 boot_id 0 응답을
구분했다. 시연 프로필에만 기동 시 통신 Fault 재검증을 켜고, STOP 재전송 및 이전
lease의 정지 확인을 보완했다. 실제 고장, 원인 미상 latch와 미확인 정지는 유지한다.

Supervisor, base 및 관련 teleop 회귀 검사 224개가 통과했다. 이후 최초 STOP의
boot 식별 및 새 STOP sequence 보완을 포함한 base/teleop 검사 157개도 통과했다.
별도 이전 lease ROS Action 검사가 통과했으며, 결과 수는 중복을 포함하므로 합산하지
않는다. 최초 회귀에서 watchdog Fault가 ACK 전에 해제되는 문제를 검출해 수정했다.
4개 관련 패키지를 캐시에 빌드했으며, STOP 유실은 합성 MCU로 검증했다.

실물에서는 PWM과 목표 0을 확인하고 런타임을 재기동했다. 이전 latch는 정상 health를
확인한 뒤 operator reset으로 정리하고, 이전 lease는 실제 STOP ACK로 종료했다.
재기동 전 측정 위치를 AMCL에 복원한 뒤 조종과 navigation의 ready, MCU disabled,
STOP ACK 및 단일 명령 publisher를 확인했다. 이 검증에서 새 이동 목표는 보내지 않았다.

원본은 데이터 루트의 `runs/navigation/traction/2026-11-08_080053_physical-validation/`
아래 `reconnect-*.json`, `reconnect-*.log`에 있으며 실패/수정 이력은
`reconnect-validation-history.json`에 구분했다. 물리 USB 단절을 다시 일으키거나
자동 주행 정지 거리를 측정한 결과는 아니다.

## 2026-11-08 폴더 정리 후 실물 주행 서버 기동

`phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경을
사용했다. 외부 자료의 현재 위치는 `data-storage-cleanup` worktree의 데이터 목록으로
조회하고, 이전 실행의 절대경로가 든 스크립트는 실행하지 않았다. 빌드 출력은
`~/.cache/cleany/builds/phone-nav2-demo`와 `hardware-deps`에 새로 생성했다.
외부 Agent와 LiDAR revision은 기존 고정 버전과 일치함을 확인했다.

24개 ROS 패키지 빌드, navigation CTest 25개 그룹과 하드웨어 프로필/슬립 문맥
pytest 19개가 통과했다. 외부 드라이버의 최초 빌드는 Micro-XRCE-DDS 의존 prefix가
없어 실패했으며, 캐시에 의존 라이브러리를 빌드한 뒤 Agent를 다시 빌드했다.
관련 원본 로그는 해당 캐시의 build/log에 보존했다.

ESP32와 CP2102 LiDAR 재연결 후 실제 메시지 수신, 단일 명령 소유자, LiDAR
장치 정보와 STOP ACK를 확인했다. MCU는 disabled, fault 0, PWM 0이었다.
기존 확장 지도를 읽어 조종 화면에 제공했으며, AMCL 초기 위치는 자동 추정하지
않았다. 자동주행 전 사용자가 현재 위치와 방향을 지정해야 한다. 이동 명령을
보내거나 실제 복구 주행을 시험하지 않았다.

입력 ID, 설정, 명령, 체크섬, 센서 수신과 정지 증거는 데이터 루트의
`runs/navigation/traction/2026-11-08_080053_physical-validation/`에 있다.
실행 진입점은 `tools/ros2/run-phone-navigation`이며 과거 `analysis/run.sh`와 구분한다.

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

## 2026-11-06 실시간 슬립 감지와 BT 복구

기준은 `4eaf919145e03ffe248732c950dab1267ded17d2`와 `phone-nav2-demo`의 미커밋
변경이다. 코드 해시는 데이터 루트 `reports/slip-bt-integration-20261106/source-manifest.json`에
보존했다. Fedora aarch64의 `ros2-humble` Distrobox에서 navigation과 teleop을 빌드했다.

navigation CTest 25개 그룹과 teleop pytest 94개가 통과했다. 기존 bag의 928개 창을
공유 감지 core로 다시 계산해 의심 39개, 증거 없음 840개, 판정 불가 49개와 사건
시각이 이전 결과와 같음을 확인했다. 전체 Nav2를 별도 도메인 191에서 띄우고
합성 휠 이동/정지 LiDAR를 입력해 `normal → braking → longitudinal_retry →
braking → failed`와 최종 0 명령을 확인했다. 같은 트리의 다음 목표가 정상 추종으로
시작하고 취소되는 검사도 통과했다. 이 입력은 물리적 슬립이나 주행 검증이 아니다.

초기 smoke는 기록 지도에서 같은 위치 목표의 경로 생성에 실패했고, 이후
명시적인 빈 시험 지도와 시작/취소 시나리오로 수정했다. 액션 서버 discovery만으로
시작하면 활성화 전 목표가 거부되어, lifecycle active 확인을 추가했다.
목표를 재사용할 때 상태를 초기화하는 문제는 목표 header와 halt cycle을 기준으로
수정하고 연속 목표 검사로 확인했다. 실제 파라미터 snapshot 재사용 시 내부
`/bond_disable_heartbeat_timeout` 중복 선언이 발생해 그 내부 키를 제거했다.

실제 로봇에는 이동 목표를 전송하지 않았다. 정지와 입력 잠금을 확인하고 Nav2
server를 재구성했으며 드라이버, AMCL과 로거는 유지했다. Omni 기본 planner와
별도 프로세스의 종방향 retry planner, MPPI 두 plugin, 충돌 감시 입력과 단일
최종 명령 소유자를 확인했다. 실물에서 슬립 후 복구 주행은 별도 확인이 필요하다.

원본은 데이터 루트 `reports/slip-bt-integration-20261106/`의
`navigation-ctest.log`, `bag-regression/`, `isolated-slip-final.json`,
`isolated-next-goal.json`, `live-final.json`, `report.html`이다. 실행 시 사용한
파라미터와 원복 자료는 `before/`, `activation/`, 절차는
`experiments/slip-bt-integration-20261106/`에 보존했다.

## 2026-11-06 기록 기반 슬립 의심 구간 분석

- 기준: `4eaf919145e03ffe248732c950dab1267ded17d2`와 `phone-nav2-demo` worktree의
  미커밋 진단 도구. 실행 코드와 입력 cache의 SHA256은 아래 manifest에 보존했다.
- 환경: Fedora aarch64, `ros2-humble` Distrobox Python. ROS 노드 생성이나 주행 명령 없이
  SQLite 읽기 전용 추출과 기존 LiDAR matcher를 사용했다.
- 입력: bag `ef507ee6-e9b6-49db-ad86-65dba4d20a9d`,
  2026-11-06 07:22:40–07:30:25 KST. scan 3,597개, odom 21,670개.
- 결과: 비교 창 928개 중 의심 39개, 증거 없음 840개, 판정 불가 49개.
  연속 감지 3구간은 모두 07:26:26–54에 집중됐다. 사람이 길을 막고 비킨 것으로
  설명한 두 경로 변화 주변 비교 구간에서는 의심 창이 없었다. 정확한 물리 행동
  시각과 전체 구간의 정답은 없으므로 정확도와 오탐률을 확정하지 않았다.
- 실패: odom gap 6개, 정합 품질 또는 교차 검사 탈락 43개는 판정 불가로 남겼다.
  단위 검사 47개 통과. 런타임 자동 감속이나 물리 동작 변경은 검증하지 않았다.
- 원본: 데이터 루트 `reports/slip-detection-20261106/report.html`,
  `full/manifest.json`, `full/inputs.npz`, `validated/{manifest,summary}.json`,
  `validated/windows.jsonl`. HTML 재생성은
  `experiments/slip-detection-20261106/report.py`를 사용한다.

## Nav2 복구 자료의 저장소 외부 보관

PR #73의 복구 자료 `recovered/nav2/`는 구현 코드와 분리해 데이터 루트의
`archives/pr73-recovery-cleanup-20261102-c978e838/recovered/nav2/`로 옮겼다.
이동 전 Git 기준은 `c978e838e9f9cca8afb1bb386fa7c960c47bfdb4`이며, 원본 출처와
검증 기록은 보관 디렉터리의 `README.md`, `INDEX.json`, `provenance/`에 있다.
활성 MPPI 구현과 설정은 `ros2_ws/src/cleany_navigation/`에 유지한다.

## PR #73 snapshot 다운로드 지연

`25b8eca28ad7f493087d68464385414203715a32`의
[CI 41379752233](https://github.com/asm17-moving-agent/cleany/actions/runs/41379752233)은
Ubuntu snapshot의 반복된 HTTP 502와 503으로 이미지 생성에서 33분 이상 경과해 중단했다.
코드 빌드와 테스트는 시작하지 않았다. 의존성 명령 재시도뿐 아니라 APT의 파일별 재시도가 누적됐다.
원본은 데이터 루트의 `reports/ci-dependency-pinning-20261102/pr73-ci-41379752233.log`다.

서명이 검증된 snapshot index의 SHA256과 크기를 유지하면서 동일 pool 파일을 archive 미러에서
먼저 받는 다운로드 도구를 추가했다. SHA256 또는 크기가 다르면 cache에 넣지 않으며 snapshot으로
돌아간다. 미러 index를 읽거나 버전을 변경하지 않는다. 설치 명령 다운로드 5분과 이미지 생성
15분 상한을 추가하고 APT 자체 재시도 1회, 명령 실행 최대 2회로 줄였다.
다운로드 도구와 기존 CI 도구의 자동 검사 24건이 통과했다. 고정 amd64 컨테이너에서
이전 실패 패키지 `libxrandr-dev`, `mesa-vulkan-drivers`, `python3-pip-whl`,
`python3-setuptools-whl`을 미러 다운로드와 SHA256 검증으로 재설치했다.
이후 버전 검사에서 Python 36개와 공통 ROS 311개가 계속 일치했다.
설치 로그는 같은 데이터 루트의 `pr73-combined/pr73-mirror-install.log`다.

## PR #73 통합 빌드의 병렬 제한과 MPPI 계약 검사

2026-11-02에 PR head `25b8eca28ad7f493087d68464385414203715a32`와 아래 수정 내용을
고정 의존성 amd64 이미지 `61625b7ef43b330932899ca41fa455e0570962dbc1549dba6ae221a583b160d0`에서 검증했다.
컨테이너는 CPU 3개 분량과 메모리 8 GiB로 제한했다. 첫 빌드에서는 colcon이
`CMAKE_BUILD_PARALLEL_LEVEL=2`에도 `-j12 -l12`를 주입해 MPPI 컴파일러 5개가 OOM으로 종료됐다.
`MAKEFLAGS=-j2 -l2`를 함께 전달한 뒤 28개 패키지가 빌드됐고 추가 OOM은 없었다.

통합 검사에서 이전 GoalDirectedController와 배포판 MPPI를 기대하던 assertion 6건이 실패했다.
기대 플러그인을 현재 Cleany MPPI로 갱신하고 Omni, 횡이동, footprint 비용과 Smac Lattice 조건을 확인했다.
검사를 제거하거나 제어 설정을 완화하지 않았다. 관련 검사 22건과 전체 2820 reported tests가
통과했다(errors/failures 0, 기존 skip 32). 별도 Nav2 synthetic ROS 실행 검사 3건,
Gazebo 센서/TF 실행 검사 1건과 motor core 검사도 통과했다.
실제 로봇 검증은 수행하지 않았다.

원본 로그는 데이터 루트의 `reports/ci-dependency-pinning-20261102/pr73-combined/`에 보관한다.

## CI 의존성의 Jetson 버전 기준

PR #73의 `20dae9af760e3212fbec8c0b5476ab73ff928d90`에서 실행한
[CI 41377740118](https://github.com/asm17-moving-agent/cleany/actions/runs/41377740118)은
이미지 생성 중 Ubuntu snapshot 서버가 패키지 4개에 HTTP 502를 반환해 실패했다.
`ros-humble-mujoco-ros2-control` 설치 단계에서 중단됐으며 코드 빌드와 테스트에는 도달하지 않았다.
고정 snapshot과 Python constraints를 유지한 채 APT 다운로드 재시도 및 의존성 준비 명령의 제한된 재시도를 추가했다.
원본은 데이터 루트의 `reports/ci-dependency-pinning-20261102/pr73-ci-41377740118-failed.log`다.

2026-11-02에 `jetson-orin`의 Ubuntu 22.04 arm64 system Python과 `dpkg-query -W`를 읽었다.
최종 검증 기준은 `c50c440540a7c5c4c0736635d9d122b4736b2a47`이며 변경은 `feat/ci-dependency-pinning` 작업 트리에서 검증했다.
실물의 패키지 설치와 설정은 변경하지 않았다.

ROS `2026-08-07` snapshot의 amd64와 arm64 package index 모두 Jetson의 ROS 패키지 388개와 upstream 버전이 일치했다.
아키텍처별 Debian 빌드 시각은 비교에서 제외했다. MuJoCo `3.11.0`과 google-genai `2.18.0`의 Python 의존성 36개를 조회해 constraints로 보존했다.
Ubuntu 저장소는 `20260807T000000Z` snapshot으로 고정했다. 이는 Jetson 전체 OS 패키지나 JetPack의 동일성을 뜻하지 않는다.

로컬 arm64 이미지 생성은 해당 ROS snapshot에 `ros-humble-ros-gz-sim`이 없어 실패했다.
같은 snapshot의 amd64에는 `0.244.25-1jammy.20260726.122251`이 있어 실제 CI와 같은 amd64에서 검증한다.
첫 amd64 생성은 베이스에 남은 TF2 `0.25.20` 등과 Jetson의 `0.25.22` 차이를 버전 검사에서 감지해 실패했다.
`rosdep install`이 이미 설치된 패키지를 갱신하지 않아, workspace 설치 전에 베이스 ROS 패키지를 snapshot으로 동기화하도록 수정했다.
재생성은 저장소의 SHA256과 대조한 APT 파일 캐시를 사용하며 workspace 빌드와 테스트 결과는 재사용하지 않는다.
첫 workspace 빌드는 BT.CPP `4.9.1`의 CMake export가 `lib/`에서 라이브러리를 찾지만 실제 파일은 `lib/x86_64-linux-gnu/`에 있어 실패했다.
같은 바이너리에 대한 호환 링크를 추가한 뒤 `cleany_manipulation_bt` 단독 빌드를 통과했다.
최종 검증은 amd64 Ubuntu 22.04, Python 3.10.12 컨테이너에서 새 workspace 빌드 디렉터리와 앞선 시도의 컴파일 캐시를 사용했다. 모든 테스트는 다시 실행했다.

초기 `326c237c` 기준의 24개 패키지 빌드와 2,561개 검사도 통과했다.
검증 도중 병합된 ToF와 Supervisor를 포함하도록 `c50c4405`에 동기화한 뒤 아래 전체 검사를 다시 실행했다.

| 검사 | 결과 |
|---|---|
| 의존성 이미지 생성과 버전 검사 | Python 36개, 공통 ROS 311개 일치 |
| `make test-ci-tools` | 13개 통과 |
| `make test`: 전체 빌드 | 27개 패키지 통과 |
| `make test`: workspace 검사 | 2,720개, 오류와 실패 0개, skip 32개 |
| `make test-motor-core` | 통과 |
| 합성 Navigation runtime | 3개 통과 |
| Gazebo sensor/TF runtime | 1개 통과 |

설치 버전, 이미지 ID와 dependency key는 원본의 `amd64-current/image.json`, `amd64-current/dpkg-packages.txt`, `amd64-current/pip-packages.txt`에 남겼다.

원본 Jetson 환경, package index, 이미지 빌드와 검사 로그는 데이터 루트의
`reports/ci-dependency-pinning-20261102/`에 보관한다.

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

## System Supervisor와 Mission Runtime 통합

검증 코드는 `ad472e2`(main `d0afd48` 위 리베이스)다. Ubuntu 22.04.5 aarch64 Distrobox, ROS 2 Humble, Python 3.10.12에서 실행했다. 원본 로그, 지연 JSON과 `verification.json`은 데이터 루트의 `runs/supervision/20261023/`에 있다.

| 검사 | 입력과 결과 | 원본 |
|---|---|---|
| `make test-supervision-core` | 50 passed | `test-supervision-core.log` |
| `make test-supervision` | 69 passed. 실제 Supervisor, safety adapter, base driver와 MockMcu, `mission_runtime`(Nav2 backend, supervised), lifecycle 복구 owner. 센서, Nav2 action, lifecycle manager와 안전 하드웨어 Bool은 합성 | `test-supervision.log` |
| 같은 Supervisor 검사, 18개 CPU busy loop | 47 passed | `test-supervisor-under-cpu-load.log` |
| `make test-mission-core`, `test-base`, `test-navigation` | 77 passed와 17 skipped, 109개, 191개 통과 | 각 로그 |
| `make test-mission-mock` | 233 passed, 2 skipped, 1 failed. `test_process_kill_restart_reports_interruption_without_action_result`는 main `d0afd48`에서도 5회 중 3회 실패 | `test-mission-mock.log`, `flaky-process-kill-*.log` |
| Workspace colcon test, `cleany_manipulation_bt` 제외 | 2218개 중 실패 3. 위 flaky 1개와 이 문서의 기존 MoveIt mock planning `status == 6`(ctest와 xunit 두 번 집계) | `workspace-test-skip-bt.log` |
| `make test` | 이 환경에 `behaviortree_cpp` v4가 없어 `cleany_manipulation_bt` build에서 중단 | `make-test.log` |

Lease 승인과 상태 파일 기록 사이에 Supervisor가 종료되면 lease가 사라지는 결함을 부하 실행에서 발견해 승인 응답 전 기록하도록 수정했다. 수정 전 같은 부하에서 재시작 재확인 검사가 8회 중 4회 실패했고 수정 후 8회 모두 통과했다.

LiDAR 무효 scan 발행부터의 지연이며, 같은 host 한 process 안의 DDS loopback과 Runtime tick 10 Hz 기준이다. 실제 Nav2 controller 정지, 모터 감속과 다중 host 지연은 포함하지 않는다.

| 구간 (5회) | 중앙값 / 최대, 무부하 | 중앙값 / 최대, CPU 부하 |
|---|---|---|
| Lease 철회 | 42 / 46 ms | 24 / 53 ms |
| Nav2 goal cancel 수신 | 46 / 50 ms | 28 / 57 ms |
| Supervisor 정지 확인 (`MISSION_STOPPED`) | 96 / 112 ms | 121 / 201 ms |
| Supervisor 종료 → Nav2 cancel | 0.995 s (heartbeat timeout 1 s) | 0.994 s |

E-stop 회로, MCU 통신 차단 시 모터 정지, 실제 Nav2 lifecycle 복구, 실물 팔 controller, Jetson 다중 프로세스와 SROS2는 검증하지 않았다.

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

## 실물 매핑 패키지 통합

기준은 원격 main `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`, 통합 작업은
`feat/hardware-mapping-integration`이다. 변경 소스 hash는 로컬
`artifacts/mapping-probe-11/source-manifest.json`에 보존한다. 환경은 Ubuntu 22.04
Distrobox, ROS 2 Humble, Linux arm64이며 실물 USB는 열지 않았다.

| 검사 | 입력과 확인한 경계 | 원본 |
|---|---|---|
| 설치 빌드 | lockfile UI와 관련 패키지 20개 빌드 | `artifacts/mapping-probe-11/build.log` |
| 패키지 회귀 | 관련 ROS 검사, ordered permission과 odom reset, 기존 navigation/Logger 회귀 | 같은 디렉터리 `packages.log` |
| 순수 core와 JS | 세션 직렬화, 저장 실패, origin yaw, PID 재사용과 지연 저장 세대 분리, 누름/해제/복구 | `core.log` |
| 설치 launch | `/tmp`에서 실행, synthetic MCU와 합성 room scan, 단일 휠 publisher와 STOP 확인 | `verification.json`, `runtime.log` |
| 세션 저장 | 초기화 2회, 서로 다른 generation과 output, 두 지도/pbstream, Logger flush와 정상 종료 | `runtime/maps/`, `runtime/recordings/` |
| CDR 재생 | domain 175에서 원본 bag 재생, raw/filtered scan, 휠 상태/명령, joint state, odom, TF와 map 수신 | `replay-verification.json`, `replay-verification.log` |
| 종료 소유권 | 기록된 PID와 시작 시각을 대조해 남은 owned child 없음 확인 | `ownership-verification.json` |
| HTTP UI | 설치 자산, token/Origin 거절, 경로 이탈 차단과 CSP | `artifacts/integrated-ui/http-verification.json` |

초기 재현은 `mapping-probe-01`부터 `-10`에 보존했다. 같은 ROS domain을 사용한 검사 간섭,
Logger의 동적 구독 제거 중 InvalidHandle, 종료 래퍼의 신호 전달, symlink 설치 자산의
경로 검사, 수신이 없거나 관측용 주기가 적용된 bag 토픽 누락을 구분해 수정했다.
Logger는 구독별 callback group과 단일 executor로 retired handle이 제어 서비스를 막지 않게 하며,
매핑용 원본은 명시적으로 수신마다 기록한다. ROS 패키지 검사는 domain 174에서 순서대로 실행한다.

브라우저 연결 호스트가 없어 실제 화면 렌더링은 이번 실행에서 확인하지 못했다.
실물 장착 TF, 물리 정지 거리, 바닥 주행과 지도 품질은 이 합성 검사의 범위가 아니다.

의존성 점검에서는 Humble rosdep에 `sllidar_ros2`, `micro_ros_agent` 규칙이 없는 것을 확인했다.
실행 의존성은 manifest에 유지하고, 일반 설치/장치 없는 CI에서는 두 외부 source-build 의존성만
명시적으로 제외한다. CI의 조종 UI 빌드는 별도 Node 24 단계로 추가했다. 이 로컬 검사는 GitHub
Actions 실행이나 실물 드라이버 설치를 대신하지 않는다.

## 매핑과 조종 진단 검증 (2026-11-02)

후속 진단 변경의 commit과 파일 hash는
`artifacts/debug-diagnostics-02/source-manifest.json`에 기록한다. 환경은 로컬 Ubuntu 22.04
ROS 2 Humble container이며 synthetic MCU와 합성 room scan을 사용했다.

20개 패키지 빌드, Python core 61개와 조이스틱 17개, 관련 ROS 검사 428개가 통과했다.
RPC 거절, 동기 및 비동기 예외, 빈 응답, 동일 실패 반복 조회, 시작과 정리의 동시 실패,
진단 파일 저장 실패, 이벤트 큐 포화와 파일 회전은 core 검사로 확인했다.
원본 로그는 같은 artifact 디렉터리의 `build.log`, `core.log`, `packages.log`에서 확인한다.

설치 launch의 두 매핑 세션에서 지도와 pbstream, Logger bag 저장과 STOP 후 종료를 확인했다.
관리자 종료 후 ROS 상태 캐시가 unavailable로 전환됐고 조종 사건 JSON Lines가 남았다.
결과와 세션 경로는 `verification.json`, 종료 후 owned PID 확인은
`ownership-verification.json`에 보존한다. 실물 USB, 실제 주행과 화면 렌더링은 검증 범위에
포함하지 않았다. HTTP 자산 제공과 UI TypeScript 빌드는 확인했다.

## 2026-10-26 ToF와 System Supervisor 통합

기반은 `main`의 `eb2c56a1`과 `feat/system-supervisor`의 `e0d5001e`를 합친
`cbe09bed8cc139e86d85f91201458f4a61693df9`다. 검증 당시 ToF 구현은 이 commit 위의 미커밋 변경이었다.
원본과 설정 SHA256은 `CLEANY_DATA_ROOT/runs/tof-integration/source-manifest.sha256`에
보존한다. KB는 Supervisor 브랜치의 gitlink를 사용하며 내부 내용을 수정하지 않았다.

Ubuntu 22.04 arm64 Distrobox, ROS 2 Humble, Python 3.10에서 `make build-tof`,
`make test-tof-core`, `make test-tof`, `make build-navigation`, `make test-navigation`을
실행했다. 마지막 ToF 패키지 재검사를 반영한 colcon 결과는 207 tests, 오류/실패 0,
선택 시나리오 skip 15다. ToF 자체 34 tests에는 합성 ROS와 pseudo-terminal이 포함된다.
Navigation colcon 결과는 194 tests, 오류/실패/skip 0이다. 중복된 core 검사는 합산하지 않는다.

선택 Mission ROS 검사는 별도로 13 passed, WebSocket 의존성 누락 skip 1이었다.
이후 WebSocket 검사도 단독 통과했다. 최초 추가 실행은 Ubuntu의 websockets 9.1에서
handshake reset으로 실패했으며 Python 3.10 호환 버전인 10.4를 별도 `testdeps` 경로에
설치해 통과했다. 제품 Bridge 코드는 수정하지 않았다. 재현 시 이 경로를 PYTHONPATH에
앞세운다. Gazebo 물리 시나리오는 실행하지 않았다.

원본 결과는 `CLEANY_DATA_ROOT/builds/feat-tof-integration/build/*/pytest.xml`,
`build-navigation/cleany_navigation/test_results/`, `log/`와 `log-navigation/`에 있다.
별도 검사 요약은 `CLEANY_DATA_ROOT/runs/tof-integration/validation.json`에 있다.

공급 펌웨어의 `evidence/range-15hz-2min.jsonl`에서 초기 10Hz 28 frames를 제외한
15Hz 1,817 frames를 오프라인 파싱했다. 평균 유효 zone 33.7347, 기본 보호 coverage를
만족한 frame은 0이었다. 마지막 통계 수신 중 추가 frame도 포함하므로 공급 보고서의
120초 집계 1,816과 범위가 다르다. 원본 경로와 결과는 같은 runs 폴더의
`firmware-replay.json`에 남겼다. 이는 신규 장치 통신이나 실물 정지 검증이 아니다.
실물 commissioning과 한계는 [CALIBRATION](CALIBRATION.md#tof-근접-보호-commissioning)과
[ToF 계약](../ros2_ws/src/cleany_tof/docs/interfaces.md)을 따른다.

## 2026-10-26 ToF 미검출과 방향별 관측 근거

아래 결과는 `905f3d7b673f755a7a7bdbe87b731392558e322e` 위의 작업 트리에서 커밋 전에 검증한 기록이다.
센서는 단독 측정만 완료했고 로봇에 미장착이라는 사용자 정정을 반영했다. 원본 firmware와
실제 센서/모터 설정은 변경하지 않았다. 정책 계약은
[ToF reference](../ros2_ws/src/cleany_tof/docs/interfaces.md)를 따른다.

`range-15hz-2min.jsonl`(SHA256
`ec02366ab656431fed101f7bcc50ca5e9b67b045fc144a1ea4f5f8aaacdcb361`)에서
세션 이전 10Hz 28 frames를 제외한 15Hz 1,817 frames, 116,288 Zone samples를 분류했다.
정상 미검출(count=0/status=255)은 53,109, 품질 조건을 통과한 RANGE는 60,868,
UNRELIABLE은 2,311, NOT_UPDATED는 0 samples였다. 1,817 frames 모두 수신된 관측으로
유지되지만 주행 가능 판정은 아니다. 기존 전체 Zone 조건을 만족한 frame은 0이었다.
이 파일의 firmware-valid 거리 최댓값은 3.22m여서 약 3.5m 미검출의 별도 사용자 관찰을
검출 보장 거리나 자유 공간 하한으로 일반화하지 않았다.

원본의 시작 wrap status, 혼합 품질, 최장 valid 거리 frame 3개를
[test fixture](../ros2_ws/src/cleany_tof/test/data/standalone_frames.json)에 측정 필드 그대로
보존했다. 단독 측정에는 RGB-D의 동시 관측이나 실제 장착 coverage가 없다.
재생 도구는 `tools/validation/tof/replay.py`, 결과는
`CLEANY_DATA_ROOT/runs/tof-coverage/standalone-audit.json`이다. offline 도구의 도착 시각은
검사 재현을 위한 device clock이며 신규 USB 지연 측정으로 해석하지 않는다.

Ubuntu 22.04 arm64 Distrobox, ROS 2 Humble, Python 3.10의 기존 개발환경과 build cache에서
다음 검사를 실행했다. 중복 검사는 합산하지 않는다.

| 명령과 범위 | 결과 |
|---|---|
| `make test-tof-core` | 77 passed |
| `make build-tof`, `make test-tof`와 최종 ToF package 재검사 | 9 packages build; colcon 231 tests, errors/failures 0, 선택 Mission/Gazebo skip 15 |
| ToF package 자체 | 58 tests, errors/failures/skips 0 |
| Humble의 `make test-navigation` | 6 packages build; colcon 199 tests, errors/failures/skips 0 |
| `git diff --check` | 통과 |

기존 finite-depth 기하와 실제 depth coverage 발행 노드→ToF 소비자 연결을 합성 RGB-D,
TF와 odometry로 검사했다. 무효 pixel, stale/잘못된 model/명령 근거의 철회, 정상 미검출의
조건부 속도 제한, 품질 불량 정지, firmware event status=255의 sensor_ready 철회와
기존 lease/Action 취소 경계를 검사했다. 초기 연결 검사에서 test TF가 평가 시각보다
미래였던 fixture를 실제 관측 시각과 일치시켰다. observer 설정이 없는 launch 프로파일도
별도로 검사하여 구성 누락 경로를 수정했다. 검사 조건을 완화해 통과시키지 않았다.

원본 XML과 로그는 `CLEANY_DATA_ROOT/builds/feat-tof-integration/{build,build-navigation,log,log-navigation}`에,
이번 변경의 소스 SHA256과 요약은 `CLEANY_DATA_ROOT/runs/tof-coverage/`에 있다.
합성 근거와 임시 sensor/body geometry는 실물 중첩을 입증하지 않는다. 실제 Nav2 전환을
포함한 full-stack 주행, Gazebo 주행 시나리오, 장착 후 제동과 접근 정확도 검사는 수행하지
않았다. `calibrated=false`를 유지하며 장착 후 항목은
[CALIBRATION](CALIBRATION.md#tof-근접-보호-commissioning)에서 확인한다.

## 2026-10-26 ToF ROI 감지와 책상 접근 단순화

기준 commit은 `162f3788a74c3674d4e1bb1f93df0f3efad8666b`이며 아래는 그 위 작업 트리의
검증이다. 사용자 지시에 따라 정상 NO_RETURN을 장애물 미검출로 처리하고, ToF 전용
RGB-D sweep proof와 전체 Zone coverage 요구를 제거했다. 위 절의 proof 정책은 이전
실행 기록이며 현재 계약은 [ToF reference](../ros2_ws/src/cleany_tof/docs/interfaces.md)다.
기존 head-pan observer, Nav2/Collision Monitor, Mission/Supervisor와 모터 watchdog은
유지했다. 센서와 firmware를 새로 구동하거나 모터 명령을 보내지 않았다.

Ubuntu 22.04 arm64 `ros2-humble` Distrobox, ROS 2 Humble, Python 3.10과 기존 build cache에서
다음 명령을 실행했다. 순수 검사와 package 검사는 일부 중복되므로 합산하지 않는다.

| 명령/범위 | 결과 |
|---|---|
| `make test-tof-core` | 91 passed |
| `make test-tof` | 9 packages build, colcon 247 tests, errors/failures 0, 기존 opt-in Mission skip 15 |
| ToF package 자체 | 74 tests, errors/failures/skips 0 |
| Humble에서 `make test-navigation` | 6 packages build, colcon 198 tests, errors/failures/skips 0 |
| `git diff --check` | 통과 |

navigation의 colcon 198 집계에는 CTest 실행 단위 9개도 포함된다. 개별 pytest/gtest
189 cases와 별도의 독립 검사 수로 합산하지 않는다.

합성 입력으로 15Hz 인접/물리 cluster와 시간 확인, flash/고립/비인접 noise, 중복 frame,
입력 공백 reset, ROI 제외, 단일 비상 return, 거리/해제 hysteresis를 검사했다. ROS 검사에서는
실제 ToF 노드에 합성 frame/odom/lease를 넣어 정상 미검출의 proof 없는 주행 허용,
전부 불량/센서 장애/stale 정지, 일부 불량 Zone의 접근 수락, 평면 소실 abort, 기존
cancel/lease/정지 확인 경계를 검사했다. 드라이버 검사는 pseudo-terminal이며 실제 USB가
아니다. graph의 기존 안전 chain은 identity 대역이다. navigation launch 검사는 ToF on/off와
기존 observer on/off의 네 조합을 조립하며 실제 프로세스를 구동하지 않는다. 제거된 proof
전용 검사와 별개로 원래 finite-depth observer 회귀는 유지했다.

첫 순수 검사에서 ROI 제외 검사와 확인 검사에 같은 timestamp를 재사용한 fixture가
실패했다. 필터의 동일 frame 중복 방지를 유지하고 별도 시나리오 사이 fixture를 reset했다.
재검사 결과는 위 표와 같다. 선택 skip은 성공으로 처리하지 않았다.

기존 `range-15hz-2min.jsonl`을 offline audit로 다시 분류했다. 원본 SHA256은
`ec02366ab656431fed101f7bcc50ca5e9b67b045fc144a1ea4f5f8aaacdcb361`이며 10Hz 28 frames를
제외한 1817 frames에서 RANGE 60868, NO_RETURN 53109, UNRELIABLE 2311 samples였다.
이 재생은 새 USB latency 측정이나 로봇의 주행 허용률 검사가 아니다.

원본 XML은 `CLEANY_DATA_ROOT/builds/feat-tof-integration/{build,build-navigation}`에,
이번 audit와 작업 트리 manifest는 `CLEANY_DATA_ROOT/runs/tof-simplification/`에 보존한다.
센서는 아직 미장착이며 `calibrated=false`다. full-stack Nav2/ToF 또는 Gazebo 주행,
실물 검출/제동/접근 정확도 검사는 하지 않았다. 장착 후 ROI와 거리/시간 후보값 튜닝,
오탐/미탐, 실제 지연과 정지는 [CALIBRATION](CALIBRATION.md#tof-근접-보호-commissioning)의
별도 검증 대상이다. ToF와 다른 센서가 같은 장애물을 모두 미검출할 수 있다.

## 2026-11-02 voxel guard 10Hz 평가 설정

기준은 `162f3788a74c3674d4e1bb1f93df0f3efad8666b` 위의 미커밋 작업 트리다.
기존 ToF 단순화 변경을 보존하고, 여러 감지 경로를 함께 사용하는 구성에서 voxel
충돌 검사와 속도 출력의 기본 주기를 0.05초에서 0.10초로 바꿨다. 공통 navigation,
Gazebo YAML과 node fallback에 같은 값을 적용했다. 지도 갱신, Collision Monitor,
ToF 주기, timeout과 제동 궤적의 `step_s`는 그대로다. 개별 robot profile의 명시적
주기 override는 계속 우선한다.

Ubuntu 22.04 arm64 `ros2-humble` Distrobox와 기존 build cache에서 다음을 확인했다.

| 검사 | 결과 |
|---|---|
| Humble `make test-navigation` | 6 packages build, colcon 198 reported tests, errors/failures/skips 0 |
| Gazebo package와 의존성 build | `colcon build --packages-up-to cleany_gazebo_sim`, 8 packages 성공 |
| 실제 ToF selector/guard의 합성 10Hz 명령 입력 | 정상 구간 49개 출력 sample의 사유 CLEAR, 입력 중단 시 COMMAND_STALE와 zero 확인 |
| `git diff --check` | 통과 |

Gazebo package만 먼저 build한 시도는 해당 install tree의 cleany_base_odometry 누락으로
실패했다. packages-up-to로 의존성을 함께 build해 해결했다. navigation의 198 집계는
CTest 실행 단위 9개를 포함하므로 독립 case 수와 합산하지 않는다.

합성 연결은 별도 DDS domain에서 15Hz frame/odom과 10Hz identity 중계로 실행했다.
상류 명령의 관측 간격 평균은 0.1000초, 최대는 0.1049초였다. 실제 voxel 계산이나
Collision Monitor, Nav2 구동은 이 중계에 포함하지 않았다. 임시 script와 result.json은
`CLEANY_DATA_ROOT/runs/voxel-rate-10hz/`, build와 XML은 기존
`CLEANY_DATA_ROOT/builds/feat-tof-integration/{build-navigation,log-navigation}`에 있다.
실제 CPU 절감, 부하 중 worst-case 지연과 물리 정지 거리는 측정하지 않았다.

## 2026-11-02 ToF PR 검증의 상태 전환 동기화

`162f3788a74c3674d4e1bb1f93df0f3efad8666b` 위의 PR 준비 작업 트리에서 순수 검사
91개를 재실행해 통과했다. 첫 `make test-tof`는 정상 미검출 전환 검사에서 이전 속도
출력과 새 OBSTACLE_HOLD 사유를 비교해 실패했다. release_s 동안의 정상 정지 동작은
유지하고, 테스트가 CLEAR와 정상 출력, AVAILABLE 및 실제 64개 NO_RETURN 수신을 함께
기다리도록 수정했다. 기존 출력 임계값과 센서 상태 조건은 유지했다. 실제 하드웨어는
사용하지 않았다. 재검사 XML은 기존 `build/cleany_tof/pytest.xml`에 있다.

수정 후 make test-tof 재실행은 colcon 247 tests, errors/failures 0, 기존 선택 skip 15였으며
ToF package 74 tests는 errors/failures/skips 0이었다. navigation 198 reported tests와
합성 10Hz 연결 검사는 같은 소스의 앞선 실행 결과다.

## 2026-11-02 PR #71 rosdep 초기화 실패 수정

PR head `ea5adf40dad37c80aa5e4d1da98acbbe942f8769`의 CI run `41363593569`는 테스트 실행 전
의존성 이미지 생성에서 실패했다. cleany_safety_adapter, cleany_system_supervisor,
cleany_tof가 ament_python을 buildtool dependency로 선언해 rosdep이 해석하지 못했다.
세 선언만 제거하고 export의 ament_python build type, 실제 runtime/test 의존성과 CI의
의존성 검사는 유지했다. 기존 로컬 overlay의 package 빌드는 이 rosdep 오류를 검출하지 못했다.

Ubuntu 22.04 arm64 Humble Distrobox와 기존 build cache에서 make test-ci-tools 9건,
make test-tof의 9 packages build와 colcon 247 tests가 errors/failures 0으로 통과했다.
기존 opt-in skip은 15건이며 ToF package 74 tests는 errors/failures/skips 0이다.
공식 기본 rosdep source와 저장소의 ros2_ws/rosdep/cleany.yaml을 별도 ROS_HOME에 준비해
전체 workspace의 rosdep install --simulate --ignore-src --rosdistro humble도 통과했다.
로컬 기본 rosdep source가 초기화되지 않아 발생한 메타데이터 누락은 이 격리 설정으로
해결했으며 전역 설정을 바꾸지 않았다. 실제 CI 이미지는 원격 workflow에서 재검증한다.

로컬 rosdep 로그와 재현 script는 CLEANY_DATA_ROOT/runs/ci-pr71-fix/에, package XML은
기존 builds/feat-tof-integration/build에 있다. 실제 센서나 모터를 사용하지 않았다.

## MoveIt 충돌 메시 최적화와 MuJoCo 동등성 검사 (2026-10-24)

`main`의 `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`에서 충돌 메시 최적화만
분리했다. 형상과 기본값은 Jetson에 적용한 로컬 `cffdddb`, 동등성 검사 수정은
`707538d`를 따른다. STL 17종과 manifest, 선택 옵션, 생성 도구와 검사를 포함한다.
MuJoCo 접촉용 MJCF, 관절, 시각 CAD와 MoveIt 자기 충돌 예외는 main을 유지한다.
cuRobo와 GPU 계산 및 미병합 physical launch의 변경은 포함하지 않는다.

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, Python 3.10.12와 MuJoCo 3.11.0의
별도 main 사본에서 description 패키지를 빌드하고 설치 자산 18개의 hash를
소스와 비교했다. `make test-description-models`에 description의 나머지 검사와
기존 main의 MoveIt 설정, 시뮬레이션 충돌 예외 및 장면 검사를 추가해
122개가 모두 통과했다. 이때 검사한 26개 core 파일은 분리 변경과 동일하다.
NumPy/SciPy 호환 범위와 deprecated API 경고 2개는 남았다.

공통 URDF 기본값 때문에 Gazebo navigation에도 최적화 메시 10개가 전달된다.
이 변경은 Jetson과 같은 기본값을 유지하며, 별도 검토한 Gazebo 원본 형상 고정은
포함하지 않는다. 앞선 122개 검사는 Gazebo 변환 모듈을 실행하지 않는다.
Gazebo 시뮬레이션 실행과 실물 모터 동작을 검증한 결과는 아니다.

원본은 Jetson의 `/home/cleany/cleany-data/reports/collision-main-review-20261024/`에
있다. `main.tar.gz`, `candidate.json`, `installation.json`, `build.log`와
`models-and-main-contracts.log/.xml`에 입력 main, core 파일 hash, 환경과 결과를
보존했다. 해당 검토의 `candidate.patch`에는 별도로 제안한 Gazebo 원본 고정이
포함되어 있으므로 이 PR의 패치와 구분한다. Jetson 운영 적용 기록은
`/home/cleany/cleany-data/reports/collision-deployment-20261024/`에 있다.

### PR 64 최신 메시와 SolidFCL 검증 (2026-10-29)

기준 commit은 `5db7b12a34d2ba85c965d7d783039a947d456afa`이며, 이 절과 함께
반영한 변경을 Jetson의 별도 workspace에서 검사했다. 최신 STL 17종의 삼각형
합계는 4,686개에서 3,966개로 줄었다. 닫힌 볼록 조각 내부의 장애물을 검출하는
`cleany/SolidFCL`을 독립 패키지로 분리하고, 요청한 detector의 로드 실패 시
계획 서비스를 종료하는 검사를 추가했다. 시간 배정, cuRobo와 GPU 실행 변경은
포함하지 않는다.

환경은 Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, MoveIt 2.5.9와 해당 버전의
Pilz 및 MoveGroup overlay다. `reviews/pr64-20261029/`의 분리된 build/install을
사용했으며 운영 install은 참조하지 않았다. 검증한 소스 42개와 설치 STL
17종의 SHA-256이 로컬 PR 작업트리와 일치한다. 소스 목록은
`pr-source-hashes.json`, 설치 확인은 `pr-installation.json`에 있다.

| 검사 | 결과 |
|---|---|
| `make build`에 `COLCON_BUILD_ARGS="--packages-up-to cleany_moveit_config"` 지정 | description, collision, scene mapping, MoveIt config 4개 패키지 빌드 성공 |
| `make test-description-models` | 모델 동등성과 메시 형상 85개 통과 |
| SolidFCL gtest | 내부 장애물, 분리된 틈, ACM, 거리, OctoMap, attached object 및 링크 경계 상자 검사 6개 통과 |
| MoveIt 설정, 시뮬레이션 충돌 예외와 장면 계약 | 18개 통과 |
| 기존 Mock 계획/실행과 종료 검사 | 3개 통과 |
| 설치된 MoveIt 서비스 | 3개 통과: 내부 장애물 36개 검출, 같은 입력의 표면 FCL 누락 재현, 잘못된 detector 요청 시 종료 |

표의 최종 결과는 중복 없는 검사 115개이며 CTest 래퍼는 합산하지 않는다.
서비스 입력은 원본 CAD에서 충돌했던 자세와 장애물 36건의 고정 fixture다.
장애물 없는 상태의 유효성과 접촉 링크도 확인한다. Mock 실행은 가상
ros2_control 하드웨어를 사용했으며 실물 드라이버나 모터 명령을 사용하지 않았다.

처음 세 패키지만 선택한 빌드는 기존 scene mapping의 미설치로 실패해
`--packages-up-to cleany_moveit_config`로 의존 패키지를 함께 빌드했다.
전체 `make test`는 내비게이션 의존성 `dwb_msgs`가 없어 빌드 단계에서 중단됐다.
서비스 검사의 환경변수를 이어 붙인 실행은 `ROS_LOCALHOST_ONLY=0:1`이 되어
2개가 통신 대기로 실패했다. CMake의 `ENV`로 값을 지정한 뒤 해당 3개를
재실행해 통과했다. NumPy/SciPy 경고 2개는 유지된다. 전체 workspace,
Gazebo 동적 실행과 새 성능 비교를 완료한 결과로 해석하지 않는다.

운영 Jetson도 별도로 재확인했다. 로컬 `707538d` 및 Jetson `4f92bc0` 이후의
미커밋 작업에서 대상 소스 32개와 설치 메시 17종이 일치했고, 운영 install의
MoveIt 서비스가 내부 장애물 36개를 모두 검출했다. 양팔 계획은 각 5회, 총
10회 성공했다. 운영 라이브러리를 교체하지 않았으며 trajectory execution을
끄고 수행했다. 이 결과와 PR의 독립 패키지 검증을 구분한다.

원본은 `/home/cleany/cleany-data/reports/pr64-update-20261029/`에 보관한다.
`native/hash-verification.json`, `native/services.json`과 MoveIt 로그가 운영 확인,
`pr-build-*.log`, `pr-models.xml`, `pr-tests/`, `pr-service-final.log`가 PR 검증
근거다. 실패 로그는 `pr-build.log`, `pr-package-tests.log`와
`pr-service-env-failure.log/.xml`에 남겼다. 로컬 사본은
`/Users/hyeon/workspace/cleany/output/pr64-update-20261029/`에 있다.

### PR 64 미사용 경로 정리 검증 (2026-10-30)

`a84daa7`에 이 절과 함께 반영한 정리 패치를 Jetson Ubuntu 22.04 ARM64,
Humble과 MoveIt 2.5.9의 별도 경로에서 검사했다. 미사용 표면 단순화와 전용
검사 3개를 제거했다. 내부 장애물 36건의 자세를 공통 3개로 묶고 원본 입력과
기대 결과가 모두 같음을 확인했다. STL 17종의 hash, 삼각형 3,966개와 조각별
형상 검증 수치는 유지했다. 설치된 메시와 manifest 18개도 소스와 일치한다.

4개 패키지 빌드, 형상 82개, 충돌과 계획 설정 및 Mock 검사 30개가 통과했다.
`make test-moveit-collision`과 기존 Mock 계획, 종료 CTest를 실행했으며 실패와
skip은 없다. NumPy/SciPy 경고 2개는 유지된다. 가상 장애물과 Mock 하드웨어를
사용했고 실물 모터, 전체 workspace 및 Gazebo 동적 검사는 실행하지 않았다.
소스 패치, 환경, 로그, JUnit과 자산 보존 근거는
`/home/cleany/cleany-data/reports/pr64-65-cleanup-20261029T233007Z/pr64/`에 보관한다.

CI 수정 중 main의 c8461d66 병합으로 생긴 Makefile과 TESTING/EVIDENCE의 append 충돌은
양쪽 target과 기록을 모두 보존해 해소했다. 병합 후 CI 도구 9건, ToF 순수 91건과
navigation colcon 198 reported tests가 통과했고 전체 rosdep simulate도 재통과했다.
remote CI는 이 통합 tree에서 다시 실행한다.

## 2026-11-02 책상 접근 ROS 검사의 수신 준비 동기화

최신 main 통합 head d82edca4의 CI run 41369304125는 의존성 이미지 생성과 환경 검증을
통과했지만 전체 2720 reported tests 중 부분 품질 책상 접근 검사 1건에서 goal 거부로
실패했다(32 skipped). 같은 제어 코드는 직전 run 41367607054에서 전체 통과했다.
기존 검사는 publisher sequence 증가만 기다려 서버의 실제 입력 수신과 권한 준비를
보장하지 않았다. 검사에서 서버가 수신한 frame/odom의 나이가 source_timeout_s의
1/4 이내이고 해당 Mission lease가 유효한지 확인한 뒤 요청하도록 바꿨다. 부분 품질
시나리오는 실제 수신 관측에 UNRELIABLE 12개가 있는지도 요구한다. goal 수락, 전진,
cancel과 정지 assertion은 유지했고 운용 코드와 freshness/lease 한계는 변경하지 않았다.

Humble Distrobox의 2 CPU affinity에서 ROS graph 11건과 make test-tof의 colcon
247 reported tests가 통과했다(errors/failures 0, 기존 opt-in skip 15). ToF package 자체는
74건, skip 0이다. 실제 로봇을 사용하지 않았다.

다른 ROS graph 검사 종료 후 두 시나리오를 2 CPU affinity에서 순차 20회씩, 총 40회
반복해 통과했다. 재현 script와 로그는 CLEANY_DATA_ROOT/runs/ci-pr71-fix의
repeat_ready.py와 readiness-repeat.log다.

## 2026-11-04 오프라인 매핑 입력 녹화 분리

코드 기준은 `15287287d4b236c71b519db989c958b233283503` 위의 미커밋 변경이다.
Jetson Orin의 Ubuntu 22.04, ROS 2 Humble, Python 3.10.12에서 실행했다.
변경 파일 hash와 원본 결과는 Fedora의
`~/.local/state/cleany/logger-fix-20261104/`와 Jetson의 같은 경로에 보존한다.

기존 Logger bag `106269c5-66b7-4e98-bafe-74359f443c76`의 스냅샷에는 raw scan
825개와 1초 초과 시각 간격 98회, 최장 약 14.11초가 있었다. Logger 상태에는
원본 크기 또는 저장 대기 한도 초과와 dropped 카운터가 남았다. Python 관측 처리의
구조 변경 실험은 충분한 연속 수신을 입증하지 못해 최종 변경에서 제외했다.

`tools/validation/record_mapping_bag.py`는 네이티브 rosbag2로 매핑 입력 6개만
별도 수집한다. SQLite 파일 조회 중 발생한 lock 종료도 확인해 `resilient` preset을
적용했다. Logger writer에도 같은 preset을 적용하고 읽기 트랜잭션을 유지한 채
쓰기 가능한지 ROS 테스트로 확인했다. 용량 조회는 같은 파일 범위를 유지하면서
순회 비용을 줄였고, writer는 가공 snapshot보다 원본 큐를 먼저 처리한다.

Jetson에서 `make build-logger`의 두 패키지 빌드와 Logger 57개, Monitor 80개,
원본 bag 도구 4개 테스트가 통과했다. Logger의 ROS opt-in과 Monitor 검사는
격리 domain 75에서 실행했다. Fedora의 ROS 없는 검사는 Logger 53개 통과,
ROS와 생성 메시지가 필요한 3개 skip이다. 도구의 시간 만료에 따른 SIGINT 종료와
metadata 마감도 별도 domain 76에서 실행했다.

실물 입력의 60초 native 검증 bag은
`~/.local/state/cleany/mapping-bags/native-validation-wal-20261104T191257`이다.
정상 종료 후 검사한 DB는 `journal_mode=wal`, `quick_check=ok`이며,
`native-wal-validation.json`에 토픽별 수와 시각 간격을 보존한다.
raw scan은 458개, 7.81 Hz, 최장 간격 0.149초였다. odom은 2,554개이며
1.379초 간격이 1회 남았다. 휠 입력에도 2.936초 간격과 sequence 115 증가가 있어
완전 무손실이나 전송 경로의 정상 동작까지 입증한 결과는 아니다.
`upstream-gap-observation.json`은 관측값이며 원인 확정이나 MCU 재부팅 증거가 아니다.

기존 SLAM, 조종 서버와 네트워크는 재시작하지 않았다. 실물 매핑은 같은
`generation=2`의 LiDAR+odom 실행을 유지했다. 기존 Logger 프로세스에는 변경을
hot reload하지 않았으며, 별도 native 수집 경로를 실행했다. 새 주행에 대한 지도
정확도와 파라미터 개선 검증은 아직 수행하지 않았다.

## 2026-11-04 주행 bag 재구성과 지도 정렬 점검

코드 기준은 로컬 `15287287d4b236c71b519db989c958b233283503`과 Jetson
`32d2d9e96e7c263ffa8ac6bc56b2b6a2ec6a5d84` 위의 미커밋 작업이다.
원본은 Jetson Ubuntu 22.04 / ROS 2 Humble에서 수집했고, 후반 재생은 Fedora의
기존 `ros2-humble` 컨테이너에서 격리 ROS domain을 사용했다. 이 비교를 위해
실물 ROS, SLAM, 조종 서버 또는 네트워크를 재시작하지 않았다.

입력 `native-user-drive-2045-20261104T204357`은 약 946.6초이며 raw scan 7,400개,
joint state 43,105개, odom 43,143개, wheel state 43,383개를 포함한다. DB SHA-256은
`c41eed98540521d06bf8adb208a41ce12c40f483726443f023d14d586814006d`다.
스캔 header 기준 최장 간격은 0.15265초이고 odom에는 1.329초 간격이 한 번 남았다.
사용자는 같은 출발 위치로 돌아왔다고 보고했으나 위치는 측량하지 않았고,
종료 방향은 같은 방향 또는 약 180도 반대로 불확실하다.

현재 실물은 63.5 mm 휠 반경, 350 mm 앞뒤 간격, 590 mm 좌우 유효 간격과
X 배치를 유지한다. 별도 원격 커밋 `be4b8e70`의 135 mm 유효 간격과 측방 부호
−1은 적용하지 않았다. 이번 입력의 독립 스캔 정합은 약 591–592 mm를 지지한다.
LiDAR 평면 TF는 CAD −Y를 전진으로 해석한 x +0.139275 m, y 0, yaw 180도다.
z=0은 평면 모델 관례이며 실제 광학 높이 측정값이 아니다.

| 오프라인 보정 | Wheel odom 복귀 차이 | LiDAR 참조 대비 전체 위치 RMSE |
|---|---:|---:|
| 원래 설정 | 2.121 m | 1.684 m |
| 이동 방향에 따른 yaw 보정 | 1.244 m | 1.065 m |
| 거리 배율과 방향 간섭 추가 | 0.948 m | 0.716 m |
| 속도와 방향 비대칭을 포함한 14계수 후보 | 0.088 m | 0.135 m |

마지막 행의 8.8 cm는 **동일 bag 안에서의 복귀 차이**다. 마지막 약 96초를
계수 피팅에서는 제외했지만 후보 선택에는 사용했고, LiDAR 참조도 전체 bag의
loop closure로 최적화했다. 해당 구간의 위치 RMSE는 약 0.127 m다. 720초까지만
학습한 같은 모델은 복귀 차이 1.189 m로 일반화에 실패했다. 따라서 이 수치를
독립 주행의 실물 정확도 또는 지도 정확도 10 cm 검증으로 사용하지 않는다.
14계수 후보와 후속 실험은 실물 설정에 반영하지 않았다.

사용자가 지적한 왼쪽 방의 가로 칸막이는 최종 지도에도 휘어짐과 겹침이 남았다.
정지에 가까운 원본 스캔의 부분 직선 피팅 잔차는 중앙값 약 0.8–0.9 cm였지만,
빔 시간 순서, 시야 범위, 지역 정합, 루프 제약, 서브맵 크기 및 SLAM Toolbox를
비교해도 모든 구역의 정렬을 해결하지 못했다. 시간 순서 반전은 일부 칸막이를
개선했으나 드라이버 오류나 단일 원인으로 확정하지 않았다.

Fedora의 데이터 루트는 `/home/ehdrms/workspace/cleany-data`다. 원본 bag과 수치 결과는
`runs/hardware-mapping-20261104/`, CAD와 설정은 `fixtures/`, 분석 스크립트는
`experiments/hardware-mapping-20261104/`, 요약과 비교 그림, Git 상태는
`reports/hardware-mapping-20261104/`에 분류했다. `summary.html`과 `summary.json`이
요약이다. 기존 압축본은 각 파일의 보존을 검증한 뒤 해체했고, 고유한 과거 버전은
`recovery/hardware-mapping-20261104/checkpoint-versions/`에 보존했다. 이동 전후 경로와
SHA-256 검증은 `migrations/storage-consolidation-20261104/`에 기록했다.
기존 실행 경로는 필요한 호환 링크로 유지하며 실제 데이터 사본은 추가하지 않는다.
후속으로 제안한 벽 앞 정지, 좌우 회전, 실측 1 m 직진·후진 및 0.5 m 횡이동 시험은
아직 수행하지 않았다. 단순 복귀 차이보다 실측 거리와 벽 정렬을 함께 확인한다.

## 2026-11-05 Logger 저장량 조정

- 기준 commit: `15287287d4b236c71b519db989c958b233283503`에 미커밋 Logger 변경 적용.
- 환경: Fedora ARM64 노트북의 `ros2-humble` Ubuntu 22.04 컨테이너.
- 입력: 세션 `d6073408-d4ba-4231-b5b0-dd1aee3fc132`의 원본 bag snapshot과 가공 DB,
  수신 시각 기준 667.869초. ROS 발행 없이 운영 RecordingControls와 Recorder로 재기록.
- 결과: 두 bag과 가공 DB 합계 179,780,667 bytes, 시간당 환산 969,067,417 bytes.
  `/scan_raw`, `/odom`, `/base/wheel_state`, `/tf`, `/tf_static`의 저장된 timestamp와
  CDR 해시, 메시지 수가 입력과 일치했다. 1시간 실물 주행 검증은 아니며 그래프 크기에
  따라 증가율이 달라질 수 있다. 기존 누락 구간은 복구하지 않았다.
- 검증: `CLEANY_RUN_ROS_TESTS=1 make test-logger` 59 passed,
  같은 옵션의 `make test-dev-monitor` 80 passed, `make build-logger` 2 packages 성공.
  Monitor의 기존 단일 bag 가정 검사 1건이 처음 실패해 두 bag의 토픽 배치와 CDR을
  검증하도록 갱신한 뒤 전체를 재실행했다. 호스트에는 pytest가 없어 컨테이너에서 실행했다.
- 원본 및 결과: `/home/ehdrms/workspace/cleany-data/reports/logger-budget-20261105/`
  (`result.json`, `cdr-integrity.json`, `previous-settings.json`, `applied.json`).
  재현 스크립트와 정책은 `/home/ehdrms/workspace/cleany-data/experiments/logger-budget-20261105/`.
  초기 메시지별 commit 실험은 중단했고 그 부분 결과는 보존했다. 최종 결과는
  `46d74ef2-d53a-48a0-a0aa-f9e5a509f6f4` 세션이다.
- 배포 경계: 실행 중이던 Logger의 configure 응답과 저장 파일에서 정책 적용을 확인했다.
  기존 용량 초과 ERROR를 정리하려고 짧은 새 세션
  `7491dd76-71b0-4f74-8589-6f7c7bbafc10`을 열었으나 제어 응답이 timeout했다.
  해당 Logger launch에 SIGINT를 보냈고 launch가 5초 후 SIGTERM으로 종료했다.
  이 복구용 세션은 완전한 기록으로 취급하지 않는다. 모터 STOP 확인은 유지했고
  하드웨어 및 네트워크를 재시작하지 않았다. 다음 주행 전 매핑 세션의 Logger
  소유 상태 복구가 필요하며 새 Logger 실행부터 분리 writer 코드가 적용된다.
- 후속 사용자 지시에 따라 분석용 정책을 별도로 저장했다. `trajectory_node_list`는
  1 Hz, `constraint_list`는 0.5 Hz이며 map, submap 목록, matched points와 가공
  관측 주기는 조정 전 정책으로 복원했다. 중복 3개 기록 제외는 유지한다.
  Logger가 실행되지 않는 상태에서 설정 검증 후 원자적으로 저장하고 다시 읽어
  일치를 확인했다. 다음 Logger 기동부터 사용하며 새 주행은 시작하지 않았다.
  이 정책은 위 0.97 GB/h 측정 정책과 다르며 용량 재측정은 수행하지 않았다.
  근거는 같은 보고서 폴더의 `analysis-rates-applied.json`과 실험 폴더의
  `analysis-recording-settings.json`이다.
- 사용자 승인으로 매핑 런타임을 재시작했다. 첫 재기동에서 `MappingBackend.start`가
  저장된 Hz와 토픽 제외 설정을 무조건 덮어쓰는 것을 실제 Logger 응답으로 확인했다.
  저장 정책이 있을 때 유지하고 첫 실행만 기본값을 설정하도록 수정했다.
  `test_mapping_core.py` 12 passed, `make build-navigation` 8 packages 성공.
- 최종 세션 `hardware-slam-20261105T180408-b17c5063`, Logger 세션
  `a41738d8-5ca8-424b-ae73-ec83a1533679`에서 running/RECORDING 확인.
  궤적 1 Hz, 제약 0.5 Hz 및 중복 3개 제외를 서비스 응답과 실제 두 bag에서 확인했다.
  약 12초 관찰에서 scan 7.68 Hz, odom과 wheel state 44.74 Hz, map 0.50 Hz.
  phone ready=true, paused=false, 모터 disabled/PWM 0/fault 0 확인. 주행 명령은 보내지 않았다.
  이전 지도와 기록은 보존했다. 근거는 logger-budget-20261105 보고서 폴더의
  `restart-live-policy.json`, `restart-status.json`, `restart-bag-topics.json`과
  laptop-hardware-20261105의 `live-inputs.json`이다.

## 2026-11-05 동일 바닥의 짧은 슬립 재현 시험

- 사용자 명시 승인으로 기존 phone controller HTTP 경로를 사용했다. 별도 모터 명령
  publisher를 만들지 않았다. 원본 bag 수집을 먼저 시작하고 fresh scan, odom,
  wheel feedback, 단일 명령 소유자와 모터 정지를 확인했다. 지도 저장 오류로 남아 있던
  maintenance pause는 시험 중 서비스로 해제하고 종료 후 원래 paused 상태로 복원했다.
- 차체 범위 밖 관측 장애물의 최소 거리는 약 0.90m였고 시험 중 0.65m 미만,
  센서 stale, MCU fault/reboot, odom 이동 0.18m 초과 시 중단하도록 제한했다.
  기존 lease, permission 및 MCU watchdog 경로를 유지했다.
- 왼쪽 2 rad/s, moving 상태 약 0.8초: odom 왼쪽 6.47cm, 독립 LiDAR 매칭
  왼쪽 5.78~5.89cm, 후방 0.48~0.97cm. 좌전방 대각 2.5 rad/s 약 0.8초:
  odom 전진 3.94cm/왼쪽 3.95cm, LiDAR 전진 3.30~3.65cm/왼쪽 3.61~3.86cm.
  각 시험의 초기/종료 정지 스캔 3쌍과 전방-only 매칭 모두 valid.
- 두 번 모두 STOP acknowledgement, disabled, PWM 전부 0 확인. 두 짧은 시험에서는
  이전 bag의 큰 후방 슬립이 재현되지 않았다. 장시간 횡이동이나 다른 바닥 위치의
  재현을 검증한 것은 아니다. 기구 및 보정 설정은 변경하지 않았다.
- 원본과 결과: `/home/ehdrms/workspace/cleany-data/reports/slip-repro-20261105/`
  아래 `184617-left`, `184707-diagonal`의 bag, result.json, analysis.json.
  실행/분석 스크립트는 cleany-data/experiments/slip-repro-20261105에 보존했다.

## 2026-11-05 2.5 rad/s 방향별 3초 반복 실물 시험

- 사용자 명시 지시: 좌/우 횡이동과 좌전/우후/우전/좌후 대각 4방향을 각각 3회.
  기존 phone controller로 반대 방향을 번갈아 총 18회 완료했다. 최대 활성 휠
  명령 2.5 rad/s 유지 구간은 피드백 수신 시각 기준 3.008~3.090초였다.
  대각선에서는 기존 mecanum 변환대로 두 바퀴가 2.5 rad/s, 다른 두 바퀴는 0이다.
- 17번째 시도의 사전 검사에서 scan stale로 이동 전에 중단했다. 5초 fresh scan
  재검사(38 samples, 최대 간격 0.138초) 후 미완료 방향만 재시도했다.
  따라서 저장 시도는 19개, 완료된 이동은 방향별 정확히 3회씩 18개다.
- 매회 센서 기록을 먼저 시작하고 기존 lease/permission/watchdog 경로를 유지했다.
  센서 stale, MCU fault/reboot, 차체 밖 거리 0.65m 미만, trial odom 이동 0.65m
  초과 시 중단했다. 모든 완료 시험에서 STOP 확인, disabled 및 PWM 전부 0.
  시험 종료 후 기존 maintenance pause 상태를 복원했다.
- 독립 LiDAR 국소 ICP를 합성해 odom과 비교했다. 긴 간격 endpoint ICP 중 일부는
  overlap 부족으로 실패하여 최종 수치에 사용하지 않았다. 수렴 실패 국소 구간은
  더 짧은 스캔 간격으로 나눠 같은 검증 기준으로 통과한 경우만 사용했다.
  18회 모두 최종 full-scan 국소 매칭 유효. front-only 독립 비교 차이는 0.2~4.5cm라
  cm 수준 수치를 절대 ground truth로 취급하지 않는다.
- 왼쪽 횡이동 세 번에서 LiDAR 후방 편향 3.3~5.0cm, 왼쪽 이동 47.4~48.4cm.
  오른쪽에서는 전방 1.8~2.7cm, 오른쪽 47.8~49.1cm. 이전 주행 bag의 큰 후진 슬립은
  이번 반복에서 재현되지 않았다. 접지 손실과 좌표계/휠 모델 편향의 원인은 미확정.
- 원본 bag, HTTP 상태/명령, trial analysis 및 summary.json:
  `/home/ehdrms/workspace/cleany-data/reports/slip-repro-repeat-20261105/`.
  실행/분석 스크립트: cleany-data/experiments/slip-repro-20261105의
  repeat_pulse.py, analyze_repeat.py. 기구나 보정값은 변경하지 않았다.

## 2026-11-05 사용자 수동 슬립 재현 기록

- 사용자 재현 완료 지시에 따라 기존 phone HTTP STOP을 보내고 MCU STOP 확인,
  disabled 및 PWM 전부 0을 확인했다. 별도 native rosbag recorder만 SIGINT 종료했다.
- `runs/manual-slip-20261105-live`에 56,660,097 bytes 기록, metadata 존재,
  SQLite quick_check=ok. 사용자 이벤트 수신 표시는 19:09:49 KST이며 정확한
  물리적 시작 시각으로 간주하지 않고 이전 45초부터 분석했다.
- 61개 짧은 명령 구간 모두 zero-seed LiDAR ICP 및 reverse-cycle 검사 통과,
  60개는 front-only 비교도 일치. 19:09:27에는 odom 전진 9.1/왼쪽 11.2cm/s,
  LiDAR 후진 1.4/왼쪽 0.6cm/s. 19:10:04에는 odom 후진 10.1/오른쪽 10.3cm/s이나
  LiDAR 이동은 약 0.1cm/s였다. 여러 초 연속으로 큰 차체/휠 이동 불일치가 관측됐다.
  해당 구간 휠 피드백은 명령을 대체로 추종하며 PWM 최대 31%였다.
- 순수 2.5 rad/s 자동 시험과 달리 수동 재현은 활성 휠 약 3.2 rad/s와 작은 반대쪽
  휠 명령이 포함되어 조건이 다르다. 마찰, 하중, 롤러 또는 접촉 장애물 원인을
  이 자료만으로 확정하지 않는다. 보정이나 기구 설정은 바꾸지 않았다.
- 분석 및 marker: `reports/manual-slip-20261105-live/summary.json`, `matches.json`,
  `events.jsonl`. 스크립트는 experiments/slip-repro-20261105의 extract_manual.py,
  compare_manual.py다. 이 재현 구간은 bag에 보존됐으며 이전 1GB 한도와 별개다.

## 2026-11-05 중단된 주행 bag의 odom 보정 및 SLAM 비교

- 기준 commit `15287287d4b236c71b519db989c958b233283503`, 기존 미커밋 변경 유지.
  Fedora ARM64의 ros2-humble 컨테이너, 별도 ROS domain 174~179에서 offline node 사용.
- 입력: Logger `a41738d8-5ca8-424b-ae73-ec83a1533679`의 18:04:10~18:24:51 기록.
  scan_raw 9,449건에서 기존 전방 180도/0.30 m 필터로 scan 재생성, odom 55,980건.
  온라인 map->odom TF는 제외하고 base->lidar 장착 TF와 odom->base_link를 사용했다.
  원래 주행의 마지막 복귀 구간이 없으므로 종점 일치 조건으로 보정하지 않았다.
- 독립 초기값 두 개가 일치하는 ICP 상대 이동 674쌍 중 348쌍으로 보정하고
  분리된 30초 블록의 326쌍에서 평가했다. 원본 odom의 상대 이동 RMSE는
  2.098 cm/0.191도, 반경·yaw 보정은 2.091 cm/0.190도,
  방향 편향 모델은 2.028 cm/0.184도였다. 동일 LiDAR에 대한 일관성 지표이며
  실측 ground truth나 전체 주행 위치 정확도가 아니다.
- 기존 odom+LiDAR, LiDAR-only, yaw 보정, 반경+yaw 보정, 방향 편향 보정의
  5개 지도와 궤적을 생성했다. SLAM 결과의 검증 상대 이동 RMSE는 각각
  1.498/3.085/1.479/1.485/1.478 cm, yaw는 0.238/0.526/0.227/0.218/0.216도.
  기존 조건 재실행도 1.498 cm/0.238도로 재현됐다. 방향 편향 후보와 기존의
  차이는 블록 bootstrap 95% 구간에 0을 포함해 뚜렷한 개선으로 판정하지 않았다.
- 칸막이 확대 비교에서 보정의 효과는 혼재하며 5 cm 지도 격자 이하의 차이가
  대부분이다. 통로의 겹침은 남았다. 실물 odom 설정은 변경하지 않았다.
- 후보 bag은 scan CDR와 timestamp 해시 동일, SQLite integrity 확인.
  원본 odom 복원 항등성 확인. 대응 odom이 기록되지 않은 TF 3건은 보간 보정량에
  원래 TF 잔차를 유지했다(최대 49.24 micrometres). 초기 검증에서 이 보간 잔차를
  발견했고 해당 입력 생성 시도도 별도 폴더에 보존했다.
- 원본·스크립트·pbstream: `/home/ehdrms/workspace/cleany-data/experiments/offline-slam-20261105-180408/`.
  비교 그림·지도·지표·SHA-256: `/home/ehdrms/workspace/cleany-data/reports/offline-slam-20261105-180408/`.

## 2026-11-05 리모컨 임시 전후진 전용 모드

- 사용자 요청으로 token 디렉터리의 `teleop-motion.json`에 forward_only=true 저장.
  조종 서버만 graceful 재시작하여 적용했고 센서, 네트워크와 SLAM은 재시작하지 않았다.
- UI는 횡이동과 회전 조작을 숨기고 포인터 및 키보드 입력을 전후 축으로 제한한다.
  서버는 별도로 횡/대각/회전 명령을 거부한다. 기본 모드는 종전과 같고 로컬 정책을
  false로 바꾸고 서버 재시작하면 복원한다. 기존 권한과 watchdog은 유지했다.
- Python teleop 검사 49 passed, Node 입력 검사 17 passed, UI 타입 검사/빌드 및
  `make build-hardware-mapping` 24 packages 성공. 실제 서버 상태 forward_only=true,
  ready=true, paused=false 확인. 금지된 세 입력은 HTTP 409로 거부됐고 모터는 계속
  disabled/PWM 0이었다. 새 JS/CSS HTTP 200 확인. 실물 이동 명령은 보내지 않았다.
- 실제 상태 근거: cleany-data/reports/laptop-hardware-20261105/forward-only-status.json.

## 2026-11-05 노트북 360도 매핑 주행 분석

입력은 22:15:26–22:44:26 KST의 Logger 세션
`ea32b1f9-80b0-477e-ac12-717f8c4f6d23`이며, Fedora ARM64 노트북의
Ubuntu 22.04 / ROS 2 Humble 컨테이너에서 읽기 전용 분석을 수행했다.
코드는 `15287287d4b236c71b519db989c958b233283503`과 보고서에 보존한 작업 트리 변경이다.
보고서는 `/home/ehdrms/workspace/cleany-data/reports/slam-360-20261105/report.html`,
분석 스크립트는 같은 데이터 루트의 `experiments/slam-360-20261105/`에 있다.
원본 bag 경로와 SHA-256은 보고서의 `storage.json`에 기록했다.

257개 이동 창 중 237개가 양방향 스캔 정합 검증을 통과했고, 해당 창의 위치 차이
중앙값은 4.61mm였다. 전체 복귀 스캔 정합은 일치점 비율 부족으로 기각했으며
절대 정확도 0.1m 또는 기존 180도 대비 개선을 입증한 결과가 아니다.
DB 무결성과 녹화 종료는 확인했지만 `raw_gap`에 저장 대기 또는 메시지 크기 한도
초과에 따른 누락이 있어 무손실 기록으로 해석하지 않는다.

## 2026-11-06 휴대폰 경로 요청과 자동주행 모니터 복구

Fedora ARM64의 Humble 컨테이너, `4eaf919145e03ffe248732c950dab1267ded17d2`
기반 `feat/phone-nav2-demo` 미커밋 변경에서 확인했다. HTTP LAN 주소에서는
`crypto.randomUUID`가 없어 목적지 선택 후 요청이 전송되지 않았다. 요청 ID를
`getRandomValues`로 생성한 뒤 같은 비보안 컨텍스트에서 preview 전송, 파란 경로 표시,
이동 goal 미전송을 검증했다. 원본은 `cleany-data/reports/phone-path-preview-lan-20261106/`에 있다.

AMCL 공분산의 `-5.7e-16` 수준 잔차로 발생한 위치 추정 거부도 재현했다.
`-1e-12` 이하 허용 범위와 실제 음수·상한 초과·non-finite 거부를 검사했다.
Supervisor 51개와 Teleop 69개 검사가 각각 통과했고 두 패키지 빌드도 완료했다.
합동 실행은 동명 `test_core` 및 상대 test helper의 수집 충돌로 실행하지 못해
패키지별 별도 프로세스로 검사했다.

정지 확인 후 로거를 종료하고 실행 묶음을 재시작했다. 이전 위치 추정값을 초기값으로
사용한 뒤 실물 피드백, 활성 fault 0개, navigation ready와 PWM 0을 확인했다.
근접 목표에 대한 읽기 전용 계획은 약 1.34초 후 경로 없음으로 끝났으며,
실물 자율 이동 성공을 검증한 결과는 아니다. RViz 모니터 설정은
`cleany-data/fixtures/phone-nav2-demo/navigation.rviz`에 저장했다.

같은 실행에서 사용자 요청에 따라 자동주행 휠 상한을 2rad/s, 장애물 감속비율을
0.25로 변경했다. 직진·회전 혼합 명령의 휠 상한과 작은 명령 비증폭을 포함한
Teleop/phone profile 검사 83개와 관련 3패키지 빌드가 통과했다. 실행 중 MPPI
속도 제한, Collision Monitor 감속비율과 phone_remote의 휠 제한 파라미터를
다시 조회했다. 근거는 `cleany-data/reports/phone-nav2-implementation-20261105/wheel-limits-20261106.json`이다.
적용 후 정지 확인과 navigation ready를 확인했으며 속도 측정을 위한 추가 실물 주행은 하지 않았다.

## 2026-11-06 제자리 회전 구간의 MPPI 정체 수정

같은 미커밋 작업 트리에서 세션 `14d7fbbe-8b60-4e3b-936a-91ae3d618e52`의
ROS 시각 1793892475 부근을 분석했다. Lattice 경로의 선두 8개 점은 동일 XY에서
yaw만 바뀌었다. 기존 critic trace에서는 PathAngle 비용이 0이고 Twirling 비용이
회전을 억제했다. 제자리 회전의 종료 yaw를 PathAngleCritic 평가에 사용하도록
선택 기능을 추가하고 longitudinal phone 프로필에 적용했다.

원본 bag에서 추출한 정지 pose·경로·local costmap으로 240회 optimizer 호출을
비교했다. 기존 body yaw 명령 평균은 0.00180rad/s, 수정 후 0.27021rad/s였다.
Y축을 반사한 합성 반대 회전에서도 -0.27021rad/s였고 검사한 최적화 궤적에서
lethal footprint 충돌은 없었다. OccupancyGrid의 중간 비용은 역변환한 근삿값이며,
고정 pose 재생은 실물 주행 또는 폐루프 완주 증거가 아니다.

원본과 비교 설정, critic trace, snapshot은
`cleany-data/reports/nav2-turn-recovery-20261106/`에 보존했다. Navigation CTest
22개 그룹이 통과했고 새 회전 목표, 각도 wrap, 일반 이동 경로 회귀를 포함한다.
정지 확인 후 실행 묶음을 재시작하고 마지막 위치 추정값을 복원했다. 실행 파라미터에서
회전 추종 기능, 출력 궤적 충돌 검증, 휠 2/0.5rad/s 제한의 설정을 확인했다.
추가 실물 이동 명령은 보내지 않았다.

후속 실물 요청에서는 과도한 회전이 관찰되어 정지했다. 사용자는 RViz 방향과 실제
정면이 일치한다고 확인했다. 두 번째 목표의 기록된 global costmap에서는 동일 footprint로
검사한 직선 연결이 가능했지만 기본 Lattice 경로에는 반복 회전이 있었다.
longitudinal 프로필을 기존 StraightFirstPlanner로 전환하고, 제자리 회전의 후보·출력
XY를 함께 0으로 제한했다. 위치 도달 전 최종 yaw 강제를 피하도록 GoalAngleCritic의
범위를 3cm로 한정했다. 일반 이동의 Twirling 벌점은 다시 유지했다.

`closed_loop.cpp`로 기록한 지도와 이상적인 속도 응답, 고정 25% 감속을 사용한
폐루프 모사를 추가했다. 50cm 직진은 약 19.2초, 약 36도 회전 후 50cm 이동은
약 30.45초에 3cm/0.1rad 조건을 만족했고 검사한 경로에서 충돌은 없었다.
이는 실제 모터·마찰·통신 지연 검증이 아니다. 원본 설정과 실패한 중간 실험도
`cleany-data/reports/nav2-turn-recovery-20261106/`에 함께 보존했다.
최종 소스로 Navigation CTest 22개 그룹이 통과했다. 재시작 후 실시간 전방 50cm
계획은 25개 경로점, 최대 방향 변화 약 0.0075rad로 생성됐고 모터는 disabled/PWM 0이었다.

## 2026-11-06 주황 영역 16cm와 절대 휠 속도 제한

사용자 요청으로 주황 영역을 바퀴 외곽 +16cm, 장애물 구간 휠 상한을 0.7rad/s로
변경했다. 기존 Nav2 CollisionMonitor를 상속하는 `phone_collision_monitor`가
같은 lifecycle, polygon, 센서와 STOP 처리를 사용하며, 선택한 polygon의 감속
비율만 입력별 절대 휠 상한으로 계산한다. 작은 명령은 다시 축소하지 않는다.

격리 ROS domain 191의 실제 노드 검사에서 빈 공간, 0.4rad/s 유지, 큰 명령 제한,
직진/횡이동/회전 혼합의 0.7rad/s 제한, STOP 우선, 비활성 상태 무출력을 확인했다.
Navigation CTest 23개 그룹과 관련 package build가 통과했다. 잘못 선택한 domain
234는 DDS 포트 범위를 초과해 최초 검사를 시작하지 못했고 유효한 격리 domain에서 다시 실행했다.

정지 확인과 위치 보존 후 실행 묶음을 재시작했다. 실제 노드의 휠 상한 0.7,
주황 영역 반경 XY 0.3985/0.4874, 빨간 영역 0.2885/0.3774를 조회했다.
원본은 `cleany-data/reports/nav2-turn-recovery-20261106/absolute-limit-applied.json`이다.
추가 실물 주행은 수행하지 않았다.

## 2026-11-06 자동 목표의 화면 의존과 격자 연결 회전 수정

환경은 Fedora aarch64 노트북과 Humble Distrobox이며, 소스는
`feat/phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2` 위 미커밋 변경이다.
입력은 `cleany-data/runs/phone-nav2-demo/runtime/recordings/bags/8d31dbaa-b40b-4d89-957a-5d770263f733/`의
실물 기록이다. 마지막 취소 시 사용자가 휴대폰 화면 잠금 또는 앱 전환을 확인했다.
수락된 자동 목표에서 browser heartbeat 만료를 제거하고 로봇 쪽 Supervisor,
odom 0.3초, scan과 pose 0.5초 감시는 유지했다. 센서 취소는 입력별 나이를 기록한다.

주행 구간 388개 scan 중 73개에서 정지 기준 이상의 반사점이 검출됐고,
모두 차체 내부 x=-0.172..-0.166m, y=0.186..0.196m에 모였다.
사용자가 케이블 또는 부품을 확인한 뒤 선을 고정했다. 고정 후 정지 상태의
47개 scan에서 정지 영역 안 점은 0개였다. 제안했던 케이블용 마스크는 적용하지 않았다.
이 관찰은 주행 중 케이블 안정성까지 검증한 것은 아니다.

격자 경로 시작에 약 2cm 연결과 반대 회전이 있었다. longitudinal 프로필에만
앞쪽 1m의 직선 연결을 전체 footprint와 최신 costmap으로 검사하도록 했다.
기록한 local costmap의 시작 경로에서 전체 yaw 변화량은 1.154rad에서 0.701rad로
줄었고 원본과 수정 경로 모두 연속 sweep 검사를 통과했다. 장애물 관통 방지,
후방 연결 보존, 남은 경로 보존과 취소 검사를 추가했다.

고정 pose MPPI 재생에서 연한 장애물 비용을 제거해도 회전 구간의 평균 명령은
약 0.2235rad/s로 같았다. 이동 구간에서는 평균 전진이 0.1117에서 0.1141m/s,
yaw가 -0.1673에서 -0.1326rad/s로 달라졌다. 중간 비용은 OccupancyGrid 역변환 근사이며
이 두 snapshot만으로 전체 와리가리 원인이나 실물 개선을 확정할 수 없다.
비용 가중치와 inflation 설정은 유지했다.

Teleop pytest 83개, Navigation CTest 23개 그룹과 관련 package build가 통과했다.
자동 목표의 실제 ROS action 검사는 합성 제어/허가 포트에서 browser heartbeat 없이
수행했다. 정지 ACK, disabled와 PWM 0 확인 후 로그를 저장하고 실행 묶음을 재시작했으며
마지막 pose를 초기 추정값으로 복원했다. 실제 파라미터의 shortcut 1m와 휠 상한
0.7rad/s를 조회했다. 이전 목표까지 경로 미리보기는 259개 점으로 생성됐고,
Supervisor readiness와 정지 상태를 확인했다. 새 실물 이동 명령은 보내지 않았다.
원본 분석과 적용 상태는 `cleany-data/reports/nav2-turn-recovery-20261106/`,
경로 및 비용 재생 비교는 `cleany-data/reports/nav2-wiggle-{seed,translate}-20261106/`에 있다.

## 2026-11-06 휴대폰 holonomic 주행 전환

환경은 Fedora aarch64와 Humble Distrobox이며 소스는 `feat/phone-nav2-demo`의
`4eaf919145e03ffe248732c950dab1267ded17d2` 위 미커밋 변경이다.
기존 Omni MPPI에 실물 geometry와 휠당 2rad/s 후보 투영, 출력 충돌 검증을 적용했다.
`PathAngleCritic.rotation_only`는 명시된 제자리 회전만 평가하며 일반 횡이동을
바라보도록 강제하지 않는다. 경로의 몸체 방향과 최종 목표 자세는 별도로 평가한다.

실행 중 기본 Omni Lattice의 옆 35cm 목표 계획에 요청 방향 대비 최대 2.291rad의
회전이 포함됐다. 기존 `StraightFirstPlanner`의 holonomic 직선 연결과 고정 방향
탐색을 사용하고, 막힌 경우 몸체 방향 탐색과 Omni Lattice로 전환하도록 구성했다.
연속 footprint 검사, 최신 costmap 재검사와 최종 목표 연결은 유지한다.
longitudinal 프로필과 기본 launch 선택은 보존했다.

Navigation CTest 23개 그룹, Teleop pytest 91개, 최종 프로필과 lattice 경로 해석
pytest 40개와 관련 package build가 통과했다. 이상적인 운동 응답과 빈 costmap의
폐루프 모사에서 전후, 좌우, 네 대각선과 최종 방향 정렬, 0.7rad/s 절대 상한의
좌측/대각선 이동까지 11개 사례가 도착 조건 3cm/0.1rad를 만족했다.
동일 방향의 옆 50cm 이동에서 최대 방향 편차는 0.00801rad였다.
최초 감속 모사는 모든 속도를 35%로 줄여 두 사례가 40초 안에 도착하지 못했다.
실제 monitor와 같은 절대 상한으로 모사를 수정했으며 시간 제한은 유지했다.
이는 물리 슬립이나 실물 완주 검증이 아니다.

전환 중 wrapper의 비어 있는 Lattice 경로를 발견해 기본 파일 해석을 수정하고
명시적 파일 override 보존 검사를 추가했다. 실패한 configure 이후 부분 lifecycle
재시작은 global costmap executor 중복으로 실패해 소유된 실행 묶음을 정상 종료 후
다시 시작했다. 기존 MCU 재부팅 latch는 fresh stable health 검증을 거치는 정식
operator reset으로 해제했다. 정지 ACK, disabled와 PWM 0을 확인하고 위치를 복원했다.

AMCL 초기화 직후 동일 측면 목표의 탐색은 한 번 실패했다. 위치 추정 안정화 후
경로 미리보기는 17개 점으로 생성됐고, 실제 bag의 `/plan`에서 목표 위치 오차 0,
목표 방향 대비 최대 차이 0.00734rad를 확인했다. 초기 native 계획과 시작 위치
추정값이 달라 planner만 바꾼 정량 A/B 비교로 해석하지 않는다.
실행 파라미터에서 Omni MPPI, Omni AMCL과 active lifecycle을 확인했으며,
상태는 `forward_only=false`, 주행 준비와 MCU 정지였다. 새 실물 이동 명령은 보내지 않았다.

입력 지도와 목표, 설정 hash, 실패 사례와 모사 결과는
`cleany-data/reports/phone-holonomic-20261106/`에 보존했다. 실제 `/plan`은
`cleany-data/runs/phone-nav2-demo/runtime/recordings/bags/0ca11a12-1916-4852-aa5d-3b4b5fff0b8b/`에 있다.

## 2026-11-06 휴대폰 자동주행 휠 상한 4/1.5rad/s

사용자 요청으로 두 시연 프로필의 빈 공간 상한과 자동 intent 상한을 4rad/s,
주황 영역의 절대 상한을 1.5rad/s로 변경했다. 정지 5cm, 감속 16cm와 횡이동
선택은 유지했다. 관련 프로필과 실제 Collision Monitor의 격리 검사 8개 및
Navigation/Teleop package build가 통과했다. 정지 ACK 확인 후 로그와 위치를
보존해 Omni 실행 묶음을 재시작했다. 실제 `ros2 param get`에서 MPPI와
phone_remote 상한 4.0, collision_monitor 상한 1.5를 조회했다. API는 주행 준비,
disabled와 PWM 0이었다. 새 속도의 실물 이동 시험은 수행하지 않았다.
환경과 commit 기준은 앞선 holonomic 전환과 같으며, 적용 상태는
`cleany-data/reports/phone-holonomic-20261106/speed-caps-status.json`에 보존했다.


## 2026-11-06 Phone Omni 방향 선택과 주행 연속성

기준 commit은 `4eaf919145e03ffe248732c950dab1267ded17d2`이며 `feat/phone-nav2-demo` 작업 트리의 기존 변경 위에서 검증했다. 환경은 Fedora aarch64 노트북의 `ros2-humble` Distrobox다. 실물 주행 입력은 기존 bag `526a70a7-e9fd-4681-91c2-8d9d59efda7a`를 읽었으며 새 이동 목표는 보내지 않았다.

`holonomic_forward_distance=0.8`, 이동 구간의 몸체 방향 평가, MPPI 후보와 출력의 가속도 제한, 경로 수직 속도 비용과 장애물 사전 감속을 적용했다. 휠 상한 4/1.5rad/s, wheel+5cm 정지 영역, wheel+16cm 감속 영역과 기존 Supervisor 정지를 유지했다. 빌드는 `make build CLEANY_BUILD_ROOT=/home/ehdrms/workspace/cleany-data/builds/phone-nav2-demo COLCON_BUILD_ARGS='--packages-select cleany_navigation cleany_teleop'`로 실행했다. Navigation CTest 23개 그룹과 teleop navigation/ROS/web control 검사 48개가 통과했다. 추가 proximity 검사와 최종 프로필 검사를 별도로 통과했다. 실제 Collision Monitor의 격리 합성 scan 검사도 Navigation 검사에 포함됐다.

같은 지도와 목표를 사용하는 이상적 50ms 폐루프 네 과제는 모두 위치 3cm와 방향 0.1rad에 도달했고 충돌 위치는 0이었다. 좌우 0.5m 이동의 평균 불필요한 전후 명령은 각각 약 71%, 58% 감소했으며 시간은 5.60/5.65초에서 6.95/6.80초로 늘었다. 회전 전환을 포함하는 두 과제의 최대 휠 명령 가속도는 약 23rad/s²에서 3rad/s² 수준으로 줄었다. CSV 반올림에는 약 0.0001rad/s² 차이가 있다. 실제 마찰, 슬립, AMCL, USB 지연을 재현한 결과가 아니다.

첫 조정은 횡이동을 약 14초로 느리게 만들어 채택하지 않았다. 가속도 한계를 유지하고 temperature 0.05, 경로 수직 속도 비용 20, 명령 변화 비용 1, 도착 접근 시간 0.5초로 재조정했다. 비교 도구의 CMake 의존 타깃과 라이브러리 경로 누락도 수정했다. 기존 bag의 재탐색 실패와 짧은 정지 전체를 해결했다는 증거는 없으며 실제 바닥 검증이 남아 있다.

정지 응답과 구동 해제를 확인하고 소유된 시연 런타임을 재시작했다. 초기 위치 복원 후 준비 상태, 새 live parameter, fault=0과 정지 대기를 확인했다. HTML은 headless Chrome으로 desktop/mobile overflow와 내부 링크를 검사했고 그래프를 HTML에 포함했다. 원본, 첫 조정 실패, 최종 비교 CSV, 설정과 소스 SHA-256은 `/home/ehdrms/workspace/cleany-data/reports/phone-motion-review-20261106/`에, 실행 도구는 `/home/ehdrms/workspace/cleany-data/experiments/phone-motion-review-20261106/`에 보존했다. 보고서 진입점은 `report.html`이다.

### 2026-11-06 병행 지도 확장 준비

- 환경: Fedora aarch64 노트북, Distrobox ROS 2 Humble, `phone-nav2-demo` 작업 트리 HEAD `4eaf919145e03ffe248732c950dab1267ded17d2`의 미커밋 변경 포함.
- 입력: `cleany-data/runs/laptop-hardware-20261105/runtime/maps/hardware-slam-20261105T221524-ff9af942/latest.pbstream`, SHA-256 `a501ebe89e60d7930f19ea3d3fb875376bec366728287e78e2be7ddaca5e8b59`.
- 실물 센서로 고정 궤적 로드, 새 궤적 생성, 지도와 추정 자세 수신, 주기 상태 저장 및 종료 시 `final.pbstream` 저장 확인. 원점 변환/PGM 저장 검사 2개 통과. 기존 Nav2와 조종 서버 프로세스 유지.
- 최초 준비 시 gflags boolean 인수를 분리해 전달해 자동 궤적이 시작된 실패는 `=false` 형식으로 수정. 실패 실행은 `cleany-data/runs/map-extension/20261106T075439`에 보존.
- 정상 준비 및 종료 결과: `cleany-data/runs/map-extension/20261106T075503`. 개발공간 실주행 및 확장 지도 품질 검증은 아직 수행하지 않음.
- 종료/재개 검증 후 전역 `/tf` 절대 remap도 추가해 Cartographer를 `/map_extension/tf`로 완전히 분리했다. 현재 실행은 `cleany-data/runs/map-extension/20261106T075650`이며 이전 검증 실행의 `final.pbstream`을 이어받는다.

## 2026-11-06 — Phone Omni 순정 Lattice와 MPPI 튜닝

- 기준: `phone-nav2-demo`, `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 작업 트리. Fedora aarch64 / ros2-humble Distrobox / Nav2 1.1.20. commit, push 또는 실물 주행 성공을 뜻하지 않는다.
- 입력: 기존 144개 primitive, 이전 지도 fixture와 현재 `/global_costmap/get_costmap`의 538×546, 5cm costmap snapshot. `tools/validation/native_lattice_mppi`에서 domain 188, 20Hz 이상적 적분과 1Hz native 재계획을 사용했다.
- 변경: Phone Omni 전역 plugin을 순정 Smac Lattice로 전환하고 오른쪽 횡이동 16개의 길이 metadata를 교정했다. MPPI 강제 회전 단계를 끄고 비용 및 3초 예측을 조정했다. 웹 목표는 미리보기 반환 자세를 표시·전송하며 3cm/0.1rad 도착 검사와 4/1.5rad/s 안전 상한은 유지했다.
- 결과: 프로필/런타임 pytest 41개, Navigation CTest 23그룹, 조종 관련 pytest 54개, TypeScript/Vite 빌드 통과. 최종 8개 적분 사례에서 표시 목표 3cm/0.1rad 이내 도착, 적분 footprint 충돌 0회. 이 중 전방 0.7m 사례는 클릭 자세와 양자화된 목표의 충돌 여부가 달랐던 경계 사례로 별도 표기했다. 막힌 목표 2개는 경로를 반환하지 않았다.
- 실패 보존: 초기 낮은 목표 비용 후보는 도착 지연이 있었고, rotation_penalty=1 후보는 불필요한 회전과 끝점 계획 실패가 있었다. 원본 길이/이전 MPPI 조합의 실패와 120초 미완료도 보존했다. 최초 live 파라미터 요청은 시작 중 timeout 후 재조회에 성공했다.
- 적용: 정지 ACK 확인 후 소유 서비스 재시작, 초기 위치 복원, 실제 planner/MPPI/Collision Monitor 파라미터 확인 및 7점 미리보기 생성. base ENABLE=false, 준비 완료. 이동 명령은 보내지 않았다.
- 원본: `/home/ehdrms/workspace/cleany-data/reports/native-lattice-mppi-20261106/`의 `report.html`, `selected-preview-results.json`, YAML, CSV, map bytes, 테스트 로그와 runtime JSON. 실물 슬립과 도착 성능은 미검증이다.

- 08:00:10 이전 bag에서 정지 odom은 x=-0.490m, y=-0.949m, yaw 약 0.57rad였으나,
  장치 재실행 후 08:01:54부터 새 bag에서는 0,0,0으로 재시작했다. 확장 SLAM을
  유지한 상태에서 이 불연속이 입력되어 지도 중첩이 발생했다. 해당 확장 결과를 보존하고
  원래 라운지 상태로 복구했다. odom 연속성 검사와 전용 전달 토픽을 추가했고 도구 검사 5개가 통과했다.

마무리 확인 중 08:03:40 KST에 phone_remote만 정상 종료 후 respawn한 기록을 확인했다. 종료를 유발한 주체는 확인하지 못했다. Nav2 PID는 유지됐으며 새 프로세스에서 정지 ACK와 경로 미리보기를 다시 확인했다. 상세는 같은 원본 폴더의 `delivery-checks.json`이다.

- 초기 위치 재점검에서 확장 SLAM이 AMCL 초기값보다 약 0.20m 어긋난 별도 결함을 확인했다.
  StartTrajectory의 시간 0 기준 상대 pose는 참조 궤적의 첫 global node를 기준으로 하는데,
  초기 도구가 마지막 submap의 local-to-global 변환을 기준으로 역변환했다.
  `read_origin.cpp`를 첫 global node 기준으로 수정했다. 원본 map 상태의 참조 값은
  (-0.00145637m, 0.01075711m, 0.00362742rad)다. 수정 후 원본 상태에서 다시 시작한
  실행은 `cleany-data/runs/map-extension/20261106T080737`에 보존한다.
- 수정된 초기 좌표 변환과 odom 센서 시각 검사로 재시작한 실행은
  `cleany-data/runs/map-extension/20261106T080832`이다. 초기 요청 pose와 실제
  Cartographer pose의 차이는 수신 표본에서 약 0.0048m, -0.083도였다.
  이는 초기값 일치 검사이며 실세계 절대 위치 정확도 측정은 아니다.
  초기화 중 콜백 대기 시간을 센서 데이터 공백으로 오인하던 검사도 센서 stamp
  차이로 수정했다. 도구 단위 검사 6개가 통과했다.

- 08:11 정지 재위치추정: 원본 라운지 `map.yaml`의 occupied 셀과 현재 `/scan`을
  비교했다. 후보 (0.22287m, -4.31555m, -1.73045rad)를 AMCL 초기값으로 적용한 후
  원본 `latest.pbstream`에서 재시작했다. 근거 데이터와 후보 비교는
  `cleany-data/runs/map-extension/relocalize-20261106T0810/{result.json,global.json,alignment.npz}`,
  새 확장 출력은 `cleany-data/runs/map-extension/20261106T081157`이다.
  후보에서 스캔 점의 약 52%가 기존 occupied 셀 10cm 안에 있어 부분 불일치가 남는다.
  이는 실세계 정답 위치 검증이 아니며 신규 공간 또는 변한 물체를 구분하지 않는다.

- 조종 화면 검은 화면 재현: 실행 중 서버가 이전 `index-Bv-ef30H.js`를 참조한
  HTML을 보존하고 있었으나 빌드가 새 `index-WOLJN-l5.js`로 교체되어 이전 JS는 404였다.
  `/`가 현재 entrypoint를 읽도록 변경하고 조종 서버만 재기동했다. UI asset 검사 4개 통과.

- 확장 주행 GIF/MP4: `cleany-data/reports/map-extension-20261106/`.
  최종 `final.pbstream`의 trajectory 1 노드 3,152개와 원래 궤적을 읽어 최종 지도 위에
  시간순 경로를 표시했다. 확장 전용 지도/pose 토픽은 bag에 없어 실시간 지도 생성
  재생이 아니며, bag의 `/scan_raw`를 최종 보정 경로로 변환한 사후 재구성이다.
  원본 궤적 시각 08:16:43~08:43:53, 6초 간격 프레임, 10fps 및 양끝 홀드 포함
  31.3초. GIF 전체 디코딩, MP4 313프레임, 1280×960, 스캔 시각이 해당 프레임
  이후가 아님을 확인했다. 파일 해시와 자세한 출처는 `provenance.json`에 있다.

- 지도 생성 GIF 수정: 최종 지도 위 주행 재구성 대신 원본 센서를 Cartographer에
  재입력했다. `cleany-data/experiments/map-growth-20261106/full4`는 ROS_DOMAIN_ID=188에서
  기존 seed 상태와 동일 설정으로 12,501개 scan을 처리하고 시간 진행 전 지도 273개를
  캡처했다. 최종 지도 또는 최종 궤적은 렌더링 입력에 사용하지 않았으며 이전 최종
  궤적 파일은 재생 구간의 시작/종료 시각에만 사용했다. 최종 노드와 비교한 재계산
  궤적 위치 차이는 중앙값 0.0139m, p95 0.0248m였다. 센서 static TF quaternion의
  기본 w=1을 덮어쓰지 않았던 첫 재실행 결과는 폐기 대상으로 분리했고 w=0으로
  수정했다. 새 결과는 `cleany-data/reports/map-growth-20261106/`에 저장했다.
  이 결과는 당시 화면 녹화가 아니라 원본 bag 재실행 중 생성된 지도의 영상이다.


## Phone Omni 고정 방향 비용과 슬립 문맥 기록 (2026-11-06)

`phone-nav2-demo`, base `4eaf919145e03ffe248732c950dab1267ded17d2`의 미커밋 작업 상태에서
횡/대각 비용 0.25/0.125 cost/m, 기존 SlipRecovery 유지와 분석 문맥 기록을 구현했다.
Fedora aarch64의 Ubuntu 22.04 Distrobox, ROS 2 Humble에서 navigation CTest 25개 그룹,
logger/teleop pytest 156개와 별도 ROS opt-in 검사를 실행했다. source 해시, 검사 출력과
[HTML 보고서](/home/ehdrms/workspace/cleany-data/reports/fixed-direction-cost-20261106/report.html)는
같은 결과 디렉터리에 보존했다.

입력은 이전 native Lattice 비교의 recorded costmap과 명시적 목표 8개다. 실제 planner/MPPI와
이상 운동학을 이용한 8개 사례 및 두 사례의 추가 반복 4회가 통과했다. 강한 비용의 최종 정렬
실패, 기존 NoiseGeneratorMain 비동기 시험의 중간 실패, 어댑터 초기 생성 오류와 수정도 원본
로그에 남겼다. 합성 전체 Nav2에서 반복 슬립 실패와 안전 출력 0을 확인하고, 별도 실제 bag
역직렬화에서 슬립 55개/복구 110개 payload가 일치했다.

기존 기록을 마감한 후 정식 runtime으로 재기동하고 같은 지도와 수동 방향 정책을 보존했다.
현재 active 설정과 실제 bag의 위치/명령 문맥을 확인했다. AMCL은 저장한 초기 추정을
관측으로 수정했으며 정지 중 TF 안정화는 확인했지만 실제 위치 정확도는 검증하지 않았다.
실제 로봇에 이동 목표를 보내지 않았으므로 실물 슬립 감소나 복구 성공 증거는 아니다.


## 2026-11-06 bag 기록 기반 MPPI 초기화 검증

기준은 `phone-nav2-demo`의 `4eaf919145e03ffe248732c950dab1267ded17d2`와 기존 dirty 변경이다.
Fedora aarch64의 Ubuntu 22.04 ROS 2 Humble 컨테이너에서 domain 188/189와
`cleany-data/builds/bag-navigation-review` 격리 설치본을 사용했다.
원본과 리포트는 `/home/ehdrms/workspace/cleany-data/reports/bag-navigation-improvements-20261106/`에 있다.

입력 bag `526a70a7-e9fd-4681-91c2-8d9d59efda7a`,
`ef507ee6-e9b6-49db-ad86-65dba4d20a9d`, `59fc31f1-1e8e-4823-87ec-dd340f3f643c`의
명령, 계획, BT 및 ROS 로그를 조사했다. 07:22:40–07:30:25 KST의 기존 원본 캐시에서
슬립 정합을 다시 계산한 928개 창은 suspect 39, no_evidence 840, unknown 49,
지속 불일치 3구간이었다. 최대 1초 이동 차이는 0.132725m다. LiDAR 참값 검증이 아니며
바닥 슬립과 장애물에 걸린 휠 회전을 분리하지 못한다. 다른 세션의 점유 목표 (0.5, 0.5)
경로 생성 실패 로그 14개는 별도로 분류했다.

기존 `NoiseGenerator`의 reset 직후 후보 복사 100회에서 잡음 없는 첫 묶음이 100회
관측됐다. reset 안에서 첫 묶음을 완성하고 상태 변경과 종료의 잠금을 맞추며 controller별
난수 엔진을 분리한 뒤에는 0회였다. 수정안 reset+복사 p50/p99는 1.454/2.246ms였다.
Navigation CTest 25개 그룹, NoiseGenerator 30회 연속 반복, 도구 및 슬립 분석 pytest
23개가 통과했다. 기존 이상 운동 8개 과제는 모두 통과했고 하네스의 사후 footprint
충돌 횟수는 0이었다. 실제 마찰이나 실시간 최악 실행 시간 검증은 아니다.

bag에서 추출한 6개 고정 global costmap 과제는 기존 1/6, 수정 0/6 성공이었다.
원본 위치/TF/계획의 수신 및 header cutoff를 적용하고 비용 구간의 상한으로 역변환했다.
정지 출발 이상 운동이며 시간에 따라 변하는 local map, AMCL, BT와 슬립은 모사하지 않았다.
시작 footprint의 lethal 판정과 도착 직전 빈 경로를 실패로 보존했다.
recorded-middle의 4회 비교는 기존 1/4, 수정 0/4 성공으로, 이 표본에서 전체 개선을
주장하거나 퇴행을 배제할 수 없다. 실패 끝점만 별도 초기값으로 넣으면 목표와
0.000655m/0.109675rad 차이에서 기존 native Lattice가 MPPI 실행 전부터 빈 경로를
반환했다. 3cm/0.1rad 도착 조건을 완화하지 않았다.

비용 가중치, BT, 휠/가속도 및 충돌 제한은 유지했다. 수정은 소스와 격리 빌드에 저장했으며
운영 설치본 교체는 보류했다. 실물 이동 명령은 보내지 않았다. 빌드 중 09:42:45 KST
메모리 부족으로 커널이 niri를 종료해 데스크톱과 ROS 서비스가 중단됐다.
CMake 병렬 환경 변수를 -D 인수로 전달한 오류와 패키지 수준 제한의 한계를 확인했고,
후속 컴파일은 -j 2로 제한했다. 로봇은 재기동하지 않았다. 원본 실패 로그, OOM 로그,
후보 patch, 입력 provenance 및 소스/바이너리 SHA-256을 보고서와 함께 보존했다.


## 2026-11-06 각도별 대각 비용 곡선 실물 시험 준비

`phone-nav2-demo`의 기준 commit `4eaf919145e03ffe248732c950dab1267ded17d2`와 기존 dirty 변경에서
사용자 합의대로 방향 비용을 `0.1 sin²θ + 0.45 sin²(2θ)` cost/m로 교체했다.
전후진 0, 횡이동 0.1, 45도 대각이동 0.5이며 최대는 약 46.6도에서 0.5014다.
목표 도착과 DiffDrive 예외, 충돌 및 휠 제한은 유지했다. 외곽 여유 비용은 추가하지 않았다.

Ubuntu 22.04 ROS 2 Humble 컨테이너에서 C++ 병렬 작업 수 2, 패키지 작업 수 1로
`make build`의 cleany_navigation을 빌드했다. 최소 critic 검사 후 프로필 pytest 9개와
Navigation CTest 25개 그룹이 통과했다. 사용자의 요청으로 bag 및 모사 주행은 실행하지 않았다.
이 설치본은 앞서 단위 검증한 NoiseGenerator 초기화 수정도 포함한다.

`cleany-phone-nav2.service`를 Omni로 시작해 controller_server가 active이며 실행 가중치가
0.1/0.45인 것을 확인했다. 휠 상한 4/1.5rad/s, goal angle weight 12를 확인했다.
새 로그 세션은 `d06dac11-5dc5-4ccd-be84-6a29a96870c2`이고 휠 피드백 457개 및
API에서 정지, PWM 0과 모터 disabled를 확인했다. 자동 이동 목표는 보내지 않았다.
Omni 시작 기본값으로 수동 횡이동도 허용된다. 사용자는 위치나 방향을 옮겼다고 답했으므로
기존 AMCL 위치를 복원하지 않았다. 자동주행 준비 완료는 주장하지 않으며 초기 위치 지정이 남았다.

소스 해시, HTML 리포트, 단위 검사와 실행 확인 원본은
`/home/ehdrms/workspace/cleany-data/reports/directional-peak-live-20261106/`에 보존했다.

## 2026-11-06 MPPI 경로 방향 추종 완화와 6번 비교

`phone-nav2-demo`의 기존 dirty 작업 상태에서 `nav2_phone_omni.yaml`의
`PathAlignCritic.use_path_orientations`를 false, `TwirlingCritic.cost_weight`를 1로
변경했다. Lattice primitive와 방향 비용 곡선, 안전 및 가속도 제한은 유지했다.
`test_phone_profile.py` 9개 검사가 통과했다. 원본 입력, 기준 commit, 파일 해시,
설정과 로그는 `/home/ehdrms/workspace/cleany-data/reports/mppi-settings-case6-20261106/`
의 `manifest.json`과 `report.html`에 있다.

과거 6번의 고정 costmap을 사용한 격리 이상 운동학 시험 2회에서 평균 통로 점유 폭은
기존 90.6cm에서 70.2/68.8cm, 목표 위치 3cm 이내 첫 진입은 95.15초에서
76.75/76.70초가 됐다. 두 실행 모두 최종 방향 정렬 전 재계획 실패가 남았으며
충돌 카운터는 0이었다. 실물 성공 또는 현재 확장 지도 시험을 의미하지 않는다.
실물 서버는 정지 상태에서 두 파라미터를 변경했고, 요청 응답 시간 초과 후 독립
조회로 저장값을 확인했다. 서버 재시작과 실물 이동 명령은 보내지 않았다.

## 2026-11-06 대칭 고정 자세 primitive 후보

기존 `phone-nav2-demo` dirty 상태에서 16개 heading 및 5cm 격자를 유지한 8방향
고정 자세 이동 후보를 생성했다. 총 224개 중 이동 128개만 재구성했고 회전 96개의
기하와 길이는 보존했다. 기존 회전 대칭 검사는 45도 배수 heading에서 반사 쌍이
존재하고 나머지 8개 heading에서 비대칭임을 확인했다.
`tests/tools/test_symmetric_primitives.py` 4개 검사가 통과했고 설치된 Smac 1.1.20의
파일 reader가 224개/16개 그룹을 정상 파싱했다. 주행 기본값, 실행 서버와 MPPI는
변경하지 않았다. 경로 검색 및 실물 시험은 수행하지 않았다.
보고서와 원본 검증은 `/home/ehdrms/workspace/cleany-data/reports/symmetric-primitives-20261106/`
의 `report.html`, `manifest.json`, `turn-audit.json`, `native-probe.log`에 있다.

2026-11-06 후속 사용자 적용 요청으로 phone Omni 기본값과 실행 중 planner의
`GridBased.lattice_filepath`를 224개 파일로 전환했다. 정지 상태에서 순정 동적
재초기화 응답 성공과 파라미터 재조회를 확인했으며 재시작·이동 명령은 없었다.
관련 설정 및 구조 검사 13개 통과. 적용 기록은 동일 보고서 폴더의
`application/result.json`에 있다. 기존 `allow_reverse_expansion=true`는 유지했다.

2026-11-06 후속 요청으로 phone Omni 기본값과 실행 중 planner를 기존
`omni-r10cm-balanced.json` 144개 구성으로 복구했다. 224개 후보는 보존했다.
MPPI와 지도는 변경하지 않았다. 정지 확인 후 동적 교체했으며 적용 원본은
위 보고서의 `revert/result.json`에 남긴다. 이 시점 주행 준비는 슬립 측정
불가에 따른 복구 실패 상태이며, 이 복구 작업에서 해당 상태를 해제하지 않았다.

## 2026-11-08 D435 LiDAR 높이 하한 미리보기

Base commit `4eaf919145e03ffe248732c950dab1267ded17d2`의 dirty `phone-nav2-demo`, Fedora / ROS 2 Humble Distrobox에서 실물 D435와 head TF를 사용했다. `min_base_z=0.192` 적용 후 8초 구간 유효 점군 107프레임의 최소 z는 0.192168m, 하한 미만 점은 0개였다. 관련 core/transport 테스트 30개 통과, RViz 표시 확인. 원본 근거는 `CLEANY_DATA_ROOT/runs/navigation/integration/2026-11-08_100401_d435-low-obstacle/lidar-height-cutoff/`의 manifest와 live-check다. 서비스 정지 후 컨테이너 자식 프로세스가 남아 식별된 미리보기 PID에 SIGINT를 전달한 뒤 교체했다. Nav2 및 Collision Monitor 미연결, 물체 검출률 및 주행 검증은 미수행이다.

## 2026-11-08 실물 depth local costmap 및 Monitor

`phone-nav2-demo`의 base commit `4eaf919145e03ffe248732c950dab1267ded17d2`와 미커밋 변경을 사용했다. Fedora aarch64 / ROS 2 Humble Distrobox에서 기존 depth 투영기를 확장하고 local VoxelLayer 및 기존 phone Collision Monitor에 연결했다. global costmap, MPPI와 slip-recovery 선택은 유지했다.

원본은 `CLEANY_DATA_ROOT/runs/navigation/integration/2026-11-08_110835_depth-local-costmap/`이다. `index.html`은 실물 의자 전후 비교, `manifest.json`은 소스 해시와 스냅샷 범위, `tests.xml`은 95개 통과 결과를 담는다. 실물 의자 중심 구역의 점유 셀은 66에서 0으로 줄었다. 넓은 구역은 책상 가장자리를 포함해 84에서 18로 줄었고, 필터 점은 1322에서 75로 줄었다. 좌표별 원본 JSON/NPZ와 적용 전후 설정을 보존했다. ROI는 관측 점군을 보고 지정했으며 의미 분할 정답이 아니다.

격리 domain 191/192에서 실제 Monitor와 VoxelLayer를 구동했다. 초기 회귀에서 Humble Monitor가 stale depth를 무시해 속도를 통과시키는 실패가 있었으며, 같은 Monitor의 required-depth 신선도 제한으로 수정 후 통과했다. live 적용 시 runtime을 정상 종료해 logger를 마무리하고 저장 pose를 복원했다. 첫 단발 초기 pose는 AMCL 수신 확인이 없어 TF가 생기지 않았고, AMCL subscriber 확인과 재전송 후 복원됐다. transient 서비스가 종료 후 수거되어 systemd-run으로 재생성했다. 마지막 TeleopStatus는 ready=true, paused=false, connected=true, stop_confirmed=true였다. 실물 이동 명령은 발행하지 않았으며 실제 회피 주행, 제동거리와 head 이동 중 costmap 동작은 미검증이다.


## 2026-11-08 depth 처리 지연과 분홍 점 관측

`runs/navigation/integration/2026-11-08_110835_depth-local-costmap/latency-and-ghosts.json`의 약 20초 관측에서 depth 입력은 14.25Hz, 장애물 출력은 14.31Hz였다. 원본 촬영 시각부터 장애물 수신까지 중앙값 107ms, p95 121ms였으며 같은 stamp의 영상 수신부터 장애물 수신까지는 중앙값 36ms였다. Local costmap의 update 직후 footprint 발행은 10.00Hz, 화면용 costmap 발행은 1.76Hz였다. 실제 제동거리나 전체 센서→제어 지연 측정은 아니다.

`diagnose_depth_costmap.py`는 수신 주기, 원본 시각, CPU와 관측 점을 수집한다. `compare_live_costmap_layers.py`는 이동 명령 없이 별도 단일 레이어 costmap을 구성한다. 진단 중 로봇 위치가 달라졌으므로 depth와 LiDAR 단독 결과를 동일 위치의 오검출 비교로 해석할 수 없다. 단일 프로세스에서 연속 lifecycle을 시작한 진단은 두 번째 단계의 서비스 발견 실패를 보존하고 단일 레이어별 독립 실행으로 확인했다. 사용자가 분홍 점이 사라졌다고 알렸으나 원인은 확정하지 않았고, 이를 수정 완료로 기록하지 않는다. 이 진단에서는 운영 필터나 갱신 주기를 바꾸지 않았다.


## 2026-11-08 누적 phone navigation 변경의 커밋 전 회귀

기반 `4eaf919145e03ffe248732c950dab1267ded17d2`에서 기능별로 분리한 코드의 마지막 커밋은 `9105eb7320d459be25818edfab0966354cd3565d`이다. Fedora aarch64의 Humble Distrobox와 호스트 Node.js에서 확인했다. 실행 근거는 `CLEANY_DATA_ROOT/runs/project/2026-11-08_114517_phone-navigation-commit-review/`의 변경 전 소스 스냅샷, manifest, 커밋 목록과 검사 로그다. 실물 서비스 재시작이나 이동 명령은 하지 않았다.

| 검증 | 결과 |
|---|---|
| `make test-navigation`, 별도 build-navigation/install-navigation 및 DDS 194 | 의존 패키지 12개 빌드, colcon 집계 405개, 오류·실패·생략 0. Python 217개와 C++ 플러그인 포함 |
| 호스트 `make test-hardware-mapping-core BASE_TEST_ROS_DOMAIN_ID=195` | Python 130개, Node 버튼 입력 17개 통과 |
| Base disarmed/stop retry 및 Supervisor 전체 pytest, DDS 196 | 84개 통과 |
| `CLEANY_RUN_ROS_TESTS=1 make test-dev-monitor`, DDS 195 | Logger 65개, Monitor 80개 통과 |
| 새 bag, 지도, slip, depth/head 분석 도구 pytest | 57개 통과 |
| `make build-teleop-ui` | TypeScript 검사와 Vite 빌드 통과 |

첫 조종기 검사는 Make recipe가 PYTHONPATH를 덮어써 ROS import에 실패했다. recipe에서 ROS 및 설치 환경을 불러오고 상속 경로와 격리 domain을 유지하도록 수정했다. 컨테이너에는 Node.js가 없어 전체 recipe는 호스트에서 다시 실행했다. 첫 Navigation 검사는 Python 묶음이 기본 60초 제한에 걸렸다. 개별 검사 조건은 유지하고 CTest 묶음 한도를 300초로 변경한 뒤 Python 묶음은 약 75초, 전체 CTest 25개 묶음은 약 82초에 완료했다. 초기 실패 로그도 보존했다.

이 결과는 관련 패키지와 합성 ROS 경계의 회귀다. 전체 workspace CI, 새로운 실물 회피 주행, 제동거리 및 재부팅 지속성 검증은 수행하지 않았다. 이전 실물 결과와 실패·미확정 사항은 각각의 실행 근거를 따른다.
