# Cleany sensor scene mapping

실험용 MoveIt OctoMap updater. MuJoCo sorting 파이프라인은 이 updater를
기본으로 사용한다. 실물 및 다른 모드는 기존 updater를 유지하며,
`depth_octomap_plugin:=cleany_scene_mapping/KnownGeometryOctomapUpdater`로
명시 선택할 수 있다. 실제 하드웨어 안전 검증을 완료한 기능이 아니다.

MoveIt의 `ShapeMask`로 점군 self-mask를 계산하고, sensor ray의 free/occupied/
model/clip 셀을 갱신한다. MoveIt이 제외 대상으로 전달한 collision shape 안에
완전히 포함되는 **기존 점유 leaf**를 free로 바꾼다. 로봇 URDF, 관측에서
구한 target OBB, 명시적으로 등록한 수거함이 shape의 출처다. MuJoCo GT나
물체 이름/정답 위치는 읽지 않으며 planning scene의 충돌 형상을 삭제하지 않는다.

가려져 현재 depth ray가 도달하지 않는 내부 잔상을 다루는 목적이다.
표준 updater와 같은 `padding_scale` / `padding_offset`을 사용한다.
한 convex body 안에 셀의 8개 모서리가 모두 포함되어야 한다. 일부만 겹치는
셀, 여러 형상 사이의 틈, 바깥 장애물, 아직 관측하지 않은 셀은 비우지 않는다.
TF는 점군 촬영 시점의 cache를 사용한다. 하나라도 누락되거나 비정상이거나,
cache 조회 중 형상이 추가·제거됐으면 **그 프레임 전체를 거부**한다.
그 경우 self-mask/raycast/내부 정리/점유 갱신/filtered-cloud 발행을 모두
수행하지 않는다. 따라서 receipt relay도 해당 촬영 시각을 처리 완료로 알리지 않는다.
로봇/OBB/TF가 틀린 경우 실제 장애물이 잘못
제외될 위험은 남으므로 이 기능이 인식 오류나 충돌 안전을 보증하지 않는다.

## 설정

`depth_cloud.known_geometry_clear_max_leaves`는 메시지당 검사하는 leaf 수의
상한이며 기본 200000이다. 초과하면 나머지를 건드리지 않고 경고한다.
free cell도 검사 예산에 포함한다. 정리 수/검사 수/소요 시간은 throttled
`DEBUG` log로 기록한다. 정상 프레임의 raycast는 이 추가 검사 예산과 무관하게 수행한다.
이 패키지는 미지정 시 `RelWithDebInfo`로 빌드한다. 점유 leaf 순회와
convex body 검사는 CPU 경로이므로 디버그 무최적화 실행 시간과 구분한다.

`depth_cloud.mask_workers`는 1–4개 CPU 작업자로 self-mask를 분할하며 기본은 4다.
모든 점을 순환 배분한 뒤 원래 순서로 합치므로 점군 밀도, mesh, padding,
range/내부/외부 판정은 바꾸지 않는다. 작업자는 각각 upstream ShapeMask와
독립적인 Body 자세 상태를 가지며, `Body::cloneAt`으로 불변 convex mesh 계산
결과만 공유한다. 원본 형상은 한 번만 생성해 시작 시 반복 hull 계산을 피한다.
형상 변경/cache 갱신은 모든 작업자가 끝날 때까지 직렬화하며 작업 예외는
프레임 전체를 거부한다. 1로 설정하면 분할 없이 원래 ShapeMask를 호출한다.
전체 CPU 작업량/메모리는 감소한다는 보장이 없으며 CPU 여유에 맞춰 설정한다.

병렬 작업 스레드는 worker 수 설정 시 한 번 생성하고 프레임 사이에는 condition
variable에서 대기한다. 기본 4 worker는 호출 스레드 1개와 상주 스레드 3개로 실행한다.
프레임마다 `std::async` 스레드를 생성·종료하지 않으며, 점군 분할 및 판정 배열도
다음 프레임에 재사용한다. 원본 점군을 순환 분할 버퍼로 복사하는 작업 자체는 유지한다.
버퍼는 처리한 최대 프레임 크기에 맞춰 용량을 보관하므로, 반복 할당 감소가
프로세스 상주 메모리 감소를 의미하지는 않는다. worker 수 재설정 또는 객체 종료 시
스레드를 종료하고 버퍼를 해제한다. 형상이 등록된 동안 worker 수 변경은 거부한다.

호출 스레드 또는 작업 스레드에서 예외가 발생하면 나머지 작업자가 모두 끝날 때까지
기다린 후 예외를 전달한다. 실패한 프레임의 일부 판정을 반환하지 않으며 다음 프레임에서
정상 복구할 수 있다. 상위 updater의 frame/geometry 직렬화와 TF·형상 세대 검사는 유지한다.

## 빌드와 검증

```bash
make test-scene-mapping
```

정지 상태에서 저장한 진단 fixture가 있다면 ROS node나 물리 명령 없이 비교한다:

```bash
make profile-scene-mask MASK_FIXTURE=/absolute/path/to/mask_fixture
```

fixture는 `robot.urdf`, `raw.cdr`(PointCloud2), `fk.cdr`(GetPositionFK response,
cloud frame의 collision link pose)를 요구한다. robot-only serial/2/4 worker 결과가
점 단위로 완전히 같아야 통과하며 부품별 내부 판정/clone 시간을 출력한다.
world/attached body는 이 프로파일에 포함되지 않는다. 별도 frame integration
검사가 해당 형상의 누락/cache 변경 시 거부와 정상 점군 통합을 검증한다.
이 수치는 실제 이동 중 지연 또는 Jetson 성능을 대신하지 않는다.

2026-09-29 버퍼/스레드 재사용 전후 비교는 저장된 robot-only fixture
(76,800점, 27개 형상, 4 worker)를 사용했다. 준비 후 전후 실행 순서를 교대해
12회 비교한 self-mask 중앙값은 365.53→363.30ms(약 0.6% 감소)였고, 모든 점의
결과는 단일 worker 기준과 일치했다. 별도 3회 계측에서 해당 호출 안의 C++
`operator new/new[]` 할당은 프레임당 30→0회였다. 프레임마다 만들던 스레드 3개는
초기화 시 생성한 스레드로 대체했다. 이 fixture에서 재사용하는 분할/판정 payload는
약 1.17MiB이며 프로세스 RSS 감소를 측정한 것은 아니다. 처리 시간 차이는 작아서
큰 속도 향상이나 전체 파이프라인 지연 급증 해소를 주장하지 않는다. 현재 실행 로그의
지도 전체 갱신 시간과도 측정 범위가 다르다. 전후 코드·원자료·계측 프로그램은 로컬
`artifacts/scene_mask_reuse_20260929/`에 보관한다.

ROS 2 Humble의 `moveit_ros_perception`, `moveit_ros_occupancy_map_monitor`,
`geometric_shapes`, `octomap`이 필요하다. 로컬 MoveIt perception prefix의
설치 조건은 루트 `docs/DEVELOPMENT_SETUP.md`를 따른다. core gtest는 내부/
외부/경계/분리 형상/회전/가변 leaf 크기/예산/TF 누락을 검사한다.
통합 gtest는 형상 세대 변경, 누락·비정상 cache, 거부 프레임의 tree/receipt
미갱신, 정상 self-mask와 주변 장애물 보존, 카메라 원점, 잘못된 점군 layout,
range clipping, stop/start, 동시 frame 직렬화 및 executor 지연 후 최신 프레임
선택을 검사한다.
이 테스트가 실제 파지·운반 성공을 증명하지는 않는다.
`test_partitioned_shape_mask`는 worker 재사용/재설정, 큰 프레임에서 작은 프레임으로의
전환, XYZ/XYZRGB layout 변경, 호출·작업 스레드 예외 후 전체 작업 종료와 복구도 검사한다.

## 프레임 처리 계약

이전 `PointCloudOctomapUpdater::updateMask` hook은 정리를 건너뛰더라도
base callback의 점유 갱신을 중단할 수 없었다. 지금은 `OccupancyMapUpdater`
plugin으로 프레임 callback 전체를 소유한다. 한 mutex로 callback을 직렬화하고,
형상 세대·cache 검사와 mask를 geometry mutex 안에서 수행한 후 body clone을
보존한다. transform provider 호출과 tree 접근 중에는 geometry mutex를 잡지
않는다(PlanningSceneMonitor의 shape/tree lock과 역순 잠금 방지).

입력은 `depth_scene_node`의 optical-frame PointCloud2 계약을 따른다:
little-endian FLOAT32 XYZ(offset 0/4/8), packed row, 완전한 data buffer.
다른 layout은 거부한다. shape mask에서 카메라 원점은 optical frame의 0이며,
raycasting에서만 촬영 시점 map-frame 카메라 위치로 변환한다. TF가 아직
도착하지 않았다면 해당 프레임을 거부하고 다음 새 프레임을 받는다. TF 대기
queue나 재발행/임의 타임스탬프는 사용하지 않는다. 정상 filtered cloud의
header는 원본 그대로이며 BEST_EFFORT SensorDataQoS로 발행한다. 입력/출력
history는 `KEEP_LAST(1)`이다. 처리 중 새 입력이 쌓이면 과거 영상을 순서대로
처리하지 않고 가장 최근 촬영본을 받는다. 한 프레임의 처리 중단이나 timestamp
재작성은 하지 않는다. 최신성 허용 기준은 coordinator에서 그대로 검사한다.
로그는 transform/cache, mask/shape snapshot, raycast, tree 갱신, receipt 발행
구간의 wall time과 입력/출력의 ROS capture age를 구분한다. lock 대기는 해당
구간 시간에 포함되므로 각 숫자를 순수 CPU 시간으로 해석하지 않는다.
추가 callback 진단은 ROS clock과 독립적인 steady clock으로 초당 최대 한 번
입장/종료를 기록한다. 원본 capture stamp, 해당 node의 ROS 시각, 마지막 갱신
시각과 종료 stage(`ros_clock_rate_limit`, `transform_cache`, `map_transform`,
`shape_mask`, `tree_update`, `integrated` 등)를 포함한다. transform cache가 false를
반환한 경우의 시간도 경고로 남긴다. throttled 로그이므로 모든 프레임의 trace는
아니며, callback 로그만으로 upstream 입력 중단과 executor 지연을 구분할 수는 없다.
입력 토픽 관측과 함께 읽어야 한다. 진단은 프레임 수용 조건이나 지도 내용,
원본 timestamp/receipt 계약을 바꾸지 않는다.

참고한 [MoveIt 2.5.9 updater](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)와
[transform cache provider](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)의
동작을 기준으로 한다. 시스템 전체의 실제 충돌 안전을 검증한 것은 아니다.

## 로그 수준

준비 완료와 실패·TF cache 누락·검사 예산 초과는 기본 출력에 남는다.
정상 프레임의 입장·종료와 `Consistent depth frame` 통계는 throttled `DEBUG`
로그이며 `move_group.scene_mapping` 로거를 사용한다. 상위 MuJoCo launch의
`log_level:=debug` 또는 standalone MoveIt의 같은 옵션으로 확인한다.
기본 `INFO`에서는 대기 중 정상 프레임 진단을 계속 출력하지 않는다.
