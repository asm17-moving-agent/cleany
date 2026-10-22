# Cleany sensor scene mapping

MoveIt의 sensor-derived OctoMap에서 알려진 collision 형상을 제외하는 선택형 updater다. MuJoCo sorting 파이프라인은 이 updater를 기본으로 사용한다. 다른 실행에서는 `depth_octomap_plugin:=cleany_scene_mapping/KnownGeometryOctomapUpdater`로 선택한다. 실물 충돌 안전을 검증한 기능은 아니다.

입력은 depth 관측과 planning scene이 등록한 로봇 URDF, 관측 기반 target mesh/OBB, 수거함 collision shape다. MuJoCo GT, 물체 이름이나 정답 위치를 읽지 않으며 planning scene의 collision shape를 삭제하지 않는다.

## 형상과 점군 처리

한 프레임의 self-mask와 점유 leaf 정리는 촬영 시각 TF와 같은 불변 형상 snapshot을 사용한다. 형상 세대와 cache가 맞지 않거나 필요한 TF가 누락되면 프레임 전체를 거부한다. 이때 raycast, tree 갱신, filtered cloud와 receipt를 발행하지 않는다. callback은 직렬화하며 transform provider와 Octree 접근 중에는 geometry mutex를 잡지 않는다.

mesh에는 convex face를 법선 방향으로 `padding_offset`만큼 확장한 volume을 사용한다. `padding_scale`은 hull vertex 평균을 중심으로 적용하고 local bounding box로도 제한한다. 점군 self-mask와 leaf 정리에 같은 volume을 적용하므로 upstream의 vertex 방사 방향 padding과 다를 수 있다. primitive는 해당 shape의 내부 판정을 사용한다.

관측된 점유 leaf의 8개 모서리가 한 convex body에 모두 포함될 때만 free로 바꾼다. 부분 겹침, 여러 형상 사이의 틈, 바깥 장애물, 미관측 셀은 비우지 않는다. 정리는 점유 갱신 뒤에 수행해 같은 프레임의 depth endpoint가 내부 잔상을 다시 만들지 않게 한다. 잘못된 모델이나 TF는 실제 장애물도 제외할 수 있다.

입력 layout은 little-endian FLOAT32 XYZ(offset 0/4/8), packed row와 완전한 data buffer다. optical frame의 원점을 카메라 원점으로 사용하고 raycasting에서 map frame으로 변환한다. 다른 layout과 비정상 cache는 거부한다.

## 병렬 처리와 설정

`depth_cloud.mask_workers`는 1–4이며 기본 4다. 형상별 convex 계산은 등록 시 준비하고 worker는 하나의 불변 프레임을 공유한다. AABB 계층으로 후보를 좁힌 뒤 같은 volume으로 정밀 판정한다. worker는 원본 점군의 서로 다른 index를 읽고 결과 순서를 유지한다. 원본 점군을 worker별 메시지로 복사하지 않는다.

기본 4 worker는 호출 스레드와 상주 스레드 3개를 사용한다. 스레드와 판정 배열은 프레임 사이에 재사용하며 작업 예외가 생기면 나머지 worker 종료를 기다린 뒤 프레임 전체를 거부한다. 형상이 등록된 동안 worker 수를 바꿀 수 없다. 단일 worker도 같은 volume을 사용한다.

광선 경로는 OctoMap의 `computeRayKeys`를 그대로 사용한다. 같은 프레임에서 이미
방문한 free key는 최대 8 MiB의 재사용 비트 배열로 걸러 반복 hash 조회를 줄인다.
배열은 매 프레임 초기화하며 점군이나 지도를 이전 프레임 것으로 대체하지 않는다.
광선 영역이 메모리 예산보다 크거나 원점이 key 범위 밖이면 기존 hash 삽입 경로로
처리한다. free key의 집합과 free→occupied→model 갱신 우선순위는 유지한다. hash 내부
순회 순서는 달라질 수 있으므로 전체 점유 데이터의 동일성을 비교한다.

`depth_cloud.known_geometry_clear_max_leaves`는 프레임당 검사 예산이며 기본 200000이다. free cell도 예산에 포함하고 초과하면 나머지를 그대로 두며 경고한다. raycast는 이 추가 검사 예산과 별개다. 빌드 기본은 `RelWithDebInfo`다.

정상 프레임의 입장, 종료와 단계별 통계는 `move_group.scene_mapping`의 throttled `DEBUG` 로그다. 준비 실패, TF 누락과 예산 초과는 기본 출력에 남는다. callback 통계만으로 upstream 입력 중단과 executor 지연을 구분하지 않는다.

## 빌드와 검증

일반 패키지 빌드는 manifest의 MoveIt perception, geometric_shapes와 OctoMap을 사용한다. Jetson의 버전 고정 overlay는 [개발환경 가이드](../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

```bash
make scene-mapping-setup moveit-octomap-lock-setup
source tools/scene-mapping-env
make test-jetson-native
```

`test_raycast`는 기준 광선의 key 집합, 기존에 prune된 지도를 갱신한 전체 점유 데이터의 동일성, 메모리 예산 fallback과 key 경계를 비교한다.

C++ 검사는 내부, 외부, 경계, 회전, 가변 leaf 크기, shape 변경, TF/cache 누락, worker 예외와 복구, 단일/병렬 결과 일치를 확인한다. mesh component 병합 경계와 검증된 geometry cache의 손상·입력 hash 불일치도 검사한다. 실물 파지 성공과 현장 안전성은 별도 검증이다. 결과와 원본 위치는 [EVIDENCE](../../../docs/EVIDENCE.md)에 남긴다.
