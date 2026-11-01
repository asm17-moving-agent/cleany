# 실물 매핑 구성

`hardware_mapping.launch.py`는 설치된 패키지로 hardware runtime을 시작한다.
Runtime은 영속 센서/base/조종 launch와 매핑 관리자를 별도 프로세스 그룹으로 소유한다.
종료 시 매핑 관리자의 저장을 먼저 기다리고 장치 launch를 종료한다.
웹 페이지를 닫아도 지도와 기록은 종료되지 않는다.

| 프로필 | 출처와 범위 |
|---|---|
| `mapping_legacy.yaml` | 이전 실물 매핑의 x=0.029m, yaw=-7.85도. 원본 측정 artifact 미확인 |
| `mapping_evaluation_20261015.yaml` | main의 평면 보정 x=-0.070807m, y=-0.038232m, yaw=79.770070도, laser frame |

두 프로필은 서로 다른 장착/프레임 자료이므로 합성하지 않는다. 둘 다 z=0은 평면 규약이며
실측 높이가 아니다. 장착을 변경했다면 해당 프로필의 사용 근거를 다시 확인한다.
기본값은 기존 매핑 동작을 보존하는 legacy 프로필이다. mock은 실물 geometry 대신
`base_synthetic.yaml`을 선택하며 Agent와 실물 LiDAR를 시작하지 않는다.

설정은 설치된 package share에서 찾고, 실행 시 state_root에 확정 설정을 보존한다.
키는 state_root/web-token(0600), 지도는 maps/, 녹화는 recordings/에 두며 저장소 소스나
`.runtime` 빌드 경로를 사용하지 않는다. 시작 명령은 [RUNNING](../../../../docs/RUNNING.md),
설치와 외부 드라이버는 [개발환경](../../../../docs/DEVELOPMENT_SETUP.md)을 따른다.
