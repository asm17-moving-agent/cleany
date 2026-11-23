# Jetson 관제 통합

Cleany와 `cleany-control-plane`을 노트북에서 개발하고
`jetson-orin-via-n100`에서 최종 검증합니다. Dashboard, Backend와
주행 서버는 모두 Jetson에서 실행합니다.

## 지도와 연결

선택한 지도는 `2026-11-08_manual-map-edit`이며 control-plane의
`apps/backend/maps/slam-20261108/`에 보관합니다. Nav2에는 같은 bundle의 `map.yaml`을
전달합니다. `metadata.json`의 원본 해시, resolution과 origin으로 지도 일치를
확인합니다. Dashboard 좌표를 역산하여 주행 목표를 만들지 않습니다.

Backend는 localhost:8080으로 수신하며 LAN에서는 `https://cleany.home.arpa`를
사용합니다. DNS와 HTTPS 설정은 control-plane의
`docs/architecture/slam-map-integration.md`가 관리합니다. Backend 원본 지도는 로봇의
실시간 `/map`이나 좌석 접근 목표를 대신하지 않습니다.

## 실물 연결 전 확인

| 경계 | 필요한 통합 |
|---|---|
| Robot Gateway와 pose 연결 | 현재 Backend의 Robot token 헤더를 클라이언트에 전달 |
| 지도 위치 | 최신 localization `map -> base_link`와 `frame_id=map`, `map_id=slam-20261108` 전달 |
| 좌석과 home | 실물 지도에서 접근 자세와 정지 조건 검증 후 설정 |
| 실행 출처 | Mission의 Nav2 `sim` 고정 값을 실제 backend 설정과 일치하도록 변경 |
| 인식과 조작 | Mission의 MockDesk 경계를 실제 관찰, 계획과 실행 서버에 연결 |
| 주행 UI | 인증과 origin 검사를 연결한 뒤 LAN 프록시 공개 |

현재 odom relay의 위치를 map 좌표라고 이름만 바꾸면 안 됩니다. 지도 위치가 없으면
Dashboard에 마커를 표시하지 않습니다. 기존 디스플레이와 CAD 서비스는 별도 프로세스이며
통합을 위해 종료하지 않습니다.

## 검증 순서

1. Backend 계약, 인증과 지도 API를 검증합니다.
2. Gateway 수락, 상태, 취소, 재연결과 중복 실행 방지를 검증합니다.
3. 실물 localization, 좌석 접근, 정지와 복귀를 검증합니다.
4. 고정 베이스에서 실제 관찰, 조작과 재관찰을 검증합니다.
5. Dashboard 요청부터 전후 관찰, 실제 책상 정리와 복귀까지 연결합니다.

미션 ID와 실행 ID로 두 저장소 commit, 설정, 지도 해시, 로그와 관찰 자료를 연결합니다.
HTTP health, DNS 응답과 화면 표시는 실물 왕복이나 책상 정리 완료의 증거가 아닙니다.

## Jetson 배포본 준비

먼저 control-plane의 `tools/deploy/jetson/host-layout/stage.py`로 새 릴리스를 만든다. 같은 디렉터리에 아래 순서로 센서와 현장 실행 환경을 추가한다. 명령은 로봇이 사용하는 ROS Humble 및 JetPack과 호환되는 Jetson에서 실행한다.

```bash
tools/ros2/stage-jetson-native "$RELEASE" "$PWD"
tools/ros2/stage-jetson-field "$RELEASE" "$PWD" "$FIELD_INSTALL" "$MOTION_INSTALL" "$CUROBO_VENV"
```

- `RELEASE`는 `/opt/cleany/releases/`의 새 후보 경로다. `FIELD_INSTALL`에는 현장 미션 패키지와 정지 자세를 검사하는 팔 드라이버, `MOTION_INSTALL`에는 실물 MoveIt 패키지가 필요하다.
- 최초 설치에는 `CLEANY_STAGE_YOLO_MODELS`, `CLEANY_STAGE_YOLO_ENV`, `CLEANY_STAGE_TORCH_SITE`로 검증한 YOLOE 파일과 GPU 환경을 지정한다. Field 의존성을 처음 등록할 때는 두 번째 명령 끝에 `--deps-from "$HARDWARE_INSTALL" "$MOVEIT_OVERLAYS"`를 붙인다. 이후에는 기존 릴리스가 가리키는 버전을 사용한다.
- CAD, GPU 런타임, 하드웨어 의존성은 `/opt/cleany/shared`에 저장한다. 각 릴리스가 사용할 버전은 링크로 고정한다. YOLOE 엔진과 인코더는 `/var/opt/cleany/models/yoloe`, 주행 지도는 `/var/opt/cleany/robot/map`에 둔다.

`configs/host/jetson-field.env.example`을 참고해 `/etc/opt/cleany/field/field.env`를 준비한다. 현장에서 확인한 지도 revision, 두 보호 입력, 센서 보정, 베이스와 팔 프로필을 사용한다. `field-targets.json`은 현재 지도에서 검증한 좌석과 Home, `field-collection.json`은 실측한 양팔과 수거함 자세를 담는다. 각 역할은 필요한 파일과 지도 해시를 확인한 뒤 기동한다.

서비스 정의는 후보의 `native/systemd-user`와 `field/systemd-user`에 있다. `cleany-field.target`은 카메라와 인식 노드를 기존 native 서비스로 관측하고 주행, 수거, 미션과 위치 관측 역할을 실행한다. 운영 서비스 계정에서 아래 명령으로 설정을 확인할 수 있다.

```bash
set -a
source /etc/opt/cleany/field/field.env
set +a
/opt/cleany/current/field/repo/tools/ros2/field-service navigation --check
```

기존 로봇의 unit을 이관할 때는 먼저 운영자 권한으로 새 정의를 `/etc/systemd/user`에 등록한다. `switch-jetson-native`와 `switch-jetson-field`는 백업 후 사용자 unit을 전역 정의에 연결한다. Field 이관은 대상 역할이 모두 중지된 상태에서 실행한다. 운영자가 지정한 mask와 drop-in은 별도로 유지한다.
