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
