# 변경을 검증하기

수정한 경계에 맞는 검사를 고르고, 자동 검사와 시뮬레이션, 실물 결과를 구분합니다.

```mermaid
flowchart TD
  Change{"변경한 경계"}
  Change -->|"core 로직"| Unit["패키지 단위 검사"]
  Change -->|"ROS 인터페이스"| ROS["빌드 + ROS 연동 검사"]
  Change -->|"가상 센서·제어"| Sim["시뮬레이션 시나리오"]
  Change -->|"실물 설정·구동"| Real["장치와 실물 시험"]
```

## 먼저 고를 검사

| 바꾼 것 | 첫 검사 | 추가로 확인할 것 |
|---|---|---|
| 순수 Python 로직 | 해당 패키지 pytest | 입력 오류, 상태 전이와 실패 경로 |
| ROS 타입·launch·설치 데이터 | 관련 package build/test | 실제 topic·service 연결 |
| 여러 패키지 연결 | 합성 ROS runtime | lifecycle, 취소, 재연결 |
| 주행·인식·조작 | 관련 시뮬레이터 runtime | 실제 장치와 주행·정지는 별도 시험 |
| UI | 실제 제공 페이지의 브라우저 검사 | 갱신, 재접속, 모바일 레이아웃 |
| 문서 | 링크·명령 구문·생성 파일 검사 | 아래 문서 검증 명령 |

## 자주 쓰는 명령

| 범위 | 명령 |
|---|---|
| 전체 ROS workspace | `make test` |
| Mission core / Supervisor core | `make test-mission-core`, `make test-supervision-core` |
| Navigation / Gazebo | `make test-navigation`, `make test-gazebo` |
| ROS Navigation runtime | `make test-navigation-runtime` |
| 조작 Mock / MuJoCo BT | `make test-manipulation`, `make test-manipulation-bt` |
| Base / ESP32 | `make test-base`, `make test-motor-core` |
| 매핑 / 슬립 분석 | `make test-hardware-mapping-core`, `make test-slip-detection` |
| Logger / Monitor | `make test-logger`, `make test-dev-monitor` |
| Telemetry 카메라 | `make test-camera-preview-core`, ROS 환경의 `make test-telemetry` |

환경과 선택 실행 조건, 전체 명령표는 [상세 참조](reference/testing.md#검사-선택)를 따릅니다. 장치가 없는 검사와 실물 실행 검사는 같은 성공으로 합산하지 않습니다.

## 문서 사이트 검사

```bash
python3 tools/docs/build_site.py --check
python3 docs/guide/build_standalone.py --check
git diff --check
```

브라우저에서는 데스크톱과 모바일에서 메뉴, 검색, 도표 확대와 내부 링크를 확인합니다.
설정 기반 그림은 [재생성 검사](../../tools/README.md#문서-그림-재생성)로 입력 해시와 PNG 일치를 확인합니다.
Chromium의 로컬 CDP 포트가 열려 있고 `websockets`가 설치돼 있다면 다음 검사는 구현 문서와 학습 가이드의 모든 페이지를 1440 px / 390 px로 엽니다. 보고서와 그림별 화면은 사용자 캐시에 저장합니다.

```bash
python3 tools/docs/check_reader.py --endpoint http://localhost:9228
```

이 검사는 도식 렌더링, 이미지 로드, 문서 가로 넘침과 확대 동작을 확인합니다. 이미지가 본문의 질문에 맞는지는 별도로 검토합니다.

## 상세 조건 찾기

| 확인할 것 | 참조 |
|---|---|
| 전체 Make target와 준비 환경 | [검사 선택](reference/testing.md#검사-선택) |
| CI 이미지와 캐시 | [CI 실행](reference/testing.md#ci-실행과-재사용) |
| 명시 실행이 필요한 ROS / UI | [선택 runtime](reference/testing.md#선택-runtime과-ui) |
| 실물 매핑 / ToF / 화면 / 모델 | [기능별 조건](reference/testing.md#실물-매핑-패키지-경계) |
| 기록 분석과 주행 모델 비교 | [오프라인 분석](reference/testing.md#기록-분석-도구) |
| 결과의 해석 | [결과와 증거](reference/testing.md#결과와-증거) |

### 위치 관측과 Backend adapter 분리 검증

`make build-pose-observation`으로 telemetry, control bridge와 bringup을 빌드한 뒤 해당
install의 `setup.bash`를 적용합니다. `make test-pose-observation`은 원본 시각과 좌표계,
오래된 입력, TF와 odom 관측, Backend payload와 인증 파일 갱신을 확인합니다. ROS 테스트는
합성 입력을 사용하므로 실물 위치 정확도를 증명하지 않습니다.

`tools/validation/telemetry_pose_pipeline.py`는 별도 ROS domain에서 메모리 DB를 사용하는
실제 Backend 서버와 합성 위치 입력을 연결합니다. 관측 유지, 인증 전송, 입력 만료와
Backend 재연결을 확인하며 운영 DB와 주행 명령은 사용하지 않습니다.

```bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
python3 tools/validation/telemetry_pose_pipeline.py \
  --backend-root "$CLEANY_CONTROL_PLANE_ROOT" \
  --backend-python "$CLEANY_CONTROL_PLANE_ROOT/apps/backend/.venv/bin/python" \
  --output "$CLEANY_DATA_ROOT/runs/platform/$(date +%F)_telemetry-pose-separation"
```
