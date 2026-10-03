# 개발 관제보드 검증

작업 브랜치: `feat/dev-monitor`. 실행 증거는 로컬 `artifacts/dev-monitor/`에 보관한다.
실제 로봇의 주행·조작 검증으로 해석하지 않는다.

## 2026-10-02 기존 관측 파이프라인

- core 및 ROS opt-in 검사: 73 passed (`final-tests.log`).
- colcon 검사: 79 tests, 0 errors, 0 failures, 7 skipped (`colcon-tests.log`).
- mock 미션 성공과 HELD 상태 취소 후 ERROR/주행 차단을 관측했다.
- 격리된 Gazebo에서 약 0.5 m Nav2 이동 후 mock 청소 완료를 관측했다.
  실제 home 복귀와 물리 로봇 청소의 증거는 아니다.
- SQLite export 무결성과 기록 재생을 확인했다 (`export-verification.json`).
- 사용자 종료 요청에 따라 이 세션이 띄운 Gazebo/Nav2와 관측 서버를 종료했다.

## 2026-10-02 shadcn/ui 개편

React/Vite/TypeScript 위에 Tailwind 4와 shadcn/ui 공통 컴포넌트를 적용했다.
공통 토큰, 사이드바, 연결 안내, 작업 요약, 키보드 탭, 반응형 배치를 정리했다.
Foxglove 앱/문서 링크를 추가했다. Bridge 설치 또는 실시간 연결 검증은 포함하지 않는다.

검사:

- TypeScript 검사와 Vite production build.
- ROS 없이 Playwright UI 검사 2개 통과. 실제 ROS/Gazebo 검사 2개는 명시 URL이 없어 건너뜀.
- 오래된 관측에서 주행 준비 상태가 `확인 불가`로 바뀌는지, 재생이 ROS 명령을 보내지 않는지 확인.
- 키보드 ROS 탭 이동, 사이드바 접기, 390 px 화면 가로 넘침 여부 확인.
- 이전 SQLite export의 HELD 취소 시점 재생을 별도 브라우저로 확인: ERROR, CANCELED, 주행 차단.
- Firefox 실화면을 Niri 창 캡처로 확인. ROS/Gazebo를 다시 실행하지 않았다.

브라우저 스크린샷은 `web/test-results/console-desktop.png`, `console-mobile.png`에 생성된다.
프런트엔드 재현 명령과 데이터 범위는 패키지 README를 따른다.

## 2026-10-03 다크 모드

- 상단 테마 전환, 시스템 초기값, localStorage 선택 저장, 다른 탭의 설정 변경 동기화를 추가했다.
- 표·코드 미리보기·상태 배지·FSM/BT/ROS 그래프에 라이트/다크 공통 토큰을 적용했다.
- production build 및 저장된 미션 기록의 5개 화면 캡처 확인.
- 브라우저에서 시스템 다크 초기값, 라이트/다크 전환 후 reload 유지, 모바일 버튼 접근을 확인했다.
- ROS 패키지 재설치는 컨테이너 실행 상태 때문에 수행하지 못했다. 로컬 미리보기는 최신 web/dist를 제공한다.
- 증거: `artifacts/dev-monitor/dark-*.png`, `dark-build.log`, `dark-browser-tests.log`.

## 2026-10-03 실시간 카메라

- 제한된 영상 디코딩, 행 패딩/BGR 순서, 압축 PNG, 미지원·잘못된 영상, lease 한도/만료,
  오래된 프레임 차단, 기록 제외와 ROS best-effort 구독/해제를 검증했다.
- `ROS_DOMAIN_ID=188 CLEANY_RUN_ROS_TESTS=1 make test-dev-monitor`: 10 passed.
- 웹 빌드와 ROS 4개 패키지 빌드 통과. Python lint 검사 통과.
- 격리 domain 188에서 합성 RGB 영상을 ROS Image로 발행하여 HTTP→브라우저 실제 표시와
  프레임 변경, 일시정지/재개를 확인했다. 지연 응답 시 이전 이미지 제거도 확인했다.
- 실물 카메라/자율주행/Gazebo는 실행하지 않았다. 기본 domain 0에는 검증 당시 영상 토픽이 없었다.
- 증거: `artifacts/dev-monitor/camera-{build,install,tests}.log`, `camera-live.png`.

## 2026-10-03 PR 최종 확인

- FSM을 둥근 상태 박스·직각 화살표·전이 조건으로 표시하고 같은 면의 입구/출구를 분리했다.
- 활성 강조는 관측된 상태마다 최소 500ms 표시한다. 실제 상태·안전 정보는 즉시 갱신하며 재생 탐색과 Runtime 재시작 시 표시 대기열을 초기화한다.
- Humble 컨테이너에서 `make test-mission-core`: 60 passed, 6 ROS opt-in skipped.
- 같은 환경에서 `make test-dev-monitor`: 8 passed, 2 ROS opt-in skipped.
- TypeScript/Vite build와 Playwright UI 검사 4개 통과. ROS/Gazebo UI 검사 2개는 명시 URL이 없어 건너뛰었다.
- 실제 제공 화면의 저장 기록 재생으로 연결선·텍스트를 확인했다. Gazebo 및 물리 로봇 동작은 추가 실행하지 않았다.
