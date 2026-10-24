# Nav2 튜닝 복구 — 2026-10-24

이 워크트리는 삭제된 실험 코드와 설정을 로컬/원격 백업에서 수집한 복구본이다.
기준 커밋은 실험 당시의 `b3ca816ba420452ee7960837bc16ec74148e07b0`이며,
현재 main의 후속 개발을 되돌리거나 그 위에 구버전 코드를 덮어쓰지 않았다.

## 바로 볼 곳

| 대상 | 경로 |
|---|---|
| 마지막 검증 MPPI 구현 (`candidate_diverse`) | `ros2_ws/src/nav2_mppi_controller/` |
| DWB 튜닝 구현과 회귀 테스트 (`44c083a`) | `ros2_ws/src/cleany_navigation/` |
| 실험 계측용 TimedController 플러그인 | `ros2_ws/src/cleany_benchmark/` |
| 마지막 MPPI + Lattice 10cm 실험 설정 | `recovered/nav2/profiles/` |
| MPPI 이전 후보, TEB/RPP 및 전체 실험 설정 이력 | `recovered/nav2/snapshots/` |
| 원본별 SHA-256과 복원 검증 | `recovered/nav2/provenance/` |
| 기존 보고서·CSV·GIF | `recovered/nav2/results/` (기존 데이터로 연결) |

세부 색인은 [recovered/nav2/README.md](recovered/nav2/README.md)에 있다.

## 복원 범위와 상태

- 원격 2026-10-17 백업에서 소스/설정/분석 도구 12,137개 파일을 선택 추출하고
  원본 manifest의 SHA-256과 대조했다. 불일치 0개.
- 로컬 실험 디렉터리에서 9,523개 파일을 복사했다. 권한이 제한된 컨테이너 생성 파일도
  사용자 namespace에서 읽었으며 원본 권한/내용을 바꾸지 않았다.
- 따로 살아 있던 초기 MPPI 실험 코드와 미커밋 변경 patch도 보존했다.
- 2026-10-16 navigation 변경은 저장된 before 파일과 patch로 복원하고,
  7개 after SHA-256을 확인했다. launch/axis 설정은 기록된 2026-10-14 실행본에서 복원했다.
- 이 복구본이 삭제 직전 폴더의 모든 미커밋 파일과 완전히 동일하다는 보장은 없다.
  확인 가능한 출처와 버전을 보존한 것이다.

현재 navigation의 기본 설정은 복원된 10월 16일 Theta* + MPPI 소스 상태다.
마지막 비교에 사용한 **MPPI + Lattice 공식 10cm 프리미티브 설정은 별도 프로파일**로
보존했다. 복구를 이유로 시뮬레이션 안전 우회나 30cm 도착 허용오차를 로봇 전역 기본값으로
적용하지 않았다. TEB와 RPP의 서로 다른 구현은 원본 스냅샷에 각각 보존했다.

## 검증과 재사용

`recovered/nav2/provenance/validation.json`에서 이번 복구 검증 결과를 확인한다.
실험 runner의 원격 절대경로와 설치 경로는 원본 그대로이므로 실행 전 이 컴퓨터에 맞게
이전해야 한다. 소스 수집 완료는 Gazebo/실물 재검증 완료를 뜻하지 않는다.
원격 x86 바이너리는 이 ARM64 노트북에 설치하지 않았으며 소스에서 빌드한다.

복구 코드/설정은 이 브랜치에 로컬 커밋으로 보존한다. 원격 push는 하지 않는다.
