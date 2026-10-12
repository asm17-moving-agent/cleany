# Hand-eye 좌표와 데이터 계약

- 절차: [CALIBRATION](../../../../docs/CALIBRATION.md)
- 모델·상수·허용 한계 원본: config와 core 코드

## 좌표·동기화

- parent_T_child: child → parent 좌표 변환
- 식: `base_T_gripper @ gripper_T_camera @ camera_T_target = base_T_target`
- 단위: m·rad·ns. Quaternion: xyzw
- 회전: orthogonal·det +1, 오차 1e-6 이내
- Image/CameraInfo: 같은 nonzero stamp·optical frame
- Fixture: left_wrist_rgb_optical_frame, RGB8 640×480, step 1920
- Fixture camera: plumb_bob D=0, R=I, fx=fy=227.751496, cx=319.5, cy=239.5
- 다른 입력을 fixture calibration으로 덮어쓰기 금지
- FK: canonical 12관절 feedback. Position IK: 팔 5관절 검사
- Capture: settle 이후 stamp. 대기 timeout: monotonic
- Position-only IK: orientation 도달 보장 없음

## Board·PnP

- Fixture: ChArUco 7×5, square 30 mm, marker 15 mm, DICT_5X5_100, legacy=false
- Corner: 24개 중 최소 16개 + 4 quadrant coverage
- 평면 PnP: IPPE; IPPE_SQUARE 제외
- Positive-depth 후보 → VVS refine → ambiguity 검사
- 상위 두 오차 비 <1.05: ambiguous. 영 오차 특수 기준: 코드의 1e-12
- 유효하지 않은 관측: 성공 샘플 처리 금지

## Dataset·평가

- Profile 고정: pose/runtime/URDF·camera K/D/R/P·source hash
- 구성: calibration 20 + held-out 5. 실패 표본 교체 금지
- 보존: random seed·탐색 제한·선정 근거·row/image hash·capture provenance
- 수집: 요청 → FK → IK → state validity → motion → settle → acquire → PnP → record
- 저장: PNG fsync → journal → JSONL atomic rename
- 부분 쓰기 복구: image 없는 row를 유효 처리 금지
- 기존 dataset 일부 재사용: fresh run으로 표시 금지
- Solver: Tsai·Park·Horaud·Andreff·Daniilidis, method별 예외 분리
- GT 평가: simulation 한정. Held-out·solver 실패 보존
- Offset·품질 gate: 명시한 실행 정책 사용
- 결과 생성: 자동 TF 게시·실물 calibration 완료 아님
