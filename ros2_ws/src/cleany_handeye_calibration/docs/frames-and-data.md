# Hand-eye 좌표와 데이터 계약

실행 절차는 [CALIBRATION](../../../../docs/CALIBRATION.md)이다.
이 문서는 계산과 재현에 필요한 불변 조건만 관리한다. 모델·상수·허용 한계 원본은 config와 core 코드다.

## 좌표와 동기화

`parent_T_child`는 child 좌표를 parent로 변환한다.
`base_T_gripper @ gripper_T_camera @ camera_T_target = base_T_target`이다.
단위는 m·rad·ns, quaternion은 xyzw다. 회전 행렬은 orthogonal 및 det +1을 1e-6 오차 이내로 검사한다.
Image/CameraInfo는 같은 nonzero stamp·optical frame을 가져야 한다.
현재 fixture는 left_wrist_rgb_optical_frame, RGB8 640×480, step 1920,
plumb_bob D=0, R=I, fx=fy=227.751496, cx=319.5, cy=239.5다.
다른 calibration 입력을 이 fixture 값으로 덮어쓰지 않는다.

12관절 canonical feedback으로 FK를 계산하고 위치 IK의 팔 5관절을 검증한다.
수집 이미지는 settle 이후 stamp여야 한다. 대기 timeout은 monotonic 기준이다.
현재 position-only IK는 orientation 도달을 보장하지 않는다.

## Board와 PnP

현재 fixture는 ChArUco 7×5, square 30 mm, marker 15 mm, DICT_5X5_100, legacy=false다.
24 inner corner 중 최소 16개와 4 quadrant coverage가 필요하다.
평면 PnP는 IPPE를 사용한다. IPPE_SQUARE는 사용하지 않는다.
Positive-depth 후보를 VVS refine하고 ambiguity를 검사한다.
상위 두 후보의 오차 비가 1.05 미만이면 ambiguous로 처리한다(영 오차 특수 기준은 코드의 1e-12).
유효하지 않은 관측을 성공 샘플로 바꾸지 않는다.

## Dataset과 결과

Materialized profile은 pose/runtime/URDF·camera K/D/R/P·source hash를 함께 고정한다.
기본 구성은 calibration 20개와 held-out 5개이며 실패 샘플을 몰래 교체하지 않는다.
Random seed와 후보 탐색 제한·선정 근거를 보존한다.
수집은 요청 → FK → IK → state validity → motion → settle → acquire → PnP → record 순서다.
PNG fsync → journal → JSONL atomic rename 순으로 기록한다. 불완전 쓰기 복구 시 없는 image를 유효 row로 만들지 않는다.
각 row/image hash와 capture provenance를 보존하며 dataset 일부 재사용을 fresh run으로 표시하지 않는다.

Tsai·Park·Horaud·Andreff·Daniilidis solver를 평가한다. 한 method의 예외는 분리해 보고한다.
Ground truth 평가는 시뮬레이션 한정이다. Held-out와 solver 실패를 결과에 남긴다.
최대 offset과 품질 gate는 실행에 명시한 정책을 따른다.
결과를 실행했다고 TF를 자동 게시하거나 실물 calibration이 완료됐다고 판단하지 않는다.
