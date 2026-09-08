# LiDAR Noise Calibration Brief

## 목표

실물 RPLIDAR A1M8의 거리 오차, noise, dropout 특성을 측정하고 동일한 조건의
Gazebo Harmonic LiDAR scan과 비교한다. 측정 결과를 바탕으로 재현 가능한 simulation
noise 설정 또는 ROS `LaserScan` 후처리 방식을 만든다.

## 작업 범위

1. 동일 물체·거리·각도에서 반복 측정할 수 있는 실험 조건과 기록 형식을 정의한다.
2. 실물과 simulation의 `LaserScan`을 같은 분석 코드로 처리한다.
3. 거리·각도별 bias, 분산, 유효 측정 비율, dropout 비율과 scan 주기를 계산한다.
4. Gazebo 기본 scan과 실물 측정 결과의 차이를 비교한다.
5. 측정 근거에 따라 다음 중 최소한의 보정 방식을 적용한다.
   - Gazebo SDF LiDAR noise parameter
   - 설정 가능한 `LaserScan` 후처리 node
6. 고정 설정 또는 random seed로 결과를 재현하고 관련 pytest와 패키지 README를 갱신한다.

## 산출물

- 실험 절차와 입력 데이터 형식
- 실물·simulation 공통 분석 도구
- 버전 관리 가능한 calibration 설정
- 보정 전후 비교 요약
- 관련 단위·구조 테스트와 실행 문서

## 제외 범위

- SLAM 알고리즘 비교 확장
- localization 또는 Nav2 성능 실험 확장
- LiDAR 장착 높이 재선정
- 측정 근거 없는 sensor parameter 확정

## 데이터 관리

- rosbag, 원본 scan dump와 생성된 그래프·결과 디렉터리는 커밋하지 않는다.
- 코드, 작은 설정 파일, 결과 schema와 재현 절차만 저장소에서 관리한다.
- 기존 변경과 기존 SLAM/localization 평가 코드는 보존한다.

## 시작 전 확정할 항목

- 측정 물체와 표면 재질
- 거리·입사각 구간과 반복 횟수
- dropout 및 유효 range 판정 규칙
- 보정 완료를 판단할 비교 지표와 허용 오차
