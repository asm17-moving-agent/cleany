# Cleany 개발 문서

<div class="hero">
<div>
<p class="eyebrow">ROBOT SOFTWARE / ROS 2 HUMBLE</p>
<p class="hero-title">구조를 보고.<br>로봇을 실행하고.<br>상태를 확인하세요.</p>
<p>주행과 매핑, 인식과 조작, MCU 제어, 시뮬레이션과 관측을 연결하는 프로젝트 문서입니다.</p>
<a class="button" href="running.md">실행할 작업 선택 →</a>
<a class="secondary" href="development_setup.md">처음 환경 준비하기</a>
</div>
</div>

## 작업에서 시작하기

<div class="cards">
<a class="card" href="how-to/simulation.md"><span class="card-number">01 / SIMULATE</span><strong>가상 로봇 실행</strong><span>Gazebo 환경과 센서, 주행 확인</span></a>
<a class="card" href="how-to/mapping.md"><span class="card-number">02 / MAP &amp; NAVIGATE</span><strong>지도와 주행</strong><span>LiDAR 지도 생성·저장, 목표 주행</span></a>
<a class="card" href="how-to/sensors.md"><span class="card-number">03 / PERCEIVE</span><strong>센서와 인식</strong><span>RGB-D, LiDAR, IMU와 ToF 관측</span></a>
<a class="card" href="how-to/manipulation.md"><span class="card-number">04 / MANIPULATE</span><strong>관찰·계획·조작</strong><span>물체 선택, MoveIt 계획, Mock·MuJoCo 실행</span></a>
<a class="card" href="how-to/base.md"><span class="card-number">05 / CONTROL</span><strong>구동과 펌웨어</strong><span>MCU 연결, 휠 피드백, STOP 확인</span></a>
<a class="card" href="how-to/monitoring.md"><span class="card-number">06 / OBSERVE</span><strong>기록과 상태 관측</strong><span>Logger, Monitor와 얼굴 디스플레이</span></a>
</div>

## 세부 작업 바로가기

| 필요한 작업 | 안내 |
|---|---|
| 처음 설치·빌드 | [개발환경](development_setup.md) |
| 휴대폰으로 목표 주행 | [화면으로 따라 하기](how-to/phone-navigation.md) |
| 외부 요청과 Mission 연결 | [Mission Runtime](how-to/mission.md) |
| 카메라·헤드·ToF 보정 | [캘리브레이션](calibration.md) |
| 변경한 코드 검증 | [검사 선택](testing.md) |

## 구조에서 찾아보기

| 궁금한 것 | 문서 |
|---|---|
| 명령과 데이터가 어떻게 연결되는가? | [시스템 연결](integration.md) |
| 각 패키지가 무엇을 맡는가? | [기능별 패키지 지도](packages.md) |
| 토픽·서비스·액션의 타입과 방향은? | [ROS 인터페이스](ros_interfaces.md) |
| 로봇 소프트웨어 개념부터 배우려면? | [학습 가이드](../guide/index.html) |
| 제품 범위와 설계 결정은? | [팀 KB](../cleany-docs/README.md) |

실물·시뮬레이션·합성 입력은 각 문서에서 구분합니다. 기능이 존재하거나 자동 검사가 통과한 것과 실물 성능이 검증된 것은 구분해서 확인합니다.
