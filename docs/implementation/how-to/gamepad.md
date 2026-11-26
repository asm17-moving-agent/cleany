# Jetson에서 게임패드로 수동 조종하기

LeadJoy Xeno Plus의 2.4 GHz 리시버를 Jetson에 연결하고 패드를 Xbox 모드로 페어링합니다. 이 경로는 Jetson의 USB 수신기와 기존 베이스 드라이버를 사용합니다.

## 드라이버와 입력 확인

`lsusb -d 4131:3519`와 `uname -r`로 장치와 커널을 확인합니다. 설치 도구는 `5.15.148-tegra` 헤더와 Module.symvers, make, gcc, curl, patch, kmod를 사용합니다. 커널 버전이 바뀌면 해당 ABI용 빌드와 실물 시험이 필요합니다.

```bash
tools/jetson-xpad build
tools/jetson-xpad load
```

`load`는 모듈을 이번 부팅에 로드합니다. 조종기 ROS 패키지를 빌드하고 두 터미널에서 ROS 환경을 로드합니다. 런타임 의존성은 `python3-evdev`입니다. 노드는 이름이 `LeadJoy Xeno Plus`인 이벤트 장치를 자동으로 찾아 수신기 식별자와 보고서 기능을 검증합니다. 해당 장치에 읽기 권한이 필요합니다. 같은 패드가 여러 대면 `device:=/dev/input/by-path/…-event-joystick`으로 사용할 포트를 지정합니다.

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=104
ros2 launch cleany_teleop gamepad.launch.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=104 ros2 topic echo /joy
```

왼쪽 스틱을 위로 밀면 `axes[1]`, 왼쪽으로 밀면 `axes[0]`가 양수가 됩니다. LB와 RB를 누르면 각각 `buttons[4]`와 `buttons[5]`가 1이 됩니다. 스틱을 일정하게 유지해도 실제 입력 보고서가 계속 들어오는지 확인합니다. LB를 누른 채 패드 전원을 끄거나 리시버를 뺐을 때 입력이 끊기거나 중립으로 바뀌는지 각각 확인합니다. RF 단절 뒤에도 마지막 이동 입력이 계속 수신되면 해당 리시버 설정에서는 실물 구동을 보류합니다.

실물 입력 시험 뒤 `tools/jetson-xpad install`로 USB 자동 로드용 모듈과 udev 규칙을 설치합니다. `plugdev` 그룹은 `/dev/input/cleany-gamepad`를 읽을 수 있습니다. 재부팅·재연결 후 같은 장치로 시험합니다. 입력 노드를 종료한 뒤 `tools/jetson-xpad remove`로 설치를 되돌릴 수 있습니다.

## 수동 조종

동일한 Jetson 사용자로 실행하는 Control Plane의 환경에 다음 값을 지정합니다. 경로는 해당 사용자의 `echo "$XDG_RUNTIME_DIR"` 결과에 맞춥니다.

```ini
CLEANY_GAMEPAD_SOCKET=/run/user/1000/cleany-gamepad.sock
```

변경한 Backend와 `cleany_teleop`를 함께 배포하고 Backend를 재시작합니다. 기존 수동 조종 서버의 토큰과 loopback 연결 설정은 [휴대폰 주행 절차](phone-navigation.md)를 따릅니다. 조종 서버, 베이스 드라이버와 MCU 피드백이 준비된 뒤 입력 확인용 launch를 종료하고 다음을 실행합니다.

```bash
ros2 launch cleany_teleop gamepad.launch.py drive:=true
```

처음에는 왼쪽 스틱과 오른쪽 스틱을 가운데에 둡니다. 왼쪽 스틱으로 전후진과 횡이동을 하고, 오른쪽 스틱을 좌우로 기울여 제자리 회전하거나 이동하면서 회전합니다. 오른쪽으로 기울이면 시계 방향입니다. 왼쪽 스틱과 오른쪽 스틱의 수평 축을 놓으면 정지합니다. 전후진과 회전만 허용된 설정에서는 횡이동 입력을 0으로 보냅니다. 입력이 끊기면 정지하고, 스틱을 민 채 다시 연결하면 새 입력으로 조종권을 얻은 뒤 주행을 재개합니다.

LT를 절반 이상 누르면 병진 속도 상한은 0.1 m/s, RT는 0.5 m/s입니다. 동시에 누르면 LT가 우선하고 두 트리거를 놓으면 기본 속도로 돌아갑니다. 회전 속도는 오른쪽 스틱의 수평 편향으로 정합니다.

오른쪽 스틱을 위아래로 기울이면 헤드 카메라가 움직이고, 놓으면 현재 각도를 유지합니다. 최초 조향 전에 수직 축을 가운데에 둡니다. 조향 노드는 `CLEANY_FIELD_REPO/configs/robot`의 헤드 보정 파일을 읽고 팔과 공유하는 직렬 버스를 단독 사용합니다. 팔 제어기가 이 포트를 사용하는 세션은 `head:=false`로 실행합니다.

LS와 RS를 놓으면 가운데를 지나 양쪽으로 튀는 동안 해당 축의 조종 입력이 0이 됩니다. 다른 축을 계속 밀고 있으면 그 축의 조작은 이어집니다. 가운데에서 안정된 스틱은 바로 조작할 수 있고, 곧바로 반대 방향으로 밀면 새 방향이 60 ms간 유지된 뒤 반영됩니다. `/joy`에서 LS의 조종값은 `axes[0:2]`, 원본은 `axes[6:8]`입니다. RS의 조종값은 `axes[3:5]`, 원본은 `axes[8:10]`입니다.

운전 중 서보 보호 정지가 발생하면 로그에 축 ID, 실측 위치, 상태 코드, 온도가 표시됩니다. 양쪽 서보에서 1초간 정상값을 확인한 뒤 RS를 가운데에 놓고 다시 기울이면 조향이 재개됩니다.

첫 구동은 바퀴를 띄운 상태에서 설정된 저속으로 확인합니다. 스틱과 버튼의 중립 복귀, 패드 전원 끄기, 리시버 분리와 어댑터 종료를 시험하고 MCU의 정지 확인 및 측정 휠 속도를 기록합니다. 입력 만료 설정과 측정한 정지 시간을 구분해 기록합니다.

## Jetson 부팅 시 수동 조종

운영 배포본은 `configs/host/systemd/jetson-gamepad`의 사용자 유닛과
`tools/ros2/run-gamepad-service`를 사용합니다. `python3-evdev`와 현재 커널용 수신기
드라이버를 설치하고, `/etc/opt/cleany/field/gamepad.env`의 `CLEANY_GAMEPAD_MCU`에는
휠 피드백으로 식별한 MCU의 `/dev/serial/by-id/` 경로를 지정합니다. Backend의
`CLEANY_GAMEPAD_SOCKET`에는 같은 사용자의 `$XDG_RUNTIME_DIR/cleany-gamepad.sock`에
해당하는 절대 경로를 설정합니다.

`cleany-gamepad.target`은 USB Agent, 베이스, loopback 조종 서버, 패드 입력과 중계를
관리합니다. 필드 주행 서비스와 수동 전용 타깃은 베이스를 단독으로 사용합니다.
수동 운용 시 타깃을 enable하고 패드가 중립인 상태에서 start합니다. 운용을 마치면
`systemctl --user stop cleany-gamepad.target`으로 종료합니다. `/joy` 입력과
`/wheel/odom` 수신, 조종 상태의 MCU 연결과 정지 피드백을 각각 확인합니다.

운영 호스트에서는 검증된 커널 모듈 경로를 인자로
`tools/ros2/install-gamepad-services /path/to/cleany_xpad.ko`를 실행합니다.
이 명령은 부팅 설정만 등록합니다. 커널 ABI가 바뀌면 새 모듈을 빌드해 다시 설치합니다.
