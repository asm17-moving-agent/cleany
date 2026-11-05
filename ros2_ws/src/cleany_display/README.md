# Cleany Jetson 디스플레이

Jetson의 로컬 브라우저에서 얼굴과 안내 문구를 전체 화면으로 표시한다. 로봇 상태는 ROS 2 Humble로 구독하며 HTML, CSS, JavaScript는 설치 폴더에서 제공한다. 인터넷 연결은 필요하지 않다. 상세 구독과 화면 우선순위는 [인터페이스](docs/interfaces.md)를 따른다.

## 독립 설치

배포 폴더는 `/home/cleany/cleany-display`다. `cleany_display`와 사용하는 로봇 버전의 `cleany_interfaces` 사본을 이 폴더에서 독립적으로 빌드한다. 기존 로봇 checkout과 build/install은 수정하지 않는다.

[개발환경 기준](../../../docs/DEVELOPMENT_SETUP.md#jetson-얼굴-디스플레이-환경)의 의존성과 그래픽 자동 로그인 세션이 필요하다. 이 패키지는 시스템 로그인 설정을 수정하지 않는다.

1. Jetson에 준비한 이 구현 레포의 루트에서 배포 파일을 복사한다. 기본 설치 위치는 현재 사용자의 `~/cleany-display`다.

   ```bash
   display_root="$HOME/cleany-display"
   mkdir -p "$display_root/ros2_ws/src" "$display_root/config"
   cp -R ros2_ws/src/cleany_display ros2_ws/src/cleany_interfaces "$display_root/ros2_ws/src/"
   cp -R ros2_ws/src/cleany_display/scripts "$display_root/"
   cp ros2_ws/src/cleany_display/scripts/standalone.Makefile "$display_root/Makefile"
   cp ros2_ws/src/cleany_display/config/display.json "$display_root/config/"
   ```

2. 설치 폴더의 `config/display.json`에 실제 토픽 이름을 지정한다. `runtime.env`가 필요하면 `ROS_DOMAIN_ID=0`처럼 실제 로봇과 같은 DDS 환경을 넣는다. 별도 지정이 없으면 ROS 기본 domain 0을 사용한다.

3. 독립 workspace와 브라우저를 설치하고 사용자 서비스를 시작한다. 그래픽 로그인한 Jetson 터미널에서 실행한다.

   ```bash
   cd "$HOME/cleany-display"
   make build
   scripts/install-browser.sh
   scripts/install-autostart.sh
   ```

설치기는 다음 사용자 설정만 추가한다.

- `~/.config/systemd/user/cleany-display.service`: ROS 구독 및 HTTP, 사용자 세션 시작 시 자동 실행
- `~/.config/systemd/user/cleany-display-kiosk.service`: 전용 Chromium 프로필로 실행, 종료 시 재시작
- `~/.config/autostart/cleany-display.desktop`: 그래픽 로그인 시 화면 환경을 전달하고 키오스크 시작

자동 로그인 후 전체 화면이 켜진다. 브라우저 설치 경로는 `browser.path`, Chromium 프로필과 캐시는 설치 폴더의 `browser-profile/`, `browser-config/`, `browser-cache/`에 둔다. 브라우저 설치에만 다운로드가 필요하고 이후 화면 실행에는 인터넷이 필요하지 않다. 배포본 갱신 시 `install-browser.sh`의 Playwright 버전을 바꾸고 재설치한 뒤 키오스크를 재시작한다. 키오스크 실행 중에는 GNOME의 idle 동작을 억제한다. HDMI를 연결하면 데스크톱이 선택한 해상도에 맞춰 화면이 다시 배치된다.

전용 브라우저는 [NVIDIA 안내](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/Graphics/GraphicsProgramming/OpenglEsProgrammingTips.html#graphics-driver-cpu-usage)에 따라 `__GL_YIELD=USLEEP`을 적용해 드라이버 대기 중 CPU 점유를 줄인다. 다른 값이 필요하면 `runtime.env`에서 지정한다.

## 운영

```bash
systemctl --user status cleany-display cleany-display-kiosk
journalctl _SYSTEMD_USER_UNIT=cleany-display.service _SYSTEMD_USER_UNIT=cleany-display-kiosk.service -n 50
curl -s http://127.0.0.1:8770/api/state
```

설정 변경 후에는 `systemctl --user restart cleany-display`를 실행한다. HTTP 주소나 포트를 바꾸면 `runtime.env`의 `CLEANY_DISPLAY_URL`도 맞추고 키오스크를 재시작한다. 웹 자산을 갱신할 때는 독립 workspace의 소스를 교체하고 설치 루트에서 `make build` 후 두 서비스를 재시작한다. 화면 상태 입력 예시는 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
source /home/cleany/cleany-display/ros2_ws/install/local_setup.bash
ros2 topic pub --once /cleany/display/state std_msgs/msg/String \
  '{data: "{\"mode\":\"charging\",\"duration_seconds\":5}"}'
```

화면을 끄고 자동 시작을 해제하려면 다음 명령을 사용한다.

```bash
systemctl --user stop cleany-display-kiosk
systemctl --user disable --now cleany-display
mv ~/.config/autostart/cleany-display.desktop /home/cleany/cleany-display/cleany-display.desktop.disabled
```

검증 방법은 [테스트 안내](../../../docs/TESTING.md#얼굴-디스플레이)를 따른다.
