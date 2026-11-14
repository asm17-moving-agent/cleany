# 개발 도구

`tools/ros2/ros2-clean`은 해당 worktree의 ROS 빌드 출력을 정리합니다. `make clean` 실행 전에 삭제 대상 경로를 확인합니다. 홈, 저장소, 데이터 루트와 그 상위 경로는 삭제를 거부합니다.

| 작업 | 위치, 명령 |
|---|---|
| 시설 생성 | [Gazebo scripts](../ros2_ws/src/cleany_gazebo_sim/scripts/) |
| micro-ROS 준비, 검사 | [ESP32 scripts](../esp32/scripts/), `esp32/host_tests/` |
| Git wrapper 링크 갱신 | `make firmware-setup` |
| Jetson USB 카메라 초기화 설치, 제거 | `tools/camera/install.py`, `make camera-host-install`, `make camera-host-uninstall` |
| 실제 MuJoCo/Gazebo 카메라 영상 수집 | [capture_cameras.py](validation/capture_cameras.py), [실행 방법](../docs/TESTING.md#카메라-영상-수집) |

실행과 검사 절차는 [테스트](../docs/implementation/testing.md), [ESP32](../esp32/README.md) 가이드를 따릅니다.

## 조립 CAD 검사

[cad_inventory.py](models/cad_inventory.py)는 `CleanyAssembly`의 저장된 BREP 형상과 부품별 경계, 배치, 해시를 읽습니다. 매크로를 실행하거나 CAD를 수정하지 않습니다. 별도 Python 환경에 `cadquery-ocp`가 필요합니다.

```bash
python3 tools/models/cad_inventory.py /path/to/Cleany-final.FCStd \
  --output "$HOME/.cache/cleany/cad-inventory.json"
```

결과 단위는 mm입니다. `bounds_mm`은 저장된 BREP의 경계이며, 상위 `assembly_placements`를 별도로 확인해야 합니다. ROS의 `base_link` 좌표나 관절 원점으로 바로 해석하지 않습니다.

## 문서 그림 재생성

[render_contract_figures.py](docs/render_contract_figures.py)는 현재 설정, 메시와 기존 테스트 fixture에서 문서용 PNG를 만듭니다. 로봇이나 시뮬레이터를 실행하지 않습니다. `matplotlib`, `numpy`, `scipy`, `PyYAML`, `pytest`, `opencv-python-headless`가 있는 별도 Python 환경에서 실행합니다. 사용한 버전과 입력 SHA-256은 [그림 출처](../docs/assets/visuals/generated.json)에 기록합니다.

```bash
python3 tools/docs/render_contract_figures.py
python3 tools/docs/render_contract_figures.py --check
python3 tools/docs/build_site.py
python3 tools/docs/build_site.py --check
```

`--check`는 PNG와 입력 해시를 다시 계산해 비교합니다. 렌더러 버전이 달라지면 그림을 검토한 뒤 다시 생성합니다. 원본 사진과 GIF는 재생성 대상이 아니며 [별도 출처](../docs/assets/visuals/imported.json)로 관리합니다.
