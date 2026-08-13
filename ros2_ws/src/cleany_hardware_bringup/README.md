# cleany_hardware_bringup

Jetson Orin NX에서 D435와 Cleany perception을 native ROS 2 Humble로 실행한다. 이
bringup은 handheld 검증용이므로 임시 `base_link`를 만들지 않고 aligned-depth의 실제
`camera_color_optical_frame`을 perception target frame으로 사용한다.

## 카메라-only 실행

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_hardware_bringup jetson_rgbd.launch.py
```

기본 설정은 color/depth `640x480x15`, exact sync, color-aligned depth와 point cloud를
활성화한다. RealSense ROS 4.58.3 aarch64 빌드의 NEON pointcloud parameter 이름 차이는
`config/jetson_d435.yaml`에서 처리하므로 별도 `ros2 param set`은 필요하지 않다.

```bash
ros2 param get /camera/camera pointcloud__neon_.enable
ros2 topic list | grep points
```

원시 point cloud의 frame은 `camera_depth_optical_frame`이며 topic은
`/camera/camera/depth/color/points`다. perception 입력에는 아래 color optical frame의
aligned 네 topic을 사용한다.

- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/aligned_depth_to_color/camera_info`

## Perception 함께 실행

Gemini runtime 준비가 끝난 후에만 perception을 활성화한다. detector-only 검증에는
point cloud가 필요하지 않으므로 Jetson 부하를 줄이기 위해 비활성화한다.

```bash
cd /home/cleany/cleany
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_hardware_bringup jetson_rgbd.launch.py \
  start_perception:=true \
  enable_pointcloud:=false
```

실물 기본 질의는 사람과 신체 부위를 제외하고 화면에 명확히 보이는 모든 물체를
검출한다. 다른 terminal에서 환경을 source하고 빈 `query`를 보내 기본 질의를 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/cleany/cleany/ros2_ws/install/setup.bash
ros2 action send_goal \
  /perception/inspect_scene \
  cleany_interfaces/action/InspectScene \
  "{query: '', snapshot_id: '', selected_object_id: 0}" \
  --feedback
```

`success: true`는 요청 처리가 정상 종료됐다는 뜻이다. 실제 검출 성공은
`detections`가 비어 있지 않고 feedback의 `detections_2d`가 1 이상인지로 판단한다.
1단계에서는 아직 선택 객체를 SAM2/3D로 처리하지 않으므로 `objects`, `target_cloud`,
`context_cloud`가 비어 있는 것이 정상이다.

bbox가 표시된 최신 RGB 이미지는 action을 한 번 실행한 다음 transient-local debug
topic에서 확인한다.

```bash
ros2 run rqt_image_view rqt_image_view
```

Image View에서 `/perception/debug_image_latched`를 선택한다. topic이 목록에 보이지 않으면
action을 먼저 실행하고 목록을 새로고침한다. Jetson GUI 부하가 크면 action 결과의 bbox
좌표만 사용한다.

SAM2 checkpoint가 준비되면 동일 launch에 `sam2_model_config`, `sam2_checkpoint`,
`sam2_device` 인자를 전달한다. API key와 모델 weight는 저장소에 기록하지 않는다.
