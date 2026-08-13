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

Gemini runtime 준비가 끝난 후에만 perception을 활성화한다.

```bash
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_hardware_bringup jetson_rgbd.launch.py \
  start_perception:=true
```

SAM2 checkpoint가 준비되면 동일 launch에 `sam2_model_config`, `sam2_checkpoint`,
`sam2_device` 인자를 전달한다. API key와 모델 weight는 저장소에 기록하지 않는다.
