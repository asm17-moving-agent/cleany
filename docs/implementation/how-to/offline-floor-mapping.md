# 19층 LiDAR와 RGB-D 기록 재처리

노트북에서 기록된 bag을 읽어 궤적과 지도를 다시 계산합니다. `bag`은 원본 rosbag2 폴더, `session`은 그 주행의 설정 폴더, `run`은 결과를 저장할 새 절대경로입니다. 원본 카메라 영상과 생성한 지도는 실행별 데이터 폴더에 보관합니다.

## LiDAR 지도

ROS 2 Humble의 Cartographer, Nav2 Map Server, rosbag2 SQLite 및 zstd 플러그인을 사용합니다. 새 bag에는 스캔, odometry, 정적 TF와 기록된 `odom → base_link`를 복사합니다. Cartographer가 `map → odom`과 submap을 다시 계산합니다.

```bash
source /opt/ros/humble/setup.bash
mkdir -p "$run/input/config"
cp "$session/config/"cartographer{,_hardware}_2d.lua "$run/input/config/"
python3 tools/validation/offline_mapping/extract_bag.py "$bag" "$run/input/navigation"
tools/validation/offline_mapping/replay.sh "$run/input" "$run/baseline" 1.0 189
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=190 python3 tools/validation/offline_mapping/verify_export.py "$run/baseline"
```

`1.0`은 제약 탐색 거리(m)입니다. 다른 거리를 비교할 때는 새 출력 폴더를 사용합니다. 검증기는 pbstream을 다시 렌더링하고 Nav2가 발행한 전체 OccupancyGrid를 PGM과 대조합니다. `map.pgm`, `map.yaml`, `map.pbstream`을 함께 보관합니다.

## 카메라 궤적

RTAB-Map core, OpenCV, Eigen, PCL 개발 헤더와 Python의 NumPy, SciPy, PyYAML이 필요합니다. 시스템 외부에 RTAB-Map을 설치했다면 `RTABMAP_PREFIX`와 동적 로더의 라이브러리 경로를 해당 설치에 맞춥니다. 다음 명령은 원본 bag의 모든 분할에서 RGB와 정렬된 depth 및 좌우 IR을 타임스탬프로 짝짓습니다.

```bash
visual=tools/validation/visual_mapping
mkdir -p "$run/input"
cp "$bag/metadata.yaml" "$run/input/source-metadata.yaml"
"$visual/build.sh" "$run/visual_slam"
splits=$(python3 -c 'import sys,yaml; print(len(yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]["files"]))' "$bag/metadata.yaml")
set -o pipefail
for ((i=0; i<splits; i++)); do
    python3 "$visual/extract_frames.py" "$bag" "$i" | tar -xf - -C "$run/input" || break
done
python3 "$visual/run_visual.py" "$run" "$bag/metadata.yaml" "$run/visual_slam" rgbd
"$run/visual_slam" --components "$run/results/rgbd"
```

`rgbd`는 컬러와 depth, `stereo`, `stereo-filtered`, `stereo-depth`는 좌우 IR로 처리합니다. 각 방식은 자체 데이터베이스, 추적 기록과 궤적을 `results/<방식>/`에 저장합니다.

### 11월 22일 고정 헤드 주행의 벽 비교

이 비교에는 05:56 주행에서 지정한 복도 벽 구간과 같은 주행의 보류 스캔이 필요합니다. 다른 장소의 오차 지표로 사용할 수 없습니다.

1. 네 가지 카메라 방식을 각각 실행하고 `--components`로 궤적을 내보냅니다.
2. 현장 PGM/YAML과 궤적 `field.csv`, 매 여덟 번째 스캔을 제외해 재계산한 궤적 `training.csv`를 `input/lidar/`에 둡니다. `geometry.py <navigation DB3> <run>/input/lidar/scans.npz`로 스캔을 추출합니다.
3. `results/fusion/`을 만들고 `match_visits.py <run> <특징점 캐시>`, `fuse_map.py <run>`, `depth_walls.py <run>`, `reconstruct.py <run>`, `render_report.py <run>`, `summarize.py <run>`을 차례로 실행합니다. 결과는 `results/index.html`과 `results/fusion/fused/map.yaml`에서 확인합니다.
