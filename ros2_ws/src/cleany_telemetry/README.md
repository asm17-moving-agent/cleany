# cleany_telemetry

Subscribes to `nav_msgs/Odometry` (default `/odom`) and sends only the latest
finite `x`/`y` as `{"x":...,"y":...}` over a WebSocket. ROS callbacks never
perform network I/O; a wall-clock worker sends at most the configured rate.
Expired input is not repeated, and simulation-clock resets invalidate the cache.
The odometry header timestamp is used to detect a reset: repeated timestamps
(including timestamp zero) do not refresh an expired sample, so a paused
simulator cannot keep a pose alive. A source that does not provide meaningful
timestamps should omit the timestamp when using the pure relay core.

Coordinates pass through unchanged in meters. The current Gazebo `/odom` uses
D-HUB world ground truth, matching the dashboard map. This node does not transform
frames: a real robot's local odometry must not be assumed to share that origin.

From the repository root, install the standard rosdep dependency
`python3-websocket` (`websocket-client`), then build and source the workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src/cleany_telemetry --ignore-src -r -y
make build-telemetry
source ros2_ws/install/setup.bash
ros2 run cleany_telemetry telemetry_node
ros2 run cleany_telemetry telemetry_node --ros-args \
  -p odom_topic:=/ground_truth/odom \
  -p url:=ws://127.0.0.1:8080/api/robots/cleany-01/pose/ws \
  -p rate:=5.0 -p input_timeout:=1.5 \
  -p reconnect_initial:=1.0 -p reconnect_max:=30.0
```

| Parameter | Default |
| --- | --- |
| `odom_topic` | `/odom` |
| `url` | `ws://127.0.0.1:8080/api/robots/cleany-01/pose/ws` |
| `rate` | `5.0` Hz |
| `input_timeout` | `1.5` s |
| `reconnect_initial` | `1.0` s |
| `reconnect_max` | `30.0` s |

The subscription uses `BEST_EFFORT`, `KEEP_LAST`, depth `1`, to accept
sensor-data odometry without retaining historical samples. The trusted local
endpoint above is the control-plane pose WebSocket; the relay does not issue
commands. The snapshot endpoint (`GET
/api/robots/cleany-01/pose`) is a backend inspection API, not the relay URL.
The backend and dashboard are in `cleany-control-plane`; run its backend on
port 8080 and open the dashboard after building it. No authentication is
implemented for this prototype: do not expose its producer endpoint publicly.
Pause/input loss expires after `input_timeout`; the backend's additional receive
timeout then marks the position stale. A backend restart reconnects automatically.

Package-only checks:

```bash
make build-telemetry
make test-telemetry
```
