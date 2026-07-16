# models/

Stable location for YOLO weights so the file path doesn't depend on where a
node was launched.

By default `detection_node` uses `weights: yolo11n-seg.pt` (detection +
instance masks), which ultralytics auto-downloads into the current working
directory on first run. Plain detect weights (e.g. `yolo11n.pt`) also work;
detections then carry no mask. To pin an exact file and a fixed location, put
the weights here and point the parameter at them:

```yaml
# config/detection.yaml
detection_node:
  ros__parameters:
    weights: <repo>/ros2_ws/src/cleany_perception/models/yolo11n-seg.pt
```

Weights are not checked in (see `.gitignore`); download or copy them per host.
