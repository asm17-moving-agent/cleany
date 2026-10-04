"""Bounded navigation observations and metrics. No driving or safety authority."""

from collections import deque
from dataclasses import dataclass
import math
import struct
from typing import Any


def vector(message: Any) -> list[float]:
    value = getattr(message, "twist", message)
    value = getattr(value, "twist", value)
    result = [float(value.linear.x), float(value.linear.y), float(value.angular.z)]
    if not all(math.isfinite(v) for v in result):
        raise ValueError("Non-finite velocity")
    return result


def cloud_points(message: Any, limit: int) -> tuple[list[list[float]], int]:
    """Read a bounded sample directly, including organized clouds and endian order."""
    count = message.width * message.height
    if (message.point_step <= 0 or message.row_step < message.width * message.point_step
            or len(message.data) < message.row_step * message.height):
        raise ValueError("Invalid cloud layout")
    fields = {f.name: f for f in message.fields}
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if not field or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError("Cloud requires scalar float x/y/z")
        if field.offset < 0 or field.offset + (4 if field.datatype == 7 else 8) > message.point_step:
            raise ValueError("Cloud field outside point")
    result = []
    stride = max(1, math.ceil(count / limit))
    order = ">" if message.is_bigendian else "<"
    for index in range(0, count, stride):
        row, column = divmod(index, message.width)
        base = row * message.row_step + column * message.point_step
        point = [struct.unpack_from(order + ("f" if fields[n].datatype == 7 else "d"),
                                    message.data, base + fields[n].offset)[0]
                 for n in ("x", "y", "z")]
        if all(math.isfinite(v) for v in point):
            result.append(point)
    return result, count


def transform_xyz(points: list[list[float]], translation: list[float], quaternion: list[float]) -> list[list[float]]:
    """Keep pitch/roll and height when projecting sensor surfaces into map XY."""
    if not all(math.isfinite(v) for v in translation + quaternion):
        raise ValueError("Non-finite transform")
    length = math.sqrt(sum(v*v for v in quaternion))
    if length < 1e-9:
        raise ValueError("Invalid transform quaternion")
    x, y, z, w = [v/length for v in quaternion]
    matrix = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    return [[translation[i]+sum(matrix[i][j]*point[j] for j in range(3)) for i in range(3)]
            for point in points if all(math.isfinite(v) for v in point)]


@dataclass
class Observation:
    data: dict
    wall: float
    monotonic: float
    stamp: float | None


class NavigationObserver:
    """Metrics use fresh measured motion, never commanded motion as ground truth."""

    def __init__(self, velocity_topics: list[str], freshness: float = .75,
                 stationary_speed: float = .005, creep_speed: float = .03, map_frame: str = "map"):
        self.velocity_topics = velocity_topics
        self.freshness = freshness
        self.stationary_speed = stationary_speed
        self.creep_speed = creep_speed
        self.map_frame = map_frame
        self.observations: dict[str, Observation] = {}
        self.history: deque = deque(maxlen=180)
        self.trail: deque = deque(maxlen=600)
        self.runs: deque = deque(maxlen=64)
        self.clock: deque = deque(maxlen=30)
        self.last_tick: tuple[float, float, float | None] | None = None
        self.previous_motion = "UNOBSERVED"
        self.previous_pose: list[float] | None = None
        self.active: dict | None = None
        self.bt_status: dict[str, str] = {}
        self.bt_replans = 0
        self.bt_seen = False
        self.completed: deque = deque(maxlen=64)
        self.timeline: deque = deque(maxlen=128)

    def note(self, kind: str, data: dict, wall: float) -> None:
        self.timeline.append(dict(kind=kind, data=data, received_at=wall))

    def ingest(self, key: str, data: dict, wall: float, monotonic: float,
               stamp: float | None = None) -> None:
        self.observations[key] = Observation(data, wall, monotonic, stamp)

    def fresh(self, item: Observation | None, monotonic: float, ros: float | None) -> bool:
        if not item or not 0 <= monotonic - item.monotonic <= self.freshness:
            return False
        return item.stamp is None or ros is None or -.1 <= ros - item.stamp <= self.freshness

    def bt(self, changes: list[dict]) -> None:
        self.bt_seen = True
        for change in changes:
            self.bt_status[change["node"]] = change["status"]
            if "ComputePath" in change["node"] and change["status"] == "RUNNING":
                self.bt_replans += 1
        # Names reflect observed events, not an inferred tree topology.
        self.bt_status = dict(list(self.bt_status.items())[-128:])

    def finish(self, result: str, wall: float) -> None:
        if self.active:
            self.active.update(result=result, end=wall, wall_s=max(0., wall-self.active["start"]))
            self.runs.append(dict(self.active))
            self.completed.append(self.active["operation_id"])
            self.active = None

    def snapshot(self, wall: float, monotonic: float, ros: float | None,
                 pose: list[float] | None, runtime: dict, runtime_fresh: bool,
                 sensor_time: float | None = None) -> tuple[dict, list[dict]]:
        events = []
        reset = bool(self.clock and ros is not None and ros < self.clock[-1][1])
        if reset:
            self.finish("CLOCK_RESET_INCOMPLETE", wall)
            self.history.clear()
            self.trail.clear()
            self.clock.clear()
            self.previous_pose = None
            self.last_tick = None
        if ros is not None:
            self.clock.append((monotonic, ros))
        rtf = None
        if len(self.clock) > 1 and self.clock[-1][0] - self.clock[0][0] > .5:
            rtf = (self.clock[-1][1]-self.clock[0][1]) / (self.clock[-1][0]-self.clock[0][0])

        sensor_time = ros if sensor_time is None else sensor_time
        measured_key = next((key for key in ("ground_truth", "odometry")
                             if self.fresh(self.observations.get(key), monotonic, sensor_time)), None)
        measured = self.observations[measured_key].data["velocity"] if measured_key else None
        speed = math.hypot(*measured[:2]) if measured else None
        commands = []
        for topic in self.velocity_topics:
            item = self.observations.get("cmd:"+topic)
            commands.append(dict(topic=topic, vector=item.data["velocity"] if item else None,
                                 age_s=max(0., monotonic-item.monotonic) if item else None,
                                 fresh=self.fresh(item, monotonic, sensor_time),
                                 received_at=item.wall if item else None))
        motion = ("UNOBSERVED" if measured is None else "STATIONARY"
                  if speed < self.stationary_speed and abs(measured[2]) < .02 else "CREEPING"
                  if speed < self.creep_speed and abs(measured[2]) < .02 else "MOVING")
        reason, source = "원인 미확정", None
        for key in ("reactive_guard", "voxel_guard", "collision", "pan"):
            item = self.observations.get(key)
            if not self.fresh(item, monotonic, sensor_time):
                continue
            decision = str(item.data.get("decision", item.data.get("reason", "")))
            if key == "pan" and item.data.get("motion_authority") is False:
                continue
            if decision and decision not in ("PASS", "PASS_OBSERVED_SPACE", "READY", "TRACKING", "HOLD_DIRECTION", "HOLD_IDLE"):
                reason, source = decision, key
                break
        if runtime_fresh and runtime.get("state") not in ("NAVIGATE_TO_TARGET", "RETURN_HOME"):
            if motion == "STATIONARY":
                reason, source = "작업/대기 단계 · 주행 중 정지와 별도", "runtime"

        nav = runtime.get("navigation", {}) if runtime_fresh else {}
        identity = str(nav.get("operation_id") or "")
        navigating = runtime_fresh and runtime.get("state") in ("NAVIGATE_TO_TARGET", "RETURN_HOME") and identity and identity not in self.completed
        if self.active and (not runtime_fresh or not navigating or identity != self.active["operation_id"]):
            self.finish("OBSERVATION_ENDED" if runtime_fresh else "RUNTIME_GAP_INCOMPLETE", wall)
        if navigating and not self.active:
            self.active = dict(operation_id=identity, target=nav.get("target", ""),
                               phase=runtime.get("state"), start=wall, result="OBSERVING",
                               wall_s=0., sim_s=0., distance_m=0., stationary_s=0., creep_s=0.,
                               moving_s=0., unobserved_s=0., stationary_episodes=0, replan_starts=0,
                               goal_error_m=None, goal_yaw_error_rad=None, gt_goal_error_m=None,
                               measured_source=measured_key, rtf=None)
            self.active["bt_start_count"] = self.bt_replans
            self.last_tick = None
            self.previous_pose = None

        dt = 0. if self.last_tick is None else monotonic-self.last_tick[1]
        if self.active:
            if measured_key:
                previous_source = self.active["measured_source"]
                self.active["measured_source"] = measured_key if previous_source is None else previous_source if previous_source == measured_key else "MIXED"
            self.active["wall_s"] = max(0., wall-self.active["start"])
            if dt > 0:
                # A delayed observer cannot certify what happened inside the gap.
                observed = dt <= .5 and measured_key is not None and motion == self.previous_motion
                bucket = {"STATIONARY": "stationary_s", "CREEPING": "creep_s", "MOVING": "moving_s"}.get(motion)
                self.active[bucket if observed and bucket else "unobserved_s"] += dt
                if ros is not None and self.last_tick[2] is not None and ros >= self.last_tick[2]:
                    self.active["sim_s"] += ros-self.last_tick[2]
            if motion == "STATIONARY" and self.previous_motion != motion:
                self.active["stationary_episodes"] += 1
            if pose and self.previous_pose and 0 < dt <= .5:
                distance = math.dist(pose[:2], self.previous_pose[:2])
                if distance < .5:  # Localization jumps are not traveled distance.
                    self.active["distance_m"] += distance
            self.active["replan_starts"] = max(0, self.bt_replans-self.active["bt_start_count"]-1) if self.bt_seen else None
            self.active["rtf"] = self.active["sim_s"]/self.active["wall_s"] if ros is not None and self.active["wall_s"] > 0 else None
            goal = nav.get("goal_pose") if nav.get("frame") == self.map_frame else None
            if pose and goal:
                self.active["goal_error_m"] = math.dist(pose[:2], [goal["x"], goal["y"]])
                self.active["goal_yaw_error_rad"] = abs(math.atan2(math.sin(pose[2]-goal["yaw"]), math.cos(pose[2]-goal["yaw"])))
            truth = self.observations.get("ground_truth")
            if goal and self.fresh(truth, monotonic, sensor_time) and truth.data.get("map_pose"):
                self.active["gt_goal_error_m"] = math.dist(truth.data["map_pose"][:2], [goal["x"], goal["y"]])
        if motion != self.previous_motion:
            events.append(dict(kind="navigation_motion", data=dict(motion=motion, reason=reason,
                               source=source, speed_mps=speed, target=nav.get("target"))))
            self.note(events[-1]["kind"], events[-1]["data"], wall)
        self.history.append([wall, *[math.hypot(*c["vector"][:2]) if c["fresh"] else None for c in commands], speed, rtf])
        if pose:
            self.trail.append([wall, *pose])
        self.previous_pose = pose
        self.previous_motion = motion
        self.last_tick = (wall, monotonic, ros)
        return dict(motion=motion, reason=reason, reason_source=source, measured=measured,
                    measured_source=measured_key, speed_mps=speed, commands=commands,
                    rtf=rtf, history=list(self.history), trail=list(self.trail),
                    active=dict(self.active) if self.active else None, runs=list(self.runs),
                    timeline=list(self.timeline),
                    bt_status=dict(self.bt_status), freshness_s=self.freshness,
                    stationary_speed_mps=self.stationary_speed, creep_speed_mps=self.creep_speed), events
