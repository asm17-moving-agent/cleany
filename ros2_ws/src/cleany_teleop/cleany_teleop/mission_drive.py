"""Attach the mission action to the phone server's existing base owner."""

import math
import time

from cleany_interfaces.action import DriveMissionTarget
from cleany_interfaces.msg import SystemState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future

from cleany_mission_manager.adapters.config import load_targets


class MissionDrive:
    """Use the parent mission's lease for exactly one named navigation leg."""

    def __init__(self, bridge, target_file, grid_revision):
        self.bridge = bridge
        self.node = bridge.node
        self.targets = load_targets(target_file)
        if not self.targets.binding or not grid_revision:
            raise ValueError(
                "mission drive requires verified targets and the commissioned occupancy-grid revision"
            )
        self.grid_revision = grid_revision
        self.state = self.goal = self.reserved = self.waiter = None
        self.received = -math.inf
        self.node.create_subscription(
            SystemState,
            "system/state",
            self.update,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.server = ActionServer(
            self.node,
            DriveMissionTarget,
            "navigation/drive_mission_target",
            execute_callback=self.execute,
            goal_callback=self.admit,
            cancel_callback=self.cancel,
        )

    def update(self, message):
        with self.bridge.lock:
            self.state, self.received = message, time.monotonic()

    def state_age(self):
        state, supervisor = self.state, self.bridge.supervision
        if (
            state is None
            or state.operating_state not in ("OPERATIONAL", "DEGRADED")
            or not supervisor._alive()
            or state.epoch != supervisor.epoch
        ):
            return math.inf
        stamp = state.header.stamp
        age = (self.node.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) * 1e-9
        elapsed = max(age, time.monotonic() - self.received)
        return elapsed if 0 <= age and elapsed <= supervisor.heartbeat_timeout else math.inf

    def authorized(self, req):
        elapsed = self.state_age()
        if (
            not math.isfinite(elapsed)
            or not req.mission_id
            or not req.lease_id
            or self.state.epoch != req.lease_epoch
        ):
            return False
        return any(
            p.granted
            and p.mission_id == req.mission_id
            and p.lease_id == req.lease_id
            and p.version == req.lease_version
            and "MOBILE_MANIPULATION" in p.capabilities
            and math.isfinite(p.valid_for_s)
            and p.valid_for_s > elapsed
            for p in self.state.permissions
        )

    def operator_blocked(self):
        if self.goal or self.reserved or not math.isfinite(self.state_age()):
            return True
        return any(
            "MOBILE_MANIPULATION" in p.capabilities and (p.granted or not p.stop_confirmed)
            for p in self.state.permissions
        )

    def allowed(self, req):
        b = self.bridge
        return (
            not b.supervision.mission_id
            and b.phase in ("idle", "succeeded", "cancelled", "failed")
            and req.target_digest == self.targets.binding["target_digest"]
            and (req.target_id == "home" or req.target_id in self.targets.seats)
            and b.map is not None
            and b.map_revision == self.grid_revision
            and b.hardware_ready()[0]
            and b.measured_standstill(time.monotonic())
            and self.authorized(req)
        )

    def admit(self, req):
        with self.bridge.lock:
            if self.goal or self.reserved or not self.allowed(req):
                return GoalResponse.REJECT
            self.reserved = req
            return GoalResponse.ACCEPT

    async def execute(self, handle):
        b, req = self.bridge, handle.request
        with b.lock:
            self.reserved = None
            if not self.allowed(req) or handle.is_cancel_requested:
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                return DriveMissionTarget.Result(detail="drive conditions changed before start")
            p = self.targets.home if req.target_id == "home" else self.targets.seats[req.target_id]
            try:
                goal = b.map.validate_goal(dict(map_id=self.grid_revision, x=p.x, y=p.y, yaw=p.yaw))
            except ValueError as error:
                handle.abort()
                return DriveMissionTarget.Result(detail=str(error))
            self.goal, self.waiter = handle, Future()
            waiter = self.waiter
            b.begin_goal(goal)
        return await waiter

    def cancel(self, handle):
        with self.bridge.lock:
            if self.goal is not None and handle.goal_id == self.goal.goal_id:
                self.bridge.cancel("Mission cancelled")
            return CancelResponse.ACCEPT

    def tick(self):
        if not self.goal:
            return
        b = self.bridge
        if b.phase in ("preparing", "active"):
            if (
                self.goal.is_cancel_requested
                or not self.authorized(self.goal.request)
                or b.map is None
                or b.map_revision != self.grid_revision
            ):
                b.cancel("Mission permission, map or goal changed")
        elif b.phase in ("succeeded", "cancelled", "failed"):
            # NavigationBridge reaches these phases only after the MCU stop ACK and fresh wheel feedback.
            handle, waiter = self.goal, self.waiter
            self.goal = self.waiter = None
            arrived = b.phase == "succeeded" and not handle.is_cancel_requested
            if handle.is_cancel_requested:
                handle.canceled()
            elif arrived:
                handle.succeed()
            else:
                handle.abort()
            waiter.set_result(
                DriveMissionTarget.Result(arrived=arrived, stop_confirmed=True, detail=b.reason)
            )
