"""Transactional target-OBB registration and ACM restoration."""

from __future__ import annotations

from copy import deepcopy
import time
from typing import Any, Callable

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.core.grasp_selection import InfrastructureError


class TargetSceneTransaction:
    def __init__(
        self,
        node: Any,
        *,
        apply_client: Any | None = None,
        get_client: Any | None = None,
        timeout_sec: float = 1.0,
        spin_once: Callable[[float], None] | None = None,
    ) -> None:
        self._apply_client = apply_client or node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        self._get_client = get_client or node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        self._timeout = timeout_sec
        self._spin_once = spin_once or (lambda duration: time.sleep(duration))
        self._object_id = ''
        self._saved_acm = None

    @property
    def active(self) -> bool:
        return bool(self._object_id)

    def _call(self, client: Any, request: Any) -> Any:
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise InfrastructureError('planning-scene service unavailable')
        future = client.call_async(request)
        deadline = time.monotonic() + self._timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                future.cancel()
                raise InfrastructureError('planning-scene service timed out')
            self._spin_once(min(0.01, remaining))
        response = future.result()
        if response is None:
            raise InfrastructureError('planning-scene service failed')
        return response

    def begin(self, candidate: Any, object_id: str) -> None:
        if self.active:
            raise InfrastructureError(
                'planning-scene transaction is already active'
            )
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        response = self._call(self._get_client, request)
        self._saved_acm = deepcopy(response.scene.allowed_collision_matrix)
        self._object_id = object_id

        collision_object = CollisionObject()
        collision_object.header.frame_id = candidate.header.frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        size = candidate.target_object.obb_size
        primitive.dimensions = [size.x, size.y, size.z]
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [deepcopy(candidate.target_object.obb_pose)]
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision_object]
        response = self._apply(scene)
        if not response.success:
            raise InfrastructureError('failed to register target OBB')

    @staticmethod
    def _with_target_permissions(saved: Any, target: str, arm: str) -> Any:
        acm = deepcopy(saved)
        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]
        while len(rows) < len(names):
            rows.append([False] * len(names))
        for row in rows:
            row.extend([False] * (len(names) - len(row)))
        if target not in names:
            names.append(target)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))
        target_index = names.index(target)
        allowed = {f'{arm}_gripper_frame', f'{arm}_moving_jaw_link'}
        for link in allowed:
            if link not in names:
                names.append(link)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
            index = names.index(link)
            rows[target_index][index] = True
            rows[index][target_index] = True
        acm.entry_names = names
        acm.entry_values = []
        for row in rows:
            entry = AllowedCollisionEntry()
            entry.enabled = row
            acm.entry_values.append(entry)
        return acm

    def allow_contacts_for(self, arm: str) -> None:
        if self._saved_acm is None or not self._object_id:
            raise InfrastructureError('target scene transaction is not active')
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = self._with_target_permissions(
            self._saved_acm, self._object_id, arm
        )
        if not self._apply(scene).success:
            raise InfrastructureError('failed to update target collision permissions')

    def disallow_target_contacts(self) -> None:
        if self._saved_acm is None or not self._object_id:
            raise InfrastructureError('target scene transaction is not active')
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = deepcopy(self._saved_acm)
        if not self._apply(scene).success:
            raise InfrastructureError('failed to clear target contact permissions')

    def _apply(self, scene: PlanningScene) -> Any:
        request = ApplyPlanningScene.Request()
        request.scene = scene
        return self._call(self._apply_client, request)

    def restore(self) -> None:
        if not self._object_id:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        remove = CollisionObject()
        remove.id = self._object_id
        remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [remove]
        if self._saved_acm is not None:
            scene.allowed_collision_matrix = deepcopy(self._saved_acm)
        response = self._apply(scene)
        if not response.success:
            raise InfrastructureError('failed to restore planning scene')
        self._object_id = ''
        self._saved_acm = None


class SceneAwarePort:
    """Switch target ACM permissions when candidate evaluation changes arm."""

    def __init__(self, adapter: Any, scene: TargetSceneTransaction) -> None:
        self._adapter = adapter
        self._scene = scene
        self._arm = ''

    def reset(self) -> None:
        self._arm = ''

    def set_target_contacts(self, arm: str | None) -> None:
        if arm == self._arm:
            return
        if arm is None:
            self._scene.disallow_target_contacts()
        else:
            self._scene.allow_contacts_for(arm)
        self._arm = arm

    def solve_position_ik(self, arm, position, seed):
        return self._adapter.solve_position_ik(arm, position, seed)

    def state_is_valid(self, arm, solution):
        return self._adapter.state_is_valid(arm, solution)

    def plan(self, arm, goal, start):
        return self._adapter.plan(arm, goal, start)
