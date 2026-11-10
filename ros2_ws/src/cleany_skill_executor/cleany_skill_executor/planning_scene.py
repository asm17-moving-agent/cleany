"""Transactional target-OBB registration and ACM restoration."""

from __future__ import annotations

from copy import deepcopy
import math
import time
from typing import Any, Callable

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.core.grasp_selection import InfrastructureError
from cleany_skill_executor.core.grasp_selection import quaternion_axis


def support_patch_from_obb(candidate: Any, object_id: str, margin: float) -> CollisionObject:
    """Finite support patch from the plane-aligned perception OBB bottom.

    This is a local planar-support assumption, not simulator ground truth.
    The perception contract constructs this OBB with its Z axis on the fitted
    support normal and its bottom on that support plane.
    """
    target = candidate.target_object
    pose = deepcopy(target.obb_pose)
    size = target.obb_size
    q = pose.orientation
    if (not math.isfinite(margin) or not 0 < margin <= .3
            or not all(math.isfinite(v) for v in (
                size.x, size.y, size.z, pose.position.x, pose.position.y, pose.position.z,
                q.x, q.y, q.z, q.w)) or min(size.x, size.y, size.z) <= 0
            or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-4):
        raise ValueError('Invalid perceived support patch dimensions or pose')
    normal = quaternion_axis((q.x, q.y, q.z, q.w), (0., 0., 1.))
    if normal[2] < math.cos(math.radians(20)):
        raise ValueError('Perceived support patch must be upward and near horizontal')
    thickness = .01
    for axis, component in zip(('x', 'y', 'z'), normal):
        setattr(pose.position, axis, getattr(pose.position, axis)-(size.z+thickness)/2*component)
    patch = CollisionObject()
    patch.header = deepcopy(candidate.header)
    patch.id = object_id
    patch.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [size.x+2*margin, size.y+2*margin, thickness]
    patch.primitives = [primitive]
    patch.primitive_poses = [pose]
    return patch


class TargetSceneTransaction:
    """Manage temporary target geometry and allowed contacts in MoveIt's scene.

    Methods mutate the shared planning scene. The caller must invoke restore()
    in its cleanup path; this object does not automatically roll back on error.
    The saved collision matrix is restored, and temporary geometry is removed.
    """
    def __init__(
        self,
        node: Any,
        *,
        apply_client: Any | None = None,
        get_client: Any | None = None,
        timeout_sec: float = 1.0,
        spin_once: Callable[[float], None] | None = None,
        support_patch_margin_m: float = 0.0,
        geometry_lookup: Callable[[Any], Any] | None = None,
    ) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('Planning scene timeout must be positive and finite')
        if not math.isfinite(support_patch_margin_m) or not 0 <= support_patch_margin_m <= .3:
            raise ValueError('Support patch margin must be in [0, .3] metres')
        self._support_margin = support_patch_margin_m
        self._support_id = ''
        self._support_acm = None
        self._geometry_lookup = geometry_lookup
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
        self._collision_object: CollisionObject | None = None
        self._attached = False

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
        if self._geometry_lookup is not None:
            deadline = time.monotonic()+self._timeout
            geometry = self._geometry_lookup(candidate)
            while geometry is None and time.monotonic() < deadline:
                self._spin_once(.01)
                geometry = self._geometry_lookup(candidate)
            if geometry is None:
                raise InfrastructureError('Exact-provenance observed collision geometry unavailable')
            collision_object.primitives = []
            collision_object.primitive_poses = []
            collision_object.meshes = [deepcopy(geometry.mesh)]
            collision_object.mesh_poses = [deepcopy(geometry.mesh_pose)]
        support_id = object_id + '/perceived_support' if self._support_margin else ''
        patch = (support_patch_from_obb(candidate, support_id, self._support_margin)
                 if support_id else None)

        # Validate geometry before acquiring transaction ownership or touching
        # MoveIt. A bad support pose must leave the next request usable.
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        response = self._call(self._get_client, request)
        saved_acm = deepcopy(response.scene.allowed_collision_matrix)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision_object]
        support_acm = None
        if patch is not None:
            scene.world.collision_objects.append(patch)
            # The object rests on its support; only this object/support pair
            # may touch. Robot links must still avoid the support patch.
            support_acm = self._with_allowed_links(saved_acm, object_id, {support_id})
            scene.allowed_collision_matrix = deepcopy(support_acm)
        # Keep ownership if an apply fails: MoveIt may have partially applied
        # the diff, so the caller must still be able to restore these objects.
        self._object_id = object_id
        self._saved_acm = saved_acm
        self._collision_object = deepcopy(collision_object)
        self._support_id = support_id
        self._support_acm = support_acm
        response = self._apply(scene)
        if not response.success:
            raise InfrastructureError('failed to register target OBB')

    def attach_to(self, arm: str) -> None:
        """Move the perceived OBB from the world into MoveIt's robot state."""
        if arm not in ('left', 'right'):
            raise ValueError("arm must be 'left' or 'right'")
        if self._collision_object is None or not self._object_id:
            raise InfrastructureError('target scene transaction is not active')
        attached = AttachedCollisionObject()
        attached.link_name = f'{arm}_gripper_frame'
        attached.touch_links = [
            f'{arm}_gripper_frame',
            f'{arm}_moving_jaw_link',
        ]
        attached.object = deepcopy(self._collision_object)
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        # MoveIt transfers an existing world object into robot state when an
        # AttachedCollisionObject with the same id is added.  Sending an
        # explicit world REMOVE in the same diff makes that transfer happen
        # twice and ApplyPlanningScene reports failure even though the object
        # ended up attached.
        scene.robot_state.attached_collision_objects = [attached]
        if not self._apply(scene).success:
            raise InfrastructureError('failed to attach target OBB')
        self._attached = True

    @staticmethod
    def _with_target_permissions(saved: Any, target: str, arm: str) -> Any:
        return TargetSceneTransaction._with_allowed_links(
            saved, target, {f'{arm}_gripper_frame', f'{arm}_moving_jaw_link'})

    @staticmethod
    def _with_allowed_links(saved: Any, target: str, allowed: set[str]) -> Any:
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
            self._support_acm if self._support_acm is not None else self._saved_acm,
            self._object_id, arm
        )
        if not self._apply(scene).success:
            raise InfrastructureError('failed to update target collision permissions')

    def disallow_target_contacts(self) -> None:
        if self._saved_acm is None or not self._object_id:
            raise InfrastructureError('target scene transaction is not active')
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = deepcopy(
            self._support_acm if self._support_acm is not None else self._saved_acm)
        if not self._apply(scene).success:
            raise InfrastructureError('failed to clear target contact permissions')

    def _apply(self, scene: PlanningScene) -> Any:
        request = ApplyPlanningScene.Request()
        request.scene = scene
        return self._call(self._apply_client, request)

    def _scene_is_restored(self) -> bool:
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        if self._attached:
            request.components.components |= PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene = self._call(self._get_client, request).scene
        owned = {self._object_id, self._support_id} - {''}
        present = {item.id for item in scene.world.collision_objects}
        present.update(item.object.id for item in scene.robot_state.attached_collision_objects)
        return (not owned.intersection(present) and self._saved_acm is not None
                and scene.allowed_collision_matrix == self._saved_acm)

    def restore(self) -> None:
        """Remove temporary geometry and restore the saved allowed-collision matrix.

        An attached object is detached before removing its resulting world copy.
        On failure, retain transaction identity so the caller can retry cleanup.

        Raises:
            InfrastructureError: MoveIt cannot confirm a required scene update.
        """
        if not self._object_id:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        remove = CollisionObject()
        remove.id = self._object_id
        remove.operation = CollisionObject.REMOVE
        if self._attached:
            attached = AttachedCollisionObject()
            attached.object.id = self._object_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects = [attached]
        else:
            scene.world.collision_objects = [remove]
        if self._support_id:
            support = CollisionObject()
            support.id = self._support_id
            support.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(support)
        if self._saved_acm is not None:
            scene.allowed_collision_matrix = deepcopy(self._saved_acm)
        response = self._apply(scene)
        if not response.success:
            try:
                restored = self._scene_is_restored()
            except InfrastructureError:
                restored = False
            if not restored:
                raise InfrastructureError('failed to restore planning scene')
            # The authoritative scene confirms that no detach/world removal
            # remains to be done despite the failed apply response.
            self._attached = False
        if self._attached:
            # Humble DETACH puts the body back into the collision world. It
            # does not remove a temporary grasp OBB. Remove that world copy
            # in a subsequent diff, after the detach has succeeded.
            self._attached = False
            scene.robot_state.attached_collision_objects = []
            scene.world.collision_objects = [remove]
            if not self._apply(scene).success:
                # Keep the id so retry removes the world copy, not an
                # already detached robot-state object.
                raise InfrastructureError('failed to remove detached target OBB')
        self._object_id = ''
        self._saved_acm = None
        self._collision_object = None
        self._attached = False
        self._support_id = ''
        self._support_acm = None


class SceneAwarePort:
    """Switch target ACM permissions when candidate evaluation changes arm."""

    def __init__(self, adapter: Any, scene: TargetSceneTransaction) -> None:
        self._adapter = adapter
        self._scene = scene
        self._arm = ''

    def reset(self) -> None:
        self._arm = ''

    @property
    def prepares_pregrasps_lazily(self) -> bool:
        return bool(getattr(self._adapter, 'prepares_pregrasps_lazily', False))

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

    def set_gripper_position(self, arm, position):
        return self._adapter.set_gripper_position(arm, position)

    def set_ik_context(self, candidate_index, arm, target, *, pregrasp):
        context = getattr(self._adapter, 'set_ik_context', None)
        if context is not None:
            context(candidate_index, arm, target, pregrasp=pregrasp)

    def solve_aligned_ik(self, arm, target, seed):
        return self._adapter.solve_aligned_ik(arm, target, seed)

    def solve_pregrasp_ik(self, arm, target, seed):
        solve = getattr(self._adapter, 'solve_pregrasp_ik', self._adapter.solve_aligned_ik)
        return solve(arm, target, seed)

    def prepare_pregrasps(self, requests):
        prepare = getattr(self._adapter, 'prepare_pregrasps', None)
        if prepare is not None:
            prepare(requests)

    def prepare_grasps(self, requests):
        prepare = getattr(self._adapter, 'prepare_grasps', None)
        if prepare is not None:
            prepare(requests)

    def begin_candidate_pass(self, retry: bool) -> bool:
        begin = getattr(self._adapter, 'begin_candidate_pass', None)
        return begin(retry) if begin is not None else False

    def state_is_valid(self, arm, solution):
        return self._adapter.state_is_valid(arm, solution)

    def plan(self, arm, goal, start):
        return self._adapter.plan(arm, goal, start)

    @property
    def planning_pass(self):
        return getattr(self._adapter, 'planning_pass', None)

    @property
    def plan_approach(self):
        """Expose the optional validated approach planner through this port."""
        return getattr(self._adapter, 'plan_approach', None)

    def plan_direct_grasp(self, arm, goal):
        return self._adapter.plan_direct_grasp(arm, goal)

    def trajectory_alignment(
        self, arm, path, orientation_xyzw, maximum_waypoints
    ):
        return self._adapter.trajectory_alignment(
            arm, path, orientation_xyzw, maximum_waypoints
        )

    def wrist_motion_cost(self, arm, pregrasp, grasp):
        metric = getattr(self._adapter, 'wrist_motion_cost', None)
        return None if metric is None else metric(arm, pregrasp, grasp)

    def current_tcp_pose(self, arm):
        return self._adapter.current_tcp_pose(arm)

    def gripper_sweep_is_valid(self, arm, solution, opening, closing, step):
        return self._adapter.gripper_sweep_is_valid(arm, solution, opening, closing, step)

    def validate_approach_path(self, arm, path, start, target):
        return self._adapter.validate_approach_path(arm, path, start, target)

    def pregrasp_is_visible(self, arm, solution):
        return self._adapter.pregrasp_is_visible(arm, solution)
