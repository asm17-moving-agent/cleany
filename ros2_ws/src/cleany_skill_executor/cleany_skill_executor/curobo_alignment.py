"""Keep wrist expansion, FK and pose-error evaluation on the CUDA device."""
from __future__ import annotations

import math
from types import ModuleType
from typing import TYPE_CHECKING, Sequence

import numpy as np

from cleany_skill_executor.core.ik_diagnostics import (
    AlignmentLimits, ScoredAlignments, summarize_error_rows,
)

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, JointSolution, PoseAlignment, PoseValue,
)

if TYPE_CHECKING:
    from torch import Tensor
    from cleany_skill_executor.curobo_engine import CuRoboEngine
    from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig


def _axes(torch: ModuleType, quaternion: Tensor) -> tuple[Tensor, Tensor]:
    quaternion = quaternion.to(dtype=torch.float64)
    quaternion = quaternion / torch.linalg.vector_norm(quaternion, dim=-1, keepdim=True)
    w, x, y, z = quaternion.unbind(dim=-1)
    closing = torch.stack((1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)), dim=-1)
    approach = -torch.stack((2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)), dim=-1)
    return approach, closing


def _unique_variants(torch: ModuleType, joints: Tensor, owners: Tensor) -> tuple[Tensor, Tensor, Tensor]:
    # Match the reference dictionary: first insertion order, last value for
    # each rounded joint key, with separate ownership for each grasp target.
    keys = torch.cat((owners[:, None], torch.round(joints*1e7).to(torch.int64)), dim=1)
    unique, inverse = torch.unique(keys, dim=0, return_inverse=True)
    indices = torch.arange(len(joints), device=joints.device)
    first = torch.full((len(unique),), len(joints), device=joints.device, dtype=torch.int64)
    last = torch.zeros_like(first)
    first.scatter_reduce_(0, inverse, indices, reduce='amin')
    last.scatter_reduce_(0, inverse, indices, reduce='amax')
    order = torch.argsort(first, stable=True)
    kept = last[order]
    return joints[kept], owners[first[order]], kept


def _rotate_wrist(torch: ModuleType, quaternions: Tensor, angles: Tensor, axis) -> Tensor:
    """Compose the TCP quaternion with a rotation about its fixed local wrist axis."""
    quaternions = quaternions.to(dtype=torch.float64)
    axis = torch.tensor(axis, device=angles.device, dtype=angles.dtype)
    half = angles[:, None] * .5
    scalar, vector = torch.cos(half), torch.sin(half) * axis
    left_scalar, left_vector = quaternions[:, :1], quaternions[:, 1:]
    return torch.cat((
        left_scalar * scalar - (left_vector * vector).sum(dim=-1, keepdim=True),
        left_scalar * vector + scalar * left_vector + torch.cross(left_vector, vector, dim=-1),
    ), dim=-1)


def _alignment_from_row(names: tuple[str, ...], row: Sequence[float]) -> PoseAlignment:
    return PoseAlignment(
        JointSolution(names, tuple(map(float, row[:5]))),
        PoseValue(tuple(map(float, row[5:8])), tuple(map(float, row[8:12]))),
        *map(float, row[12:15]))


def _compact_group(names: tuple[str, ...], rows: np.ndarray, limits: AlignmentLimits,
                   preferred_roll: float | None) -> ScoredAlignments:
    errors = rows[:, 12:15]
    feasible = (errors <= (limits.position_m, limits.approach_degrees,
                           limits.closing_degrees)).all(axis=1)
    retained = list(np.flatnonzero(feasible))
    rejected = np.flatnonzero(~feasible)
    if len(rejected):
        if np.isfinite(errors[rejected]).all() and (
                preferred_roll is None or math.isfinite(preferred_roll)):
            travel = (np.abs(rows[rejected, 4]-preferred_roll) if preferred_roll is not None
                      else np.zeros(len(rejected)))
            order = np.lexsort((rows[rejected, 12], rows[rejected, 14],
                                rows[rejected, 13], travel))
            best = rejected[order[0]]
        else:
            # Preserve Python tuple ordering for nonfinite failure evidence.
            best = min(rejected, key=lambda index: (
                abs(float(rows[index, 4])-preferred_roll) if preferred_roll is not None else 0.,
                *map(float, rows[index, 13:15]), float(rows[index, 12])))
        retained.append(best)
        retained.sort()
    candidates = {index: _alignment_from_row(names, rows[index]) for index in retained}
    def witness(index):
        return candidates[index] if index in candidates else _alignment_from_row(names, rows[index])
    summary = summarize_error_rows(errors, limits, witness)
    return ScoredAlignments(list(candidates.values()), summary, limits)


def score_wrist_batch(
    engine: CuRoboEngine, arm: str, targets: Sequence[PoseValue], joints: Tensor,
    owners: Tensor, state: dict[str, float], config: MoveItAdapterConfig,
    preferred_rolls: Sequence[float | None],
) -> list[list[PoseAlignment] | ScoredAlignments]:
    """Return reference-ordered alignments after one final host transfer.

    Joint rows and target owners may come directly from GPU IK. All poses are
    scoped to this call's full robot state; collision decisions are independent.
    """
    torch = engine.torch
    names = ARM_JOINT_NAMES[arm]
    output = [[] for _ in targets]
    if len(joints) == 0:
        return output
    device = joints.device
    joints = joints.to(dtype=torch.float64)
    owners = owners.to(device=device, dtype=torch.int64)
    target_positions = torch.tensor([t.position for t in targets], device=device, dtype=torch.float64)
    target_quaternions = torch.tensor(
        [t.orientation_xyzw for t in targets], device=device, dtype=torch.float64)[:, [3, 0, 1, 2]]
    target_approach, target_closing = _axes(torch, target_quaternions)
    preferred = torch.tensor(
        [float('nan') if value is None else value for value in preferred_rolls],
        device=device, dtype=torch.float64)[owners]
    lower, upper = engine.model.limits[names[-1]]
    offsets = torch.tensor(config.ik_seed_wrist_offsets_degrees, device=device,
                           dtype=torch.float64) * (math.pi/180.)
    rolls = (joints[:, -1, None]+offsets).clamp(lower, upper)
    valid = torch.ones_like(rolls, dtype=torch.bool)
    originals = original_positions = original_quaternions = None
    if config.align_wrist_roll:
        originals, inverse = torch.unique(joints, dim=0, return_inverse=True)
        original_positions, original_quaternions = engine.fk_tensor(arm, originals, state)
        approach, closing = _axes(torch, original_quaternions[inverse])
        axis = approach / torch.linalg.vector_norm(approach, dim=-1, keepdim=True)
        desired = target_closing[owners]
        source = closing-(closing*axis).sum(dim=-1, keepdim=True)*axis
        target = desired-(desired*axis).sum(dim=-1, keepdim=True)*axis
        source_norm = torch.linalg.vector_norm(source, dim=-1, keepdim=True)
        target_norm = torch.linalg.vector_norm(target, dim=-1, keepdim=True)
        projected = (source_norm[:, 0] > 1e-9) & (target_norm[:, 0] > 1e-9)
        source = source/source_norm.clamp_min(1e-30)
        target = target/target_norm.clamp_min(1e-30)
        angle = torch.atan2((axis*torch.cross(source, target, dim=-1)).sum(dim=-1),
                            (source*target).sum(dim=-1))
        aligned = joints[:, -1]+angle
        first = torch.ceil((lower-aligned)/math.tau)
        last = torch.floor((upper-aligned)/math.tau)
        turns = first[:, None]+torch.arange(
            math.floor((upper-lower)/math.tau)+1, device=device)[None, :]
        exact = aligned[:, None]+turns*math.tau
        exact_valid = projected[:, None] & (turns <= last[:, None])
        order = torch.argsort(torch.abs(exact-joints[:, -1, None]), dim=1, stable=True)
        exact = exact.gather(1, order)
        exact_valid = exact_valid.gather(1, order)
        delta = preferred[:, None]-exact
        tolerance = math.radians(config.closing_tolerance_degrees)*(1.-1e-6)
        boundary = exact+torch.copysign(delta.abs().clamp_max(tolerance), delta)
        boundary_valid = exact_valid & torch.isfinite(preferred[:, None]) & (
            (boundary >= lower) & (boundary <= upper))
        # The reference prepends each boundary/exact pair in reverse roll order.
        aligned_rolls = torch.stack((boundary, exact), dim=2).flip(1).flatten(1)
        aligned_valid = torch.stack((boundary_valid, exact_valid), dim=2).flip(1).flatten(1)
        rolls = torch.cat((preferred[:, None].clamp(lower, upper), rolls, aligned_rolls), dim=1)
        valid = torch.cat((torch.isfinite(preferred[:, None]), valid, aligned_valid), dim=1)
    expanded = joints[:, None, :].expand(-1, rolls.shape[1], -1).clone()
    expanded[:, :, -1] = rolls
    expanded_owners = owners[:, None].expand_as(rolls)
    variants, variant_owners, kept = _unique_variants(
        torch, expanded[valid], expanded_owners[valid])
    wrist_axis = engine._coaxial_wrist_axes.get(arm)
    if originals is None:
        positions, quaternions = engine.fk_tensor(arm, variants, state)
    elif wrist_axis is not None:
        # Only the final wrist angle changed. The URDF proves that its TCP
        # position is invariant, so reuse FK and compose the local rotation.
        bases = inverse[:, None].expand_as(rolls)[valid][kept]
        positions = original_positions[bases]
        quaternions = _rotate_wrist(torch, original_quaternions[bases],
                                   variants[:, -1] - originals[bases, -1], wrist_axis)
    else:
        # Reuse original FK, including proposals shared by different targets.
        union, inverse = torch.unique(torch.cat((originals, variants)), dim=0, return_inverse=True)
        original_indices, variant_indices = inverse[:len(originals)], inverse[len(originals):]
        missing = torch.ones(len(union), device=device, dtype=torch.bool)
        missing[original_indices] = False
        positions = torch.empty((len(union), 3), device=device, dtype=original_positions.dtype)
        quaternions = torch.empty((len(union), 4), device=device, dtype=original_quaternions.dtype)
        positions[original_indices], quaternions[original_indices] = original_positions, original_quaternions
        missing_joints = union[missing]
        if len(missing_joints):
            positions[missing], quaternions[missing] = engine.fk_tensor(arm, missing_joints, state)
        positions, quaternions = positions[variant_indices], quaternions[variant_indices]
    approach, closing = _axes(torch, quaternions)
    expected_approach, expected_closing = target_approach[variant_owners], target_closing[variant_owners]
    approach = approach/torch.linalg.vector_norm(approach, dim=-1, keepdim=True)
    closing = closing/torch.linalg.vector_norm(closing, dim=-1, keepdim=True)
    expected_approach = expected_approach/torch.linalg.vector_norm(expected_approach, dim=-1, keepdim=True)
    expected_closing = expected_closing/torch.linalg.vector_norm(expected_closing, dim=-1, keepdim=True)
    errors = torch.stack((
        torch.linalg.vector_norm(positions-target_positions[variant_owners], dim=-1),
        torch.acos((approach*expected_approach).sum(dim=-1).clamp(-1., 1.))*(180./math.pi),
        torch.acos((closing*expected_closing).sum(dim=-1).clamp(-1., 1.))*(180./math.pi),
    ), dim=1)
    packed = torch.cat((variant_owners[:, None], variants, positions,
                        quaternions[:, [1, 2, 3, 0]], errors), dim=1).detach().cpu()
    if getattr(engine, '_compact_alignment_output', False):
        values = packed.numpy()
        limits = AlignmentLimits(config.position_tolerance_m, config.approach_tolerance_degrees,
                                 config.closing_tolerance_degrees)
        return [_compact_group(names, values[values[:, 0] == owner, 1:], limits,
                               state.get(names[-1]) if roll is None else roll)
                for owner, roll in enumerate(preferred_rolls[:len(targets)])]
    for row in packed.tolist():
        owner = int(row[0])
        output[owner].append(_alignment_from_row(names, row[1:]))
    return output
