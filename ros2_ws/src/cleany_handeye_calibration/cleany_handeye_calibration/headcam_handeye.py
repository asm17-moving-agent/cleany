"""Read-only head calibration geometry and estimated-time stationary gates."""
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


@dataclass(frozen=True)
class HeadJointMap:
    calibration: dict
    signs: dict

    def radians(self, raw: dict) -> dict:
        period = self.calibration['encoder_period_ticks']
        angles = {}
        for name in ('pan', 'tilt'):
            joint = self.calibration['joints'][name]
            value = raw[name]['raw_tick']
            if raw[name]['id'] != joint['id'] or not 0 <= value < period:
                raise ValueError(f'{name}: invalid ID or encoder reading')
            low, high = joint['observed_min_unwrapped_tick'], joint['observed_max_unwrapped_tick']
            if high - low >= period or self.signs[name] not in (-1, 1):
                raise ValueError(f'{name}: ambiguous observed range or invalid sign')
            candidates = [value + turn * period for turn in range(
                math.ceil((low-value)/period), math.floor((high-value)/period)+1)]
            if len(candidates) != 1:
                raise ValueError(f'{name}: position outside demonstrated range')
            angles[name] = self.signs[name] * (
                candidates[0] - joint['zero_candidate_raw_tick']) * 2 * math.pi / period
        return angles


class HeadKinematics:
    """Read the nominal base-to-tilt chain; do not substitute a measured mount."""

    def __init__(self, xacro: Path):
        root = ET.parse(xacro).getroot()
        names = ('top_base_joint', 'head_pan_joint', 'head_tilt_joint')
        self.chain = []
        parent = 'base_link'
        for name in names:
            joint = root.find(f'.//joint[@name="{name}"]')
            if joint is None or joint.find('parent').get('link') != parent:
                raise ValueError('Unexpected head kinematic chain')
            parent = joint.find('child').get('link')
            origin = joint.find('origin')
            xyz = np.array([float(v) for v in origin.get('xyz').split()])
            roll, pitch, yaw = [float(v) for v in origin.get('rpy').split()]
            transform = np.eye(4)
            rx = cv2.Rodrigues(np.array([roll, 0., 0.]))[0]
            ry = cv2.Rodrigues(np.array([0., pitch, 0.]))[0]
            rz = cv2.Rodrigues(np.array([0., 0., yaw]))[0]
            transform[:3, :3] = rz @ ry @ rx
            transform[:3, 3] = xyz
            axis, limits = None, None
            if joint.get('type') == 'revolute':
                axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                if not np.isclose(np.linalg.norm(axis), 1):
                    raise ValueError('Expected normalized joint axis')
                limits = tuple(float(joint.find('limit').get(v)) for v in ('lower', 'upper'))
            self.chain.append((name, transform, axis, limits))
        if parent != 'head_tilt_link':
            raise ValueError('Unexpected chain endpoint')

    def base_T_tilt(self, angles: dict) -> np.ndarray:
        result = np.eye(4)
        for name, origin, axis, limits in self.chain:
            result = result @ origin
            if axis is not None:
                q = angles['pan' if name == 'head_pan_joint' else 'tilt']
                if not math.isfinite(q) or not limits[0] <= q <= limits[1]:
                    raise ValueError(f'{name}: outside nominal URDF limits')
                rotation = np.eye(4)
                rotation[:3, :3] = cv2.Rodrigues(axis * q)[0]
                result = result @ rotation
        return result


def stationary_pair(rows: list[dict], frame_ns: int, exposure_ns: int,
                    now_ns: int, joint_map: HeadJointMap, policy: dict) -> dict:
    """Bracket SDK-estimated frame time with host register-read intervals.

    This is software time association, not a hardware synchronization claim.
    Full exposure +/- uncertainty and pre/post hold must be covered by feedback.
    """
    if exposure_ns <= 0 or frame_ns <= 0:
        raise ValueError('Missing exposure or image timestamp')
    if not 0 <= now_ns - frame_ns <= policy['maximum_frame_age_ms'] * 1e6:
        raise ValueError('Image timestamp stale or in future')
    margin = exposure_ns + int(policy['timestamp_guard_ms'] * 1e6)
    begin = frame_ns - margin - int(policy['settle_seconds'] * 1e9)
    end = frame_ns + margin + int(policy['post_hold_ms'] * 1e6)
    starts = [r['host_monotonic_start_ns'] for r in rows]
    ends = [r['host_monotonic_end_ns'] for r in rows]
    if any(b < a for a, b in zip(starts, ends)) or any(
            b <= a for a, b in zip(ends, starts[1:])):
        raise ValueError('Joint read timestamps overlap or regress')
    previous = [i for i, stamp in enumerate(ends) if stamp <= begin]
    following = [i for i, stamp in enumerate(starts) if stamp >= end]
    if not previous or not following:
        raise ValueError('Waiting for stationary history/exposure coverage')
    window = rows[previous[-1]:following[0]+1]
    if len(window) < 3:
        raise ValueError('Insufficient feedback')
    gap = int(policy['maximum_joint_gap_ms'] * 1e6)
    if any(b['host_monotonic_end_ns'] - a['host_monotonic_start_ns'] > gap
           for a, b in zip(window, window[1:])):
        raise ValueError('Joint feedback gap too large')
    angles = [joint_map.radians(row['joints']) for row in window]
    span = {name: max(q[name] for q in angles) - min(q[name] for q in angles)
            for name in ('pan', 'tilt')}
    if max(span.values()) > policy['maximum_stationary_span_rad']:
        raise ValueError('Head moving; hold both axes still')
    before = next(r for r in reversed(window) if r['host_monotonic_end_ns'] <= frame_ns)
    after = next(r for r in window if r['host_monotonic_start_ns'] >= frame_ns)
    a = (before['host_monotonic_start_ns'] + before['host_monotonic_end_ns']) / 2
    b = (after['host_monotonic_start_ns'] + after['host_monotonic_end_ns']) / 2
    alpha = (frame_ns-a)/(b-a)
    qa, qb = joint_map.radians(before['joints']), joint_map.radians(after['joints'])
    return {'angles_rad': {name: qa[name] + alpha*(qb[name]-qa[name]) for name in qa},
            'interpolation_alpha': alpha, 'before': before, 'after': after,
            'stationary_span_rad': span, 'stationary_window': window,
            'frame_monotonic_estimate_ns': frame_ns, 'exposure_us': exposure_ns/1000,
            'timing_method': 'SDK global-time estimate + host read-interval bracketing',
            'hardware_synchronized': False}


def board_pose(found: dict, board, camera: dict, policy: dict) -> dict:
    obj = np.ascontiguousarray(board.chessboardCorners[found['ids'].flatten()], dtype=float)
    image = np.ascontiguousarray(found['corners'], dtype=float)
    k, d = np.array(camera['K']), np.array(camera['D'])
    ok, rotations, translations, _ = cv2.solvePnPGeneric(
        obj, image, k, d, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise ValueError('PnP failed')
    candidates = []
    for r, t in zip(rotations, translations):
        r, t = cv2.solvePnPRefineVVS(obj, image, k, d, r.copy(), t.copy())
        rotation = cv2.Rodrigues(r)[0]
        if not np.isfinite(rotation).all() or not np.isfinite(t).all():
            continue
        if np.min((obj @ rotation.T + t.reshape(1, 3))[:, 2]) <= 0:
            continue
        projected = cv2.projectPoints(obj, r, t, k, d)[0]
        error = float(np.sqrt(np.mean(np.sum((image-projected)**2, axis=2))))
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, t.flatten()
        candidates.append({'rmse_px': error, 'camera_T_board': transform.tolist()})
    candidates.sort(key=lambda x: x['rmse_px'])
    if not candidates or candidates[0]['rmse_px'] > policy['maximum_pnp_rmse_px']:
        raise ValueError('PnP reprojection error too large or no positive-depth pose')
    if len(candidates) > 1:
        aa, bb = np.array(candidates[0]['camera_T_board']), np.array(candidates[1]['camera_T_board'])
        # Refinement may converge both initial IPPE solutions to the same pose.
        distinct = (np.linalg.norm(aa[:3, 3]-bb[:3, 3]) > 1e-5 or
                    np.linalg.norm(cv2.Rodrigues(aa[:3, :3].T @ bb[:3, :3])[0]) > 1e-4)
        if distinct and candidates[1]['rmse_px'] / max(candidates[0]['rmse_px'], 1e-12) < policy['minimum_pnp_ratio']:
            raise ValueError('Ambiguous planar pose; change board viewing angle')
    return {'best': candidates[0], 'candidates': candidates}
