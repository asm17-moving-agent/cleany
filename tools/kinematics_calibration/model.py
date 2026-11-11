"""Offline camera-constrained calibration of a five-axis URDF chain."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def pose(vector: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    result[:3, 3] = vector[3:6]
    return result


def vector(matrix: np.ndarray) -> np.ndarray:
    return np.r_[Rotation.from_matrix(matrix[:3, :3]).as_rotvec(), matrix[:3, 3]]


def transverse_basis(axis: np.ndarray) -> np.ndarray:
    first = np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))])
    first /= np.linalg.norm(first)
    return np.array([first, np.cross(axis, first)])


@dataclass
class Observation:
    name: str
    angles: np.ndarray
    points: np.ndarray
    pixels: np.ndarray


@dataclass
class Chain:
    names: list[str]
    origins: np.ndarray
    axes: np.ndarray

    @classmethod
    def read(cls, path: Path, end: str) -> Chain:
        root = ET.parse(path).getroot()
        incoming = {j.find('child').get('link'): j for j in root.findall('joint')}
        joints, seen = [], set()
        while end != 'base_link':
            if end in seen or end not in incoming:
                raise ValueError('Chain must connect to base_link without a cycle')
            seen.add(end)
            joint = incoming[end]
            joints.append(joint)
            end = joint.find('parent').get('link')
        names, origins, axes = [], [], []
        pending = np.eye(4)
        for joint in reversed(joints):
            origin = joint.find('origin')
            offset = np.eye(4)
            if origin is not None:
                offset[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                offset[:3, :3] = Rotation.from_euler(
                    'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
                ).as_matrix()
            pending = pending @ offset
            if joint.get('type') == 'fixed':
                continue
            if joint.get('type') not in ('continuous', 'revolute') or joint.find('mimic') is not None:
                raise ValueError('Only independent revolute joints are supported')
            axis = joint.find('axis')
            axis = np.fromstring(axis.get('xyz', '1 0 0') if axis is not None else '1 0 0', sep=' ')
            if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) == 0:
                raise ValueError('Invalid joint axis')
            names.append(joint.get('name'))
            origins.append(pending)
            axes.append(axis / np.linalg.norm(axis))
            pending = np.eye(4)
        if len(names) != 5 or not np.allclose(pending, np.eye(4)):
            raise ValueError('Endpoint must be the fifth moving joint child')
        return cls(names, np.array(origins), np.array(axes))

    def corrected(self, parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        origins, axes = self.origins.copy(), self.axes.copy()
        if len(parameters) >= 23:
            for index in range(1, 5):
                axis = axes[index]
                first, second = transverse_basis(axis)
                tilt = parameters[15 + 2 * (index - 1):17 + 2 * (index - 1)]
                axes[index] = Rotation.from_rotvec(first * tilt[0] + second * tilt[1]).apply(axis)
        if len(parameters) == 25:
            for index, delta in zip((2, 3), parameters[23:25]):
                translation = origins[index, :3, 3]
                origins[index, :3, 3] += delta * translation / np.linalg.norm(translation)
        return origins, axes

    def axis_positions(self, parameters: np.ndarray) -> np.ndarray:
        positions = np.zeros((5, 3))
        if len(parameters) == 31:
            for index in range(1, 5):
                components = parameters[23 + 2 * (index - 1):25 + 2 * (index - 1)]
                positions[index] = components @ transverse_basis(self.axes[index])
        return positions

    def fk(self, angles: np.ndarray, parameters: np.ndarray) -> np.ndarray:
        origins, axes = self.corrected(parameters)
        positions = self.axis_positions(parameters)
        angles = np.array(angles, dtype=float, copy=True)
        if len(parameters) >= 15:
            angles[:, 1:4] += parameters[12:15]
        result = np.broadcast_to(np.eye(4), (len(angles), 4, 4)).copy()
        for index, (origin, axis) in enumerate(zip(origins, axes)):
            rotation = np.broadcast_to(np.eye(4), result.shape).copy()
            rotation[:, :3, :3] = Rotation.from_rotvec(angles[:, index, None] * axis).as_matrix()
            rotation[:, :3, 3] = positions[index] - rotation[:, :3, :3] @ positions[index]
            result = result @ origin @ rotation
        return result


PARAMETER_COUNTS = {'extrinsics': 12, 'zeros': 15, 'axes': 23, 'geometry': 25, 'screw_axes': 31}


@dataclass
class Problem:
    chain: Chain
    observations: list[Observation]
    camera_matrix: np.ndarray
    distortion: np.ndarray
    zero_sigma_rad: float = np.deg2rad(10)
    axis_sigma_rad: float = np.deg2rad(2)
    length_sigma_m: float = 0.003
    pixel_sigma_px: float = 1.0

    def residual(self, parameters: np.ndarray, indices: list[int]) -> np.ndarray:
        observations = [self.observations[index] for index in indices]
        world_camera = self.chain.fk(np.array([s.angles for s in observations]), parameters) @ pose(parameters[:6])
        board = pose(parameters[6:12])
        rotations = world_camera[:, :3, :3].transpose(0, 2, 1)
        translations = (rotations @ (board[:3, 3] - world_camera[:, :3, 3])[:, :, None])[:, :, 0]
        xyz = np.concatenate([
            (s.points @ board[:3, :3].T) @ rotations[j].T + translations[j]
            for j, s in enumerate(observations)
        ]).astype(float)
        pixels = cv2.projectPoints(
            xyz, np.zeros(3), np.zeros(3), self.camera_matrix, self.distortion,
        )[0].reshape(-1, 2)
        return (pixels - np.concatenate([s.pixels for s in observations])).ravel()

    def prior_sigma(self, count: int) -> np.ndarray:
        sigma = []
        if count >= 15:
            sigma.extend([self.zero_sigma_rad] * 3)
        if count >= 23:
            sigma.extend([self.axis_sigma_rad] * 8)
        if count == 25:
            sigma.extend([self.length_sigma_m] * 2)
        if count == 31:
            sigma.extend([self.length_sigma_m] * 8)
        return np.array(sigma)

    def fit(self, family: str, indices: list[int], seed: np.ndarray, *, max_evaluations: int = 180):
        count = PARAMETER_COUNTS[family]
        start = np.zeros(count)
        shared = min(len(seed), count, 23)
        start[:shared] = seed[:shared]
        if len(seed) == count:
            start[shared:] = seed[shared:]
        sigma = self.prior_sigma(count)

        def objective(parameters: np.ndarray) -> np.ndarray:
            return np.r_[self.residual(parameters, indices) / self.pixel_sigma_px, parameters[12:] / sigma]

        # The first yaw and last roll zeros are gauges of the board and camera
        # transforms. They remain fixed; raw encoder resolution stays 4096/rev.
        lower = np.r_[np.full(12, -np.inf), -5 * sigma]
        upper = np.r_[np.full(12, np.inf), 5 * sigma]
        start[12:] = np.clip(start[12:], lower[12:] + 1e-9, upper[12:] - 1e-9)
        return least_squares(
            objective, start, bounds=(lower, upper), x_scale='jac',
            max_nfev=max_evaluations, ftol=1e-9, xtol=1e-9, gtol=1e-8,
        )

    def evaluate(self, parameters: np.ndarray, indices: list[int]) -> dict:
        rows = []
        camera = pose(parameters[:6])
        board = pose(parameters[6:12])
        for index in indices:
            sample = self.observations[index]
            delta = self.residual(parameters, [index]).reshape(-1, 2)
            world_camera = self.chain.fk(sample.angles[None, :], parameters)[0] @ camera
            camera_points = (sample.points @ board[:3, :3].T + board[:3, 3] - world_camera[:3, 3]) @ world_camera[:3, :3]
            rows.append({
                'sample': sample.name, 'corners': len(delta),
                'rms_px': float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))),
                'minimum_depth_m': float(camera_points[:, 2].min()),
            })
        return {
            'rms_px': float(np.sqrt(np.average([r['rms_px']**2 for r in rows], weights=[r['corners'] for r in rows]))),
            'all_points_in_front': all(r['minimum_depth_m'] > 0 for r in rows),
            'views': rows,
        }
