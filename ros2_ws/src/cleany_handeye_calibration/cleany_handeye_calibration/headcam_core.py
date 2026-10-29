"""Offline head RGB intrinsic calibration; independent of hardware and ROS."""
from pathlib import Path
import json

import cv2
import numpy as np


def make_board(config: dict):
    """Construct the separately configured real target on OpenCV 4.5.4."""
    square = float(config['square_length_m'])
    marker = float(config['marker_length_m'])
    if not 0 < marker < square or not np.isfinite(square):
        raise ValueError('Require finite 0 < marker length < square length')
    nx, ny = int(config['squares_x']), int(config['squares_y'])
    if min(nx, ny) < 3:
        raise ValueError('Board must have at least three squares per axis')
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, config['dictionary']))
    board = cv2.aruco.CharucoBoard_create(nx, ny, square, marker, dictionary)
    return dictionary, board


def detect(image: np.ndarray, dictionary, board, minimum: int) -> dict:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    result = {'marker_corners': corners, 'marker_ids': ids,
              'corners': None, 'ids': None, 'valid': False}
    if ids is None:
        return result
    _, points, point_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board)
    result.update(corners=points, ids=point_ids)
    if point_ids is not None and len(point_ids) >= minimum:
        obj = board.chessboardCorners[point_ids.flatten()]
        result['valid'] = bool(np.linalg.matrix_rank(obj[:, :2] - obj[0, :2]) == 2)
    return result


def calibrate_directory(directory: Path) -> dict:
    """Solve RGB intrinsics only, preserving camera defaults and robot TF."""
    manifest = json.loads((directory / 'manifest.json').read_text())
    config = manifest['config']
    dictionary, board = make_board(config)
    corners, ids, names = [], [], []
    size = (config['width'], config['height'])
    for sample in sorted(directory.glob('sample_*')):
        if not (sample / 'record.json').exists():
            continue
        image = cv2.imread(str(sample / 'rgb.png'))
        if image is None or (image.shape[1], image.shape[0]) != size:
            raise ValueError(f'Invalid image size: {sample}')
        found = detect(image, dictionary, board, config['minimum_corners'])
        if found['valid']:
            corners.append(found['corners'])
            ids.append(found['ids'])
            names.append(sample.name)
    if len(names) < config['minimum_calibration_views']:
        raise ValueError(f'Only {len(names)} valid views; need '
                         f"{config['minimum_calibration_views']}")
    rms, k, d, rotations, translations = cv2.aruco.calibrateCameraCharuco(
        corners, ids, board, size, None, None)
    if not np.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(d).all():
        raise ValueError('Non-finite calibration result')
    errors = []
    for points, point_ids, r, t in zip(corners, ids, rotations, translations):
        projected, _ = cv2.projectPoints(
            board.chessboardCorners[point_ids.flatten()], r, t, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum(
            (points.reshape(-1, 2) - projected.reshape(-1, 2)) ** 2, axis=1)))))
    return {'kind': 'rgb_intrinsics_candidate', 'review_required': True,
            'width': size[0], 'height': size[1], 'K': k.tolist(),
            'D': d.flatten().tolist(), 'rms_px': float(rms),
            'samples': names, 'per_view_rmse_px': errors,
            'note': 'No robot extrinsics or depth calibration. Validate on held-out views.'}
