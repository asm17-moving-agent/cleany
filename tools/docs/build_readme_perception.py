#!/usr/bin/env python3
"""Run the current detector and geometry core on a recorded RGB-D frame."""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image
import yaml
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/cleany_perception'))
from cleany_perception.adapters.yoloe_detector import YoloeDetector
from cleany_perception.core.geometry import (
    deproject_masked_depth, fit_plane_ransac, reconstruct_supported_obb,
)
from cleany_perception.core.models import CameraIntrinsics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--model-directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--generic-cpu', action='store_true',
                        help='Disable optional CPU convolution kernels on unsupported hosts')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.generic_cpu:
        torch.backends.mkldnn.enabled = False
        torch.backends.nnpack.enabled = False
    metadata = json.loads((args.input / 'capture.json').read_text())
    width, height = metadata['aligned_resolution']
    k = metadata['K']
    intrinsics = CameraIntrinsics(width, height, k[0][0], k[1][1], k[0][2], k[1][2])
    rgb_path, depth_path = args.input/'color-000.rgb8', args.input/'depth-000.z16'
    rgb = np.fromfile(rgb_path, dtype=np.uint8).reshape(height, width, 3)
    depth = np.fromfile(depth_path, dtype='<u2').reshape(height, width)
    depth = (depth * metadata['depth_scale_m_per_unit']).astype(np.float32)
    config = ROOT/'ros2_ws/src/cleany_perception/config/yoloe_real32.yaml'
    classes = yaml.safe_load(config.read_text())['perception_inspector']['ros__parameters']['yoloe_classes']
    model = args.model_directory/'yoloe-26m-seg.pt'
    detector = YoloeDetector(str(model), classes, device='cpu', require_masks=True,
                            text_encoder_directory=str(args.model_directory))
    detections = detector.detect(rgb, 'Detect visible movable objects on the tabletop.')
    records = []
    for i, detection in enumerate(detections):
        mask = detection.segmentation_mask
        assert mask is not None and mask.shape == depth.shape
        Image.fromarray(mask.astype(np.uint8)*255).save(args.output/f'mask-{i:02d}.png')
        records.append({'index': i, 'label': detection.label, 'confidence': detection.confidence,
                        'mask_pixels': int(mask.sum()),
                        'bbox_xyxy': [detection.bbox.x_min, detection.bbox.y_min,
                                      detection.bbox.x_max, detection.bbox.y_max]})
    print(json.dumps(records), flush=True)
    candidates = list(enumerate(detections))
    if not candidates:
        raise RuntimeError('No target found; no manual mask is substituted')
    index, target = max(candidates, key=lambda item: item[1].confidence)
    mask = target.segmentation_mask
    support = np.ones(depth.shape, dtype=bool)
    for detection in detections:
        support &= ~detection.segmentation_mask
    support_points = deproject_masked_depth(depth, intrinsics, support, .15, 1.5, stride=4)
    plane = fit_plane_ransac(support_points, 300, .008, 100, .35)
    points = deproject_masked_depth(depth, intrinsics, mask, .15, 1.5, stride=1)
    box = reconstruct_supported_obb(points, plane, .006, .005, 30)
    corners_local = np.array(list(itertools.product((-1, 1), repeat=3))) * box.size / 2
    corners = corners_local @ box.rotation.T + box.center
    visible = points[(points @ plane.normal + plane.offset) >= .006]
    # A fixed sampling grid keeps the plotted cloud reproducible.
    visible = visible[::max(1, len(visible)//1800)]
    overlay = rgb.copy()
    overlay[mask] = (.5*rgb[mask]+.5*np.array([62, 216, 175])).astype(np.uint8)
    fig = plt.figure(figsize=(15, 5), dpi=120, facecolor='#f3f7f3')
    for slot, (title, array) in enumerate([('Detection', rgb), ('Segmentation', overlay)], 1):
        ax = fig.add_subplot(1, 3, slot)
        ax.imshow(array)
        fig.text(.015+(slot-1)*.33, .915, title, fontsize=16, color='#294e43')
        ax.axis('off')
        if slot == 1:
            bbox = target.bbox
            ax.add_patch(Rectangle((bbox.x_min, bbox.y_min), bbox.x_max-bbox.x_min,
                                   bbox.y_max-bbox.y_min, fill=False, edgecolor='#3ed8af', linewidth=2))
            ax.text(.02, .04, f'{target.label}  {target.confidence:.0%}', transform=ax.transAxes,
                    fontsize=11, color='white', bbox={'facecolor':'#294e43', 'edgecolor':'none', 'pad':6})
    ax = fig.add_subplot(1, 3, 3, projection='3d', facecolor='#f3f7f3')
    fig.text(.675, .915, '3D target', fontsize=16, color='#294e43')
    # Plot in the fitted plane basis, centered at the OBB. This is not base_link.
    p = (visible-box.center) @ box.rotation * 100
    c = (corners-box.center) @ box.rotation * 100
    ax.scatter(*p.T, c=p[:, 2], cmap='winter', s=3, alpha=.75)
    for a in range(8):
        for b in range(a+1, 8):
            if sum(corners_local[a] != corners_local[b]) == 1:
                ax.plot(*c[[a, b]].T, color='#d39b36', linewidth=2)
    extent = float(max(box.size)*100/2)*1.2
    for setter in (ax.set_xlim, ax.set_ylim, ax.set_zlim):
        setter(-extent, extent)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel('x (cm)', fontsize=9); ax.set_ylabel('y (cm)', fontsize=9)
    ax.set_zlabel('z (cm)', fontsize=9)
    ax.view_init(elev=24, azim=-55)
    ax.tick_params(labelsize=8)
    fig.subplots_adjust(left=.015, right=.98, bottom=.1, top=.88, wspace=.08)
    fig.savefig(args.output/'perception-pipeline.png', dpi=120)
    plt.close(fig)
    result = {
        'kind': 'offline inference on recorded aligned RGB-D',
        'input': str(args.input), 'frame': metadata['records'][0],
        'input_hashes': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in (rgb_path, depth_path, args.input/'capture.json')},
        'model': str(model), 'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
        'classes_config': str(config), 'detections': records, 'target_index': index,
        'camera_frame_center_m': box.center.tolist(), 'size_m': box.size.tolist(),
        'rotation_camera': box.rotation.tolist(), 'plane_normal': plane.normal.tolist(),
        'plane_offset_m': plane.offset, 'plane_inliers': plane.inlier_count,
        'object_points': len(points), 'plot_frame': 'OBB-centered support-plane basis',
        'core_functions': ['deproject_masked_depth', 'fit_plane_ransac', 'reconstruct_supported_obb'],
        'gemini_called': False, 'robot_command_sent': False,
        'generic_cpu': args.generic_cpu, 'labels': 'detector predictions, not ground truth',
    }
    (args.output/'perception-pipeline.json').write_text(json.dumps(result, indent=2)+'\n')
    print('PERCEPTION_COMPLETE', result['size_m'], flush=True)


if __name__ == '__main__':
    main()
