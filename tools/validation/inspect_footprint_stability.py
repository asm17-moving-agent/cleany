#!/usr/bin/env python3
"""Capture passive RGB-D samples and isolate footprint measurement variation."""
from __future__ import annotations

import argparse
from dataclasses import asdict, fields
import json
from pathlib import Path
import time

import numpy as np


def capture(output: Path, count: int, interval: float) -> None:
    """Subscribe to the inspector's inputs without sending an inspection Goal."""
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import CameraInfo, Image
    from cleany_perception.core.models import PipelineConfig
    from cleany_perception.rgbd_snapshot import RgbdSnapshotBuffer, snapshot_from_messages

    if (output / 'captures.npz').exists():
        raise FileExistsError('Use a new output directory to preserve captured evidence')
    rclpy.init()
    node = rclpy.create_node('footprint_diagnostic_capture')
    try:
        names = [f.name for f in fields(PipelineConfig)] + [
            'yoloe_model_path', 'yoloe_classes', 'yoloe_device', 'yoloe_image_size',
            'yoloe_iou_threshold', 'yoloe_text_encoder_directory',
            'yoloe_class_confidence_thresholds', 'depth_16u_scale_m',
            'color_image_topic', 'color_info_topic', 'depth_image_topic', 'depth_info_topic',
        ]
        client = node.create_client(GetParameters, '/perception_inspector/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Inspector parameters unavailable')
        parameters = {}
        for name in names:
            future = client.call_async(GetParameters.Request(names=[name]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise TimeoutError(f'Inspector parameter lookup timed out: {name}')
            values = future.result().values
            parameters[name] = parameter_value_to_python(values[0]) if values else None
        output.mkdir(parents=True, exist_ok=True)
        (output / 'parameters.json').write_text(json.dumps(parameters, indent=2))
        buffer = RgbdSnapshotBuffer()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        for msg_type, key, callback in (
            (Image, 'color_image_topic', buffer.add_color),
            (CameraInfo, 'color_info_topic', buffer.add_color_info),
            (Image, 'depth_image_topic', buffer.add_depth),
            (CameraInfo, 'depth_info_topic', buffer.add_depth_info),
        ):
            node.create_subscription(msg_type, parameters[key], callback, qos)
        snapshots, sequence, next_sample = [], 0, 0.0
        deadline = time.monotonic() + 15 + count * interval
        while len(snapshots) < count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if buffer.sequence <= sequence or time.monotonic() < next_sample:
                continue
            messages = buffer.wait_for_new(sequence, .1)
            sequence = messages.sequence
            snapshot = snapshot_from_messages(messages, parameters['depth_16u_scale_m'])
            snapshots.append(snapshot)
            next_sample = time.monotonic() + interval
        if len(snapshots) != count:
            raise TimeoutError(f'Captured {len(snapshots)}/{count} synchronized samples')
        np.savez_compressed(output / 'captures.npz',
            rgb=np.stack([s.rgb for s in snapshots]),
            depth=np.stack([s.depth_m for s in snapshots]),
            stamps=np.array([s.stamp_ns for s in snapshots], dtype=np.int64))
        metadata = dict(intrinsics=asdict(snapshots[0].intrinsics),
                        source_frame=snapshots[0].source_frame, count=count,
                        elapsed_s=(snapshots[-1].stamp_ns-snapshots[0].stamp_ns)/1e9)
        (output / 'capture.json').write_text(json.dumps(metadata, indent=2))
        print(json.dumps(metadata), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def analyze(output: Path, label: str) -> None:
    """Use the deployed local detector and geometry functions on saved inputs."""
    from cleany_perception.adapters.yoloe_detector import YoloeDetector
    from cleany_perception.core.geometry import deproject_masked_depth, reconstruct_supported_obb
    from cleany_perception.core.models import CameraIntrinsics, ObjectMask, PipelineConfig, RgbdSnapshot
    from cleany_perception.core.pipeline import InspectionPipeline

    parameters = json.loads((output / 'parameters.json').read_text())
    config = PipelineConfig(**{f.name: parameters[f.name] for f in fields(PipelineConfig)})
    metadata = json.loads((output / 'capture.json').read_text())
    arrays = np.load(output / 'captures.npz')
    detector = YoloeDetector(parameters['yoloe_model_path'], parameters['yoloe_classes'],
        device=parameters['yoloe_device'], image_size=parameters['yoloe_image_size'],
        confidence_threshold=config.minimum_detection_confidence,
        iou_threshold=parameters['yoloe_iou_threshold'], maximum_detections=config.maximum_detections,
        text_encoder_directory=parameters['yoloe_text_encoder_directory'], require_masks=True,
        class_confidence_thresholds=parameters['yoloe_class_confidence_thresholds'] or ())
    detector.prepare()
    pipeline = InspectionPipeline(detector, None, None, 'base_link', config)
    snapshots, detections, targets, planes, points_list, rows = [], [], [], [], [], []
    intrinsics = CameraIntrinsics(**metadata['intrinsics'])
    for index, stamp in enumerate(arrays['stamps']):
        snapshot = RgbdSnapshot(arrays['rgb'][index], arrays['depth'][index], intrinsics,
                               int(stamp), metadata['source_frame'])
        detected = pipeline._validate_detections(snapshot, detector.detect(snapshot.rgb, ''))
        candidates = [d for d in detected if d.label == label]
        if len(candidates) != 1:
            raise ValueError(f'Sample {index}: expected one {label}, found {len(candidates)}')
        target = candidates[0]
        plane, boxes = pipeline._reconstruct(snapshot, detected,
            (ObjectMask(target, target.segmentation_mask, target.confidence),), True)
        points = deproject_masked_depth(snapshot.depth_m, intrinsics, target.segmentation_mask,
                                        config.minimum_depth_m, config.maximum_depth_m)
        snapshots.append(snapshot)
        detections.append(detected)
        targets.append(target)
        planes.append(plane)
        points_list.append(points)
        rows.append(dict(index=index, stamp_ns=int(stamp), label=target.label,
            bbox=asdict(target.bbox), confidence=target.confidence, mask_pixels=int(target.segmentation_mask.sum()),
            plane_normal=plane.normal.tolist(), plane_offset_m=plane.offset,
            plane_inliers=plane.inlier_count, size_m=boxes[0].size.tolist()))
        print(f'sample {index}: size={boxes[0].size.tolist()}', flush=True)

    def box_for(points, plane):
        return reconstruct_supported_obb(points, plane, config.minimum_object_height_m,
                                          config.minimum_obb_extent_m, config.minimum_object_points)

    for index, row in enumerate(rows):
        # Counterfactuals change one input family while keeping the rest fixed.
        fixed_mask_points = deproject_masked_depth(snapshots[index].depth_m, intrinsics,
            targets[0].segmentation_mask, config.minimum_depth_m, config.maximum_depth_m)
        fixed_depth_points = deproject_masked_depth(snapshots[0].depth_m, intrinsics,
            targets[index].segmentation_mask, config.minimum_depth_m, config.maximum_depth_m)
        row['depth_only_size_m'] = box_for(fixed_mask_points, planes[0]).size.tolist()
        row['mask_only_size_m'] = box_for(fixed_depth_points, planes[0]).size.tolist()
        row['plane_only_size_m'] = box_for(points_list[0], planes[index]).size.tolist()
        row['plane_angle_to_first_deg'] = float(np.degrees(np.arccos(np.clip(
            planes[0].normal @ planes[index].normal, -1., 1.))))
        mask_a, mask_b = targets[0].segmentation_mask, targets[index].segmentation_mask
        row['mask_iou_to_first'] = float(np.count_nonzero(mask_a & mask_b)/np.count_nonzero(mask_a | mask_b))
        points, plane = points_list[index], planes[index]
        heights = points @ plane.normal + plane.offset
        kept = points[heights >= config.minimum_object_height_m]
        row['above_plane_points'] = len(kept)
        row['depth_quantiles_m'] = np.quantile(points[:, 2], [0,.01,.05,.5,.95,.99,1]).tolist()
        box = box_for(points, plane)
        local = (kept-box.center) @ box.rotation
        row['trimmed_1pct_same_axes_m'] = np.diff(np.quantile(local[:, :2], [.01,.99], axis=0), axis=0)[0].tolist()
        row['trimmed_5pct_same_axes_m'] = np.diff(np.quantile(local[:, :2], [.05,.95], axis=0), axis=0)[0].tolist()
        row['extent_endpoint_depths_m'] = [
            [float(kept[np.argmin(local[:, axis]), 2]), float(kept[np.argmax(local[:, axis]), 2])]
            for axis in (0, 1)]
    np.savez_compressed(output / 'masks.npz', masks=np.stack([t.segmentation_mask for t in targets]))
    report = dict(scope='New passive observations; not the expired historical RGB-D frames',
                  config=asdict(config), observations=rows)
    (output / 'analysis.json').write_text(json.dumps(report, indent=2))
    for key in ('size_m', 'depth_only_size_m', 'mask_only_size_m', 'plane_only_size_m'):
        widths = [min(row[key][:2])*100 for row in rows]
        print(key, 'smaller_extent_cm_range', min(widths), max(widths),
              'over_11cm',sum(w > 11 for w in widths), '/', len(widths), flush=True)


def report(output: Path, depth_split: float) -> None:
    """Visualize observed contamination; the depth split is diagnostic only."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from cleany_perception.core.geometry import deproject_masked_depth, reconstruct_supported_obb
    from cleany_perception.core.models import CameraIntrinsics, Plane

    raw = np.load(output / 'captures.npz')
    masks = np.load(output / 'masks.npz')['masks']
    analysis = json.loads((output / 'analysis.json').read_text())
    rows, config = analysis['observations'], analysis['config']
    intrinsics = CameraIntrinsics(**json.loads((output / 'capture.json').read_text())['intrinsics'])
    diagnostics = []
    for index, row in enumerate(rows):
        points = deproject_masked_depth(raw['depth'][index], intrinsics, masks[index],
                                       config['minimum_depth_m'], config['maximum_depth_m'])
        plane = Plane(np.array(row['plane_normal']), row['plane_offset_m'], row['plane_inliers'])
        heights = points @ plane.normal + plane.offset
        retained = heights >= config['minimum_object_height_m']
        far = retained & (points[:, 2] > depth_split)
        box = reconstruct_supported_obb(points[points[:, 2] <= depth_split], plane,
            config['minimum_object_height_m'], config['minimum_obb_extent_m'], config['minimum_object_points'])
        valid = masks[index] & np.isfinite(raw['depth'][index])
        valid &= (raw['depth'][index] >= config['minimum_depth_m']) & (raw['depth'][index] <= config['maximum_depth_m'])
        y, x = np.nonzero(valid)
        diagnostics.append(dict(index=index, retained_points=int(retained.sum()),
            far_points=int(far.sum()), far_pixels=np.column_stack((x[far], y[far])).tolist(),
            far_depth_m=points[far, 2].tolist(), far_height_mm=(heights[far]*1000).tolist(),
            without_far_size_m=box.size.tolist()))
    summary = dict(depth_split_m=depth_split, scope='Diagnostic only; no runtime filter change',
                   observations=diagnostics)
    (output / 'diagnostics.json').write_text(json.dumps(summary, indent=2))
    actual = np.array([min(r['size_m'][:2])*100 for r in rows])
    clean = np.array([min(r['without_far_size_m'][:2])*100 for r in diagnostics])
    print('without_far_cm_range',float(clean.min()),float(clean.max()))
    print('far_points_range',min(r['far_points'] for r in diagnostics),max(r['far_points'] for r in diagnostics))
    example = int(np.argmax(actual))
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    axes[0, 0].imshow(raw['rgb'][example])
    axes[0, 0].contour(masks[example], levels=[.5], colors=['#16a085'], linewidths=.8)
    xy = np.array(diagnostics[example]['far_pixels'])
    if xy.size:
        axes[0, 0].scatter(xy[:, 0], xy[:, 1], s=27, c='#e63946', label='Far depth retained as object')
    bbox = rows[example]['bbox']
    axes[0, 0].set_xlim(bbox['x_min']-18, bbox['x_max']+18)
    axes[0, 0].set_ylim(bbox['y_max']+12, bbox['y_min']-12)
    axes[0, 0].set_title(f'Sample {example+1}: {diagnostics[example]["far_points"]} contaminating pixels')
    axes[0, 0].legend(loc='lower left', fontsize=8)
    axes[0, 0].axis('off')
    depth = raw['depth'][example][masks[example]]
    depth = depth[np.isfinite(depth) & (depth >= config['minimum_depth_m']) & (depth <= config['maximum_depth_m'])]
    axes[0, 1].hist(depth*100, bins=70, color='#457b9d', log=True)
    axes[0, 1].axvline(depth_split*100, color='#e63946', ls='--', label='Diagnostic split')
    axes[0, 1].set(xlabel='Camera depth (cm)', ylabel='Pixels (log scale)', title='Depth inside the can mask')
    axes[0, 1].legend()
    sample_numbers = np.arange(1, len(rows)+1)
    axes[1, 0].plot(sample_numbers, actual, 'o-', color='#e63946', label='Current measurement')
    axes[1, 0].plot(sample_numbers, clean, 'o-', color='#16a085', label='Diagnostic: remove far depth points')
    axes[1, 0].axhline(11, color='#333333', ls='--', label='Current size limit: 11 cm')
    axes[1, 0].set(xlabel='Passive sample', ylabel='Smaller footprint extent (cm)', title='Same scene, 20 RGB-D captures')
    axes[1, 0].legend(fontsize=8)
    for key, label, color in (
        ('depth_only_size_m', 'Depth changes; mask/plane fixed', '#e63946'),
        ('mask_only_size_m', 'Mask changes; depth/plane fixed', '#457b9d'),
        ('plane_only_size_m', 'Plane changes; points fixed', '#f4a261'),
    ):
        axes[1, 1].plot(sample_numbers, [min(r[key][:2])*100 for r in rows], '.-', label=label, color=color)
    axes[1, 1].set(xlabel='Passive sample', ylabel='Smaller footprint extent (cm)', title='Separate input effects')
    axes[1, 1].legend(fontsize=8)
    fig.suptitle('Can footprint instability: passive recapture, not historical frame replay', fontsize=14)
    fig.savefig(output / 'diagnostic.png', dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('capture', 'analyze', 'report'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--count', type=int, default=16)
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('--label', default='beverage can')
    parser.add_argument('--depth-split-m', type=float,
                        help='Diagnostic separation selected from the measured depth distribution')
    args = parser.parse_args()
    if args.count < 2 or args.interval <= 0:
        parser.error('count must be at least two and interval positive')
    if args.mode == 'capture':
        capture(args.output, args.count, args.interval)
    elif args.mode == 'analyze':
        analyze(args.output, args.label)
    else:
        if args.depth_split_m is None or not np.isfinite(args.depth_split_m) or args.depth_split_m <= 0:
            parser.error('report requires a positive --depth-split-m')
        report(args.output, args.depth_split_m)


if __name__ == '__main__':
    main()
