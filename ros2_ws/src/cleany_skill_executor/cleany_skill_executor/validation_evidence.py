"""Optional JSON and 3x3 image evidence for plan-only pick validation."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw, ImageFont
from sensor_msgs.msg import Image

from cleany_skill_executor.core.target_filter import TargetDecision


def image_message_to_pil(message: Image | None) -> PilImage.Image | None:
    if message is None or message.encoding not in ('rgb8', 'bgr8'):
        return None
    rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        int(message.height), int(message.step)
    )
    array = rows[:, : int(message.width) * 3].reshape(
        int(message.height), int(message.width), 3
    )
    if message.encoding == 'bgr8':
        array = array[:, :, ::-1]
    return PilImage.fromarray(np.ascontiguousarray(array))


def filter_overlay(
    base: PilImage.Image | None,
    detections: Sequence[Any],
    decisions: Sequence[TargetDecision],
    selected_object_id: int,
) -> PilImage.Image:
    image = (
        base.convert('RGB').copy()
        if base is not None
        else PilImage.new('RGB', (640, 480), (25, 29, 38))
    )
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    by_id = {item.object_id: item for item in decisions}
    for detection in detections:
        object_id = int(detection.object_id)
        decision = by_id[object_id]
        selected = object_id == selected_object_id
        color = (40, 230, 90) if selected else (
            (240, 75, 75) if not decision.eligible else (255, 190, 45)
        )
        width = 5 if selected else 3
        box = (
            float(detection.x_min), float(detection.y_min),
            float(detection.x_max), float(detection.y_max),
        )
        draw.rectangle(box, outline=color, width=width)
        state = 'SELECTED' if selected else decision.detail
        label = f'{object_id} {detection.label}: {state}'
        top = max(0.0, float(detection.y_min) - 14.0)
        draw.rectangle(
            (float(detection.x_min), top, float(detection.x_min) + 7 * len(label), top + 13),
            fill=(0, 0, 0),
        )
        draw.text((float(detection.x_min) + 2, top + 1), label, fill=color, font=font)
    return image


def _status_panel(title: str, lines: Sequence[str]) -> PilImage.Image:
    image = PilImage.new('RGB', (640, 480), (24, 29, 39))
    draw = ImageDraw.Draw(image)
    draw.text((24, 24), title, fill=(105, 205, 255))
    draw.multiline_text((24, 62), '\n'.join(lines), fill=(235, 240, 248), spacing=10)
    return image


def _crop_selected(base: PilImage.Image | None, detection: Any) -> PilImage.Image:
    if base is None:
        return _status_panel('Selected target', ['RGB image unavailable'])
    margin = 24
    box = (
        max(0, int(detection.x_min) - margin),
        max(0, int(detection.y_min) - margin),
        min(base.width, int(detection.x_max) + margin),
        min(base.height, int(detection.y_max) + margin),
    )
    crop = base.crop(box)
    crop.thumbnail((640, 480), PilImage.Resampling.LANCZOS)
    canvas = PilImage.new('RGB', (640, 480), (24, 29, 39))
    canvas.paste(crop, ((640 - crop.width) // 2, (480 - crop.height) // 2))
    return canvas


def write_validation_evidence(
    output_root: Path,
    *,
    snapshot_id: str,
    query: str,
    detections: Sequence[Any],
    decisions: Sequence[TargetDecision],
    selected_detection: Any,
    selected_object: Any,
    candidates: Sequence[Any],
    selection: Any,
    latencies: Sequence[tuple[str, float]],
    phase1_debug: Image | None,
    phase2_debug: Image | None,
    grasp_debug: Image | None,
    target_attempts: Sequence[dict[str, Any]] = (),
) -> tuple[str, str]:
    scene_dir = output_root / snapshot_id
    scene_dir.mkdir(parents=True, exist_ok=True)
    phase1 = image_message_to_pil(phase1_debug)
    phase2 = image_message_to_pil(phase2_debug)
    grasp = image_message_to_pil(grasp_debug)
    selected_id = int(selected_detection.object_id)
    panels = (
        phase1 or _status_panel('Object detections', ['debug image unavailable']),
        filter_overlay(phase1, detections, decisions, selected_id),
        _crop_selected(phase1, selected_detection),
        phase2 or _status_panel('YOLOE-seg mask', ['debug image unavailable']),
        _status_panel(
            'Selected 3D object',
            [
                f'id={selected_object.object_id} label={selected_object.label}',
                f'depth={selected_detection.median_depth_m:.3f} m',
                f'bbox size={selected_detection.estimated_width_m:.3f} x '
                f'{selected_detection.estimated_height_m:.3f} m',
                f'OBB={selected_object.obb_size.x:.3f} x '
                f'{selected_object.obb_size.y:.3f} x {selected_object.obb_size.z:.3f} m',
            ],
        ),
        grasp or _status_panel('AnyGrasp', ['debug image unavailable']),
        _status_panel(
            'Requested grasp',
            [
                f'candidate={selection.selected_candidate_index}/{len(candidates)}',
                f'score={selection.selected_candidate.score:.4f}',
                f'opening={selection.selected_candidate.required_opening_m:.3f} m',
                f'depth={selection.selected_candidate.grasp_depth_m:.3f} m',
                f'arm={selection.selected_arm}',
            ],
        ),
        _status_panel(
            'Robot FK alignment',
            [
                f'pre pos={selection.pregrasp_position_error_m:.4f} m',
                f'pre axes={selection.pregrasp_approach_error_degrees:.1f}/'
                f'{selection.pregrasp_closing_error_degrees:.1f} deg',
                f'grasp pos={selection.grasp_position_error_m:.4f} m',
                f'grasp axes={selection.grasp_approach_error_degrees:.1f}/'
                f'{selection.grasp_closing_error_degrees:.1f} deg',
                f'trajectory max={selection.trajectory_max_approach_error_degrees:.1f}/'
                f'{selection.trajectory_max_closing_error_degrees:.1f} deg',
            ],
        ),
        _status_panel(
            'Plan-only result',
            [
                f'planner={selection.planner_id}',
                f'gripper joint={selection.gripper_joint_position:.4f} rad',
                *(f'{name}: {seconds:.3f} s' for name, seconds in latencies),
            ],
        ),
    )
    titles = (
        '1 detections', '2 denylist/size filter', '3 selected RGB',
        '4 YOLOE-seg selected', '5 selected 3D', '6 AnyGrasp candidates',
        '7 requested grasp', '8 actual FK', '9 motion plan-only',
    )
    cell_width, cell_height, title_height = 640, 480, 28
    grid = PilImage.new(
        'RGB',
        (cell_width * 3, (cell_height + title_height) * 3),
        (14, 18, 25),
    )
    draw = ImageDraw.Draw(grid)
    for index, (panel, title) in enumerate(zip(panels, titles, strict=True)):
        panel = panel.convert('RGB')
        panel.thumbnail((cell_width, cell_height), PilImage.Resampling.LANCZOS)
        column, row = index % 3, index // 3
        x = column * cell_width + (cell_width - panel.width) // 2
        y0 = row * (cell_height + title_height)
        y = y0 + title_height + (cell_height - panel.height) // 2
        grid.paste(panel, (x, y))
        draw.text((column * cell_width + 8, y0 + 8), title, fill=(235, 240, 248))
    grid_path = scene_dir / 'pipeline_grid_3x3.png'
    grid.save(grid_path)

    payload = {
        'schema': 'cleany.validate_pick/v1',
        'snapshot_id': snapshot_id,
        'query': query,
        'detections': [
            {
                'object_id': int(item.object_id),
                'label': str(item.label),
                'confidence': float(item.confidence),
                'median_depth_m': float(item.median_depth_m),
                'estimated_width_m': float(item.estimated_width_m),
                'estimated_height_m': float(item.estimated_height_m),
                'decision': asdict(decisions[index]),
            }
            for index, item in enumerate(detections)
        ],
        'selected_object_id': selected_id,
        'candidate_count': len(candidates),
        'selected_candidate_index': int(selection.selected_candidate_index),
        'selected_arm': str(selection.selected_arm),
        'requested_grasp': {
            'position': [
                float(selection.selected_candidate.tcp_pose.position.x),
                float(selection.selected_candidate.tcp_pose.position.y),
                float(selection.selected_candidate.tcp_pose.position.z),
            ],
            'orientation_xyzw': [
                float(selection.selected_candidate.tcp_pose.orientation.x),
                float(selection.selected_candidate.tcp_pose.orientation.y),
                float(selection.selected_candidate.tcp_pose.orientation.z),
                float(selection.selected_candidate.tcp_pose.orientation.w),
            ],
            'opening_m': float(selection.selected_candidate.required_opening_m),
            'depth_m': float(selection.selected_candidate.grasp_depth_m),
            'score': float(selection.selected_candidate.score),
        },
        'moveit': {
            'planner_id': str(selection.planner_id),
            'plan_only': True,
            'gripper_joint_position': float(selection.gripper_joint_position),
            'pregrasp_position_error_m': float(selection.pregrasp_position_error_m),
            'grasp_position_error_m': float(selection.grasp_position_error_m),
            'pregrasp_axis_error_degrees': [
                float(selection.pregrasp_approach_error_degrees),
                float(selection.pregrasp_closing_error_degrees),
            ],
            'grasp_axis_error_degrees': [
                float(selection.grasp_approach_error_degrees),
                float(selection.grasp_closing_error_degrees),
            ],
            'trajectory_axis_error_degrees': [
                float(selection.trajectory_max_approach_error_degrees),
                float(selection.trajectory_max_closing_error_degrees),
            ],
        },
        'latency_seconds': dict(latencies),
        'target_attempts': list(target_attempts),
    }
    json_path = scene_dir / 'result.json'
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=str) + '\n',
        encoding='utf-8',
    )
    _write_summary_grid(output_root)
    return str(json_path), str(grid_path)


def write_target_attempt_evidence(
    output_root: Path, *, snapshot_id: str, attempts: Sequence[dict[str, Any]],
    success: bool, error_code: int, message: str,
) -> str:
    """Retain all object outcomes even when no selected grasp/grid exists."""
    scene_dir = output_root / snapshot_id
    scene_dir.mkdir(parents=True, exist_ok=True)
    path = scene_dir / 'target_attempts.json'
    path.write_text(json.dumps({
        'schema': 'cleany.target_attempts/v1', 'snapshot_id': snapshot_id,
        'success': bool(success), 'error_code': int(error_code), 'message': message,
        'target_attempts': list(attempts),
    }, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return str(path)


def _write_summary_grid(output_root: Path) -> None:
    paths = sorted(output_root.glob('*/pipeline_grid_3x3.png'))
    if not paths:
        return
    cell_width, cell_height = 640, 500
    columns = min(4, len(paths))
    rows = (len(paths) + columns - 1) // columns
    canvas = PilImage.new(
        'RGB', (columns * cell_width, rows * cell_height), (14, 18, 25)
    )
    draw = ImageDraw.Draw(canvas)
    for index, path in enumerate(paths):
        image = PilImage.open(path).convert('RGB')
        image.thumbnail((cell_width, cell_height - 28), PilImage.Resampling.LANCZOS)
        column, row = index % columns, index // columns
        x = column * cell_width + (cell_width - image.width) // 2
        y = row * cell_height + 28
        canvas.paste(image, (x, y))
        draw.text((column * cell_width + 8, row * cell_height + 8), path.parent.name, fill=(235, 240, 248))
    canvas.save(output_root / 'validate_pick_summary_grid.png')
