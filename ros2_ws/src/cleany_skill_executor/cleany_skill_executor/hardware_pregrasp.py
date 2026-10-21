"""Execute one live perception-to-pregrasp cycle on the physical robot."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import threading

import rclpy
from cleany_interfaces.action import ValidatePick
from cleany_interfaces.srv import ObserveObjectReference
from rclpy.executors import MultiThreadedExecutor

from .pregrasp_pipeline import PregraspPipeline


class HardwarePregraspPipeline(PregraspPipeline):
    """Physical execution that stops after the selected pregrasp trajectory."""

    def __init__(self, **kwargs):
        super().__init__(mode='hardware', **kwargs)

    def before_execution(self, selected):
        self.real.verify(selected.selected_arm)

    def run_pregrasp(self, query: str):
        report = {'mode': 'hardware_pregrasp', 'success': False,
                  'pregrasp_executed': False, 'grasp_executed': False}
        selected = None
        execution_started = False
        try:
            self.scene_monitor.wait()
            selected = self.run_action(
                self.validation, ValidatePick.Goal(query=query),
                self.get_parameter('pipeline_timeout_sec').value,
                allow_aborted=True,
            )
            report['capture_stamp_ns'] = (
                selected.detections.header.stamp.sec * 10**9
                + selected.detections.header.stamp.nanosec
            )
            if not selected.success:
                report.update(error_code=selected.error_code,
                              detection_count=len(selected.detections.detections))
                raise RuntimeError(selected.message)
            report.update(
                arm=selected.selected_arm,
                candidate_index=selected.selected_candidate_index,
                planner_id=selected.planner_id,
                evidence_json_path=selected.evidence_json_path,
                detection_count=len(selected.detections.detections),
                candidate_count=len(selected.candidates),
            )
            if not selected.reference_id:
                raise RuntimeError('Enable pin_object_reference and YOLOE-seg reference observation')
            before = self.observe(selected.reference_id, self.get_clock().now().nanoseconds)
            center = selected.selected_object.obb_pose.position
            observed = before.observed_center
            extent = selected.selected_object.obb_size
            bound = math.sqrt(extent.x**2 + extent.y**2 + extent.z**2) / 2
            if math.dist(
                (center.x, center.y, center.z),
                (observed.x, observed.y, observed.z),
            ) > bound + .01:
                raise RuntimeError('Fresh target surface is outside the planned object; replan required')
            self.before_execution(selected)
            execution_started = True
            arrival = self.execute_pregrasp(selected)
            report['pregrasp_executed'] = True
            report['arrival_stamp_ns'] = arrival
            after = self.observe(selected.reference_id, arrival)
            old = (before.observed_center.x, before.observed_center.y, before.observed_center.z)
            new = (after.observed_center.x, after.observed_center.y, after.observed_center.z)
            displacement = math.dist(old, new)
            report['observed_surface_displacement_m'] = displacement
            if displacement > self.get_parameter('maximum_target_translation_m').value:
                raise RuntimeError('Target surface changed; a new grasp plan is required')
            report.update(success=True, message='Physical pregrasp reached and verified; no grasp requested')
        except Exception as error:
            report['message'] = str(error)
            if execution_started:
                report['stop'] = self.real.fail_stop()
        finally:
            if selected and selected.reference_id:
                request = ObserveObjectReference.Request(
                    operation=ObserveObjectReference.Request.CLEAR,
                    reference_id=selected.reference_id,
                )
                try:
                    self.call(self.reference, request, 10.)
                except Exception as error:
                    report['cleanup_error'] = str(error)
                    report['success'] = False
        return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--query', default='')
    parser.add_argument('--output', required=True)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = HardwarePregraspPipeline()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    report = {'success': False, 'message': 'Interrupted'}
    try:
        report = node.run_pregrasp(options.query)
    finally:
        output = Path(options.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(report, ensure_ascii=False))
        node.scene_monitor.close()
        executor.remove_node(node)
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if report['success'] else 1)
