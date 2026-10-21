"""Opt-in installed launch/profile/Python wiring check against real ROS services."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import pytest
from test_curobo_gpu import ros_node

pytestmark=pytest.mark.skipif(os.environ.get('CLEANY_RUN_HYBRID_APPLIED')!='1',
    reason='Enable with make test-motion-hybrid-applied-gpu')


def test_standalone_hybrid_launch_uses_pinned_python_and_retry_parameters(ros_node,tmp_path):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    root=Path(__file__).resolve().parents[4]
    log=tmp_path/'selector-launch.log'
    environment=dict(os.environ,ROS_DOMAIN_ID=str(ros_node.context.get_domain_id()))
    with log.open('w') as output:
        process=subprocess.Popen(['ros2','launch','cleany_skill_executor','grasp_selection.launch.py',
            'motion_backend:=curobo_ik_moveit','motion_runtime:=native',f'repository_root:={root}'],
            env=environment,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        client=ros_node.create_client(GetParameters,'/grasp_selection_server/get_parameters')
        try:
            end=time.monotonic()+45.
            while not client.wait_for_service(timeout_sec=.2):
                assert process.poll() is None, log.read_text()[-4000:]
                assert time.monotonic()<end, log.read_text()[-4000:]
            names=['motion_backend','maximum_candidates','curobo_ik_primary_seeds',
                   'curobo_ik_fallback_seeds','curobo_ik_retry_failed_candidates',
                   'curobo_ik_pregrasp_retry_passes','curobo_ik_cpu_fallback',
                   'curobo_planning_retries','validate_motion_paths']
            future=client.call_async(GetParameters.Request(names=names))
            while not future.done():
                assert process.poll() is None and time.monotonic()<end
                rclpy.spin_once(ros_node,timeout_sec=.02)
            values=future.result().values
            assert values[0].string_value=='curobo_ik_moveit'
            assert [v.integer_value for v in values[1:4]]==[32,16,32]
            assert values[4].bool_value and values[5].integer_value==0
            assert not values[6].bool_value and values[7].integer_value==1
            assert values[8].bool_value
            pid=int(re.search(r'process started with pid \[(\d+)\]',log.read_text()).group(1))
            command=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
            assert command[0].decode()==str(root/'runtime/curobo-venv/bin/python')
            receipt=dict(scope='installed standalone hybrid launch / native ROS parameter services',
                motion_backend=values[0].string_value,pinned_python=command[0].decode(),
                maximum_candidates=32,primary_seeds=16,fallback_seeds=32,
                failed_candidate_retries=True,primary_retry_passes=0,
                cpu_ik_fallback=False,planning_retries=1,validate_motion_paths=True,
                physical_motors_accessed=False,cuda_ik_executed=False)
            (tmp_path/'launch-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        finally:
            ros_node.destroy_client(client)
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=10.)
