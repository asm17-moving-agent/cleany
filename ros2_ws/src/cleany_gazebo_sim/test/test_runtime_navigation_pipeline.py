"""Opt-in CAD/Gazebo test of the common full navigation pipeline."""
from __future__ import annotations

import json
import math
import hashlib
import os
from pathlib import Path
import signal
import subprocess
from uuid import uuid4
from xml.etree import ElementTree as ET

import numpy as np
import yaml

import pytest


def test_cad_gazebo_full_pipeline_reaches_saved_map_goal(runtime_test_options, tmp_path, monkeypatch):
    if runtime_test_options.profile != 'harmonic':
        pytest.skip('The saved-map CAD evaluation fixture currently targets Harmonic')
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    tools = Path(__file__).resolve().parents[4]/'tools/navigation_evaluation'
    monkeypatch.syspath_prepend(str(tools))
    from run_safety_evaluation import prepare
    from safety_probe import SafetyProbe
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('GZ_PARTITION', 'cleany_navigation_'+uuid4().hex)
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    scenario = prepare(tmp_path, share/'maps/study_cafe_30cm.yaml', robot_model='cad_frame',
                       obstacle_memory=True, guard_3d=True)
    # Use a matched open-room map/world. The historical study-cafe map was
    # recorded with a different model and is not a CAD localization benchmark.
    tree = ET.parse(tmp_path/'world.sdf')
    world = tree.getroot().find('world')
    for model in list(world.findall('model')):
        if model.get('name') not in ('ground_plane', 'cleany_mecanum'):
            world.remove(model)
    spawn = [0., 0., .38, 0., 0., 0.]
    world.find("model[@name='cleany_mecanum']/pose").text = ' '.join(map(str, spawn))
    for name, pose, size in [('north', '0 3.05 1 0 0 0', '6.2 .1 2'),
                             ('south', '0 -3.05 1 0 0 0', '6.2 .1 2'),
                             ('east', '3.05 0 1 0 0 0', '.1 6.2 2'),
                             ('west', '-3.05 0 1 0 0 0', '.1 6.2 2')]:
        world.append(ET.fromstring(f'<model name="test_wall_{name}"><static>true</static>'
            f'<pose>{pose}</pose><link name="wall"><collision name="wall"><geometry>'
            f'<box><size>{size}</size></box></geometry></collision><visual name="wall">'
            f'<geometry><box><size>{size}</size></box></geometry></visual></link></model>'))
    tree.write(tmp_path/'world.sdf', encoding='unicode')
    pixels = np.full((124, 124), 254, dtype=np.uint8)
    pixels[:2] = pixels[-2:] = 0
    pixels[:, :2] = pixels[:, -2:] = 0
    (tmp_path/'map.pgm').write_bytes(b'P5\n124 124\n255\n'+pixels.tobytes())
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.05,
        origin=[-3.1, -3.1, 0.], negate=0, occupied_thresh=.65, free_thresh=.2)))
    scenario['spawn_pose'] = spawn
    # Known exact spawn and ideal odometry isolate motion policy from relocalization.
    scenario['initial_pose_covariance'] = [.005**2, .005**2, math.radians(.5)**2]
    scenario['map_source'] = 'test fixture: matched 6m open room'
    scenario['map_sha256'] = hashlib.sha256((tmp_path/'map.pgm').read_bytes()).hexdigest()
    (tmp_path/'scenario.json').write_text(json.dumps(scenario, indent=2))
    # Use a tighter goal checker so pose-estimator drift does not consume the
    # ground-truth progress bound while exercising each motion direction.
    axis_file = tmp_path/'config/nav2_axis_motion.yaml'
    axis = yaml.safe_load(axis_file.read_text())
    controller = axis['controller_server']['ros__parameters']
    controller['goal_checker'] = {'xy_goal_tolerance': .05, 'yaw_goal_tolerance': .08}
    controller['FollowPath']['xy_goal_tolerance'] = .05
    axis_file.write_text(yaml.safe_dump(axis))
    scenario['evaluation']['avoid']['goal'] = [.6, 0., 0.]
    scenario['evaluation']['avoid']['timeout_wall_s'] = 120.
    process = node = None
    rclpy.init()
    try:
        with (tmp_path/'launch.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', str(tmp_path/'run.launch.py')],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node = SafetyProbe(tmp_path, scenario)
            node.initialize()
            results = []
            commands = {topic: [] for topic in ('/nav2/cmd_vel', '/pan_ready/cmd_vel',
                                                '/safety_2d/cmd_vel', '/cmd_vel')}
            from geometry_msgs.msg import Twist
            for topic in commands:
                node.create_subscription(Twist, topic, lambda msg, topic=topic:
                    commands[topic].append((msg.linear.x, msg.linear.y, msg.angular.z)), 10)
            for goal in ([.6, 0., 0.], [0., 0., 0.], [0., .6, math.pi/2]):
                scenario['evaluation']['avoid']['goal'] = goal
                result = node.run_avoid(inject_obstacle=False)
                results.append(dict(goal=goal, **result))
                (tmp_path/'result.json').write_text(json.dumps(results, indent=2))
                assert result['checks']['nav2_succeeded'], result
                # Physical fixture error includes AMCL offset as well as goal tolerance.
                assert math.dist(result['final_truth'][:2], goal[:2]) < .2, result
                yaw_error = result['final_truth'][2]-goal[2]
                assert abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))) < .2, result
            for topic, samples in commands.items():
                assert samples, topic
                assert all(abs(y) < 1e-6 and not (abs(x) > 1e-6 and abs(w) > 1e-6)
                           for x, y, w in samples), (topic, samples)
                assert any(x > .01 for x, _, _ in samples), topic
                assert any(x < -.01 for x, _, _ in samples), topic
                assert any(abs(w) > .01 for _, _, w in samples), topic
            (tmp_path/'commands.json').write_text(json.dumps(commands))
            node.wait(1.)
            output = node.latest.get('/cmd_vel')
            assert output is not None
            assert max(abs(output.linear.x), abs(output.linear.y), abs(output.angular.z)) < .001
    except Exception:
        print((tmp_path/'launch.log').read_text()[-16000:])
        raise
    finally:
        if node is not None:
            node.save()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
