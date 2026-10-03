"""Contract tests for the observation-only rqt launch."""
import ast
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext


SOURCE = Path(__file__).parents[1] / 'launch' / 'base_monitor.launch.py'
spec = importlib.util.spec_from_file_location('base_monitor_launch', SOURCE)
monitor_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor_launch)


def nodes_for(wheel='fl'):
    context = LaunchContext()
    context.launch_configurations['wheel'] = wheel
    return monitor_launch._nodes(context)


def arguments(node):
    return node._Node__arguments


def test_default_wheel_curves_and_gui_only_graph():
    speed, pwm, diagnostic = nodes_for('fl')
    assert arguments(speed) == ['--force-discover', '--empty',
        '/base/wheel_state/target_rad_s[0]',
        '/base/wheel_state/commanded_rad_s[0]',
        '/base/wheel_state/velocity_rad_s[0]']
    assert arguments(pwm) == ['--force-discover', '--empty',
                             '/base/wheel_state/pwm_percent[0]']
    remap = diagnostic._Node__remappings[0]
    assert [part.text for side in remap for part in side] == [
        'diagnostics_agg', '/diagnostics']
    assert len({node._Node__node_name for node in (speed, pwm, diagnostic)}) == 3
    assert [(node._Node__package, node._Node__node_executable)
            for node in (speed, pwm, diagnostic)] == [
        ('rqt_plot', 'rqt_plot'), ('rqt_plot', 'rqt_plot'),
        ('rqt_robot_monitor', 'rqt_robot_monitor')]


def test_all_wheels_curves_preserve_wire_order():
    speed, pwm, _ = nodes_for('all')
    assert arguments(speed)[2:] == [
        f'/base/wheel_state/{field}[{index}]'
        for index in range(4)
        for field in ('target_rad_s', 'commanded_rad_s', 'velocity_rad_s')]
    assert arguments(pwm)[2:] == [
        f'/base/wheel_state/pwm_percent[{index}]' for index in range(4)]


@pytest.mark.parametrize('wheel', ['fr', 'rl', 'rr'])
def test_single_wheel_indices(wheel):
    speed, pwm, _ = nodes_for(wheel)
    index = monitor_launch.WHEELS[wheel]
    assert arguments(speed)[2:] == [
        f'/base/wheel_state/{field}[{index}]'
        for field in ('target_rad_s', 'commanded_rad_s', 'velocity_rad_s')]
    assert arguments(pwm)[2:] == [f'/base/wheel_state/pwm_percent[{index}]']


def test_invalid_wheel_rejected():
    with pytest.raises(RuntimeError, match='wheel must be'):
        nodes_for('front')


def test_description_has_only_observation_nodes():
    description = monitor_launch.generate_launch_description()
    assert len(description.entities) == 3
    argument = description.entities[0]
    assert argument.name == 'wheel'
    assert [part.text for part in argument.default_value] == ['fl']
    assert not any(token in SOURCE.read_text() for token in (
        'base_driver_node', 'agent', 'rviz', 'cmd_vel', 'base/enable'))


def test_gui_dependencies_are_declared():
    manifest = ET.parse(SOURCE.parents[1] / 'package.xml')
    dependencies = {element.text for element in manifest.findall('exec_depend')}
    assert {'rqt_gui', 'rqt_plot', 'rqt_robot_monitor'} <= dependencies


def test_saved_preset_launches_only_rqt_with_diagnostics_remap(monkeypatch):
    monkeypatch.setattr(monitor_launch, 'get_package_share_directory',
                        lambda package: '/share/' + package)
    context = LaunchContext()
    context.launch_configurations['preset'] = 'true'
    nodes = monitor_launch._nodes(context)
    assert len(nodes) == 1
    node = nodes[0]
    assert (node._Node__package, node._Node__node_executable) == ('rqt_gui', 'rqt_gui')
    assert arguments(node) == ['--force-discover', '--perspective-file',
                              '/share/cleany_base_driver/config/base_monitor.perspective']
    remap = node._Node__remappings[0]
    assert [part.text for side in remap for part in side] == [
        '/diagnostics_agg', '/diagnostics']


def test_invalid_preset_rejected():
    context = LaunchContext()
    context.launch_configurations['preset'] = 'maybe'
    with pytest.raises(RuntimeError, match='preset must be'):
        monitor_launch._nodes(context)


def test_perspective_has_only_two_matplots_and_robot_monitor():
    path = SOURCE.parents[1] / 'config' / 'base_monitor.perspective'
    data = json.loads(path.read_text())
    manager = data['groups']['pluginmanager']
    plugins = ast.literal_eval(manager['keys']['running-plugins']['repr'])
    assert plugins == {'rqt_plot/Plot': [1, 2], 'rqt_robot_monitor/RobotMonitor': [1]}
    assert set(manager['groups']) == {
        'plugin__rqt_plot__Plot__1', 'plugin__rqt_plot__Plot__2',
        'plugin__rqt_robot_monitor__RobotMonitor__1'}
    # Store the internal dock layout, not machine-specific window placement.
    assert set(data['groups']['mainwindow']['keys']) == {'state'}
    for serial, expected_fields, limits in (
        (1, ('target_rad_s', 'commanded_rad_s', 'velocity_rad_s'), [-15, 15]),
        (2, ('pwm_percent',), [-100, 100]),
    ):
        settings = manager['groups'][f'plugin__rqt_plot__Plot__{serial}'][
            'groups']['plugin']['keys']
        values = {key: ast.literal_eval(value['repr']) for key, value in settings.items()}
        assert values['plot_type'] == 1  # rqt_plot MatPlot backend.
        assert values['autoscroll'] is True
        assert list(map(float, values['x_limits'])) == [0, 10]
        assert list(map(float, values['y_limits'])) == limits
        assert values['topics'] == [
            f'/base/wheel_state/{field}[{index}]'
            for index in range(4) for field in expected_fields]
