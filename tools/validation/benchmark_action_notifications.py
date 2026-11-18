"""Measure stock/event JTC delivery and exercise terminal races without motors.

Only mock_components/GenericSystem is instantiated, in localhost ROS domain 219.
The installed upstream JTC library remains unchanged. CPU is controller process
CPU as a percentage of one logical core; notification latency is measured from
the controller's success log timestamp to the client's result callback.
"""
from __future__ import annotations

import argparse
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
import json
import os
from pathlib import Path
import random
import re
import signal
import statistics
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

STOCK = 'joint_trajectory_controller/JointTrajectoryController'
EVENT = 'cleany_trajectory_controller/JointTrajectoryController'


def stats(values):
    ordered = sorted(values)
    return {'median': statistics.median(ordered),
            'p95': ordered[min(len(ordered)-1, int(len(ordered)*.95))],
            'max': max(ordered), 'samples': values}


class Rig:
    def __init__(self, directory, label, plugin, rate):
        assert plugin in (STOCK, EVENT)
        self.label, self.plugin, self.rate = label, plugin, rate
        self.log_path = directory/(label+'.log')
        self.feedback, self.stamps = [], []
        names = [f'bench_{i}' for i in range(4)]
        xml = '<robot name="notification_benchmark"><link name="base"/>'
        for i in range(4):
            xml += f'<link name="link{i}"/><joint name="joint{i}" type="revolute"><parent link="base"/><child link="link{i}"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="2" effort="10"/></joint>'
        xml += '<ros2_control name="Mock" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>'
        for i in range(4):
            xml += f'<joint name="joint{i}"><command_interface name="position"/><state_interface name="position"><param name="initial_value">0</param></state_interface></joint>'
        xml += '</ros2_control></robot>'
        config = {'controller_manager': {'ros__parameters': {'robot_description': xml, 'update_rate': 50}}}
        for i, name in enumerate(names):
            config['controller_manager']['ros__parameters'][name] = {'type': plugin}
            config[name] = {'ros__parameters': {'joints': [f'joint{i}'], 'command_interfaces': ['position'],
                'state_interfaces': ['position'], 'state_publish_rate': 50., 'action_monitor_rate': float(rate),
                'constraints': {'goal_time': 1., f'joint{i}': {'goal': .001, 'trajectory': 0.}}}}
        path = directory/(label+'.yaml')
        path.write_text(yaml.safe_dump(config))
        self.log = self.log_path.open('w')
        self.process = subprocess.Popen(['/opt/ros/humble/lib/controller_manager/ros2_control_node',
            '--ros-args', '--params-file', str(path)], stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.node = Node('completion_benchmark')
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.client = ActionClient(self.node, FollowJointTrajectory, '/bench_0/follow_joint_trajectory')
        self.switch = self.node.create_client(SwitchController, '/controller_manager/switch_controller')
        self.subscription = self.node.create_subscription(JointTrajectoryControllerState, '/bench_0/state',
            lambda message: self.stamps.append(message.header.stamp.sec+message.header.stamp.nanosec/1e9), 100)
        try:
            subprocess.run(['ros2','run','controller_manager','spawner',*names,
                '--controller-manager-timeout','15'],check=True,stdout=self.log,stderr=subprocess.STDOUT,timeout=35)
            assert self.client.wait_for_server(timeout_sec=5.)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def wait(future, timeout=5.):
        event = threading.Event()
        received = []
        future.add_done_callback(lambda _: (received.append(time.time_ns()/1e9), event.set()))
        if not event.wait(timeout):
            raise TimeoutError('Action/service result missing')
        return future.result(), received[0]

    def send(self, position, duration, *, delay=.08, invalid=False, strict_path=False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['unknown' if invalid else 'joint0']
        start = time.time_ns()+round(delay*1e9)
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = divmod(start, 10**9)
        point = JointTrajectoryPoint(positions=[float(position)])
        point.time_from_start = Duration(sec=int(duration), nanosec=round((duration-int(duration))*1e9))
        goal.trajectory.points = [point]
        if strict_path:
            goal.path_tolerance = [JointTolerance(name='joint0', position=1e-8)]
        handle, _ = self.wait(self.client.send_goal_async(goal,
            feedback_callback=lambda message: self.feedback.append(time.time_ns()/1e9)))
        if invalid:
            assert not handle.accepted
            return None
        assert handle.accepted
        return handle, handle.get_result_async(), start/1e9+duration

    def ticks(self):
        values = Path(f'/proc/{self.process.pid}/stat').read_text().split()
        return sum(int(values[i]) for i in (13,14))

    def cpu_phase(self, callback):
        start_ticks, start = self.ticks(), time.monotonic()
        value = callback()
        elapsed = time.monotonic()-start
        cpu = (self.ticks()-start_ticks)/os.sysconf('SC_CLK_TCK')/elapsed*100
        return value, cpu, elapsed

    def measure(self, trials):
        randomizer = random.Random(68)
        received, overhead = [], []
        def run():
            for i in range(trials+4):
                h,f,endpoint = self.send(.1 if i % 2 else -.1, randomizer.uniform(.085,.185))
                result, stamp = self.wait(f)
                assert result.status == 4 and result.result.error_code == 0
                received.append(stamp)
                overhead.append(stamp-endpoint)
                time.sleep(randomizer.uniform(.001,.017))
        feedback_before, stamp_before = len(self.feedback),len(self.stamps)
        _,cpu,elapsed = self.cpu_phase(run)
        finish=[float(value) for value in re.findall(
            r'\[([0-9]+\.[0-9]+)\] \[bench_0\]: Goal reached, success!', self.log_path.read_text())]
        assert len(finish) == len(received) == trials+4,(len(finish),len(received))
        latency=[end-start for start,end in zip(finish[4:],received[4:])]
        assert min(latency) >= 0
        periods=[b-a for a,b in zip(self.stamps[stamp_before:],self.stamps[stamp_before+1:]) if b>a]
        feedback_count=len(self.feedback)-feedback_before
        _,idle_cpu,_=self.cpu_phase(lambda:time.sleep(1.5))
        return {'mode':self.label,'plugin':self.plugin,'action_rate_hz':self.rate,'trials':trials,
                'controller_cpu_one_core_percent':cpu,'idle_cpu_one_core_percent':idle_cpu,
                'elapsed_sec':elapsed,'feedback_messages':feedback_count,
                'notification_sec':stats(latency),'after_scheduled_endpoint_sec':stats(overhead[4:]),
                'state_period_sec':stats(periods),
                'loaded_libraries':[line.split()[-1] for line in Path(f'/proc/{self.process.pid}/maps').read_text().splitlines()
                    if 'libcleany_trajectory_controller' in line or 'libjoint_trajectory_controller' in line]}

    def functional(self):
        checks=[]
        def terminal(label, command, status, code=None, began=None):
            handle,f,endpoint=command
            result,stamp=self.wait(f,timeout=3.)
            assert result.status==status,(label,result.status,result.result)
            if code is not None:assert result.result.error_code==code,(label,result.result)
            delay=stamp-(began if began is not None else endpoint)
            assert delay < .3,(label,delay)
            checks.append({'case':label,'status':result.status,'error_code':result.result.error_code,
                           'delivery_after_reference_sec':delay})
            return result
        # At 1 Hz feedback all these terminals must arrive before its next tick.
        terminal('success_without_feedback_tick',self.send(.2,.13),4,0)
        start=time.time()
        terminal('path_tolerance_failure',self.send(-.5,.4,strict_path=True),6,-4,began=start)
        command=self.send(.2,.5)
        time.sleep(.02)
        start=time.time();cancel,_=self.wait(command[0].cancel_goal_async())
        assert cancel.goals_canceling
        terminal('accepted_cancel',command,5,began=start)
        old=self.send(-.2,.7);time.sleep(.02)
        start=time.time();new=self.send(.1,.12)
        terminal('superseded_goal_aborted',old,6,-1,began=start)
        terminal('replacement_success',new,4,0)
        self.send(.1,.12,invalid=True)
        checks.append({'case':'invalid_goal_rejected','passed':True})
        # Accepted cancel racing a control update must never lose its result.
        randomizer=random.Random(73)
        for index in range(24):
            command=self.send(.1 if index%2 else -.1,.1,delay=.04)
            time.sleep(randomizer.uniform(.11,.17))
            started=time.time();cancel,_=self.wait(command[0].cancel_goal_async())
            result,stamp=self.wait(command[1],timeout=2.)
            assert result.status in (4,5),(index,result)
            if cancel.goals_canceling:assert result.status==5
            assert stamp-started < .3
            checks.append({'case':f'cancel_finish_race_{index}','status':result.status,
                           'cancel_accepted':bool(cancel.goals_canceling)})
        assert self.switch.wait_for_service(timeout_sec=3.)
        command=self.send(.3,.6)
        start=time.time()
        response,_=self.wait(self.switch.call_async(SwitchController.Request(
            deactivate_controllers=['bench_0'],strictness=2,timeout=Duration(sec=3))))
        assert response.ok
        terminal('deactivate_aborts_active_goal',command,6,-1,began=start)
        response,_=self.wait(self.switch.call_async(SwitchController.Request(
            activate_controllers=['bench_0'],strictness=2,timeout=Duration(sec=3))))
        assert response.ok
        terminal('reactivated_success',self.send(-.1,.14),4,0)
        before=len(self.feedback)
        terminal('low_rate_progress',self.send(.1,2.2),4,0)
        progress=len(self.feedback)-before
        assert 1 <= progress <= 3,progress
        checks.append({'case':'feedback_rate_1hz','messages_in_2_2sec':progress})
        return checks

    def close(self):
        self.client.destroy()
        self.node.destroy_client(self.switch)
        self.node.destroy_subscription(self.subscription)
        self.executor.shutdown();self.thread.join(timeout=3.)
        self.node.destroy_node()
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM);self.process.wait(timeout=3.)
        self.log.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--blocks',type=int,default=3)
    parser.add_argument('--trials',type=int,default=24)
    parser.add_argument('--functional-only',action='store_true')
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='219' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    report={'physical_motor_commands_sent':False,'domain':219,'control_update_hz':50,'controllers':4,
            'scope':'Jetson native JTC, GenericSystem; four one-joint controllers, one commanded',
            'runs':[],'functional':[]}
    try:
        modes=[('stock20',STOCK,20.),('event20',EVENT,20.),('stock100',STOCK,100.)]
        if not args.functional_only:
            for block in range(args.blocks):
                for name,plugin,rate in modes[block:]+modes[:block]:
                    rig=Rig(args.output,f'{name}-{block}',plugin,rate)
                    try:
                        row=rig.measure(args.trials);row['variant']=name;row['block']=block
                        report['runs'].append(row)
                        print(json.dumps({'variant':name,'block':block,'cpu':row['controller_cpu_one_core_percent'],
                                          'notification_median_ms':row['notification_sec']['median']*1000}),flush=True)
                    finally:rig.close()
                    (args.output/'results.json').write_text(json.dumps(report,indent=2)+'\n')
        rig=Rig(args.output,'event-functional',EVENT,1.)
        try:report['functional']=rig.functional()
        finally:rig.close()
        (args.output/'results.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'functional_checks':len(report['functional']),'success':True}),flush=True)
    finally:rclpy.shutdown()


if __name__=='__main__':main()
