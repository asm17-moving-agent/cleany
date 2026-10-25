"""Measure actual RGB-D and mapping capture age without commanding motion."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds',type=float,default=10.)
parser.add_argument('--name',default='scene-latency')
args = parser.parse_args()
if Path(args.name).name != args.name:
    parser.error('name must be a basename')
rclpy.init()
node = rclpy.create_node('scene_latency_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
samples = {}
for key, topic, kind in (
    ('depth','/camera/aligned_depth_to_color/image_raw',Image),
    ('cloud','/perception/scene_cloud',PointCloud2),
    ('filtered','/perception/scene_cloud_filtered',PointCloud2)):
    samples[key] = []
    def receive(message,key=key):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now = node.get_clock().now().nanoseconds*1e-9
        if now > 0:
            samples[key].append(dict(wall=time.monotonic(),stamp=stamp,age_s=now-stamp,
                elements=message.width*message.height))
    node.create_subscription(kind,topic,receive,qos_profile_sensor_data)
end = time.monotonic()+args.seconds
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.05)
report = {}
for key, values in samples.items():
    ages = [v['age_s'] for v in values]
    report[key] = dict(count=len(values),age_p50_s=float(np.median(ages)) if ages else None,
        age_p95_s=float(np.percentile(ages,95)) if ages else None,
        maximum_age_s=max(ages,default=None),samples=values)
Path('/output/evaluations',args.name+'.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:{a:b for a,b in v.items() if a!='samples'} for k,v in report.items()}),flush=True)
node.destroy_node()
rclpy.shutdown()
