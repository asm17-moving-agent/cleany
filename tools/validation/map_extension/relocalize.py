"""Estimate a stopped robot pose from a live scan and a saved occupancy map."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def transform(points,p):
    c,s=np.cos(p[2]),np.sin(p[2])
    return points@np.array([[c,s],[-s,c]])+p[:2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--map',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Node('map_extension_relocalize');b=Buffer();l=TransformListener(b,n);data={}
    n.create_subscription(LaserScan,'/scan',lambda m:data.update(scan=m),qos_profile_sensor_data)
    n.create_subscription(Odometry,'/odom',lambda m:data.update(odom=m),qos_profile_sensor_data)
    end=time.monotonic()+6
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    scan=data['scan'];odom=data['odom']
    if math.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y)>.01 or abs(odom.twist.twist.angular.z)>.02:raise RuntimeError('Robot must be stopped')
    extr=b.lookup_transform('base_link',scan.header.frame_id,Time()).transform
    current=b.lookup_transform('map','base_link',Time()).transform
    guess=np.array([current.translation.x,current.translation.y,angle(current.rotation)])
    ranges=np.array(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid=np.isfinite(ranges)&(ranges>.5)&(ranges<8)
    points=np.column_stack([ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])])
    points=transform(points,[extr.translation.x,extr.translation.y,angle(extr.rotation)])
    if len(points)<80:raise RuntimeError('Too few useful scan returns')
    points=points[::max(1,len(points)//300)]
    info=yaml.safe_load(a.map.read_text());im=np.array(Image.open(a.map.parent/info['image']));res=info['resolution'];origin=info['origin']
    iy,ix=np.where(im<65)
    walls=transform(np.column_stack([(ix+.5)*res,(im.shape[0]-iy-.5)*res]),origin)
    tree=cKDTree(walls)
    def residual(p):return tree.query(transform(points,p))[0]
    def score(p):
        d=residual(p);return np.mean(np.minimum(d,.3)**2)
    candidates=[]
    for dyaw in np.linspace(-math.pi/3,math.pi/3,25):
        for dx in np.arange(-1.5,1.51,.25):
            for dy in np.arange(-1.5,1.51,.25):
                p=guess+[dx,dy,dyaw];candidates.append((score(p),p))
    candidates.sort(key=lambda r:r[0]);results=[]
    for _,seed in candidates:
        if any(np.linalg.norm(seed[:2]-r['pose'][:2])<.3 and abs(seed[2]-r['pose'][2])<.15 for r in results):continue
        fit=least_squares(residual,seed,loss='soft_l1',f_scale=.06,max_nfev=150,diff_step=1e-4)
        p=fit.x;dist=residual(p)
        results.append(dict(pose=p.tolist(),score=float(score(p)),median_m=float(np.median(dist)),p80_m=float(np.percentile(dist,80)),inliers_10cm=float(np.mean(dist<.10))))
        if len(results)>=15:break
    results.sort(key=lambda r:r['score']);report=dict(initial_pose=guess.tolist(),initial_score=float(score(guess)),scan_stamp=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,scan_points=len(points),map=str(a.map),candidates=results)
    (a.output/'result.json').write_text(json.dumps(report,indent=2));np.savez(a.output/'alignment.npz',walls=walls,points=points,initial=guess,selected=results[0]['pose'])
    print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
