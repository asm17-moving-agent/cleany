"""Shadow-only 3D stopping-volume comparison for the parked Gazebo robot.

No velocity publisher: existing Collision Monitor remains authoritative.
"""
from __future__ import annotations
import ast
import json
from itertools import product
from xml.etree import ElementTree as ET
import xacro
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from cleany_gazebo_sim.body_geometry import load_body_boxes, nearest_body, swept_hits
from cleany_gazebo_sim.obstacle_memory_node import transform
from cleany_gazebo_sim.world.cad_frame import rpy, freeze, origin


def quaternion(rotation: np.ndarray) -> tuple[float,float,float,float]:
    roll,pitch,yaw = rpy(rotation)
    cr,cp,cy = np.cos(np.array([roll,pitch,yaw])/2)
    sr,sp,sy = np.sin(np.array([roll,pitch,yaw])/2)
    return sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy


class BodyGuardObserver(Node):
    def __init__(self):
        super().__init__('body_guard_observer')
        share=Path(get_package_share_directory('cleany_gazebo_sim'))
        defaults=dict(profile=str(share/'config/cad_frame.yaml'), legacy_config='', output_directory='',
                      margin_m=.02, reaction_s=.15, linear_deceleration=.4, angular_deceleration=.6,
                      source_timeout_s=.6, update_period_s=.2, sweep_step_s=.05, min_points=3)
        for key,value in defaults.items(): self.declare_parameter(key,value)
        self.p={key:self.get_parameter(key).value for key in defaults}
        if not np.isfinite([self.p[k] for k, v in defaults.items() if isinstance(v, (float, int))]).all():
            raise ValueError('Non-finite observer parameters')
        if not self.p['legacy_config'] or not self.p['output_directory']:
            raise ValueError('legacy_config and output_directory are required')
        if self.p['margin_m'] < 0 or self.p['reaction_s'] < 0 or min(self.p[k] for k in ('linear_deceleration','angular_deceleration','source_timeout_s','update_period_s','sweep_step_s','min_points')) <= 0:
            raise ValueError('Invalid observer parameters')
        self.boxes=load_body_boxes(Path(get_package_share_directory('cleany_description')),Path(self.p['profile']))
        old=yaml.safe_load(Path(self.p['legacy_config']).read_text())['collision_monitor']['ros__parameters']
        self.polygon=np.array(ast.literal_eval(old['StopZone']['points']))
        self.min_z,self.max_z=old['depth']['min_height'],old['depth']['max_height']
        self.output=Path(self.p['output_directory']);self.output.mkdir(parents=True,exist_ok=True)
        (self.output/'geometry.json').write_text(json.dumps([dict(name=b.name,center=b.center.tolist(),rotation=b.rotation.tolist(),half_size=b.half_size.tolist()) for b in self.boxes],indent=2))
        (self.output/'parameters.json').write_text(json.dumps(self.p,indent=2))
        self.latest={};self.command=Twist();self.command_time=-1e9
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(LaserScan,'/scan',lambda m:self.latest.update(lidar=m),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/camera/head/depth/points',lambda m:self.latest.update(depth=m),qos_profile_sensor_data)
        self.create_subscription(Twist,'/nav2/cmd_vel',self.on_command,10)
        durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers=self.create_publisher(MarkerArray,'/body_guard/markers',durable)
        self.state=self.create_publisher(String,'/body_guard/state',durable)
        self.visual_markers=self.make_visual_markers()
        self.previous=None
        self.create_timer(self.p['update_period_s'],self.update)

    def on_command(self,msg):
        self.command=msg;self.command_time=self.get_clock().now().nanoseconds/1e9

    def points(self,source,msg):
        age=(self.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
        if not 0 <= age <= self.p['source_timeout_s']: raise ValueError('Stale or future sensor stamp')
        if source=='lidar':
            distances=np.asarray(msg.ranges);angles=msg.angle_min+np.arange(len(distances))*msg.angle_increment
            valid=np.isfinite(distances)&(distances>=msg.range_min)&(distances<=msg.range_max)
            points=np.column_stack([distances[valid]*np.cos(angles[valid]),distances[valid]*np.sin(angles[valid]),np.zeros(valid.sum())])
        else:
            fields={f.name:f for f in msg.fields}
            if not all(k in fields and fields[k].datatype==PointField.FLOAT32 for k in ('x','y','z')): raise ValueError('Expected XYZ float32 cloud')
            dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
            array=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
            points=np.column_stack([array[k].ravel() for k in ('x','y','z')])
            distances=np.linalg.norm(points,axis=1)
            points=points[np.isfinite(points).all(axis=1)&(distances>.01)&(distances<4.)]
        pose=self.buffer.lookup_transform_full('base_link',Time(),msg.header.frame_id,Time.from_msg(msg.header.stamp),'odom').transform
        points=transform(points,pose)
        return points[(points[:,2]>=self.min_z)&(points[:,2]<=self.max_z)]

    def marker(self,kind,index,namespace):
        m=Marker();m.header.frame_id='base_link';m.header.stamp=self.get_clock().now().to_msg()
        m.ns,m.id,m.type=namespace,index,kind;m.pose.orientation.w=1.;return m

    def make_visual_markers(self) -> list[Marker]:
        """Render original visual geometry in the same frozen posture as the guard."""
        description=Path(get_package_share_directory('cleany_description'))
        urdf=ET.fromstring(xacro.process_file(str(description/'urdf/cleany.urdf.xacro')).toxml())
        config=yaml.safe_load(Path(self.p['profile']).read_text())
        transforms=freeze(urdf,config['park_joints'],config.get('simulation_park_limits'))
        markers=[]
        for link in urdf.findall('link'):
            for visual in link.findall('visual'):
                shape=next(iter(visual.find('geometry')))
                kind={'mesh':Marker.MESH_RESOURCE,'box':Marker.CUBE,
                      'sphere':Marker.SPHERE,'cylinder':Marker.CYLINDER}[shape.tag]
                m=self.marker(kind,len(markers),'robot_visual')
                pose=transforms[link.get('name')]@origin(visual.find('origin'))
                if config.get('dynamic_pan', False) and link.get('name').startswith(('head_pan_link', 'head_tilt_link', 'head_camera')):
                    pose=np.linalg.inv(transforms['head_camera_depth_frame'])@pose
                    m.header.frame_id='head_camera_depth_frame'
                    m.frame_locked=True
                m.pose.position=Point(x=float(pose[0,3]),y=float(pose[1,3]),z=float(pose[2,3]))
                m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,quaternion(pose[:3,:3]))
                if shape.tag=='mesh':
                    m.mesh_resource=shape.get('filename')
                    scale=np.fromstring(shape.get('scale','1 1 1'),sep=' ')
                elif shape.tag=='box':
                    scale=np.fromstring(shape.get('size'),sep=' ')
                elif shape.tag=='sphere':
                    scale=np.full(3,2*float(shape.get('radius')))
                else:
                    scale=[2*float(shape.get('radius'))]*2+[float(shape.get('length'))]
                m.scale.x,m.scale.y,m.scale.z=map(float,scale)
                material=visual.find('material')
                if material is not None and material.find('color') is None:
                    material=next((v for v in urdf.findall('material') if v.get('name')==material.get('name')),None)
                color=material.find('color') if material is not None else None
                rgba=np.fromstring(color.get('rgba'),sep=' ') if color is not None else [.65,.75,.8,1.]
                m.color.r,m.color.g,m.color.b,m.color.a=map(float,rgba)
                markers.append(m)
        return markers

    def update(self):
        start=time.monotonic();now=self.get_clock().now().nanoseconds/1e9
        velocity=(self.command.linear.x,self.command.linear.y,self.command.angular.z) if now-self.command_time<self.p['source_timeout_s'] else (0.,0.,0.)
        report=dict(mode='shadow_only',profile='fixed_park_posture',sim_s=now,margin_m=self.p['margin_m'],velocity=velocity,sources={})
        # Remove old filled-box namespaces when attaching to an existing RViz.
        reset=self.marker(Marker.CUBE,0,'');reset.action=Marker.DELETEALL
        markers=[reset]+self.visual_markers
        stamp=self.get_clock().now().to_msg()
        for m in self.visual_markers:m.header.stamp=stamp
        signs=np.array(list(product((-1.,1.),repeat=3)))
        edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
        for i,b in enumerate(self.boxes):
            margin=self.marker(Marker.LINE_LIST,i,'margin_outlines')
            margin.scale.x=.001
            margin.color.g,margin.color.a=1.,.15
            corners=(signs*(b.half_size+self.p['margin_m']))@b.rotation.T+b.center
            margin.points=[Point(x=float(corners[k,0]),y=float(corners[k,1]),z=float(corners[k,2])) for edge in edges for k in edge]
            markers.append(margin)
        failed=False;total_hits=0
        for source in ('lidar','depth'):
            try:
                points=self.points(source,self.latest[source])
                # Bound expensive sweep to its maximum reachable body radius.
                radius=max(np.linalg.norm(b.center[:2])+np.linalg.norm(b.half_size) for b in self.boxes)
                speed=float(np.hypot(*velocity[:2]));horizon=self.p['reaction_s']+max(speed/self.p['linear_deceleration'],abs(velocity[2])/self.p['angular_deceleration'])
                points=points[np.linalg.norm(points[:,:2],axis=1)<=radius+self.p['margin_m']+speed*horizon+.1]
                distance,indices=nearest_body(points,self.boxes)
                legacy=((points[:,:2]>=self.polygon.min(axis=0))&(points[:,:2]<=self.polygon.max(axis=0))).all(axis=1)
                hits=swept_hits(points,self.boxes,velocity,self.p['margin_m'],self.p['reaction_s'],self.p['linear_deceleration'],self.p['angular_deceleration'],self.p['sweep_step_s'])
                total_hits+=int(hits.sum())
                report['sources'][source]=dict(legacy_hits=int(legacy.sum()),body_hits=int((distance<=0).sum()),margin_hits=int((distance<=self.p['margin_m']).sum()),swept_hits=int(hits.sum()),
                    legacy_min_body_distance_m=float(distance[legacy].min()) if legacy.any() else None,
                    legacy_nearest_parts=sorted(set(self.boxes[i].name for i in indices[legacy])))
                np.savez_compressed(self.output/(source+'_latest.npz'),points=points,legacy=legacy,distance=distance,swept=hits)
                for ns,mask,color in [('legacy_points',legacy,(1.,0.,1.)),('body_risk_points',hits,(1.,0.,0.))]:
                    m=self.marker(Marker.POINTS,0,source+'_'+ns);m.scale.x=m.scale.y=.015
                    m.color.r,m.color.g,m.color.b=color;m.color.a=1.
                    m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in points[mask]];markers.append(m)
            except (KeyError,ValueError,TransformException) as exc:
                report['sources'][source]=dict(unavailable=str(exc));failed=True
                for ns in ('legacy_points', 'body_risk_points'):
                    m=self.marker(Marker.POINTS,0,source+'_'+ns)
                    m.action=Marker.DELETE;markers.append(m)
        report['decision']='STOP_UNAVAILABLE' if failed else ('STOP' if total_hits>=self.p['min_points'] else 'NO_OBSERVED_COLLISION')
        report['observability']='Unobserved space and moving obstacles are not certified clear'
        report['update_wall_ms']=(time.monotonic()-start)*1000
        self.markers.publish(MarkerArray(markers=markers));self.state.publish(String(data=json.dumps(report)))
        (self.output/'latest.json').write_text(json.dumps(report,indent=2))
        signature=(report['decision'],json.dumps(report['sources'],sort_keys=True))
        if signature!=self.previous:
            with (self.output/'changes.jsonl').open('a') as f:f.write(json.dumps(report)+'\n')
            self.previous=signature


def main(args=None):
    rclpy.init(args=args);node=BodyGuardObserver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
