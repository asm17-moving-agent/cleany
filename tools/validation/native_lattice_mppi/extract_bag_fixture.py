#!/usr/bin/env python3
"""Extract a causal, frozen costmap fixture from recorded navigation messages.

This does not replay live ROS topics or predict physical motion after a command
change. Header and receipt cutoffs prohibit future input in the extracted state.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sqlite3


def occupancy_to_cost(value: int) -> int:
    """Invert Nav2's lossy display table using the upper bound of each bin."""
    if value == -1:
        return 255
    if value == 100:
        return 254
    if value == 99:
        return 253
    if value == 0:
        return 0
    if not 1 <= value <= 98:
        raise ValueError('Invalid OccupancyGrid value')
    return min(252, math.ceil(value*251/97))


def yaw(q) -> float:
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def compose(a: list[float], b: list[float]) -> list[float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2]]


def extract(bag: Path, stamp: float, output: Path) -> dict:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    found = {}
    transforms = []
    needed = ('/global_costmap/costmap', '/odom', '/plan')
    for file in sorted(bag.rglob('*.db3')):
        with sqlite3.connect(f'file:{file}?mode=ro', uri=True) as db:
            db.execute('BEGIN')
            topics = {name: (i, typ) for i, name, typ in db.execute('select id,name,type from topics')}
            for name in needed:
                if name not in topics:
                    continue
                ident, typ = topics[name]
                row = db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1', (ident, round(stamp*1e9))).fetchone()
                if row and (name not in found or row[0] > found[name][0]):
                    found[name] = (row[0], deserialize_message(row[1], get_message(typ)), str(file))
            if '/tf' not in topics:
                continue
            ident, typ = topics['/tf']
            for received, blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp', (ident, round((stamp-2)*1e9), round(stamp*1e9))):
                for t in deserialize_message(blob, get_message(typ)).transforms:
                    header = t.header.stamp.sec+t.header.stamp.nanosec*1e-9
                    if t.header.frame_id == 'map' and t.child_frame_id == 'odom' and header <= stamp:
                        p, q = t.transform.translation, t.transform.rotation
                        transforms.append((header, received/1e9, [p.x, p.y, yaw(q)]))
    if set(found) != set(needed) or not transforms:
        raise ValueError('Missing recorded input or map-to-odom TF')
    tf_stamp, tf_received, tf = max(transforms)
    grid, odom, plan = [found[k][1] for k in needed]
    ages = {k: stamp-found[k][0]/1e9 for k in needed}
    if ages['/global_costmap/costmap'] > 3 or ages['/odom'] > .2 or ages['/plan'] > 2 or stamp-tf_stamp > .5:
        raise ValueError(f'Stale fixture inputs: {ages}, TF {stamp-tf_stamp}')
    for m in (grid, odom, plan):
        assert m.header.stamp.sec+m.header.stamp.nanosec*1e-9 <= stamp, 'Future header'
    assert grid.header.frame_id == plan.header.frame_id == 'map'
    assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_link'
    assert abs(yaw(grid.info.origin.orientation)) < 1e-6
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    start = compose(tf, [p.x, p.y, yaw(q)])
    if not plan.poses:
        raise ValueError('Recorded plan is empty')
    goal = plan.poses[-1].pose
    goal_pose = [goal.position.x, goal.position.y, yaw(goal.orientation)]
    output.mkdir(parents=True, exist_ok=False)
    info = grid.info
    (output/'map.txt').write_text(f'{info.width} {info.height} {info.resolution} {info.origin.position.x} {info.origin.position.y}\n')
    (output/'map.bin').write_bytes(bytes(occupancy_to_cost(v) for v in grid.data))
    (output/'poses.txt').write_text('\n'.join(' '.join(map(str,p)) for p in [start,goal_pose])+'\n')
    metadata = dict(bag=str(bag),stamp=stamp,receipt_ages_s=ages,tf_header=tf_stamp,
        tf_received=tf_received,map_to_odom=tf,start=start,goal=goal_pose,
        recorded_speed=[odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z],
        files={k:found[k][2] for k in needed},cost_conversion='upper_bound_of_occupancy_bin',
        scope='Frozen recorded geometry; ideal kinematics from rest, not full bag resimulation')
    (output/'provenance.json').write_text(json.dumps(metadata,indent=2))
    return metadata


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--at',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(extract(args.bag,args.at,args.output)))
