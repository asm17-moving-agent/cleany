"""Build grid-aligned eight-direction translations; audit retained turn primitives."""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def build(source: dict) -> dict:
    """Preserve heading bins and turns, replacing fixed-yaw translations only."""
    result = copy.deepcopy(source)
    resolution = source['lattice_metadata']['grid_resolution']
    headings = source['lattice_metadata']['heading_angles']
    primitives = []
    for index, yaw in enumerate(headings):
        old = [p for p in source['primitives'] if p['start_angle_index'] == index]
        forward = next(p for p in old if p['trajectory_length'] > 0 and
                       p['start_angle_index'] == p['end_angle_index'] and
                       abs(wrap(math.atan2(p['poses'][-1][1], p['poses'][-1][0])-yaw)) < 1e-6)
        gx, gy = [round(v/resolution) for v in forward['poses'][-1][:2]]
        divisor = math.gcd(abs(gx), abs(gy))
        gx, gy = gx//divisor, gy//divisor
        primitives.extend(copy.deepcopy(p) for p in old if p['trajectory_length'] == 0 or
                          p['start_angle_index'] != p['end_angle_index'])
        for xsign, ysign in [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]:
            x = (xsign*gx-ysign*gy)*resolution
            y = (xsign*gy+ysign*gx)*resolution
            distance = math.hypot(x,y)
            count = max(1, math.ceil(distance/(resolution/2)))
            primitives.append(dict(start_angle_index=index, end_angle_index=index,
                left_turn=True, trajectory_radius=0.0, trajectory_length=distance,
                arc_length=0.0, straight_length=distance,
                poses=[[x*k/count,y*k/count,yaw] for k in range(1,count+1)]))
    for i,p in enumerate(primitives):
        p['trajectory_id'] = i
    result['primitives'] = primitives
    result['lattice_metadata']['number_of_trajectories'] = len(primitives)
    # The output is reproducible; date_generated identifies the source asset.
    return result


def audit(source: dict) -> dict:
    """Report geometric symmetry, without treating unequal turns as a pair."""
    headings = source['lattice_metadata']['heading_angles']
    report = []
    for index, yaw in enumerate(headings):
        rows = [p for p in source['primitives'] if p['start_angle_index']==index]
        turns = [p for p in rows if p['start_angle_index'] != p['end_angle_index']]
        pairs = []
        for p in turns:
            delta = wrap(headings[p['end_angle_index']]-yaw)
            if delta<=0:continue
            reflected = [[math.cos(2*yaw)*x+math.sin(2*yaw)*y,
                          math.sin(2*yaw)*x-math.cos(2*yaw)*y,
                          (2*yaw-a)%(2*math.pi)] for x,y,a in p['poses']]
            matches=[]
            for q in turns:
                if abs(wrap(headings[q['end_angle_index']]-yaw)+delta)>1e-6:continue
                if len(reflected)!=len(q['poses']):continue
                err=max(max(abs(a[0]-b[0]),abs(a[1]-b[1]),abs(wrap(a[2]-b[2]))) for a,b in zip(reflected,q['poses']))
                if err<3e-5 and abs(p['trajectory_length']-q['trajectory_length'])<3e-5:
                    matches.append(q['trajectory_id'])
            pairs.append(dict(left_id=p['trajectory_id'],right_ids=matches,
                yaw_change_deg=math.degrees(delta),length_m=p['trajectory_length'],
                type='rotation' if p['trajectory_length']==0 else 'curve'))
        report.append(dict(heading_deg=math.degrees(yaw),left_turns=pairs,
            rotation_changes_deg=[math.degrees(wrap(headings[p['end_angle_index']]-yaw))
                for p in turns if p['trajectory_length']==0]))
    return {'tolerance_m_rad':3e-5,'headings':report}


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--audit',type=Path,required=True)
    args=parser.parse_args()
    source=json.loads(args.source.read_text())
    args.output.write_text(json.dumps(build(source),indent=2)+'\n')
    args.audit.write_text(json.dumps(audit(source),indent=2)+'\n')


if __name__=='__main__':
    main()
