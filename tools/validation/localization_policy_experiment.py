#!/usr/bin/env python3
"""Compare recovery admission policies on recorded, stationary LiDAR windows.

Recorded AMCL poses are references, not surveyed ground truth. Offset trials
measure residual injected error relative to those references, not physical accuracy.
No ROS publishers or robot motion commands are created.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

from cleany_navigation.localization_alignment import (
    AlignmentMap, AlignmentScore, AlignmentSettings, search_pose, transform, wrap)

SETTINGS = replace(AlignmentSettings(), min_inlier_ratio=.30)
FLOORS = (.30, .40, .50, .55, .65)
OFFSETS = ((0.,0.,0.), (.4,0.,0.), (-.4,0.,0.), (0.,.4,0.),
           (0.,-.4,0.), (0.,0.,.35), (0.,0.,-.35))
TOPICS = {'/map','/tf','/tf_static','/scan_raw','/amcl_pose','/odom','/rosout','/localization/status'}


def stamp(message) -> float:
    return message.header.stamp.sec+message.header.stamp.nanosec/1e9


def yaw(q) -> float:
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


@dataclass
class Observation:
    received: float
    stamp: float
    pose: np.ndarray
    points: np.ndarray
    model: AlignmentMap
    score: AlignmentScore
    odom_pose: np.ndarray
    stationary: bool
    revision: str


def read_events(bag: Path) -> tuple[list, list]:
    events, sources = [], []
    for path in sorted(bag.rglob('*.db3')):
        digest=hashlib.sha256()
        with sqlite3.connect(f'file:{path.resolve()}?mode=ro',uri=True) as db:
            db.execute('PRAGMA query_only=ON');db.execute('BEGIN')
            topics={i:(n,k) for i,n,k in db.execute('select id,name,type from topics') if n in TOPICS}
            if not topics:continue
            ids=','.join(map(str,topics))
            for i,t,data in db.execute(f'select topic_id,timestamp,data from messages where topic_id in ({ids}) order by timestamp,id'):
                n,k=topics[i];events.append((t,n,k,data))
                digest.update(f'{n}:{t}:'.encode());digest.update(data)
        sources.append({'path':str(path.resolve()),'selected_message_sha256':digest.hexdigest(),
                        'file_sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    events.sort(key=lambda e:e[0])
    return events,sources


def extract(bag: Path, end: float | None = None) -> tuple[list[Observation], dict]:
    """Reconstruct fresh observations at causal 5Hz ticks from each bag's own map."""
    events,sources=read_events(bag)
    if end is not None:events=[e for e in events if e[0]/1e9<=end]
    buffer=Buffer(cache_time=Duration(seconds=30.))
    scan=pose=model=odom=None
    revision=''
    classes, observations, resets, recorded = {},[],[],{}
    pointer=0;last=-math.inf;rejected={}
    for tick in range(events[0][0],events[-1][0]+1,200_000_000):
        now=tick/1e9
        while pointer<len(events) and events[pointer][0]<=tick:
            t,n,k,data=events[pointer];pointer+=1
            if k not in classes:classes[k]=get_message(k)
            m=deserialize_message(data,classes[k])
            if n in ('/tf','/tf_static'):
                for tr in m.transforms:
                    (buffer.set_transform_static if n=='/tf_static'else buffer.set_transform)(tr,'recording')
            elif n=='/map':
                cells=np.asarray(m.data).reshape(m.info.height,m.info.width)
                revision=hashlib.sha256(cells.tobytes()).hexdigest()
                model=AlignmentMap(cells,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation)))
            elif n=='/scan_raw':scan=m
            elif n=='/amcl_pose':pose=m
            elif n=='/odom':odom=m
            elif n=='/rosout' and m.name=='amcl' and m.msg=='initialPoseReceived':resets.append(t/1e9)
            elif n=='/localization/status':
                report=json.loads(m.data)
                if report.get('score') and report.get('stamp'):recorded[round(report['stamp'],5)]=report['score']
        reason=None
        if scan is None or pose is None or model is None or odom is None:reason='missing_input'
        elif not 0<=now-stamp(scan)<=.5:reason='scan_stale'
        elif not 0<=now-stamp(pose)<=1.:reason='pose_stale'
        elif not 0<=now-stamp(odom)<=.3:reason='odom_stale'
        elif pose.header.frame_id!='map':reason='pose_frame'
        elif any(not math.isfinite(v)for v in pose.pose.covariance) or any(not -1e-9<=pose.pose.covariance[i]<=.5 for i in (0,7,35)):reason='covariance'
        if reason:
            rejected[reason]=rejected.get(reason,0)+1;continue
        if stamp(scan)<=last:continue
        try:
            mount=buffer.lookup_transform('base_link',scan.header.frame_id,Time()).transform
            current=buffer.lookup_transform('map','base_link',Time.from_msg(scan.header.stamp)).transform
        except Exception:
            rejected['tf']=rejected.get('tf',0)+1;continue
        ranges=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        mask=np.isfinite(ranges)&(ranges>=.5)&(ranges<=min(scan.range_max,8.))
        points=np.column_stack((ranges[mask]*np.cos(angles[mask]),ranges[mask]*np.sin(angles[mask])))
        points=transform(points,np.array([mount.translation.x,mount.translation.y,yaw(mount.rotation)]))
        if len(points)>SETTINGS.max_points:points=points[np.linspace(0,len(points)-1,SETTINGS.max_points,dtype=int)]
        position=np.array([current.translation.x,current.translation.y,yaw(current.rotation)])
        twist=odom.twist.twist;op=odom.pose.pose
        observations.append(Observation(now,stamp(scan),position,points,model,model.score(points,position,SETTINGS),
            np.array([op.position.x,op.position.y,yaw(op.orientation)]),
            math.hypot(twist.linear.x,twist.linear.y)<.01 and abs(twist.angular.z)<.03,revision))
        last=stamp(scan)
    errors=[]
    for o in observations:
        other=recorded.get(round(o.stamp,5))
        if other:errors.append([abs(o.score.inlier_ratio-other['inlier_ratio']),abs(o.score.mean_distance-other['mean_distance'])])
    return observations,{'sources':sources,'start':events[0][0]/1e9,'end':events[-1][0]/1e9,
        'observations':len(observations),'rejected_ticks':rejected,'pose_resets':resets,
        'score_crosscheck':{'matched_scans':len(errors),'max_abs_errors':np.max(errors,axis=0).tolist()if errors else None}}


def stable(scores: list[AlignmentScore], stamps: list[float], settings: AlignmentSettings,
           *, recovery: bool = False, duration: float = 1.5) -> int | None:
    """Return first stable-run endpoint, resetting evidence across gaps or failures."""
    since=None;previous=None
    for i,(score,t) in enumerate(zip(scores,stamps)):
        if previous is not None and (t<=previous or t-previous>.6):since=None
        if score.acceptable(settings,recovery=recovery):
            if since is None:since=t
            if t-since>=duration:return i
        else:since=None
        previous=t
    return None


def candidate_gate(result: dict, floor: float, require_improvement: bool) -> tuple[bool,str]:
    if 'validation' not in result:return False,result['reason']
    score=AlignmentScore(**result['validation'])
    if not score.acceptable(replace(SETTINGS,recovery_inlier_ratio=floor),recovery=True):
        return False,'candidate_not_well_aligned'
    if result['alternative_gap'] is not None and result['alternative_gap']<SETTINGS.ambiguity_gap:
        return False,'ambiguous_candidate'
    if require_improvement and result['baseline']['mean_distance']-score.mean_distance<SETTINGS.improvement:
        return False,'insufficient_improvement'
    return True,'accepted'


def select_windows(observations: list[Observation], resets: list[float], count: int) -> list[list[Observation]]:
    windows=[]
    for i,o in enumerate(observations):
        if not o.stationary or len(o.points)<120:continue
        w=[x for x in observations[i:i+20] if x.stamp-o.stamp<=3.]
        if len(w)<6 or w[-1].stamp-o.stamp<2.:continue
        if any(not x.stationary or x.revision!=o.revision or len(x.points)<120 for x in w):continue
        if any(b.stamp-a.stamp>.6 for a,b in zip(w,w[1:])):continue
        if any(o.received-2<=t<=w[-1].received+1 for t in resets):continue
        if any(np.linalg.norm(x.odom_pose[:2]-o.odom_pose[:2])>.02 or abs(wrap(x.odom_pose[2]-o.odom_pose[2]))>.03 for x in w):continue
        windows.append(w)
    # Evenly sample normal and difficult stationary evidence, avoiding adjacent windows.
    selected=[]
    for normal in (True,False):
        candidates=[w for w in windows if (stable([x.score for x in w],[x.stamp for x in w],SETTINGS)is not None)==normal]
        spaced=[]
        for w in candidates:
            if not spaced or w[0].stamp-spaced[-1][0].stamp>=15:spaced.append(w)
        for i in np.linspace(0,len(spaced)-1,min((count+1)//2,len(spaced)),dtype=int):
            selected.append(spaced[i])
    return sorted(selected,key=lambda w:w[0].stamp)


def evaluate(window: list[Observation], offset: tuple[float,float,float]) -> dict:
    reference=window[0];seed=reference.pose+np.array(offset);seed[2]=wrap(seed[2])
    # Rigidly offset the whole recorded pose stream; never let future unperturbed
    # AMCL poses silently erase the injected error.
    def shifted(pose: np.ndarray) -> np.ndarray:
        local=transform((pose[:2]-reference.pose[:2])[None,:],np.array([0.,0.,offset[2]]))[0]
        return np.array([*(seed[:2]+local),wrap(pose[2]+offset[2])])
    current_scores=[x.model.score(x.points,shifted(x.pose),SETTINGS)for x in window]
    result=search_pose(reference.model,reference.points,seed,replace(SETTINGS,recovery_inlier_ratio=.30),
                       require_improvement=not current_scores[0].acceptable(SETTINGS))
    candidate=np.array(result.get('pose',seed))
    # Propagate a candidate only by recorded wheel odometry during the stopped window.
    candidate_scores=[]
    for x in window:
        local=transform((x.odom_pose[:2]-reference.odom_pose[:2])[None,:],
                        np.array([0.,0.,-reference.odom_pose[2]]))[0]
        world=transform(local[None,:],candidate)[0]
        cp=np.array([*world,wrap(candidate[2]+x.odom_pose[2]-reference.odom_pose[2])])
        candidate_scores.append(x.model.score(x.points,cp,SETTINGS))
    next_index=next((i for i,x in enumerate(window) if x.stamp-reference.stamp>.15),None)
    policies={}
    def outcome(accepted: bool, kind: str, pose: np.ndarray, reason: str, index: int = 0) -> dict:
        return {'accepted':accepted,'operation':kind if accepted else 'reject','reason':reason,
                'latency_observation_s':window[index].stamp-reference.stamp if accepted else None,
                'pose':pose.tolist()if accepted else None,
                'reference_translation_error_m':float(np.linalg.norm(pose[:2]-reference.pose[:2]))if accepted else None,
                'reference_yaw_error_rad':abs(wrap(pose[2]-reference.pose[2]))if accepted else None}
    for floor in FLOORS:
        ok,reason=candidate_gate(result,floor,not current_scores[0].acceptable(SETTINGS))
        if ok and (next_index is None or not candidate_scores[next_index].acceptable(
                replace(SETTINGS,recovery_inlier_ratio=floor),recovery=True)):
            ok,reason=False,'failed_new_scan'
        policies[f'floor_{floor:.2f}']=outcome(ok,'reseed',candidate,reason,next_index or 0)
    settled=stable(current_scores,[x.stamp for x in window],SETTINGS)
    if settled is not None:
        policies['structured']=outcome(True,'keep',seed,'current_pose_reverified',settled)
    else:
        ok,reason=candidate_gate(result,.30,True)
        validated=stable(candidate_scores[1:],[x.stamp for x in window[1:]],SETTINGS)
        if ok and validated is None:ok,reason=False,'candidate_not_stable'
        policies['structured']=outcome(ok,'reseed',candidate,reason,validated+1 if validated is not None else 0)
    return {'offset':offset,'seed':seed.tolist(),'reference':reference.pose.tolist(),
            'reference_stable':stable([x.score for x in window],[x.stamp for x in window],SETTINGS)is not None,
            'seed_score':asdict(current_scores[0]),'search':result,'policies':policies}


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,action='append',required=True)
    parser.add_argument('--end',type=float)
    parser.add_argument('--windows-per-bag',type=int,default=6)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    settings={'normal':asdict(SETTINGS),'floors':FLOORS,'offsets':OFFSETS,
              'stationary_window_s':3.,'stable_duration_s':1.5,'windows_per_bag':args.windows_per_bag,
              'reference_warning':'Recorded AMCL is not ground truth. Offset residuals are reference-relative.',
              'structured':'Keep current pose after stable normal observations; otherwise 30% candidate floor, improvement, uniqueness and 1.5s fresh-scan stability.',
              'baseline':'Only recovery ratio changes; same bounded search, improvement/ambiguity checks and one subsequent fresh scan.',
              'boundary':'Candidate admission comparison; AMCL after reseeding is not re-simulated; no physical success claim.'}
    (args.output/'protocol.json').write_text(json.dumps(settings,indent=2))
    rclpy.init(domain_id=196)
    metadata=[];cases=[]
    try:
        for index,bag in enumerate(args.bag):
            observations,meta=extract(bag,args.end)
            windows=select_windows(observations,meta['pose_resets'],args.windows_per_bag)
            meta.update(bag=bag.name,windows=len(windows),split='development'if index<3 else'holdout')
            metadata.append(meta)
            print(json.dumps({'bag':bag.name,'observations':len(observations),'windows':len(windows)}),flush=True)
            for wi,window in enumerate(windows):
                for oi,offset in enumerate(OFFSETS):
                    began=time.monotonic();result=evaluate(window,offset)
                    result.update(bag=bag.name,split=meta['split'],window=wi,time=window[0].stamp,
                                  compute_s=time.monotonic()-began)
                    cases.append(result)
                print(json.dumps({'bag':bag.name,'window':wi,'cases':len(cases)}),flush=True)
                (args.output/'results.json').write_text(json.dumps(cases,indent=2))
            # Save exact selected inputs so extraction and policy search can be audited separately.
            for wi,w in enumerate(windows):
                np.savez_compressed(args.output/f'window-{index}-{wi}.npz',
                    cells=w[0].model.cells,resolution=w[0].model.resolution,origin=w[0].model.origin,
                    stamps=np.array([x.stamp for x in w]),poses=np.array([x.pose for x in w]),
                    odom=np.array([x.odom_pose for x in w]),
                    **{f'points_{j}':x.points for j,x in enumerate(w)})
            (args.output/'inputs.json').write_text(json.dumps(metadata,indent=2))
    finally:rclpy.shutdown()
    (args.output/'source-sha256.txt').write_text(hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    print(json.dumps({'complete':True,'cases':len(cases)}),flush=True)


if __name__=='__main__':
    main()
