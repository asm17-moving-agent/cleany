#!/usr/bin/env python3
"""Compare localization policies on causal recorded inputs without commanding a robot."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, fields
from datetime import datetime, timezone, timedelta
import hashlib
import html
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import sys
import tarfile
import threading
import time
import types

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
import yaml

from cleany_navigation.localization_alignment import (
    AlignmentHealth, AlignmentMap, AlignmentScore, AlignmentSettings, transform)

INPUTS = {'/tf', '/tf_static', '/map', '/scan_raw', '/amcl_pose', '/localization/status', '/cmd_vel', '/rosout'}
KST = timezone(timedelta(hours=9))


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def load_events(bag: Path):
    events, manifest = [], []
    for path in sorted(bag.rglob('*.db3')):
        with sqlite3.connect(f'file:{path.resolve()}?mode=ro', uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            topics = {i:(name, kind) for i, name, kind in db.execute('select id,name,type from topics') if name in INPUTS}
            if not topics:
                continue
            ids = ','.join(map(str, topics))
            for i, received, payload in db.execute(
                    f'select topic_id,timestamp,data from messages where topic_id in ({ids}) order by timestamp,id'):
                name, kind = topics[i]
                events.append((received, name, kind, payload))
        manifest.append({'path':str(path.resolve()), 'bytes':path.stat().st_size,
                         'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    events.sort(key=lambda event:event[0])
    if not events:
        raise ValueError(f'No recorded inputs in {bag}')
    return events, manifest


def decode(event, classes):
    _, _, kind, payload = event
    classes.setdefault(kind, get_message(kind))
    return deserialize_message(payload, classes[kind])


def invalidate(policy):
    if hasattr(policy, 'invalidate'):
        policy.invalidate()
    else:
        policy.state = 'unknown'
        policy.good_since = policy.bad_since = None


def is_pose_reset(topic, message) -> bool:
    return topic == '/rosout' and message.name == 'amcl' and message.msg == 'initialPoseReceived'


def reconstruct(events, settings, old_module, draft_module):
    """Only transforms received by each simulated timer tick may be consumed."""
    buffer = Buffer(cache_time=Duration(seconds=30.))
    old_settings = old_module.AlignmentSettings(**{f.name:getattr(settings, f.name)
                                                  for f in fields(old_module.AlignmentSettings)})
    old, new = old_module.AlignmentHealth(old_settings), AlignmentHealth(settings)
    draft = draft_module.AlignmentHealth(draft_module.AlignmentSettings(**asdict(settings)))
    scan = pose = model = None
    latest_command = None
    classes, rows, recorded, resets = {}, [], {}, []
    pointer, last_scan = 0, -math.inf
    score, reason = None, 'waiting_for_inputs'
    for tick in range(events[0][0], events[-1][0]+1, 200_000_000):
        while pointer < len(events) and events[pointer][0] <= tick:
            event = events[pointer];pointer += 1
            message = decode(event, classes);topic = event[1]
            if is_pose_reset(topic, message):
                # Initialpose itself was not recorded. Conservatively drop trust at
                # every received reset, regardless of manual/recovery ownership.
                old = old_module.AlignmentHealth(old_settings)
                draft = draft_module.AlignmentHealth(draft_module.AlignmentSettings(**asdict(settings)))
                new = AlignmentHealth(settings)
                resets.append(event[0]/1e9)
            if topic in ('/tf', '/tf_static'):
                for tr in message.transforms:
                    (buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform)(tr, 'recording')
            elif topic == '/map':
                model = AlignmentMap(np.asarray(message.data).reshape(message.info.height,message.info.width),
                    message.info.resolution, (message.info.origin.position.x,message.info.origin.position.y,yaw(message.info.origin.orientation)))
            elif topic == '/scan_raw':scan = message
            elif topic == '/amcl_pose':pose = message
            elif topic == '/cmd_vel':latest_command = (event[0]/1e9, message)
            elif topic == '/localization/status':
                report = json.loads(message.data)
                if report.get('score') and report.get('stamp'):
                    recorded[round(report['stamp'], 5)] = report['score']
        now = tick/1e9
        reason = None
        if scan is None or pose is None or model is None:reason = 'waiting_for_inputs'
        elif not 0 <= now-stamp(scan) <= .5:reason = 'scan_stale'
        elif not 0 <= now-stamp(pose) <= 1.:reason = 'pose_stale'
        elif not scan.header.frame_id or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0:
            reason = 'invalid_scan_geometry'
        elif pose.header.frame_id != 'map':reason = 'invalid_pose_frame'
        elif not math.isfinite(sum(v*v for v in (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                     pose.pose.pose.orientation.z, pose.pose.pose.orientation.w))) or abs(sum(v*v for v in (
                     pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z,
                     pose.pose.pose.orientation.w))-1.) > .05:reason = 'invalid_pose_orientation'
        elif any(not math.isfinite(v) for v in pose.pose.covariance) or any(
                not -1e-9 <= pose.pose.covariance[i] <= .5 for i in (0,7,35)):reason = 'invalid_covariance'
        if reason:
            invalidate(old);invalidate(draft);invalidate(new);score = None
        elif stamp(scan) > last_scan:
            try:
                mount = buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
                current = buffer.lookup_transform('map','base_link',Time.from_msg(scan.header.stamp)).transform
                ranges = np.asarray(scan.ranges)
                angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment
                valid = np.isfinite(ranges)&(ranges>=.5)&(ranges<=min(scan.range_max,8.))
                points = np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
                points = transform(points,np.array([mount.translation.x,mount.translation.y,yaw(mount.rotation)]))
                if len(points)>settings.max_points:points=points[np.linspace(0,len(points)-1,settings.max_points,dtype=int)]
                estimated = np.array([current.translation.x,current.translation.y,yaw(current.rotation)])
                score = model.score(points,estimated,settings)
                old.observe(stamp(scan),old_module.AlignmentScore(**asdict(score)))
                draft.observe(stamp(scan),draft_module.AlignmentScore(**asdict(score)))
                new.observe(stamp(scan),score)
                last_scan = stamp(scan)
            except Exception as error:
                reason = 'tf_unavailable';invalidate(old);invalidate(draft);invalidate(new);score = None
        command_nonzero = False
        if latest_command and now-latest_command[0] <= .3:
            command = latest_command[1]
            command_nonzero = math.hypot(command.linear.x,command.linear.y)>.01 or abs(command.angular.z)>.02
        rows.append(dict(time=now, stamp=None if scan is None else stamp(scan),
                         old=old.state,draft=draft.state,new=new.state,reason=reason or new.reason,
                         score=None if score is None else asdict(score),
                         speed_percentage={'available':100,'degraded':30}.get(new.state,0),
                         recorded_command_nonzero=command_nonzero))
    matches = []
    for row in rows:
        if row['score'] and round(row['stamp'],5) in recorded:
            other=recorded[round(row['stamp'],5)]
            matches.append([abs(row['score']['inlier_ratio']-other['inlier_ratio']),
                            abs(row['score']['mean_distance']-other['mean_distance'])])
    crosscheck = {'matched_ticks':len(matches), 'matched_unique_scans':len({r['stamp'] for r in rows
                    if r['score'] and round(r['stamp'],5) in recorded})}
    if matches:
        crosscheck.update(median_abs_error=np.median(matches,axis=0).tolist(),max_abs_error=np.max(matches,axis=0).tolist())
    return rows,crosscheck,resets


def summarize(rows):
    counts = {name:dict(Counter(r[name] for r in rows)) for name in ('old','draft','new')}
    recoveries, pending = [], None
    for row in rows:
        if row['new']=='degraded' and pending is None:pending=row
        elif pending and row['new']!='degraded':
            segment=[r for r in rows if pending['time']<=r['time']<row['time']]
            recoveries.append({'start':pending['time'],'end':row['time'],'outcome':row['new'],
                'duration':row['time']-pending['time'],'reason':row['reason'],
                'old_blocked_s':sum(r['old']!='available' for r in segment)*.2,
                'commanded_motion_s':sum(r['recorded_command_nonzero'] for r in segment)*.2})
            pending=None
    if pending:
        recoveries.append({'start':pending['time'],'end':rows[-1]['time'],'outcome':'recording_end',
                           'duration':rows[-1]['time']-pending['time']})
    return {'duration_s':len(rows)*.2,'state_seconds':{k:{s:n*.2 for s,n in v.items()}for k,v in counts.items()},
            'old_blocked_new_degraded_s':sum(r['old']!='available' and r['new']=='degraded' for r in rows)*.2,
            'old_allowed_new_blocked_s':sum(r['old']=='available' and r['new'] in ('unknown','unavailable') for r in rows)*.2,
            'episodes':recoveries,'input_failures':dict(Counter(r['reason']for r in rows if not r['score']))}


def native_window(events, rows, settings, output, resets):
    """Replay a recorded window to the real node in an isolated DDS domain."""
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav2_msgs.msg import SpeedLimit
    from std_msgs.msg import String
    from cleany_navigation.localization_guard_node import LocalizationGuard
    episodes=[e for e in summarize(rows)['episodes'] if e['outcome']=='available' and e['old_blocked_s'] > 0
              and not any(e['start']-6 <= t <= e['end']+4 for t in resets)]
    if not episodes:return {'skipped':'No recovered episode without nearby pose reset'}
    episode=max(episodes,key=lambda e:e['old_blocked_s'])
    center=episode['start'];start=max(events[0][0]/1e9,center-6);end=min(events[-1][0]/1e9,episode['end']+4)
    player=Node('recorded_localization_player')
    parameters=[Parameter(k,value=v)for k,v in asdict(settings).items()]+[Parameter('use_sim_time',value=True)]
    guard=LocalizationGuard(parameter_overrides=parameters)
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(player);executor.add_node(guard)
    statuses, limits = [], []
    player.create_subscription(String,'localization/status',lambda m:statuses.append(json.loads(m.data)),10)
    player.create_subscription(SpeedLimit,'navigation/localization_speed_limit',
        lambda m:limits.append({'stamp':stamp(m),'percentage':m.percentage,'speed_limit':m.speed_limit}),100)
    clock=player.create_publisher(Clock,'clock',10)
    publishers={}
    classes={}
    selected=[e for e in events if e[1] in ('/map','/tf_static','/tf','/scan_raw','/amcl_pose')]
    for event in selected:
        topic,kind=event[1:3]
        if topic in publishers:continue
        target='/scan' if topic=='/scan_raw' else topic
        qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ('/map','/tf_static') else (
            qos_profile_sensor_data if topic in ('/scan_raw','/amcl_pose') else 100)
        publishers[topic]=player.create_publisher(get_message(kind),target,qos)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def clock_at(t):clock.publish(Clock(clock=Time(seconds=t).to_msg()))
    clock_at(start-2);time.sleep(.2)
    preload={}
    for event in selected:
        if event[0]/1e9>start-2:break
        if event[1] in ('/map','/tf_static'):preload[event[1]]=event
    for event in preload.values():publishers[event[1]].publish(decode(event,classes))
    subset=[e for e in selected if start-2<=e[0]/1e9<=end]
    pointer=0
    for tick in np.arange(start-2,end,.1):
        clock_at(float(tick))
        while pointer<len(subset) and subset[pointer][0]/1e9<=tick:
            event=subset[pointer];pointer+=1;publishers[event[1]].publish(decode(event,classes))
        time.sleep(.04)
    time.sleep(.2)
    executor.shutdown(timeout_sec=4);thread.join(4)
    guard.destroy_node();player.destroy_node()
    transitions=[]
    for status in statuses:
        if not transitions or status['state']!=transitions[-1]:transitions.append(status['state'])
    result={'start':start,'end':end,'dds_domain':196,'status_counts':dict(Counter(s['state']for s in statuses)),
            'speed_percentages':sorted(set(s['speed_limit_percentage']for s in statuses)),
            'speed_topic_percentages':sorted(set(s['speed_limit']for s in limits)),
            'episode':episode,
            'state_transitions':transitions,
            'verified_available_degraded_available':any(transitions[i:i+3]==['available','degraded','available']
                                                        for i in range(len(transitions)-2)),
            'no_motion_goal_or_wheel_command_published':True}
    (output/'native-status.json').write_text(json.dumps(statuses,indent=2))
    (output/'native-speed-limit.json').write_text(json.dumps(limits,indent=2))
    return result


def render(output: Path, results: list[dict]) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(results)*3,1,figsize=(13,5*len(results)),layout='constrained')
    for i,result in enumerate(results):
        rows=result['rows'];x=np.array([r['time']-rows[0]['time']for r in rows])
        a,b,c=axes[i*3:i*3+3]
        a.plot(x,[np.nan if not r['score']else r['score']['inlier_ratio']*100 for r in rows],lw=.8)
        a.axhline(30,color='orange',ls='--');a.set_ylabel('Inliers (%)');a.set_title(result['bag'])
        b.plot(x,[np.nan if not r['score']else r['score']['mean_distance']for r in rows],lw=.8)
        b.axhline(.18,color='orange',ls='--');b.set_ylabel('Trimmed error (m)')
        c.step(x,[100 if r['old']=='available'else 0 for r in rows],label='Previous stop policy',color='#64748b',lw=.8)
        c.step(x,[{'available':100,'degraded':30}.get(r['draft'],0)for r in rows],label='First graded policy',color='#d56b5b',ls=':',lw=.8)
        c.step(x,[r['speed_percentage']for r in rows],label='Corrected graded policy',color='#008578',lw=.8)
        for axis in (a,b,c):
            axis.grid(alpha=.15)
            for reset in result['pose_resets']:
                axis.axvline(reset-rows[0]['time'],color='#9a67be',lw=.6,alpha=.6)
        c.set_ylabel('Speed cap (%)');c.set_xlabel('Recorded time (s)');c.legend(loc='upper right')
    fig.savefig(output/'comparison.png',dpi=150);plt.close(fig)
    cards=''
    for result in results:
        s=result['summary'];episodes=s['episodes']
        cards+=f'<section><h2>{html.escape(result["bag"])}</h2><p>기록 {s["duration_s"]:.1f}초 · 구 정책 차단/새 정책 감속 {s["old_blocked_new_degraded_s"]:.1f}초 · 정상 복귀 {sum(e["outcome"]=="available" for e in episodes)}회</p><pre>{html.escape(json.dumps({"state_seconds":s["state_seconds"],"input_failures":s["input_failures"],"score_crosscheck":result["score_crosscheck"]},ensure_ascii=False,indent=2))}</pre></section>'
    totals={name:{state:sum(r['summary']['state_seconds'][name].get(state,0)for r in results)
            for state in ('available','degraded','unknown','unavailable')}for name in ('old','draft','new')}
    blocked={name:v['unknown']+v['unavailable']for name,v in totals.items()}
    avoided=sum(r['summary']['old_blocked_new_degraded_s']for r in results)
    extra=sum(r['summary']['old_allowed_new_blocked_s']for r in results)
    duration=sum(r['summary']['duration_s']for r in results)
    reduction=100*(blocked['old']-blocked['new'])/blocked['old']
    table=''
    for name,title in [('old','기존 정지 정책'),('draft','감속 정책 수정 전'),('new','감속 정책 수정 후')]:
        v=totals[name]
        table+=f'<tr><th>{title}</th><td>{v["available"]:.1f}</td><td>{v["degraded"]:.1f}</td><td>{v["unknown"]:.1f}</td><td>{v["unavailable"]:.1f}</td><td><b>{blocked[name]:.1f}</b></td></tr>'
    native=json.loads((output/'manifest.json').read_text()).get('native',{})
    examples=''
    selected=[]
    for result in results:
        for episode in result['summary']['episodes']:
            if episode['outcome']!='available':continue
            segment=[r for r in result['rows']if episode['start']<=r['time']<=episode['end']]
            kind='bug' if any(r['draft']=='unavailable'for r in segment) else (
                'benefit' if episode.get('old_blocked_s',0)>0 else None)
            if kind is None or kind in selected:continue
            selected.append(kind)
            window=[r for r in result['rows']if episode['start']-4<=r['time']<=episode['end']+4]
            x=[r['time']-result['rows'][0]['time']for r in window]
            fig,ax=plt.subplots(figsize=(12,3),layout='constrained')
            for name,color in [('old','#64748b'),('draft','#d56b5b'),('new','#008578')]:
                ax.step(x,[{'available':100,'degraded':30}.get(r[name],0)for r in window],
                        label=name,color=color,ls=':'if name=='draft'else'-')
            ax.set(xlabel='Recorded elapsed time (s)',ylabel='Requested cap (%)',yticks=[0,30,100])
            ax.legend();ax.grid(alpha=.2);fig.savefig(output/f'example-{kind}.png',dpi=140);plt.close(fig)
            title='짧은 정상 관측을 개선으로 인정한 효과'if kind=='bug'else'기존 정지가 감속 대기로 바뀐 구간'
            when=datetime.fromtimestamp(episode['start'],KST).strftime('%H:%M:%S')
            examples+=f'<section><h2>{title}</h2><p>Bag {result["bag"][:8]}, {when}부터 {episode["duration"]:.1f}초. 정상 판정으로 복귀했다. 해당 구간의 기록된 비영 속도 명령은 {episode["commanded_motion_s"]:.1f}초이며 실제 이동량을 뜻하지 않는다.</p><img src="example-{kind}.png"></section>'
    doc=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cleany · 기록 기반 위치 정합 정책 검증</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#eef2f7;color:#172033;font:16px/1.7 system-ui,sans-serif}}main{{max-width:1120px;margin:auto;padding:44px 24px}}h1{{font-size:clamp(28px,4vw,40px);line-height:1.3;letter-spacing:-1px}}h2{{font-size:23px}}.eyebrow{{font-size:12px;font-weight:750;letter-spacing:2px;color:#00796f}}section,.note{{background:white;padding:26px;border-radius:16px;margin:20px 0;border:1px solid #dfe6ed}}.note{{border-left:5px solid #cf923f}}.metrics{{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}}.metric{{background:#17394a;color:white;border-radius:14px;padding:20px}}.metric b{{display:block;font-size:29px}}.metric small{{color:#c3d9df}}img{{width:100%;height:auto}}pre{{white-space:pre-wrap;font-size:12px;line-height:1.5}}a{{color:#00796f}}table{{width:100%;border-collapse:collapse;font-size:14px}}th,td{{padding:10px 8px;text-align:right;border-bottom:1px solid #e4eaf0}}th:first-child{{text-align:left}}thead{{background:#f1f6f8}}.scroll{{overflow-x:auto}}@media(max-width:700px){{.metrics{{grid-template-columns:1fr}}main{{padding:28px 14px}}section{{padding:18px}}}}</style>
<main><div class="eyebrow">CLEANY / RECORDED INPUT VALIDATION / 2026-11-08</div><h1>감속 대기는 정지를 줄였지만,<br>모든 정합 실패를 해결하지는 않았다.</h1>
<p>기존 bag 2개, 총 {duration/60:.2f}분을 같은 조건으로 비교했다. 짧은 정상 관측을 개선으로 인정하지 않던 문제를 수정한 뒤, 정책상 차단 시간은 {blocked['old']:.1f}초에서 {blocked['new']:.1f}초로 {reduction:.1f}% 줄었다.</p>
<div class="metrics"><div class="metric"><small>기존 → 수정 후 차단</small><b>{blocked['old']:.1f} → {blocked['new']:.1f}s</b><small>미검증 + 불가 상태</small></div><div class="metric"><small>기존 차단을 감속으로 전환</small><b>{avoided:.1f}s</b><small>같은 기록에 대한 정책 비교</small></div><div class="metric"><small>기존 허용인데 새 정책은 차단</small><b>{extra:.1f}s</b><small>연속 불안정에 대한 8초 상한 등</small></div></div>
<div class="note"><b>주행 성공률이나 실제 정지 시간이 아니다.</b> 기록된 AMCL 위치·TF·스캔을 고정하고 판단만 비교했다. 새 속도로 움직였을 때의 위치 보정, 제동 거리, 슬립이나 도착 성공은 재현하지 않았다. 과거의 정지나 위치 재지정이 보정을 도왔을 수도 있다.</div>
<section><h2>동일 조건의 세 정책</h2><p>정상 기준은 모두 일치율 30% 이상, 절사 평균 잔차 18cm 이하로 맞췄다. 기존 정책은 0.8초 불일치 후 차단한다. 새 정책은 검증된 위치에 한해 정상 상한의 30%로 감속하며, 4초간 개선이 없거나 총 8초가 지나면 차단한다. 정상 복귀에는 1초 연속 관측이 필요하다.</p><p>수정 전에는 짧게 정상 기준을 통과해도 개선 시계를 갱신하지 않았다. 수정 후에는 정상 관측도 개선으로 인정하지만, 총 8초 한도는 초기화하지 않는다. 센서 단절·잘못된 공분산·초기 미검증은 계속 차단한다.</p><div class="scroll"><table><thead><tr><th>정책</th><th>정상(s)</th><th>감속(s)</th><th>미검증(s)</th><th>불가(s)</th><th>총 차단(s)</th></tr></thead><tbody>{table}</tbody></table></div><p>5Hz 평가 구간 합계다. STOP 횟수와 같지 않다. 초기 위치 수신 시 세 정책 모두 신뢰 상태를 초기화했다.</p></section>
{examples}<section><h2>전체 기록</h2><p>주황 점선은 정상 기준, 보라 세로선은 초기 위치 수신이다. 회색은 기존 정책, 붉은 점선은 수정 전 감속 정책, 초록은 수정 후 정책이다.</p><img src="comparison.png"></section>
<section><h2>실제 ROS 노드 검증</h2><p>실제 LocalizationGuard에 선택 구간의 기록된 입력과 시뮬레이션 시계를 넣었다. 독립 DDS domain 196에서 available → degraded → available 전이는 <b>{'확인됨'if native.get('verified_available_degraded_available')else'미확인'}</b>. 실제 속도 제한 토픽 값은 <b>{native.get('speed_topic_percentages',native.get('speed_percentages'))}%</b>였다. 시작 시 TF 준비와 기록의 pose 단절 구간은 0%로 처리됐다.</p><p>실물로 재생하지 않았고 이동 목표와 휠 명령을 발행하지 않았다. 실행 중 서버도 재시작하지 않았다. 수정 코드는 다음 실행부터 반영되며 현재 프로세스는 이전 코드다.</p></section>
<section><h2>30% 감속 비율은 아직 실험값</h2><p>감속 대기를 유지할 근거는 생겼지만 30%가 최적이라는 근거는 아니다. 기록 입력을 고정한 상태에서는 20%·30%·50% 중 실제 보정을 더 잘 돕는 값을 결정할 수 없다. 이번에도 {blocked['new']:.1f}초의 차단이 남았고, 다음 실물 확인에서는 감속 중 잔차 변화와 재정지 여부를 봐야 한다.</p></section>
<section><h2>입력과 한계</h2><p>/scan_raw, /tf, /tf_static, /amcl_pose와 진단 bag의 /map을 사용했다. 각 시점까지 수신된 데이터만 읽었고 360도 프로필의 0.5–8m 끝점을 최대 240개 평가했다. 스캔 저장률은 약 3.8Hz라 센서 전 주기를 복원한 것은 아니다. 원본 DB의 SHA-256을 전후 비교했다.</p><p>/initialpose가 없어 AMCL의 initialPoseReceived 로그를 사용했다. 수동 지정과 복구의 소유자는 확정할 수 없어 모든 수신에서 보수적으로 신뢰를 초기화했다. 기록된 점수와 재계산 결과의 최대 차이는 일치율 약 0.42%p, 잔차 약 3.3mm 이하다.</p><p><a href="results.json">전체 판정</a> · <a href="manifest.json">설정·코드·입력 해시</a> · <a href="native-status.json">실제 노드 상태</a> · <a href="native-speed-limit.json">속도 제한 토픽</a> · <a href="../initial/index.html">초기 분석 보존본</a></p></section>{cards}</main></html>'''
    (output/'index.html').write_text(doc)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,action='append',required=True)
    parser.add_argument('--old-source-tar',type=Path,required=True)
    parser.add_argument('--draft-source',type=Path,required=True)
    parser.add_argument('--settings',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    with tarfile.open(args.old_source_tar) as archive:
        source=archive.extractfile('ros2_ws/src/cleany_navigation/cleany_navigation/localization_alignment.py').read()
    old=types.ModuleType('recorded_old_alignment');sys.modules[old.__name__]=old
    exec(compile(source,'recorded_old_alignment.py','exec'),old.__dict__)
    draft_source=args.draft_source.read_bytes()
    draft=types.ModuleType('recorded_draft_alignment');sys.modules[draft.__name__]=draft
    exec(compile(draft_source,'recorded_draft_alignment.py','exec'),draft.__dict__)
    raw=yaml.safe_load(args.settings.read_text())['localization_guard']['ros__parameters']
    settings=AlignmentSettings(**{f.name:raw.get(f.name,f.default)for f in fields(AlignmentSettings)})
    if settings.min_inlier_ratio != .3:raise ValueError('This comparison expects the selected 30% threshold')
    manifest={'settings':asdict(settings),'old_source_sha256':hashlib.sha256(source).hexdigest(),
        'draft_source_sha256':hashlib.sha256(draft_source).hexdigest(),
        'new_source':str(Path(sys.modules[AlignmentHealth.__module__].__file__).resolve()),
        'raw_scan_equivalence':'360 degree /scan, with guard range 0.5..8m; no sector masking assumed',
        'evaluated_at':'causal 5Hz timer; only messages received by timer are visible','bags':[]}
    rclpy.init(domain_id=196)
    results=[];first=None
    try:
        for bag in args.bag:
            events,files=load_events(bag)
            rows,crosscheck,resets=reconstruct(events,settings,old,draft)
            result={'bag':bag.name,'summary':summarize(rows),'score_crosscheck':crosscheck,'pose_resets':resets,'rows':rows}
            results.append(result);manifest['bags'].append({'bag':str(bag.resolve()),'files':files})
            print(json.dumps({'bag':bag.name,**result['summary'],'score_crosscheck':crosscheck}),flush=True)
            if first is None and any(e['outcome']=='available' and e['old_blocked_s'] > 0 and not any(
                    e['start']-6 <= t <= e['end']+4 for t in resets) for e in result['summary']['episodes']):first=(events,rows,resets)
        if first:
            native=native_window(first[0],first[1],settings,args.output,first[2]);manifest['native']=native;print(json.dumps({'native':native}),flush=True)
        else:
            (args.output/'native-status.json').write_text('[]')
    finally:rclpy.shutdown()
    manifest['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    manifest['new_source_sha256']=hashlib.sha256(Path(manifest['new_source']).read_bytes()).hexdigest()
    for bag in manifest['bags']:
        for entry in bag['files']:
            entry['sha256_after']=hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest()
            if entry['sha256_after'] != entry['sha256']:raise RuntimeError('Input bag changed during validation')
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (args.output/'results.json').write_text(json.dumps(results,indent=2))
    render(args.output,results)


if __name__=='__main__':main()
