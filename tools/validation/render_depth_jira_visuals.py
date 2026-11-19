#!/usr/bin/env python3
"""SCRUM-556 검토용 실측 재생과 설정 설명 그림을 읽기 전용으로 생성합니다.

rosbags, numpy, scipy, matplotlib, Pillow가 필요합니다. ROS 노드를 실행하지
않으며 원본 실행 폴더를 수정하지 않습니다. 재생에는 해당 시점까지 수신되고
header 시각도 지나간 메시지만 사용합니다. 출력 manifest에 출처와 한계를 남깁니다.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import hashlib
import html
import json
from pathlib import Path
import sqlite3
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch, Polygon, Rectangle
from matplotlib.lines import Line2D
from PIL import Image, ImageDraw, ImageFont
import numpy as np
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO/'ros2_ws/src/cleany_navigation'))
from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.depth_obstacles import ObstacleFilter, filter_obstacles

KST = timezone(timedelta(hours=9))
BG, FG, TEAL, ORANGE, BLUE = '#f3f7f5', '#203d3a', '#137c70', '#d16b32', '#507fc0'
plt.rcParams.update({'font.family': 'NanumGothic', 'font.size': 11,
    'axes.unicode_minus': False, 'figure.facecolor': BG, 'axes.facecolor': 'white',
    'text.color': FG, 'axes.labelcolor': FG, 'axes.titleweight': 'bold',
    'axes.spines.top': False, 'axes.spines.right': False, 'savefig.facecolor': BG})
STORE = get_typestore(Stores.ROS2_HUMBLE)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def time_label(t: float) -> str:
    return datetime.fromtimestamp(t, KST).strftime('%H:%M:%S')


def time_value(s: str) -> float:
    return datetime.fromisoformat('2026-11-08T'+s).replace(tzinfo=KST).timestamp()


def matrix(translation, quaternion) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = translation
    return result


def ros_matrix(t) -> np.ndarray:
    p, q = t.translation, t.rotation
    return matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def dict_matrix(t: dict) -> np.ndarray:
    return matrix([t['translation'][k] for k in ('x','y','z')],
                  [t['rotation'][k] for k in ('x','y','z','w')])


def apply(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


@dataclass
class Series:
    """수신과 발행 시각을 모두 지난 뒤 사용할 수 있는 표본을 보관합니다."""
    times: list
    messages: list

    def at(self, t: float, max_age: float = float('inf')):
        index = bisect_right(self.times, t)-1
        if index < 0 or t-self.times[index] > max_age:
            return None
        return self.messages[index]


class Recording:
    """저장된 SQLite 메시지를 역직렬화하고 과거 표본만 조회합니다."""
    def __init__(self, bag: Path):
        self.bag = bag
        entries, transforms = {}, {}
        wanted = {'/odom','/plan','/scan_raw','/map','/local_costmap/costmap',
                  '/local_costmap/published_footprint','/cmd_vel','/nav2/cmd_vel',
                  '/localization/status','/rosout'}
        self.sources = sorted(bag.rglob('*.db3'))
        self.counts = {}
        for path in self.sources:
            with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
                for ident, name, typ in db.execute('SELECT id,name,type FROM topics'):
                    if name not in wanted | {'/tf','/tf_static'}:
                        continue
                    target = entries.setdefault(name, [])
                    count = 0
                    for ns, raw in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (ident,)):
                        m = STORE.deserialize_cdr(raw, typ)
                        received = ns*1e-9
                        if name in ('/tf','/tf_static'):
                            for tf in m.transforms:
                                key = (tf.header.frame_id, tf.child_frame_id)
                                if name == '/tf' and key[1] not in ('base_link','odom','d435_pan_link','d435_tilt_link','d435_link'):
                                    continue
                                available = max(received, stamp(tf))
                                transforms.setdefault(key, []).append((available, ros_matrix(tf.transform), name == '/tf_static'))
                        else:
                            available = max(received, stamp(m)) if hasattr(m, 'header') else received
                            target.append((available, m))
                        count += 1
                    self.counts[name] = self.counts.get(name,0)+count
        self.data = {k: Series([p[0] for p in sorted(v, key=lambda p:p[0])],
                              [p[1] for p in sorted(v, key=lambda p:p[0])]) for k,v in entries.items() if v}
        self.transforms = {}
        for key, values in transforms.items():
            values.sort(key=lambda p:p[0])
            self.transforms[key] = (Series([x[0] for x in values], [x[1] for x in values]), values[0][2])
        self.audit = []

    def tf(self, target: str, source: str, t: float) -> np.ndarray:
        if target == source:
            return np.eye(4)
        graph = {}
        for (parent, child), (series, static) in self.transforms.items():
            value = series.at(t, float('inf') if static else 1.5)
            if value is None:
                continue
            graph.setdefault(child, []).append((parent, value))
            graph.setdefault(parent, []).append((child, np.linalg.inv(value)))
        pending, seen = [(source, np.eye(4))], set()
        while pending:
            frame, value = pending.pop()
            if frame == target:
                return value
            if frame in seen:
                continue
            seen.add(frame)
            pending.extend((other, transform@value) for other,transform in graph.get(frame, []) if other not in seen)
        raise ValueError(f'TF 없음: {source} → {target}, {time_label(t)}')


class Bundle:
    def __init__(self, out: Path):
        self.out = out
        self.out.mkdir(parents=True, exist_ok=True)
        self.items = read_json(out/'manifest.json')['items'] if (out/'manifest.json').exists() else []
        self.metrics = read_json(out/'metrics.json') if (out/'metrics.json').exists() else {}

    def add(self, number: int, title: str, filename: str, scope: str, sources: list[str], note: str, **extra):
        self.items = [item for item in self.items if item['number'] != number]
        self.items.append(dict(number=number, title=title, file=filename, scope=scope,
                               sources=sources, note=note, **extra))

    def save(self, fig, filename: str):
        fig.savefig(self.out/filename, dpi=140)
        plt.close(fig)
        print('생성:', filename, flush=True)


def figure(title: str, subtitle: str, rows=1, cols=1, size=(12,7)):
    fig, axes = plt.subplots(rows, cols, figsize=size, squeeze=False)
    fig.suptitle(title, x=.06, y=.97, ha='left', fontsize=21, weight='bold')
    fig.text(.06,.895,subtitle,fontsize=10,color='#58736b')
    fig.subplots_adjust(left=.075,right=.96,bottom=.14,top=.79,wspace=.26,hspace=.38)
    return fig, axes


def footer(fig, text: str):
    fig.text(.06,.035,text,fontsize=9,color='#58736b')


def axes_xy(ax, frame='odom'):
    ax.set(xlabel=f'{frame} X (m)',ylabel=f'{frame} Y (m)',aspect='equal')
    ax.grid(alpha=.15)


def grid_rgba(cells: np.ndarray, local=False) -> np.ndarray:
    result = np.zeros((*cells.shape,4), dtype=float)
    if local:
        result[cells>0] = [.64,.79,.72,.5]
        result[cells==99] = [.86,.68,.40,.45]
        result[cells==100] = [.12,.25,.23,.9]
        result[cells<0] = [.7,.73,.72,.22]
    else:
        result[:] = [.97,.98,.96,1]
        result[cells<0] = [.86,.9,.88,1]
        result[cells>50] = [.32,.4,.37,1]
    return result


def extent(info) -> list[float]:
    x,y = info.origin.position.x,info.origin.position.y
    return [x,x+info.width*info.resolution,y,y+info.height*info.resolution]


def show_grid(ax, m, local=False):
    assert abs(m.info.origin.orientation.z)<1e-6, '회전 grid 원점 미지원'
    ax.imshow(grid_rgba(m.data.reshape(m.info.height,m.info.width),local), origin='lower',
              extent=extent(m.info),interpolation='nearest',zorder=0)


def point_array(points) -> np.ndarray:
    return np.array([[p.x,p.y,p.z] for p in points]).reshape(-1,3)


def robot(ax, transform, footprint=None):
    if footprint is None:
        footprint = np.array([[-.2885,-.3774,0],[-.2885,.3774,0],[.2885,.3774,0],[.2885,-.3774,0]])
    p = apply(footprint,transform)
    ax.add_patch(Polygon(p[:,:2],closed=True,facecolor=ORANGE,edgecolor='white',alpha=.85,zorder=8))
    direction = transform[:2,0]*.45
    ax.arrow(*transform[:2,3],*direction,width=.025,color=ORANGE,zorder=9)


def pose_matrix(odom) -> np.ndarray:
    p,q=odom.pose.pose.position,odom.pose.pose.orientation
    return matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])


def scan_points(m) -> np.ndarray:
    r=np.asarray(m.ranges); angle=m.angle_min+np.arange(len(r))*m.angle_increment
    keep=np.isfinite(r)&(r>=m.range_min)&(r<=m.range_max)
    return np.column_stack((r[keep]*np.cos(angle[keep]),r[keep]*np.sin(angle[keep]),np.zeros(keep.sum())))


def trajectory(rec: Recording, start: float, end: float, frame='map') -> np.ndarray:
    result=[]
    series=rec.data['/odom']
    for t,m in zip(series.times,series.messages):
        if not start<=t<=end:
            continue
        try:
            transform=rec.tf(frame,m.header.frame_id,t)@pose_matrix(m)
            result.append((t,*transform[:2,3]))
        except ValueError:
            result.append((t,np.nan,np.nan))
    return np.asarray(result)


def split_trace(trace: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
    """누락 구간과 위치 보정 점프를 연속 이동으로 연결하지 않습니다."""
    x,y=trace[:,1].copy(),trace[:,2].copy()
    gaps=(np.diff(trace[:,0])>1)|(np.linalg.norm(np.diff(trace[:,1:3],axis=0),axis=1)>.5)
    x[1:][gaps]=np.nan; y[1:][gaps]=np.nan
    return x,y


def render_replay(bundle: Bundle, rec: Recording, start: float, end: float, number: int, close=False):
    title='장애물 인접 구간 확대' if close else '실제 주행 기록 재생'
    filename=f'{number:02d}-'+('obstacle-closeup.gif' if close else 'driving-replay.gif')
    step=.25 if close else .4
    times=np.arange(start,end+1e-6,step)
    trace=trajectory(rec,start,end,'odom' if close else 'map')
    all_xy=trace[:,1:3]; valid=all_xy[np.isfinite(all_xy).all(axis=1)]
    limits=[valid.min(axis=0)-1.8,valid.max(axis=0)+1.8]
    frames=[]; audits=[]
    for index,t in enumerate(times):
        fig,axs=figure(title,f'실물 기록 재구성  |  2026-11-08 {time_label(t)} KST  |  {step/.1:g}배속',cols=2,size=(12,6.5))
        fig.subplots_adjust(bottom=.20)
        map_ax,local_ax=axs[0]
        odom=rec.data['/odom'].at(t,.5)
        cost=rec.data['/local_costmap/costmap'].at(t,1.5)
        plan=rec.data['/plan'].at(t,2)
        footprint=rec.data['/local_costmap/published_footprint'].at(t,1.5)
        frame='odom' if close else 'map'
        if not close:
            show_grid(map_ax,rec.data['/map'].at(t))
        past=trace[trace[:,0]<=t]
        if len(past):
            x,y=split_trace(past);map_ax.plot(x,y,color=TEAL,lw=2,label='기록 궤적')
        missing=[]
        if odom:
            try:
                base_odom=pose_matrix(odom);base=rec.tf(frame,'odom',t)@base_odom
                if footprint:
                    fp=point_array(footprint.polygon.points)
                    map_ax.add_patch(Polygon(apply(fp,rec.tf(frame,footprint.header.frame_id,t))[:,:2],color=ORANGE,alpha=.75,zorder=8))
                    local_ax.add_patch(Polygon(fp[:,:2],color=ORANGE,alpha=.7,zorder=8))
                else:
                    missing.append('footprint')
                for ax,tr in [(map_ax,base),(local_ax,base_odom)]:
                    ax.scatter(*tr[:2,3],color=ORANGE,s=25,zorder=9)
                    ax.arrow(*tr[:2,3],*(tr[:2,0]*.4),color=ORANGE,width=.025,zorder=9)
            except ValueError:
                missing.append('로봇 TF')
        else:
            missing.append('odom')
        if cost:
            show_grid(local_ax,cost,True)
            if close:
                show_grid(map_ax,cost,True)
        else:
            missing.append('costmap')
        if plan and plan.poses:
            p=point_array([x.pose.position for x in plan.poses])
            for ax,target in [(map_ax,frame),(local_ax,'odom')]:
                try:
                    xy=apply(p,rec.tf(target,plan.header.frame_id,t)); ax.plot(xy[:,0],xy[:,1],color=BLUE,lw=2,ls='--',label='기록 계획 경로')
                except ValueError:
                    missing.append('plan TF')
        scan=rec.data['/scan_raw'].at(t,.5)
        if scan:
            try:
                points=scan_points(scan)
                for ax,target in [(map_ax,frame),(local_ax,'odom')]:
                    tr=rec.tf(target,scan.header.frame_id,stamp(scan))
                    xy=apply(points,tr);ax.scatter(xy[:,0],xy[:,1],s=2,c=BLUE,alpha=.45,zorder=3)
            except ValueError:
                missing.append('scan TF')
        map_ax.set_xlim(limits[0][0],limits[1][0]);map_ax.set_ylim(limits[0][1],limits[1][1])
        if cost:
            e=extent(cost.info);local_ax.set_xlim(e[:2]);local_ax.set_ylim(e[2:])
        axes_xy(map_ax,frame);axes_xy(local_ax,'odom')
        map_ax.set_title('관측과 기록 궤적' if close else '지도와 기록 궤적')
        age=t-stamp(cost) if cost else float('nan')
        local_ax.set_title(f'통합 Local Costmap  |  표본 나이 {age:.2f}s')
        fig.legend(handles=[Patch(color=FG,label='장애물 셀 100'),Patch(color='#e8d9bf',label='내접 비용 99'),
            Patch(color='#d2e3d9',label='주변 비용'),Patch(color=ORANGE,label='기록 footprint'),
            Line2D([],[],color=BLUE,ls='--',label='계획 경로'),Line2D([],[],color=BLUE,marker='.',ls='',label='LiDAR'),
            Line2D([],[],color=TEAL,label='궤적')],
            loc='lower center',bbox_to_anchor=(.5,.065),ncol=7,fontsize=8,frameon=False)
        footer(fig,'실측 bag 재생 · LiDAR+depth 통합 결과 · 원본 depth 미저장'+('  |  누락: '+', '.join(sorted(set(missing))) if missing else ''))
        fig.canvas.draw(); frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3]).copy())
        plt.close(fig)
        audits.append(dict(time=float(t),costmap_header=stamp(cost) if cost else None,missing=missing))
        if index%40==0:print(filename,index,'/',len(times),flush=True)
    frames[0].save(bundle.out/filename,save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=False)
    frames[len(frames)//2].save(bundle.out/filename.replace('.gif','-poster.png'))
    bundle.add(number,title,filename,'실물 기록 재생',[str(p) for p in rec.sources],
        '기록된 costmap을 표시합니다. 원본 depth 점군 미저장으로 sensor별 기여는 분리하지 않습니다. 위치 보정 점프와 1초 초과 궤적 공백은 연결하지 않습니다.',
        start=time_label(start),end=time_label(end),sample_step_s=step,playback_speed=step/.1,frames=len(frames))
    bundle.metrics[f'replay_{number}']=audits


def grid_world_cells(msg, anchor: np.ndarray) -> tuple[dict, float]:
    """움직이는 rolling grid를 고정 odom 격자에 맞춰 공통 관측 셀만 비교합니다."""
    assert msg.header.frame_id == 'odom'
    assert abs(msg.info.origin.orientation.z)<1e-6
    r=msg.info.resolution
    origin=np.array([msg.info.origin.position.x,msg.info.origin.position.y])
    offset=(origin+r/2-anchor)/r
    assert np.max(abs(offset-np.round(offset)))<1e-3, 'costmap 격자 원점 불일치'
    offset=np.round(offset).astype(int)
    cells=msg.data.reshape(msg.info.height,msg.info.width)
    yy,xx=np.nonzero(cells>=0)
    return {(int(x+offset[0]),int(y+offset[1])):int(cells[y,x]) for x,y in zip(xx,yy)},r


def render_forward(bundle: Bundle, rec: Recording):
    """전진 관측을 선택하고 셀 상태 변화와 센서 방향을 분리해 표시합니다."""
    start,end=time_value('11:26:05'),time_value('11:26:19')
    times=np.arange(start,end+1e-6,.2)
    initial=rec.data['/local_costmap/costmap'].at(start,1.5)
    anchor=np.array([initial.info.origin.position.x,initial.info.origin.position.y])+initial.info.resolution/2
    trace=trajectory(rec,start,end,'odom')
    frames=[];audits=[]
    for index,t in enumerate(times):
        odom=rec.data['/odom'].at(t,.5)
        cost=rec.data['/local_costmap/costmap'].at(t,1.5)
        previous=rec.data['/local_costmap/costmap'].at(t-1,1.5)
        assert odom is not None and cost is not None and previous is not None
        assert stamp(cost)<=t and stamp(previous)<=t-1
        base=pose_matrix(odom)
        camera=rec.tf('odom','d435_depth_optical_frame',t)
        camera_base=np.linalg.inv(base)@camera
        optical=camera_base[:3,2]
        assert optical[0]>0.8, '정면 카메라 구간이 아닙니다'
        vel=odom.twist.twist
        assert vel.linear.x>-.01, '후진 표본 포함'
        current,r=grid_world_cells(cost,anchor);old,_=grid_world_cells(previous,anchor)
        common=current.keys()&old.keys()
        added=[k for k in common if old[k]<100 and current[k]==100]
        removed=[k for k in common if old[k]==100 and current[k]<100]
        points=lambda keys: np.asarray(keys,dtype=float).reshape(-1,2)*r+anchor
        added_xy,removed_xy=points(added),points(removed)
        fig,axs=figure('전진 중 장애물 관측과 costmap 변화',
            f'실물 기록 재구성  |  2026-11-08 {time_label(t)} KST  |  2배속  |  전후 속도 {vel.linear.x:+.2f}m/s',
            cols=2,size=(13,7.5))
        fig.subplots_adjust(bottom=.25,left=.075,top=.78)
        left,right=axs[0]
        for ax in (left,right):
            show_grid(ax,cost,True)
            e=extent(cost.info);ax.set(xlim=e[:2],ylim=e[2:]);axes_xy(ax)
            fp=rec.data['/local_costmap/published_footprint'].at(t,1)
            if fp is not None:
                ax.add_patch(Polygon(point_array(fp.polygon.points)[:,:2],facecolor=ORANGE,edgecolor=ORANGE,alpha=.35,zorder=7))
            ax.arrow(*base[:2,3],*(base[:2,0]*.5),width=.025,color=ORANGE,zorder=9)
        past=trace[trace[:,0]<=t];left.plot(*split_trace(past),color=TEAL,lw=1.5)
        c=camera[:2,3];direction=camera[:2,2];direction=direction/np.linalg.norm(direction)
        left.plot([c[0],c[0]+direction[0]*1.5],[c[1],c[1]+direction[1]*1.5],color='#8b4d9e',ls='--',lw=2,zorder=10)
        left.scatter(*c,marker='*',color='#8b4d9e',s=90,zorder=10)
        world_vel=base[:2,:2]@np.array([vel.linear.x,vel.linear.y])
        if np.linalg.norm(world_vel)>.01:
            left.arrow(*base[:2,3],*(world_vel/np.linalg.norm(world_vel)*.9),width=.018,color=BLUE,zorder=9)
        scan=rec.data['/scan_raw'].at(t,.5);missing=[]
        if scan:
            try:
                xy=apply(scan_points(scan),rec.tf('odom',scan.header.frame_id,stamp(scan)))
                left.scatter(xy[:,0],xy[:,1],s=3,color=BLUE,alpha=.55,zorder=4)
            except ValueError:missing.append('scan TF')
        else:missing.append('scan')
        right.scatter(added_xy[:,0],added_xy[:,1],c='#c94573',s=20,marker='s',zorder=8)
        right.scatter(removed_xy[:,0],removed_xy[:,1],edgecolors='#1587b5',facecolors='none',s=27,marker='s',linewidths=1.3,zorder=8)
        left.set_title('카메라 정면 관측과 실제 이동 방향',fontsize=12)
        right.set_title(f'약 1초 전과 비교 · 100으로 {len(added)} / 100 미만으로 {len(removed)}',fontsize=11)
        fig.legend(handles=[Line2D([],[],color=ORANGE,label='차체 +X'),Line2D([],[],color='#8b4d9e',ls='--',label='카메라 광축'),
            Line2D([],[],color=BLUE,label='odom 이동 방향'),Line2D([],[],marker='.',ls='',color=BLUE,label='LiDAR 점'),
            Patch(color='#c94573',label='장애물 값 100으로 변경'),Patch(edgecolor='#1587b5',facecolor='none',label='100 미만으로 변경')],
            loc='lower center',bbox_to_anchor=(.5,.115),ncol=3,fontsize=9,frameon=False)
        fig.text(.06,.065,'두 costmap에 모두 존재하는 동일 odom 셀만 비교합니다. 새로 들어온 화면 가장자리와 미관측 셀은 제외합니다.',fontsize=9,color='#58736b')
        footer(fig,'Nav2 명령 없는 실물 전진 구간 · LiDAR+depth 통합 costmap · depth 단독 효과 또는 회피 성공 증명 아님'+(' · 누락: '+','.join(missing) if missing else ''))
        fig.canvas.draw();frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3]).copy());plt.close(fig)
        audits.append(dict(time=float(t),costmap_header=stamp(cost),previous_costmap_header=stamp(previous),
            forward_velocity=float(vel.linear.x),camera_forward_axis_base=optical.tolist(),
            added_cells=len(added),removed_cells=len(removed),common_cells=len(common),missing=missing))
        if index%25==0:print('전진 GIF',index,'/',len(times),flush=True)
    filename='01-driving-replay.gif'
    frames[0].save(bundle.out/filename,save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=False)
    frames[len(frames)//2].save(bundle.out/'01-driving-replay-poster.png')
    bundle.metrics['replay_1']=audits
    bundle.add(1,'전진 중 장애물 관측과 costmap 변화',filename,'실물 전진 기록 재구성',[str(p) for p in rec.sources],
        'Nav2 명령 없는 전진 구간입니다. 차체 전방, 카메라 광축, 이동 방향과 약 1초 전 대비 공통 odom 격자의 값 변화를 표시합니다. 통합 costmap의 변화이며 depth 단독 기여는 분리할 수 없습니다.',
        start=time_label(start),end=time_label(end),sample_step_s=.2,playback_speed=2,frames=len(frames))


def render_motion(bundle: Bundle, rec: Recording):
    start,end=time_value('11:25:30'),time_value('11:43:20')
    trace=trajectory(rec,start,end)
    stops=[];segment=[]
    for t,m in zip(rec.data['/odom'].times,rec.data['/odom'].messages):
        if not start<=t<=end:continue
        v=m.twist.twist
        stationary=np.hypot(v.linear.x,v.linear.y)<.01 and abs(v.angular.z)<.02
        if segment and (not stationary or t-segment[-1]>1):
            if segment[-1]-segment[0]>=2:stops.append((segment[0],segment[-1]))
            segment=[]
        if stationary:segment.append(t)
    if segment and segment[-1]-segment[0]>=2:stops.append((segment[0],segment[-1]))
    bundle.metrics['odom_stop_candidates']=[{'start':a,'end':b,'duration_s':b-a} for a,b in stops]
    fig,axs=figure('주행 궤적 요약','실물 기록  |  2026-11-08 11:25:30–11:43:20 KST  |  map 기준 위치 추정',cols=2,size=(13,7))
    for ax,frame in zip(axs[0],['map','odom']):
        tr=trajectory(rec,start,end,frame); x,y=split_trace(tr)
        if frame=='map':show_grid(ax,rec.data['/map'].at(end))
        ax.plot(x,y,color=TEAL,lw=2)
        valid=tr[np.isfinite(tr[:,1:]).all(axis=1)]
        ax.scatter(valid[0,1],valid[0,2],s=90,color=TEAL,label='구간 시작',zorder=5)
        ax.scatter(valid[-1,1],valid[-1,2],s=90,color=ORANGE,marker='s',label='구간 끝',zorder=5)
        nav=tr[(tr[:,0]>=time_value('11:38:28'))&(tr[:,0]<=time_value('11:39:21'))]
        ax.plot(*split_trace(nav),color=BLUE,lw=3,label='Nav2 명령 기록 구간')
        first=True
        for a,b in stops:
            selected=tr[(tr[:,0]>=a)&(tr[:,0]<=b)]
            selected=selected[np.isfinite(selected[:,1:]).all(axis=1)]
            if len(selected):
                xy=np.median(selected[:,1:],axis=0)
                ax.scatter(*xy,s=80,facecolors='none',edgecolors='#796497',linewidths=1.5,
                           label='정지 후보 ≥2s' if first else None,zorder=6)
                first=False
        low=valid[:,1:].min(axis=0)-1; high=valid[:,1:].max(axis=0)+1
        ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);axes_xy(ax,frame)
        ax.set_title('지도 기준 궤적' if frame=='map' else '연속 odometry 기준 궤적');ax.legend(fontsize=9)
    footer(fig,'정지 후보: odom 속도 <0.01m/s, 각속도 <0.02rad/s가 2초 이상 · 외부 측량 정답 아님 · 끝점은 목표 도착을 뜻하지 않음')
    bundle.save(fig,'02-driving-trajectory.png')
    bundle.add(2,'주행 궤적 요약','02-driving-trajectory.png','실물 기록 분석',[str(p) for p in rec.sources],
               'map과 odom 궤적을 분리합니다. 위치 보정 점프를 이동으로 연결하지 않습니다. 목표 성공 판정은 포함하지 않습니다.')
    fig,axs=figure('주행 명령과 관측 상태','실물 기록  |  2026-11-08 KST  |  메시지 수신 시각 기준',rows=4,size=(13,9))
    fig.subplots_adjust(left=.14)
    reference=time_value('11:25:00')
    for name,color,label in [('/cmd_vel',TEAL,'출력 cmd_vel'),('/nav2/cmd_vel',BLUE,'Nav2 cmd_vel'),('/odom',ORANGE,'odom 관측')]:
        series=rec.data[name];ts=[];vs=[];ws=[]
        for t,m in zip(series.times,series.messages):
            if not start<=t<=end:continue
            twist=m.twist.twist if name=='/odom' else m
            if ts and t-reference-ts[-1]>1:ts.append(np.nan);vs.append(np.nan);ws.append(np.nan)
            ts.append(t-reference);vs.append(np.hypot(twist.linear.x,twist.linear.y));ws.append(twist.angular.z)
        axs[0,0].plot(ts,vs,'.-',markersize=2,lw=.8,color=color,label=label)
        axs[1,0].plot(ts,ws,'.-',markersize=2,lw=.8,color=color,label=label)
    axs[0,0].set_ylabel('평면 속도 (m/s)');axs[1,0].set_ylabel('각속도 (rad/s)');axs[0,0].legend(ncol=3,fontsize=9)
    states={'unknown':0,'unavailable':1,'degraded':2,'available':3}
    status=rec.data['/localization/status'];xs=[];ys=[]
    for t,m in zip(status.times,status.messages):
        if start<=t<=end:
            xs.append(t-reference);ys.append(states.get(json.loads(m.data).get('state'),0))
    axs[2,0].scatter(xs,ys,s=5,color=TEAL)
    axs[2,0].set_yticks(range(4),states.keys());axs[2,0].set_ylabel('위치 정합 상태')
    logs=rec.data['/rosout'];warnings=[]
    for t,m in zip(logs.times,logs.messages):
        if start<=t<=end and 'depth' in m.msg.lower() and ('not been updated' in m.msg or m.level>=30):warnings.append(t-reference)
    axs[3,0].hist(warnings,bins=np.arange(start-reference,end-reference+5,5),color=ORANGE)
    axs[3,0].set_ylabel('depth 경고 수 / 5s')
    ticks=np.arange(0,1200,120)
    for ax in axs[:,0]:
        ax.axvspan(time_value('11:38:28')-reference,time_value('11:39:21')-reference,color=BLUE,alpha=.08)
        ax.set_xlim(start-reference,end-reference);ax.set_xticks(ticks,[time_label(reference+t) for t in ticks]);ax.grid(alpha=.15)
    footer(fig,'1초 초과 메시지 공백은 선을 끊음 · 경고 시각과 감속의 동시성만 표시하며 인과관계나 실제 제동 성능을 판정하지 않음')
    bundle.save(fig,'11-driving-timeline.png')
    bundle.add(11,'주행 상태 타임라인','11-driving-timeline.png','실물 기록 분석',[str(p) for p in rec.sources],
               '명령과 odom 관측을 구분합니다. 경고와 정지의 인과관계는 별도 검증 대상입니다.')


def render_chair(bundle: Bundle, run: Path):
    fig,axs=figure('의자 배치와 제거 전후','실물 정지 관측  |  2026-11-08  |  같은 odom 영역 비교',rows=2,cols=2,size=(12,10))
    compare=read_json(run/'chair-comparison.json')
    for col,stem in enumerate(['chair-present','chair-removed']):
        data=np.load(run/f'{stem}.npz');meta=read_json(run/f'{stem}.json')
        points=apply(data['marking'],dict_matrix(meta['costmap_to_base']))
        sc=axs[0,col].scatter(points[:,0],points[:,1],c=points[:,2],s=2,cmap='viridis',vmin=.192,vmax=1.4)
        info=meta['costmap']['info'];o=info['origin']['position'];r=info['resolution'];cells=data['costmap']
        ex=[o['x'],o['x']+cells.shape[1]*r,o['y'],o['y']+cells.shape[0]*r]
        axs[1,col].imshow(grid_rgba(cells,True),origin='lower',extent=ex,interpolation='nearest')
        state='의자 배치' if col==0 else '의자 제거'
        axs[0,col].set_title(state+' · depth 장애물 점군')
        axs[1,col].set_title('통합 costmap · 중심 구역 '+str(compare[stem]['chair_core_occupied_cells'])+'셀')
        for ax in axs[:,col]:
            ax.add_patch(Rectangle((.8,-.4),.4,.55,fill=False,color=ORANGE,lw=2))
            ax.set(xlim=(-.4,2),ylim=(-1,1));axes_xy(ax)
    fig.colorbar(sc,ax=list(axs.flat),label='base_link Z (m)',shrink=.65)
    fig.legend(handles=[Patch(color=FG,label='장애물 셀 100'),Patch(color='#e8d9bf',label='내접 비용 99'),
        Patch(color='#d2e3d9',label='주변 비용')],loc='lower center',bbox_to_anchor=(.5,.065),ncol=3,frameon=False)
    footer(fig,'표시한 중심 구역 66 → 0셀 · 영역은 수동 지정이며 의자 의미 분할 아님 · 통합 costmap에는 LiDAR 관측도 포함')
    bundle.save(fig,'04-chair-before-after.png')
    bundle.add(4,'의자 배치·제거 전후','04-chair-before-after.png','실물 정지 관측',
               [str(run/f'{s}.{ext}') for s in ['chair-present','chair-removed'] for ext in ['npz','json']],
               '다른 시점의 주행 GIF와 동기화된 장면이 아닙니다. 의자 중심 영역의 통합 costmap 66→0셀 관측입니다.')


def render_sensors(bundle: Bundle, run: Path):
    d=np.load(run/'latency-and-ghosts.npz');meta=read_json(run/'latency-and-ghosts.json')
    # 원본 진단 도구가 각 메시지의 TF로 odom에 변환한 누적 관측입니다.
    inv=np.linalg.inv(dict_matrix(meta['odom_to_base']))
    depth=apply(d['marking'][::25],inv);scan=apply(d['scan'][::6],inv)
    fig,axs=figure('LiDAR와 depth가 관측한 영역','실물 정지 진단  |  최근 최대 60개 메시지의 odom 점군을 기준 pose로 변환',cols=2,size=(13,7))
    sc=axs[0,0].scatter(depth[:,0],depth[:,1],c=depth[:,2],cmap='viridis',s=3,vmin=.192,vmax=1.4,alpha=.6)
    axs[0,0].scatter(scan[:,0],scan[:,1],s=4,color=ORANGE,label='LiDAR',alpha=.65)
    axes_xy(axs[0,0],'기준 base');axs[0,0].set(xlim=(-2,3),ylim=(-2.5,2.5),title='평면 투영')
    axs[0,0].legend()
    axs[0,1].scatter(depth[:,0],depth[:,2],c=depth[:,2],cmap='viridis',s=3,vmin=.192,vmax=1.4,alpha=.6)
    axs[0,1].set(xlim=(-.5,3),ylim=(-.15,1.5),xlabel='기준 base X (m)',ylabel='기준 base Z (m)',title='Depth 높이 분포')
    axs[0,1].axhline(.192,c=ORANGE,ls='--',label='설정상 marking 하한 0.192m');axs[0,1].legend(fontsize=9)
    fig.colorbar(sc,ax=list(axs[0]),label='기준 base Z (m)',shrink=.65)
    footer(fig,'주행 중 동기 프레임 아님 · 관측 범위 비교이며 센서별 costmap 기여도 아님 · LiDAR 높이는 기존 평면 TF 관례이므로 측면도에서 제외')
    bundle.save(fig,'05-lidar-depth-observation.png')
    bundle.add(5,'LiDAR와 depth 관측 비교','05-lidar-depth-observation.png','실물 정지 관측 재구성',
               [str(run/'latency-and-ghosts.npz'),str(run/'latency-and-ghosts.json')],
               '최근 여러 메시지의 누적 점군이며 단일 동기 프레임이 아닙니다. LiDAR의 물리 장착 높이는 이 자료로 입증하지 않습니다.')


def body_boxes(path: Path) -> list[BodyBox]:
    return [BodyBox(x['name'],np.array(x['center']),np.eye(3),np.array(x['half_size'])) for x in read_json(path)['boxes']]


def render_filter(bundle: Bundle, run: Path):
    data=np.load(run/'floor-obstacles/obstacle-preview.npz')
    config_path=REPO/'ros2_ws/src/cleany_navigation/config/d435_obstacles_lidar_height.json'
    box_path=REPO/'ros2_ws/src/cleany_navigation/config/d435_fixed_body_boxes.json'
    cfg=ObstacleFilter(**read_json(config_path))
    points,counts=filter_obstacles(data['raw'],data['origin'],body_boxes(box_path),cfg)
    fig,axs=figure('장애물 필터 적용 전후','기록 점군에 현재 설정을 오프라인 적용  |  당시 preview 출력과 구분',cols=3,size=(15,7))
    for ax,p,title in zip(axs[0],[data['raw'],data['obstacles'],points],['기록 원본 점군','당시 preview 필터 출력','현재 설정으로 재처리']):
        ax.scatter(p[:,0],p[:,2],s=2,c=p[:,2],cmap='viridis',vmin=-.1,vmax=1.4,alpha=.5)
        ax.set(xlim=(-.2,3),ylim=(-.2,1.5),xlabel='base_link X (m)',ylabel='base_link Z (m)',title=f'{title}\n{len(p):,}점')
        ax.axhline(.192,c=ORANGE,ls='--',lw=1);ax.grid(alpha=.15)
    footer(fig,'현재 min_base_z=0.192m 적용 · 출력점 감소는 정확도 점수가 아님 · 원본은 정지 장면이며 현재 로봇 실행 결과가 아님')
    bundle.save(fig,'06-filter-comparison.png')
    bundle.metrics['filter_counts']=counts
    bundle.add(6,'필터 적용 전후','06-filter-comparison.png','기록 입력의 오프라인 재처리',
               [str(run/'floor-obstacles/obstacle-preview.npz'),str(config_path),str(box_path)],
               '당시 preview 결과와 현재 min_base_z 설정을 적용한 결과를 별도 패널로 표시합니다. 재실행한 실물 관측이 아닙니다.')


def render_rays(bundle: Bundle, run: Path, rec: Recording):
    data=np.load(run/'chair-present.npz');meta=read_json(run/'chair-present.json')
    samples=meta['clouds']
    print('점군 메타데이터:',str(samples)[:180],flush=True)
    # 캡처 프로그램이 저장한 마지막 clearing 표본의 좌표계를 같은 촬영 시각 TF로 변환합니다.
    clearing_meta=samples['clearing'][-1]
    t=clearing_meta['stamp'];frame=clearing_meta['frame']
    tr=rec.tf('base_link',frame,t)
    clearing=apply(data['clearing'],tr);marking=data['marking'];origin=tr[:3,3]
    fig,axs=figure('장애물 표시와 빈 공간 관측','실물 정지 캡처  |  위: 평면 위치와 관측 방향  /  아래: 높이와 바닥 제외',rows=2,cols=2,size=(14,12))
    fig.subplots_adjust(top=.84,bottom=.13,hspace=.48)
    rays=clearing[np.linspace(0,len(clearing)-1,100,dtype=int)]
    for row,(u,v,view) in enumerate([(1,0,'평면'),(0,2,'측면')]):
        clear_ax,mark_ax=axs[row]
        for p in rays:
            clear_ax.plot([origin[u],p[u]],[origin[v],p[v]],c=TEAL,alpha=.13,lw=.8)
        clear_ax.scatter(clearing[::4,u],clearing[::4,v],s=2,c=TEAL,alpha=.3)
        mark_ax.scatter(marking[:,u],marking[:,v],s=3,c=ORANGE,alpha=.45)
        clear_ax.set_title(f'{view} | Clearing용 관측점과 광선')
        mark_ax.set_title(f'{view} | Marking용 장애물 후보')
        for ax in axs[row]:
            ax.scatter(origin[u],origin[v],marker='*',s=150,c=BLUE,zorder=5,label='카메라 원점')
            ax.scatter(0,0,marker='+',s=65,c=FG,zorder=5,label='base_link 원점')
            ax.set_aspect('equal');ax.grid(alpha=.15)
            if row==0:
                # 전방은 위쪽, 로봇의 왼쪽인 +Y는 화면 왼쪽으로 표시합니다.
                ax.set(xlim=(1.6,-1.6),ylim=(-.3,3),xlabel='base_link Y (m)  ← 왼쪽(+)',ylabel='base_link X (m)  전방(+) →')
            else:
                ax.set(xlim=(-.2,3),ylim=(-.2,1.6),xlabel='base_link X (m)',ylabel='base_link Z (m)')
            ax.legend(fontsize=9,loc='upper right')
    axs[1,1].axhline(.192,c=TEAL,ls='--',label='설정상 marking 하한 0.192m')
    axs[1,1].legend(fontsize=9,loc='upper right')
    footer(fig,'같은 점군을 두 방향으로 투영합니다. 두 입력은 비동기 최신 캡처입니다.\n광선은 설명용 부분 표본이며 실제 voxel 삭제 이력이 아닙니다. 평면에서 선이 겹쳐도 전체 높이의 빈 공간을 뜻하지 않습니다.')
    bundle.save(fig,'07-marking-clearing.png')
    bundle.metrics['clearing_tf']={'stamp':t,'frame':frame,'matrix':tr.tolist()}
    bundle.add(7,'Marking과 clearing 평면·측면 비교','07-marking-clearing.png','실측 점군과 광선 설명',
               [str(run/'chair-present.npz'),str(run/'chair-present.json'),str(rec.sources[0])],
               '위는 X–Y 평면, 아래는 X–Z 측면입니다. 같은 관측 데이터를 투영하며 두 입력은 비동기 캡처입니다. 광선은 설명용이며 실제 voxel 삭제 이력이나 전체 높이의 빈 공간 판정이 아닙니다.')


def render_walls(bundle: Bundle, run: Path):
    from compare_depth_lidar_wall import camera_pose, rotation
    fig,axs=figure('카메라 방향별 벽 정합','실물 정지 관측  |  각 방향의 동일 시각에 가까운 depth와 LiDAR 비교',rows=2,cols=2,size=(12,10))
    mount=read_json(run/'cad-mount-provenance.json');sources=[];metrics={}
    for ax,name,label in zip(axs.flat,['start','pan_minus','pan_plus','tilt_plus'],['시작 방향','Pan -','Pan +','Tilt +']):
        folder=run/'pan-tilt-wall'/name;capture=read_json(folder/'capture.json');report=read_json(folder/'wall-comparison.json')
        frame=report['frames'][len(report['frames'])//2];i=frame['index'];t=capture['depth_times'][i]['stamp']
        depths=np.load(folder/'depth.npz')['depth_mm'];z=depths[i].astype(float)*.001
        h,w=z.shape;vv,uu=np.mgrid[int(h*.2):int(h*.8):4,int(w*.2):int(w*.8):4]
        zz=z[vv,uu].ravel();k=np.asarray(capture['camera_info']['k']).reshape(3,3)
        p=np.column_stack(((uu.ravel()-k[0,2])*zz/k[0,0],(vv.ravel()-k[1,2])*zz/k[1,1],zz));p=p[(zz>.25)&(zz<3)]
        r,origin=camera_pose(capture,t,mount);p=p@r.T+origin
        scan=min(capture['scans'],key=lambda s:abs(s['stamp']+s['scan_time']/2-t))
        ranges=np.asarray(scan['ranges'],dtype=float);angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        keep=np.isfinite(ranges)&(ranges>.1)&(ranges<3)
        lp=np.column_stack((ranges[keep]*np.cos(angles[keep]),ranges[keep]*np.sin(angles[keep]),np.zeros(keep.sum())))
        tf=capture['tf_static'][scan['frame']];lp=lp@rotation(tf['xyzw']).T+tf['xyz']
        ax.scatter(p[:,1],p[:,0],s=2,color=TEAL,alpha=.3,label='Depth 중앙 ROI')
        ax.scatter(lp[:,1],lp[:,0],s=9,color=ORANGE,label='LiDAR')
        delta=report['delta_wall_x_m']['median']*1000;angle=report['delta_yaw_deg']['median']
        ax.set(xlim=(-.6,.6),ylim=(.4,1.4),xlabel='base_link Y (m)',ylabel='base_link X (m)',
               title=f'{label} | 벽 X 차이 {delta:+.1f}mm / 각도 차이 {angle:+.1f}°')
        ax.legend(fontsize=9);ax.grid(alpha=.15)
        metrics[name]={'delta_wall_x_mm':delta,'delta_yaw_deg':angle,'frame_index':i,'sensor_skew_s':frame['skew_s']}
        sources.extend([str(folder/f) for f in ['capture.json','depth.npz','wall-comparison.json']])
    footer(fig,'방향별 중앙 표본 표시 · 수치는 각 방향 기록 전체의 중앙값 · 관측 높이가 다름 · 벽 평탄성 정답 및 완전한 외부 보정 검증 아님')
    bundle.save(fig,'08-wall-alignment.png');bundle.metrics['wall_alignment']=metrics
    bundle.add(8,'방향별 벽 정합 비교','08-wall-alignment.png','실물 정지 관측',sources,
               '같은 벽의 서로 다른 높이를 비교한 기록입니다. 임계값 합격 판정이나 전체 6DoF 보정 결과가 아닙니다.')


def render_body(bundle: Bundle):
    path=REPO/'ros2_ws/src/cleany_navigation/config/d435_fixed_body_boxes.json';boxes=body_boxes(path)
    fig,axs=figure('고정 차체 자기 필터 범위','현재 설정의 AABB 투영  |  실물 관측 및 전체 로봇 CAD와 구분',cols=2,size=(12,7))
    for ax,(u,v),title in zip(axs[0],[(0,2),(1,2)],['측면 X–Z','정면 Y–Z']):
        for box in boxes:
            low=box.center-box.half_size
            ax.add_patch(Rectangle((low[u],low[v]),2*box.half_size[u],2*box.half_size[v],facecolor=TEAL,edgecolor=TEAL,alpha=.18))
        ax.autoscale();ax.margins(.2);ax.set_aspect('equal');ax.set(xlabel=('X' if u==0 else 'Y')+' (m)',ylabel='Z (m)',title=title);ax.grid(alpha=.15)
        ax.axhline(.192,c=ORANGE,ls='--',lw=1,label='marking 높이 하한');ax.legend(fontsize=9)
    footer(fig,f'{len(boxes)}개 고정 박스 · 팔의 움직이는 자세는 포함하지 않음 · 이 그림은 차체 충돌 안전 범위나 최신 CAD 전체 형상이 아님')
    bundle.save(fig,'09-body-self-filter.png')
    bundle.add(9,'고정 차체 필터 범위','09-body-self-filter.png','현재 설정 설명',[str(path)],
               '고정 AABB 자기 관측 제거용 설정이며 움직이는 팔 또는 전체 충돌 모델을 나타내지 않습니다.')


def render_timing(bundle: Bundle, run: Path):
    path=run/'latency-and-ghosts.json';meta=read_json(path);data=meta['timing_samples']
    fig,axs=figure('Depth 입력 지연과 관측 주기','실물 정지 상태 20초 진단  |  입력 나이, 처리 구간, 발행 주기의 구분',rows=2,cols=2,size=(13,9))
    fig.subplots_adjust(left=.14)
    for name,label,color in [('image','Depth 영상',BLUE),('marking','Marking 점군',TEAL),('clearing','Clearing 점군',ORANGE)]:
        a=np.asarray(data[name]);axs[0,0].plot(a[:,0]-a[0,0],a[:,2]*1000,'.-',ms=2,lw=.7,label=label,c=color)
    axs[0,0].set(xlabel='각 토픽 첫 수신 이후 (s)',ylabel='촬영→수신 나이 (ms)',title='센서 촬영 시각 기준 나이');axs[0,0].legend(fontsize=9)
    arrivals={r[1]:r[0] for r in data['image']};matched=[(r[0],(r[0]-arrivals[r[1]])*1000) for r in data['marking'] if r[1] in arrivals]
    a=np.array(matched);axs[0,1].plot(a[:,0]-a[0,0],a[:,1],'.-',color=TEAL,ms=3,lw=.7)
    axs[0,1].axhline(np.median(a[:,1]),color=ORANGE,ls='--',label=f'중앙값 {np.median(a[:,1]):.1f}ms')
    axs[0,1].set(xlabel='매칭 첫 수신 이후 (s)',ylabel='수신 간격 (ms)',title='동일 촬영 stamp: 영상 수신→Marking 수신');axs[0,1].legend(fontsize=9)
    keys=['image','marking','clearing','update_footprint','costmap'];labels=['Depth 영상','Marking','Clearing','Footprint','Costmap 발행']
    hz=[meta['timing'][k]['received_hz'] for k in keys]
    axs[1,0].barh(labels,hz,color=TEAL);axs[1,0].invert_yaxis();axs[1,0].set(xlabel='진단 수신 빈도 (Hz)',title='토픽별 수신 빈도')
    for i,v in enumerate(hz):axs[1,0].text(v+.1,i,f'{v:.2f}',va='center',fontsize=9)
    axs[1,0].set_xlim(0,17)
    p50=[meta['timing'][k]['interval_ms_p50_p95_max'][0] for k in keys];p95=[meta['timing'][k]['interval_ms_p50_p95_max'][1] for k in keys]
    xs=np.arange(5);axs[1,1].bar(xs-.18,p50,.36,label='중앙값',color=TEAL);axs[1,1].bar(xs+.18,p95,.36,label='95백분위',color=ORANGE)
    axs[1,1].set_xticks(xs,labels,rotation=15);axs[1,1].set(ylabel='수신 간격 (ms)',title='토픽 주기 분포');axs[1,1].legend(fontsize=9)
    footer(fig,'CPU 순수 처리 시간 또는 실제 제동 지연이 아님 · Footprint는 내부 갱신의 간접 관측 · Costmap 발행 주기와 제어 주기는 다름')
    bundle.save(fig,'10-depth-timing.png')
    bundle.add(10,'입력 지연과 갱신 주기','10-depth-timing.png','실물 정지 진단',[str(path)],
               '촬영→수신 나이와 동일 stamp 영상→점군 수신 지연을 구분합니다. 제동 시간 측정이 아닙니다.')


MERMAID = '''flowchart TB
  A["D435 깊이 영상 + CameraInfo"] --> B{"촬영 시각 TF · 입력 신선도 · 유효 깊이"}
  B -->|무효| X["점군 출력 안 함"]
  B -->|유효| C["고정 차체 및 가려진 ray 제외"]
  C --> D["Clearing: nav_points / 카메라 좌표"]
  C --> E["거리 · 높이 · 고립점 · voxel 필터"]
  E --> F["Marking: nav_obstacle_points / base_link"]
  D --> G["Local VoxelLayer: 관측 광선으로 clearing"]
  F --> G
  L["LiDAR"] --> H["Local ObstacleLayer"]
  G --> I["통합 Local Costmap + Inflation"]
  H --> I
  I --> J["MPPI Controller"]
  J --> K["기존 주행 명령 제한 경로"]
  K --> M["Collision Monitor: 근접 영역 + 필수 depth 신선도 검사"]
  F --> M
  L --> M
  X -.->|필수 depth 만료 시| M
  M --> N["출력 속도 명령"]
  classDef sensor fill:#e7f0fa,stroke:#507fc0,color:#203d3a;
  classDef proc fill:#e4f2eb,stroke:#137c70,color:#203d3a;
  classDef guard fill:#fff0df,stroke:#d16b32,color:#203d3a;
  class A,L sensor;
  class C,D,E,F,G,H,I,J,K proc;
  class B,X,M,N guard;
'''


def render_structure(bundle: Bundle):
    (bundle.out/'12-depth-pipeline.mmd').write_text(MERMAID)
    (bundle.out/'12-depth-pipeline.md').write_text('# Depth 장애물 처리 구조\n\n현재 depth overlay 기준의 요약입니다. 주행 명령 제한 경로의 전체 노드 목록은 생략합니다.\n\n```mermaid\n'+MERMAID+'```\n\nGlobal Costmap에 depth를 직접 추가하는 구성은 아닙니다. 속도 명령 0과 실물 정지는 별도 확인 대상입니다.\n')
    bundle.add(12,'Depth 처리 구조','12-depth-pipeline.png','현재 구현 설명',
               [str(REPO/'ros2_ws/src/cleany_navigation/config/depth_local.yaml'),str(REPO/'ros2_ws/src/cleany_navigation/docs/interfaces.md')],
               '현재 depth overlay 중심의 요약이며 중간 주행 제한 노드를 모두 나열하지 않습니다. Mermaid 원문을 함께 제공합니다.')


def package(bundle: Bundle, rec: Recording):
    import shutil
    import subprocess
    # Mermaid는 저장소의 고정 vendor와 별도 렌더러로 SVG/PNG를 생성합니다.
    shutil.copyfile(REPO/'docs/guide/vendor/mermaid.tiny.js',bundle.out/'mermaid.tiny.js')
    (bundle.out/'diagram.html').write_text('<!doctype html><meta charset="utf-8"><style>body{background:#f3f7f5;color:#203d3a;font-family:NanumGothic,sans-serif;padding:28px}h1{font-size:28px}#diagram{width:1300px}svg{width:100%;max-width:none!important}</style><h1>12 · Depth 장애물 처리 구조</h1><p>현재 구현 설명 · 실물 실행 증거와 구분 · 편집 가능한 Mermaid 원문 제공</p><div id="diagram"></div><script src="mermaid.tiny.js"></script><script>mermaid.initialize({startOnLoad:false,securityLevel:"strict",theme:"base",fontFamily:"NanumGothic",flowchart:{htmlLabels:false},themeVariables:{primaryColor:"#e4f2eb",lineColor:"#137c70",fontSize:"18px"}});mermaid.render("depthflow",'+json.dumps(MERMAID)+').then(r=>{document.getElementById("diagram").innerHTML=r.svg;document.body.dataset.ready="true"})</script>')
    cards=[]
    for item in sorted(bundle.items,key=lambda x:x['number']):
        name=item['file'];n=item['number']
        links=f'<a href="{name}" download>원본 다운로드</a>'
        if n==12:links+=' · <a href="12-depth-pipeline.mmd" download>Mermaid 원문</a> · <a href="12-depth-pipeline.md">Markdown</a>'
        cards.append(f'<section id="v{n}"><span class="tag">{html.escape(item["scope"])}</span><h2>{n:02d} · {html.escape(item["title"])}</h2><img src="{name}" alt="{html.escape(item["title"])}"><p>{html.escape(item["note"])}</p><p>{links}</p></section>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SCRUM-556 · 시각자료 12종</title><style>body{margin:0;background:#edf3ef;color:#203d3a;font:16px/1.7 system-ui,sans-serif}main{max-width:1250px;margin:auto;padding:40px 24px}h1{font-size:38px}section{background:white;padding:28px;border:1px solid #d8e5dc;border-radius:16px;margin:24px 0}img{width:100%;height:auto;border-radius:8px}a{color:#137c70}.tag{font-size:13px;color:#137c70;font-weight:700}nav{display:flex;flex-wrap:wrap;gap:16px}header p{max-width:900px}</style><main><header><p class="tag">CLEANY / SCRUM-556</p><h1>Depth 장애물 연동 · 시각자료 12종</h1><p>2026-11-08 실물 기록과 현재 설정을 구분한 검토용 자료입니다. 주행 화면은 기록된 ROS 데이터를 다시 그린 것이며 외부 촬영 영상이 아닙니다. 점군 비교는 별도 정지 실험입니다.</p><p><a href="contact-sheet.png">전체 미리보기</a> · <a href="manifest.json">출처와 생성 정보</a> · <a href="metrics.json">분석 수치</a></p><nav>'''+''.join(f'<a href="#v{x["number"]}">{x["number"]:02d} {html.escape(x["title"])}</a>' for x in sorted(bundle.items,key=lambda x:x['number']))+'</nav></header>'+''.join(cards)+'</main></html>'
    (bundle.out/'index.html').write_text(page)
    sources=sorted({p for i in bundle.items for p in i['sources']} | {
        str(Path(__file__)),str(REPO/'tools/validation/render_depth_jira_diagram.mjs'),
        str(REPO/'tools/validation/compare_depth_lidar_wall.py'),
        str(REPO/'ros2_ws/src/cleany_navigation/cleany_navigation/depth_obstacles.py'),
        str(REPO/'ros2_ws/src/cleany_navigation/cleany_navigation/depth_clearance.py'),
        str(REPO/'ros2_ws/src/cleany_navigation/cleany_navigation/body_geometry.py')})
    fingerprints=[]
    for p in sources:
        path=Path(p);h=hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(4*1024*1024),b''):h.update(block)
        fingerprints.append(dict(path=p,bytes=path.stat().st_size,sha256=h.hexdigest()))
    manifest=dict(created_at=datetime.now(KST).isoformat(),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        command=sys.argv,items=sorted(bundle.items,key=lambda x:x['number']),sources=fingerprints,
        bag_topic_counts=rec.counts,temporal_policy='availability=max(receipt,header); latest past message; no future dynamic TF; no motion interpolation; >1s path gaps not connected',
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (bundle.out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    (bundle.out/'metrics.json').write_text(json.dumps(bundle.metrics,ensure_ascii=False,indent=2))


def finish(out: Path):
    """이미지와 GIF를 모두 디코딩하고 검토용 묶음을 만듭니다."""
    import zipfile
    manifest=read_json(out/'manifest.json');metrics=read_json(out/'metrics.json')
    assert sorted(x['number'] for x in manifest['items'])==list(range(1,13))
    validation=[];thumbs=[]
    font=ImageFont.truetype('/usr/share/fonts/naver-nanum-gothic-fonts/NanumGothic.ttf',19)
    for item in manifest['items']:
        path=out/item['file'];duration=0;hashes=set()
        with Image.open(path) as im:
            count=getattr(im,'n_frames',1);size=im.size
            for index in range(count):
                im.seek(index);im.load();duration+=im.info.get('duration',0)
                hashes.add(hashlib.sha256(im.convert('RGB').tobytes()).hexdigest())
            if path.suffix=='.gif':
                # Pillow는 연속된 동일 화면을 합치되 총 표시 시간을 보존합니다.
                assert count<=item['frames'] and duration==item['frames']*100 and len(hashes)>1
            im.seek(count//2);thumb=im.convert('RGB');thumb.thumbnail((590,350))
            card=Image.new('RGB',(620,410),'white');card.paste(thumb,((620-thumb.width)//2,42+(350-thumb.height)//2))
            ImageDraw.Draw(card).text((16,10),f'{item["number"]:02d}  {item["title"]}',font=font,fill=FG)
            thumbs.append(card)
        validation.append(dict(file=path.name,size=size,frames=count,duration_ms=duration,bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for key in ['replay_1','replay_3']:
        assert all(x['costmap_header'] is None or x['costmap_header']<=x['time'] for x in metrics[key])
    sheet=Image.new('RGB',(1880,1670),BG)
    for index,thumb in enumerate(thumbs):sheet.paste(thumb,(10+index%3*620,10+index//3*410))
    sheet.save(out/'contact-sheet.png')
    (out/'validation.json').write_text(json.dumps(dict(assets=validation,no_future_costmap=True),ensure_ascii=False,indent=2))
    # 생성 도구는 레포에 두고 배포 묶음에는 결과와 출처만 포함합니다.
    with zipfile.ZipFile(out/'scrum-556-visuals.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(out.iterdir()):
            if path.is_file() and path.suffix!='.zip':archive.write(path,path.name)
    print(json.dumps(validation,ensure_ascii=False,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--only',default='all',choices=['all','static','motion','forward','rays'])
    parser.add_argument('--finalize',action='store_true')
    args=parser.parse_args();root=args.data_root
    if args.finalize:
        finish(args.output)
        return
    if root is None:parser.error('--data-root가 필요합니다')
    bundle=Bundle(args.output)
    bag=root/'runs/navigation/traction/2026-11-08_080053_physical-validation/runtime/recordings/bags/37707477-1292-47e4-b6f7-2537af79e2cf'
    run=root/'runs/navigation/integration/2026-11-08_110835_depth-local-costmap'
    low=root/'runs/navigation/integration/2026-11-08_100401_d435-low-obstacle'
    print('기록 읽기:',bag,flush=True);rec=Recording(bag)
    if args.only=='rays':
        render_rays(bundle,run,rec)
    if args.only=='forward':
        render_forward(bundle,rec)
    if args.only in ('all','static'):
        render_motion(bundle,rec);render_chair(bundle,run);render_sensors(bundle,run)
        render_filter(bundle,low);render_rays(bundle,run,rec);render_walls(bundle,low)
        render_body(bundle);render_timing(bundle,run);render_structure(bundle)
    if args.only in ('all','motion'):
        render_forward(bundle,rec)
        # 동일 Nav2 구간 안에서 costmap 장애물이 가장 가까운 움직이는 표본을 찾습니다.
        candidates=[]
        for t,m in zip(rec.data['/local_costmap/costmap'].times,rec.data['/local_costmap/costmap'].messages):
            if not time_value('11:38:33')<=t<=time_value('11:38:54'):continue
            odom=rec.data['/odom'].at(t,.5)
            if odom is None or np.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y)<.015:continue
            cells=m.data.reshape(m.info.height,m.info.width);yy,xx=np.nonzero(cells==100)
            xy=np.column_stack((m.info.origin.position.x+(xx+.5)*m.info.resolution,m.info.origin.position.y+(yy+.5)*m.info.resolution))
            if len(xy):candidates.append((float(np.min(np.linalg.norm(xy-pose_matrix(odom)[:2,3],axis=1))),t))
        if not candidates:raise ValueError('확대할 움직임과 costmap 동시 표본이 없습니다')
        distance,t=min(candidates)
        begin=max(time_value('11:38:28'),t-6);segment_end=min(time_value('11:39:21'),t+9)
        bundle.metrics['closeup_selection']={'time':t,'nearest_occupied_cell_center_distance_m':distance,'meaning':'zoom selection only, not body clearance or collision verdict'}
        render_replay(bundle,rec,begin,segment_end,3,True)
    package(bundle,rec)


if __name__=='__main__':
    main()
