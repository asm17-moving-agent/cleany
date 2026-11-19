#!/usr/bin/env python3
"""SCRUM-557의 실물 상태 기록과 오프라인 복구 평가를 구분해 시각화합니다.

원본 SQLite와 실험 NPZ/JSON은 읽기만 합니다. 실제 ROS 명령을 발행하지
않습니다. 후보 비교의 기준은 기록된 AMCL이며 물리 정답으로 취급하지 않습니다.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import html
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

import render_depth_jira_visuals as common
from cleany_navigation.localization_alignment import AlignmentMap, AlignmentSettings, transform
from report_localization_policy_experiment import summarize

REPO=common.REPO
TEAL,ORANGE,BLUE,FG,BG=common.TEAL,common.ORANGE,common.BLUE,common.FG,common.BG
RED='#b84e58'
POLICIES=['floor_0.30','floor_0.40','floor_0.50','floor_0.55','floor_0.65','structured']
LABELS=['30%','40%','50%','55%','65%','구조 분리 실험안']
STATE_COLORS={'unknown':'#8c939a','unavailable':RED,'degraded':ORANGE,'available':TEAL}
PHASE_LABELS={'idle':'대기','searching':'후보 탐색','verifying_amcl':'AMCL 재검증',
              'recovered':'복구 완료 보고','failed':'복구 실패 보고'}


class Bundle:
    def __init__(self,out: Path,experiment: Path):
        self.out=out;out.mkdir(parents=True,exist_ok=True)
        self.experiment=experiment
        self.items=[];self.metrics={};self.sources=set()
        self.cases=common.read_json(experiment/'results.json')
        self.inputs=common.read_json(experiment/'inputs.json')
        self.protocol=common.read_json(experiment/'protocol.json')
        self.settings=AlignmentSettings(**self.protocol['normal'])
        self.sources.update(experiment/name for name in ['results.json','inputs.json','protocol.json','summary.json'])

    def add(self,number,title,file,scope,note,**more):
        self.items.append(dict(number=number,title=title,file=file,scope=scope,note=note,**more))

    def save(self,fig,file):
        common.footer(fig,getattr(fig,'boundary','기록 AMCL 기준의 비교이며 실측 정답 또는 실물 복구 성공률이 아닙니다.'))
        fig.savefig(self.out/file,dpi=140);plt.close(fig);print('생성:',file,flush=True)

    def window(self,case):
        index=next(i for i,x in enumerate(self.inputs) if x['bag']==case['bag'])
        path=self.experiment/f'window-{index}-{case["window"]}.npz';self.sources.add(path)
        with np.load(path) as data:
            values={k:data[k].copy() for k in data.files}
        model=AlignmentMap(values['cells'],float(values['resolution']),values['origin'])
        assert np.allclose(values['poses'][0],case['reference'])
        return model,values


def map_panel(ax,model,points,pose,title,bounds,reference=None):
    assert abs(model.origin[2])<1e-6
    h,w=model.cells.shape;r=model.resolution;x,y=model.origin[:2]
    ax.imshow(common.grid_rgba(model.cells),origin='lower',extent=[x,x+w*r,y,y+h*r],interpolation='nearest')
    score=model.score(points,np.array(pose),AlignmentSettings(min_inlier_ratio=.30))
    residuals,_=model.residuals(points,np.array(pose),.6)
    xy=transform(points,np.array(pose));colors=np.where(residuals<=.15,TEAL,ORANGE)
    ax.scatter(xy[:,0],xy[:,1],s=10,c=colors,zorder=4)
    ax.scatter(pose[0],pose[1],c=BLUE,s=65,marker='^',zorder=6)
    ax.arrow(pose[0],pose[1],.4*np.cos(pose[2]),.4*np.sin(pose[2]),width=.025,color=BLUE,zorder=5)
    if reference is not None:
        ax.scatter(reference[0],reference[1],facecolor='none',edgecolor=RED,s=120,marker='o',zorder=6)
    ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='map X (m)',ylabel='map Y (m)',aspect='equal',title=title)
    return score


def bounds_for(points,poses):
    xy=np.concatenate([transform(points,np.array(p)) for p in poses])
    low=np.quantile(xy,.02,axis=0)-.35;high=np.quantile(xy,.98,axis=0)+.35
    return [low[0],high[0],low[1],high[1]]


def point_legend(fig,y=.07):
    fig.legend(handles=[Line2D([],[],marker='.',color=TEAL,ls='',label='점유 셀까지 ≤15cm'),
        Line2D([],[],marker='.',color=ORANGE,ls='',label='>15cm 또는 미확인'),
        Line2D([],[],marker='^',color=BLUE,ls='',label='투영에 사용한 위치')],
        loc='lower center',bbox_to_anchor=(.5,y),ncol=3,frameon=False,fontsize=9)


def examples(bundle):
    case=next(x for x in bundle.cases if x['reference_stable'] and x['offset']==[.4,0.,0.] and
              x['policies']['floor_0.50']['accepted'] and x['policies']['floor_0.50']['reference_translation_error_m']<.2)
    model,data=bundle.window(case);points=data['points_0'];pose0=case['reference'];seed=case['seed']
    b=bounds_for(points,[pose0,seed])
    fig,axs=common.figure('같은 scan, 서로 다른 위치','기록된 정지 scan + 인위적 위치 오차  |  오프라인 투영 비교',cols=2,size=(13,8))
    fig.subplots_adjust(bottom=.20)
    for ax,p,title in zip(axs[0],[pose0,seed],['기록 AMCL 위치','X 방향 0.4m 오차 주입']):
        s=map_panel(ax,model,points,p,title,b)
        ax.set_title(f'{title}\n일치율 {s.inlier_ratio:.1%} · 절사 평균 잔차 {s.mean_distance*100:.1f}cm')
    point_legend(fig)
    bundle.save(fig,'01-scan-map-alignment.png')
    bundle.add(1,'지도와 LiDAR 정합 비교','01-scan-map-alignment.png','기록 입력의 오프라인 비교',
               '기록 AMCL 위치와 인위적으로 0.4m 어긋낸 위치의 비교입니다. 로봇의 실제 위치를 이동시킨 시험이 아닙니다.')
    candidate=case['policies']['floor_0.50']['pose']
    frames=[]
    for p,title in [(seed,'1 / 3 · 위치 오차 주입'),(candidate,'2 / 3 · 50% 정책의 수락 후보'),(pose0,'3 / 3 · 기록 AMCL 기준 위치')]:
        fig,axs=common.figure('복구 후보 위치 비교',f'오프라인 단계 비교  |  {title}  |  실제 복구 동작 영상 아님',size=(10,8))
        fig.subplots_adjust(bottom=.20)
        score=map_panel(axs[0,0],model,points,p,'',b,pose0)
        axs[0,0].set_title(f'동일 전체 scan 재평가: 일치율 {score.inlier_ratio:.1%}, 잔차 {score.mean_distance*100:.1f}cm')
        point_legend(fig)
        common.footer(fig,'붉은 원은 기록 기준 위치 · 탐색/검증 빔 분리 점수와 이 전체 scan 재평가 점수는 구분합니다.')
        fig.canvas.draw();frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3]).copy());plt.close(fig)
    frames[0].save(bundle.out/'04-recovery-candidate.gif',save_all=True,append_images=frames[1:],duration=1800,loop=0)
    frames[1].save(bundle.out/'04-recovery-candidate-poster.png')
    bundle.add(4,'복구 전 위치와 후보 비교','04-recovery-candidate.gif','오프라인 후보 비교',
               '세 위치를 단계별로 바꿔 보여줍니다. AMCL 재초기화 이후의 실제 동작이나 수렴 과정을 재현한 것이 아닙니다.',frames=3)
    bundle.metrics['example_case']=case


def policy_figures(bundle):
    summary=common.read_json(bundle.experiment/'summary.json')['all']
    for p in POLICIES:assert summarize(bundle.cases,p)==summary[p]
    fig,axs=common.figure('복구 후보 탐색 모델 비교','기존 정상 복귀(already_aligned) 분기 미포함 · 전체 실행 정책의 성능 비교가 아님',cols=2,size=(15,8))
    fig.subplots_adjust(left=.13,bottom=.22,wspace=.38)
    baseline=np.array([summary[p]['baseline_accepted'] for p in POLICIES]);y=np.arange(6)
    axs[0,0].barh(y,baseline,color=TEAL);axs[0,0].set_yticks(y,LABELS);axs[0,0].invert_yaxis();axs[0,0].set(xlim=(0,20),xlabel='수락한 기준 구간 수',title='오차 미주입 기준 18구간')
    for i,n in enumerate(baseline):axs[0,0].text(n+.2,i,f'{n}/18',va='center',fontsize=10)
    offsets=np.zeros(6)
    for key,color,label in [('within_tolerance',TEAL,'수락 · 기준 잔차 범위 이내'),('large_residual',RED,'수락 · 기준 잔차 큼'),('rejected','#c8d2ce','거부')]:
        values=np.array([summary[p][key] for p in POLICIES]);axs[0,1].barh(y,values,left=offsets,color=color,label=label)
        for i,n in enumerate(values):
            if n:axs[0,1].text(offsets[i]+n/2,i,str(n),ha='center',va='center',fontsize=10,color='white' if key!='rejected' else FG)
        offsets+=values
    assert np.all(offsets==108)
    axs[0,1].set_yticks(y,LABELS);axs[0,1].invert_yaxis();axs[0,1].set(xlim=(0,108),xlabel='오차 주입 조건 수 / 108',title='기준 안정 구간의 오차 주입')
    axs[0,1].legend(loc='upper center',bbox_to_anchor=(.5,-.14),fontsize=9)
    fig.boundary='기준 안정 18구간 × 6오차 = 108조건 · 큰 잔차: 기록 AMCL 대비 >0.2m 또는 >0.15rad · 실제 위치 정답/실물 복구 성공률 아님'
    bundle.save(fig,'05-policy-comparison.png')
    bundle.add(5,'복구 정책 비교','05-policy-comparison.png','기존 오프라인 실험 집계',
               '기존 실험 모델에 정상 복귀 분기가 누락되어 실제 실행 정책의 우열 비교에는 사용할 수 없습니다. 후보 탐색 모델의 수락과 큰 잔차 사례이며 기준 AMCL은 외부 측량 정답이 아닙니다.')
    fig,axs=common.figure('거부된 후보의 사유','기준 안정 18구간의 108개 오차 주입 조건  |  기존 실험이 기록한 거부 이유만 집계',size=(12,7))
    fig.subplots_adjust(left=.20,bottom=.22);ax=axs[0,0];left=np.zeros(6)
    reasons=[('candidate_not_well_aligned','후보 정합 기준 미달',ORANGE),('failed_new_scan','다음 scan 재검증 실패',BLUE)]
    for key,label,color in reasons:
        values=np.array([summary[p]['rejection_reasons'].get(key,0) for p in POLICIES]);ax.barh(y,values,left=left,label=label,color=color)
        for i,n in enumerate(values):
            if n:ax.text(left[i]+n/2,i,str(n),ha='center',va='center',color='white',fontsize=10)
        left+=values
    assert all(int(left[i])==summary[p]['rejected'] for i,p in enumerate(POLICIES))
    ax.set_yticks(y,LABELS);ax.invert_yaxis();ax.set(xlim=(0,110),xlabel='거부 조건 수 / 108');ax.legend(loc='upper center',bbox_to_anchor=(.5,-.13),ncol=2,fontsize=10)
    fig.boundary='이 비교군에는 두 거부 사유만 기록됨 · 실물 bag의 ambiguous_candidate, amcl_verification_failed 등과 별도 집계'
    bundle.save(fig,'07-rejection-reasons.png')
    bundle.add(7,'복구 거부 이유 분포','07-rejection-reasons.png','기존 오프라인 실험 집계',
               '정해진 108조건의 거부 사유입니다. 실제 로봇의 모든 복구 실패 유형을 포괄하지 않습니다.')
    selected=[]
    for policy,keep in [('floor_0.50',False),('structured',True)]:
        candidates=[x for x in bundle.cases if x['reference_stable'] and any(x['offset']) and x['policies'][policy]['accepted'] and
                    (x['policies'][policy]['reference_translation_error_m']>.2 or x['policies'][policy]['reference_yaw_error_rad']>.15) and
                    (not keep or x['policies'][policy]['operation']=='keep')]
        selected.append((policy,max(candidates,key=lambda x:x['policies'][policy]['reference_translation_error_m'])))
    fig,axs=common.figure('수락했지만 기록 기준과 다른 위치','진단 사례  |  50% 정책의 큰 잔차 사례와 구조 분리안의 잘못된 위치 유지 사례',rows=2,cols=2,size=(13,12))
    fig.subplots_adjust(bottom=.16,top=.83,hspace=.30)
    for row,(policy,case) in enumerate(selected):
        model,data=bundle.window(case);points=data['points_0'];result=case['policies'][policy];b=bounds_for(points,[case['reference'],result['pose']])
        map_panel(axs[row,0],model,points,case['reference'],'기록 AMCL 기준 위치',b)
        name='50% 정책 수락 후보' if row==0 else '구조 분리안: 주입 위치 유지'
        map_panel(axs[row,1],model,points,result['pose'],f'{name}\n기준 대비 {result["reference_translation_error_m"]*100:.1f}cm / {np.degrees(result["reference_yaw_error_rad"]):.1f}°',b,case['reference'])
    point_legend(fig,.065)
    fig.boundary='정합이 좋아 보여도 다른 위치를 수락할 수 있음 · 각 범주의 기준 대비 이동 잔차 최대 사례 선택 · 물리 정답과의 오차로 해석하지 않음'
    bundle.save(fig,'06-admitted-residual-cases.png')
    bundle.add(6,'기준 위치와 다른 후보 수락 사례','06-admitted-residual-cases.png','기존 오프라인 실험 사례',
               '50% 후보 수락과 구조 분리안의 현재 위치 유지에서 각각 큰 기준 잔차 사례를 선택했습니다. 실제 잘못된 물리 위치를 측량한 결과는 아닙니다.')
    bundle.metrics['large_residual_cases']=[dict(policy=p,case=c) for p,c in selected]


def statuses(rec):
    s=rec.data['/localization/status']
    return [(t,json.loads(m.data)) for t,m in zip(s.times,s.messages)]


def real_timeline(bundle,rec,filename='03-alignment-speed-timeline.png'):
    begin,end=common.time_value('11:38:10'),common.time_value('11:39:04')
    rows=[(t,x) for t,x in statuses(rec) if begin<=t<=end]
    ts=np.array([t-begin for t,x in rows]);fig,axs=common.figure('기준 미달과 정상 복귀 대기','실물 기록  |  2026-11-08 11:38:10–11:39:04 KST  |  기준 충족 후에도 1초 연속 정상 확인 필요',rows=5,size=(13,12))
    fig.subplots_adjust(left=.17,right=.96,bottom=.12,top=.84,hspace=.40)
    ratios=[x.get('score',{}).get('inlier_ratio',np.nan)*100 for t,x in rows]
    residuals=[x.get('score',{}).get('mean_distance',np.nan)*100 for t,x in rows]
    axs[0,0].plot(ts,ratios,'.-',ms=3,c=TEAL);axs[0,0].set(ylabel='일치율 (%)',ylim=(20,65))
    axs[0,0].axhline(bundle.settings.min_inlier_ratio*100,color=RED,ls='--',label='정상 하한 30%')
    axs[0,0].legend(loc='upper left',fontsize=10)
    axs[1,0].plot(ts,residuals,'.-',ms=3,c=ORANGE);axs[1,0].set(ylabel='불일치 거리 (cm)',ylim=(9,24))
    axs[1,0].axhline(bundle.settings.max_mean_distance*100,color=RED,ls='--',label='정상 상한 18 cm')
    axs[1,0].legend(loc='upper left',fontsize=10)
    keys=['unknown','unavailable','degraded','verifying_reconvergence','available']
    states=[('verifying_reconvergence' if x['state']=='degraded' and x.get('reconvergence_reason')=='verifying_reconvergence' else x['state']) for t,x in rows]
    colors={**STATE_COLORS,'verifying_reconvergence':BLUE}
    axs[2,0].scatter(ts,[keys.index(k) for k in states],s=18,c=[colors[k] for k in states])
    axs[2,0].set_yticks(range(5),['미확인','주행 불가','저하: 불일치','저하: 복귀 대기','정상'])
    exhausted=next((t-begin for t,x in rows if x.get('reconvergence_reason')=='reconvergence_budget_exhausted'),None)
    if exhausted is not None:
        axs[2,0].annotate('8초 대기 소진',xy=(exhausted,1),xytext=(exhausted-3,-1.2),fontsize=9,color=RED,arrowprops=dict(arrowstyle='->',color=RED))
    axs[2,0].set_ylim(-1.5,4.5)
    axs[3,0].step(ts,[x.get('speed_limit_percentage',np.nan) for t,x in rows],where='post',c=TEAL);axs[3,0].set(ylabel='제한 요청 (%)',ylim=(-5,110),yticks=[0,30,100])
    for topic,label,color in [('/nav2/cmd_vel','Nav2 명령',BLUE),('/cmd_vel','출력 명령',TEAL),('/odom','odom 관측',ORANGE)]:
        s=rec.data[topic];xs=[];ys=[];last=None
        for t,m in zip(s.times,s.messages):
            if not begin<=t<=end:continue
            if last is not None and t-last>1:xs.append(np.nan);ys.append(np.nan)
            v=m.twist.twist if topic=='/odom' else m
            xs.append(t-begin);ys.append(np.hypot(v.linear.x,v.linear.y));last=t
        axs[4,0].plot(xs,ys,'.-',ms=2,lw=.8,label=label,c=color)
    axs[4,0].set_ylabel('평면 속도 (m/s)');axs[4,0].legend(ncol=3,fontsize=9)
    for ax in axs[:,0]:
        ax.set_xlim(0,end-begin);ax.set_xticks(np.arange(0,55,10),[common.time_label(begin+t) for t in np.arange(0,55,10)]);ax.grid(alpha=.15)
    fig.text(.06,.073,'점은 약 0.6초 간격으로 저장된 상태 표본 · 선 사이의 판정 입력은 기록되지 않았을 수 있음',fontsize=10,color='#58736b')
    fig.boundary='불일치 거리: 큰 오차 25% 제외 평균 · 복귀 대기는 기록된 판정 사유 · 속도 제한 요청만으로 실제 감속/제동 성능 판정 불가'
    bundle.save(fig,filename)
    bundle.add(3,'정합 상태와 속도 타임라인',filename,'실물 상태 기록',
               '당시 기준선과 기록된 정상 복귀 대기 사유를 분리합니다. 저장 표본 사이의 판정 입력은 추정하지 않으며 요청이 실제 감속을 일으켰다는 인과 판정은 하지 않습니다.')


def observed_points(rec,report,t):
    scan_time=report.get('stamp')
    if scan_time is None:return None,None
    s=rec.data['/scan_raw'];i=int(np.searchsorted(s.times,t,side='right'))
    options=[m for m in s.messages[max(0,i-15):i] if abs(common.stamp(m)-scan_time)<.001]
    if not options:return None,None
    m=options[-1];r=np.asarray(m.ranges);angle=m.angle_min+np.arange(len(r))*m.angle_increment
    good=np.isfinite(r)&(r>=.5)&(r<=min(m.range_max,8.))
    p=np.column_stack((r[good]*np.cos(angle[good]),r[good]*np.sin(angle[good]),np.zeros(good.sum())))
    p=common.apply(p,rec.tf('base_link',m.header.frame_id,scan_time))[:,:2]
    if len(p)>240:p=p[np.linspace(0,len(p)-1,240,dtype=int)]
    return p,common.stamp(m)


def live_replay(bundle,rec,start,end,bonus=False):
    all_status=statuses(rec);s=common.Series([t for t,x in all_status],[x for t,x in all_status])
    grid=rec.data['/map'].at(start);model=AlignmentMap(grid.data.reshape(grid.info.height,grid.info.width),grid.info.resolution,[grid.info.origin.position.x,grid.info.origin.position.y,0])
    selected=[x for t,x in all_status if start<=t<=end and 'pose' in x]
    poses=np.array([x['pose'] for x in selected]);low=poses[:,:2].min(axis=0)-[2,1.5];high=poses[:,:2].max(axis=0)+[2,1.5]
    width=max(high[0]-low[0],(high[1]-low[1])*1.55)
    center=(low[0]+high[0])/2;low[0]=center-width/2;high[0]=center+width/2
    bounds=[low[0],high[0],low[1],high[1]]
    times=np.arange(start,end+1e-5,.4);frames=[];audit=[];last_visual=None
    state_names={'available':'정합 정상','degraded':'정합 저하','unavailable':'정합 불량','unknown':'판정 불가'}
    title='복구 상태 전환 실물 기록' if bonus else '실제 주행 중 지도 정합'
    filename='bonus-recorded-recovery.gif' if bonus else '02-live-alignment.gif'
    for index,t in enumerate(times):
        x=s.at(t,.65);fig=plt.figure(figsize=(12,9));ax=fig.add_axes([.055,.235,.89,.55])
        fig.text(.055,.95,title,fontsize=24,weight='bold')
        fig.text(.055,.905,'실물 bag 재생 · 2026-11-08 · 2배속',fontsize=13,color='#58736b')
        fig.text(.945,.905,common.time_label(t),fontsize=16,ha='right')
        scan_stamp=None;recomputed=None;display_stamp=None
        if x and 'pose' in x:
            try:p,scan_stamp=observed_points(rec,x,t)
            except ValueError:p=None
            if p is not None:
                recomputed=model.score(p,np.asarray(x['pose']),bundle.settings)
        # 누락된 scan을 보간하지 않고 마지막 관측의 위치와 점군을 함께 고정합니다.
        end_index=int(np.searchsorted(s.times,t,side='right'))
        for report_time,report in reversed(all_status[max(0,end_index-24):end_index]):
            if t-report_time>10:break
            if 'pose' not in report:continue
            try:vp,vs=observed_points(rec,report,t)
            except ValueError:continue
            if vp is not None and vs<=t:
                if last_visual is None or vs>last_visual[2]:last_visual=(vp,report['pose'],vs)
                break
        if last_visual is not None:
            vp,pose,display_stamp=last_visual
            age=t-display_stamp;held=age>1.2
            map_panel(ax,model,vp,pose,'',bounds)
            for collection in ax.collections:collection.set_alpha(.35 if held else 1)
            for patch in ax.patches:patch.set_alpha(.35 if held else 1)
            caption=f'이전 관측 고정 표시 · {age:.1f}초 전 점군과 위치' if held else f'최근 기록 관측 · {age:.1f}초 전'
        else:
            common.show_grid(ax,grid);ax.set(xlim=bounds[:2],ylim=bounds[2:],aspect='equal')
            caption='표시 가능한 대응 관측을 기다립니다'
        ax.set(xlabel='',ylabel='',xticks=[],yticks=[])
        for spine in ax.spines.values():spine.set_visible(False)
        state=x['state'] if x else 'unknown'
        fig.text(.055,.835,state_names[state],fontsize=23,weight='bold',color=STATE_COLORS[state])
        if bonus:
            phase=x.get('phase','idle') if x else 'idle'
            secondary='최근 복구 단계: '+PHASE_LABELS.get(phase,phase)
        else:
            limit=x.get('speed_limit_percentage') if x else None
            secondary=f'속도 제한 요청 {limit:.0f}%' if limit is not None else '상태 기록 대기'
        fig.text(.945,.84,secondary,fontsize=16,ha='right')
        fig.text(.055,.195,caption,fontsize=14,color='#58736b')
        fig.legend(handles=[Line2D([],[],marker='o',color=TEAL,ls='',label='지도와 가까운 점 (≤15 cm)'),
            Line2D([],[],marker='o',color=ORANGE,ls='',label='먼 점 / 미확인'),
            Line2D([],[],marker='^',color=BLUE,ls='',label='해당 관측의 로봇 위치')],
            loc='lower center',bbox_to_anchor=(.5,.13),ncol=3,frameon=False,fontsize=12)
        progress=fig.add_axes([.055,.09,.89,.018]);progress.set(xlim=(start,end),ylim=(0,1));progress.axis('off')
        # 상태 띠는 기록 전체의 요약이며 점군에는 미래 표본을 사용하지 않습니다.
        for j,rt in enumerate(times):
            status=s.at(float(rt),.65)
            progress.axvspan(rt,min(rt+.4,end),color=STATE_COLORS[status['state'] if status else 'unknown'])
        progress.axvline(t,c=FG,lw=3)
        fig.text(.055,.063,'기록 상태 흐름',fontsize=10,color='#58736b')
        fig.text(.945,.063,f'{t-start:.0f} / {end-start:.0f}초',fontsize=10,ha='right',color='#58736b')
        common.footer(fig,'복구 단계는 노드의 보고입니다. 이후 실패도 포함하며 실물 안전 복구 전체 검증은 아닙니다.' if bonus else '점군은 저장된 관측만 표시합니다. 정합 판정과 속도 수치는 별도 타임라인에서 확인합니다.')
        fig.canvas.draw();frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3]).copy());plt.close(fig)
        audit.append(dict(time=float(t),scan_stamp=scan_stamp,display_stamp=display_stamp,state=x.get('state') if x else None,phase=x.get('phase') if x else None,
                          recorded_score=x.get('score') if x else None,recomputed_score=asdict(recomputed) if recomputed else None))
        if index%40==0:print(filename,index,'/',len(times),flush=True)
    frames[0].save(bundle.out/filename,save_all=True,append_images=frames[1:],duration=200,loop=0,optimize=False)
    candidates=[i for i,a in enumerate(audit) if a['display_stamp'] is not None
                and (not bonus or (a['phase']=='recovered' and a['state']=='available'))]
    poster=min(candidates,key=lambda i:abs(i-len(frames)//2)) if candidates else len(frames)//2
    frames[poster].save(bundle.out/filename.replace('.gif','-poster.png'))
    bundle.add(10 if bonus else 2,title,filename,'실물 기록 재생',
               '탐색→AMCL 재검증→복구 완료 보고 이후 후속 재검증/실패도 포함합니다. 완전한 안전 복구 성공 판정은 아닙니다.' if bonus else
               '상태 메시지의 scan stamp와 일치하는 기록을 사용합니다. 1.2초보다 오래된 관측은 흐리게 고정하고 나이를 표시합니다. 움직임을 보간하거나 현재 설정으로 상태를 재판정하지 않습니다.',
               frames=len(frames),frame_duration_ms=200,start=common.time_label(start),end=common.time_label(end),playback_speed=2,bonus=bonus)
    bundle.metrics['recovery_replay' if bonus else 'driving_replay']=audit


STATE_DIAGRAM='''stateDiagram-v2
    direction LR
    [*] --> unknown
    unknown --> available: 새로운 정상 관측 1초
    unknown --> unavailable: 정합 실패 지속
    available --> degraded: 가벼운 불일치
    degraded --> available: 새로운 정상 관측 1초
    degraded --> unavailable: 개선 없음 4초 / 총 8초 / 심한 불일치
    unavailable --> available: 새로운 정상 관측 1초
    available --> unknown: 입력 무효 또는 새 초기 위치
    degraded --> unknown: 입력 무효 또는 새 초기 위치
    unavailable --> unknown: 입력 무효 또는 새 초기 위치
    note right of available: 다른 안전 조건도 만족하면 주행 허용
    note right of degraded: 목표 유지 / 속도 제한 30% 요청
    note left of unknown: 주행 허용 안 함
    note right of unavailable: 정지와 허가 회수 / 취소 목표 자동 재개 없음
'''

RECOVERY_DIAGRAM='''sequenceDiagram
    participant S as Supervisor
    participant B as Base와 기존 목표
    participant G as Localization guard
    participant A as AMCL
    S->>B: 위치 정합 Fault로 주행 허가 회수
    B-->>S: 기존 목표 정지 확인과 base_disarmed 안정
    S->>G: localization/recover 요청
    G->>G: 정지 확인 후 주변 후보 탐색
    G->>G: 탐색/검증 빔 분리와 후보 유일성 검사
    G->>G: 다음 신선한 scan으로 후보 재검사
    alt 검사 통과
        G->>A: initialpose 갱신
        A-->>G: 새로운 pose와 scan
        G->>G: 1.5초 정상 정합 확인
        G-->>S: 복구 성공 보고
        S->>S: 신선한 health 재확인
        Note over S,B: 취소된 이전 목표 자동 재개 없음
    else 모호한 후보 / 입력 이상 / 움직임 / 취소
        G-->>S: 후보 적용 거부 또는 복구 중단
        Note over S,B: 자동 재시도 총 2회 / 실패 시 주행 차단 유지
    end
'''


def diagrams(bundle):
    source=REPO/'ros2_ws/src/cleany_navigation/docs/localization.md';bundle.sources.add(source)
    for number,title,stem,code in [(8,'상태별 주행 허가','08-localization-states',STATE_DIAGRAM),(9,'정지 상태 복구 순서','09-recovery-sequence',RECOVERY_DIAGRAM)]:
        (bundle.out/(stem+'.mmd')).write_text(code)
        (bundle.out/(stem+'.md')).write_text(f'# {title}\n\n현재 구현 문서 기준의 설명입니다. 당시 실물 동작 검증과 구분합니다.\n\n```mermaid\n'+code+'```\n')
        page='<!doctype html><meta charset="utf-8"><style>body{background:#f3f7f5;color:#203d3a;font:18px NanumGothic,sans-serif;padding:28px}#diagram{width:1320px}svg{width:100%;max-width:none!important}</style><h1>'+title+'</h1><p>현재 구현 설명 · 당시 실물 기록과 구분 · Mermaid 원문 제공</p><div id="diagram"></div><script src="mermaid.tiny.js"></script><script>mermaid.initialize({startOnLoad:false,securityLevel:"strict",theme:"base",fontFamily:"NanumGothic",themeVariables:{primaryColor:"#e4f2eb",primaryTextColor:"#203d3a",lineColor:"#137c70",fontSize:"19px"}});mermaid.render("diagramSvg",'+json.dumps(code)+').then(x=>{document.getElementById("diagram").innerHTML=x.svg;document.body.dataset.ready="true"})</script>'
        (bundle.out/(stem+'.html')).write_text(page)
        bundle.add(number,title,stem+'.png','현재 구현 설명','현재 구현 문서의 상태/복구 계약을 요약합니다. 실물 bag에서 모든 분기의 동작을 검증했다는 뜻은 아닙니다.',mermaid=stem+'.mmd')


def write_package(bundle,rec):
    bundle.items.sort(key=lambda x:x['number'])
    bundle.sources.update(rec.sources)
    bundle.sources.update([Path(__file__),REPO/'tools/validation/render_localization_jira_diagrams.mjs',REPO/'tools/validation/render_depth_jira_visuals.py',
                          REPO/'tools/validation/report_localization_policy_experiment.py',REPO/'ros2_ws/src/cleany_navigation/cleany_navigation/localization_alignment.py'])
    events=[];previous=None
    for t,x in statuses(rec):
        if x.get('phase')!=previous:
            events.append(dict(time=common.time_label(t),phase=x.get('phase'),state=x['state'],reason=x.get('recovery_reason')));previous=x.get('phase')
    bundle.metrics['recorded_recovery_phase_changes']=events
    shutil.copyfile(REPO/'docs/guide/vendor/mermaid.tiny.js',bundle.out/'mermaid.tiny.js')
    cards=[]
    for item in bundle.items:
        prefix='보조' if item.get('bonus') else f'{item["number"]:02d}'
        links=f'<a href="{item["file"]}" download>원본 다운로드</a>'
        if item.get('mermaid'):links+=f' · <a href="{item["mermaid"]}">Mermaid 원문</a>'
        cards.append(f'<section id="v{item["number"]}"><span>{item["scope"]}</span><h2>{prefix} · {item["title"]}</h2><img src="{item["file"]}" alt="{item["title"]}"><p>{html.escape(item["note"])}</p><p>{links}</p></section>')
    header='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SCRUM-557 시각자료</title><style>body{margin:0;background:#edf3ef;color:#203d3a;font:16px/1.7 system-ui,sans-serif}main{max-width:1250px;margin:auto;padding:40px 24px}h1{font-size:36px}section{background:white;border:1px solid #d8e5dc;border-radius:16px;padding:24px;margin:24px 0}img{width:100%;height:auto}span,a{color:#137c70}nav{display:flex;gap:14px;flex-wrap:wrap}</style><main><h1>위치 정합과 정지 상태 복구</h1><p>SCRUM-557 · 시각자료 9종 + 실물 복구 상태 보조 GIF</p><p>실물 bag, 오프라인 후보 비교, 현재 구현 설명을 구분합니다. 기록 AMCL은 외부 측량 정답이 아니며 후보 수락률은 실제 복구 성공률이 아닙니다.</p><p><a href="contact-sheet.png">전체 미리보기</a> · <a href="manifest.json">출처</a> · <a href="metrics.json">분석 수치와 단계 기록</a></p><nav>'''
    (bundle.out/'index.html').write_text(header+''.join(f'<a href="#v{x["number"]}">{"보조" if x.get("bonus") else x["number"]} {x["title"]}</a>' for x in bundle.items)+'</nav>'+''.join(cards)+'</main></html>')
    sources=[]
    for p in sorted(bundle.sources):
        digest=hashlib.sha256()
        with p.open('rb') as stream:
            for block in iter(lambda:stream.read(4*1024*1024),b''):digest.update(block)
        sources.append(dict(path=str(p),bytes=p.stat().st_size,sha256=digest.hexdigest()))
    manifest=dict(items=bundle.items,sources=sources,command=sys.argv,git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
                  policy='Recorded state decisions are not reclassified with current settings. Last matching scan and pose remain frozen until a newer matching observation; age is labeled and observations older than 1.2s are faded. This is a visual hold, not a controller freshness rule. Dynamic scans and TF use only past available messages. The status strip summarizes the entire recording interval. Offline AMCL reference is not surveyed truth.')
    (bundle.out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    (bundle.out/'metrics.json').write_text(json.dumps(bundle.metrics,ensure_ascii=False,indent=2))


def finalize(out):
    manifest=common.read_json(out/'manifest.json');metrics=common.read_json(out/'metrics.json')
    assert [x['number'] for x in manifest['items']]==list(range(1,11))
    font=ImageFont.truetype('/usr/share/fonts/naver-nanum-gothic-fonts/NanumGothic.ttf',19)
    sheet=Image.new('RGB',(1880,1660),BG);checks=[]
    for i,item in enumerate(manifest['items']):
        path=out/item['file'];hashes=set();duration=0
        with Image.open(path) as im:
            count=getattr(im,'n_frames',1)
            for frame in range(count):
                im.seek(frame);im.load();hashes.add(hashlib.sha256(im.convert('RGB').tobytes()).hexdigest());duration+=im.info.get('duration',0)
            if path.suffix=='.gif':
                # GIF 인코더는 연속 동일 프레임을 합치므로 총 재생 시간도 대조합니다.
                frame_ms=item.get('frame_duration_ms',1800 if item['number']==4 else 100)
                assert 1<count<=item['frames'] and len(hashes)>1
                assert duration==item['frames']*frame_ms
            im.seek(count//2);thumb=im.convert('RGB')
            poster=path.with_name(path.stem+'-poster.png')
            if path.suffix=='.gif' and poster.exists():
                with Image.open(poster) as preview:thumb=preview.convert('RGB')
            thumb.thumbnail((590,350))
            card=Image.new('RGB',(620,410),'white');card.paste(thumb,((620-thumb.width)//2,45+(350-thumb.height)//2))
            label='보조' if item.get('bonus') else f'{item["number"]:02d}'
            ImageDraw.Draw(card).text((12,10),label+' '+item['title'],font=font,fill=FG)
            sheet.paste(card,(10+i%3*620,10+i//3*410))
        checks.append(dict(file=path.name,frames=count,duration_ms=duration,bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for key in ['driving_replay','recovery_replay']:
        assert all(r['scan_stamp'] is None or r['scan_stamp']<=r['time'] for r in metrics[key])
        assert all(r['display_stamp'] is None or r['display_stamp']<=r['time'] for r in metrics[key])
        shown=[r['display_stamp'] for r in metrics[key] if r['display_stamp'] is not None]
        assert all(a<=b for a,b in zip(shown,shown[1:]))
        for row in metrics[key]:
            if row['recomputed_score'] is not None:
                for metric in ['inlier_ratio','mean_distance']:
                    assert abs(row['recorded_score'][metric]-row['recomputed_score'][metric])<1e-10
    sheet.save(out/'contact-sheet.png')
    (out/'validation.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2))
    with zipfile.ZipFile(out/'scrum-557-visuals.zip','w',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(out.iterdir()):
            if path.is_file() and path.suffix!='.zip':z.write(path,path.name)
    print(json.dumps(checks,ensure_ascii=False,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--data-root',type=Path);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--finalize',action='store_true')
    args=parser.parse_args()
    if args.finalize:finalize(args.output);return
    if args.data_root is None:parser.error('--data-root가 필요합니다')
    exp=args.data_root/'runs/navigation/traction/2026-11-08_094147_recovery-policy-comparison'
    bundle=Bundle(args.output,exp)
    examples(bundle);policy_figures(bundle);diagrams(bundle)
    bag=args.data_root/'runs/navigation/traction/2026-11-08_080053_physical-validation/runtime/recordings/bags/37707477-1292-47e4-b6f7-2537af79e2cf'
    print('주행 bag 읽기',flush=True);rec=common.Recording(bag)
    real_timeline(bundle,rec)
    live_replay(bundle,rec,common.time_value('11:38:18'),common.time_value('11:39:03'))
    live_replay(bundle,rec,common.time_value('11:12:00'),common.time_value('11:12:40'),True)
    write_package(bundle,rec)


if __name__=='__main__':main()
