from pathlib import Path
import csv,json,math
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
ROOT=OUT.parent
records=[];signals={}
def flat(doc,prefix=''):
    result={}
    for key,value in doc.items():
        if isinstance(value,dict):result.update(flat(value,prefix+key+'.'))
        else:result[prefix+key]=value
    return result

for current in sorted((ROOT/'lattice').glob('[0-7]_*')):
    if not current.is_dir():continue
    prior=OUT/'prior-diverse8'/current.name
    pair={};controllers=[];planners=[];libraries=[]
    for label,folder in [('prior',prior),('current',current)]:
        result=json.loads((folder/'result.json').read_text())
        params=yaml.safe_load((folder/'parameters.yaml').read_text())
        ctrl=flat(params['controller_server']);ctrl.pop('ros__parameters.FollowPath.benchmark_timing_file',None)
        controllers.append(ctrl);planners.append(params['planner_server']);libraries.append(result['loaded_mppi_libraries_sha256'])
        start,end=result['navigation_sim_start_s'],result['navigation_sim_end_s']
        data=np.loadtxt(folder/'velocity.csv',delimiter=',');data=data[(data[:,0]>=start)&(data[:,0]<=end)]
        _,indices=np.unique(data[:,0],return_index=True);data=data[np.sort(indices)]
        t=np.arange(data[0,0],data[-1,0],.05)
        velocity=np.column_stack([np.interp(t,data[:,0],data[:,i]) for i in [1,2,3]])
        smooth=np.column_stack([np.convolve(velocity[:,i],np.ones(5)/5,mode='same') for i in range(3)])
        jerk=np.gradient(np.gradient(smooth,.05,axis=0),.05,axis=0)[5:-5]
        tt=t[5:-5]-start;linear=np.linalg.norm(jerk[:,:2],axis=1)
        edge=(tt<.75)|(tt>result['navigation_sim_s']-.75)
        command_rows=[row for row in csv.reader((folder/'commands.csv').open()) if row[2]=='/nav2/cmd_vel' and start<=float(row[0])<=end]
        intervals=np.diff(np.array([[float(row[0]),float(row[1])] for row in command_rows]),axis=0)
        intervals=intervals[intervals[:,0]>0]
        plans=json.loads((folder/'plans.json').read_text());pt=np.array([p['sim_s'] for p in plans])
        pair[label]={'linear_jerk_rms':float(np.sqrt(np.mean(linear**2))),
            'angular_jerk_rms':float(np.sqrt(np.mean(jerk[:,2]**2))),
            'linear_peak':float(max(linear)),'linear_peak_sim_s':float(tt[np.argmax(linear)]),
            'first_last_0p75s_linear_squared_energy_fraction':float(np.sum(linear[edge]**2)/np.sum(linear**2)),
            'duration_sim_s':result['navigation_sim_s'],'actual_rtf':result['achieved_rtf'],
            'command_median_interval_sim_s':float(np.median(intervals[:,0])),
            'command_median_interval_wall_s':float(np.median(intervals[:,1])),
            'plan_count':len(plans),'plan_median_interval_sim_s':float(np.median(np.diff(pt))),
            'control_deadline_warnings':result['controller_deadline_warning_count'],
            'model_dt':params['controller_server']['ros__parameters']['FollowPath']['model_dt']}
        signals[current.name,label]=(t-start,velocity,tt,linear,np.abs(jerk[:,2]),pt-start)
    records.append({'route':current.name,'controller_equal':controllers[0]==controllers[1],
        'planner_equal':planners[0]==planners[1],'mppi_binary_equal':bool(libraries[0]) and libraries[0]==libraries[1],**pair})

summary={label:{metric:float(np.mean([r[label][metric] for r in records[:7]]))
    for metric in ['linear_jerk_rms','angular_jerk_rms']} for label in ['prior','current']}
(OUT/'analysis.json').write_text(json.dumps({'routes':records,'first7_equal_route_weight_means':summary,
    'jerk_definition':'Same original definition: interpolate GT twist at 50ms sim time, 5-sample centered moving average, two numerical gradients, trim 5 samples each end.',
    'interpretation':'Timing conditions differ despite identical controller parameters and MPPI libraries. More frequent simulation-time commands/replans and short-route endpoint transients are observed; their causal contributions have not been isolated with a matched-load rerun.',
    'source_rate_controller':'https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_behavior_tree/plugins/decorator/rate_controller.cpp'},indent=2))

fig,axes=plt.subplots(2,2,figsize=(13,7),layout='constrained')
colors={'prior':'#677483','current':'#2464c7'}
for label in ['prior','current']:
    t,v,jt,j,a,pt=signals['5_lateral',label]
    axes[0,0].plot(t,np.linalg.norm(v[:,:2],axis=1),color=colors[label],label=label,lw=1.5)
    axes[0,1].plot(jt,j,color=colors[label],label=label,lw=1.5)
    t,v,jt,j,a,pt=signals['4_distant',label]
    axes[1,0].plot(t,v[:,2],color=colors[label],label=label,lw=1)
    axes[1,1].plot(jt,a,color=colors[label],label=label,lw=1)
axes[0,0].set(title='Short lateral route: recorded speed',ylabel='m/s')
axes[0,1].set(title='Short lateral route: departure / arrival jerk',ylabel='m/s³')
axes[1,0].set(title='Distant route: angular corrections',ylabel='rad/s')
axes[1,1].set(title='Distant route: angular jerk',ylabel='rad/s³')
for ax in axes.flat:ax.set_xlabel('Elapsed simulation time (s)');ax.grid(alpha=.2);ax.legend()
fig.suptitle('Same Lattice + MPPI binaries/settings, different runtime timing',fontsize=15)
fig.savefig(OUT/'jerk-prior-vs-current.png',dpi=165)
print(json.dumps(summary));print('All controller/planner/binary equal:',all(r['controller_equal'] and r['planner_equal'] and r['mppi_binary_equal'] for r in records))
