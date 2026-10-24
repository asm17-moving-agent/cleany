#!/usr/bin/env python3
from pathlib import Path
import json, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
rows=json.loads((ROOT/'comparison.json').read_text())
colors={'dwb':'#2563eb','mppi':'#059669','teb':'#ea580c','rpp':'#9333ea'}
fields=[('position_error_cm','Position error (cm)'),('yaw_error_deg','Heading error (deg)'),('sim_s','Navigation time (sim s)'),('controller_cpu_one_core_pct','Controller CPU (% of one core; host-dependent)'),('linear_jerk_rms_m_s3','Linear jerk RMS (m/s³)'),('angular_jerk_rms_rad_s3','Angular jerk RMS (rad/s³)')]
fig,axes=plt.subplots(2,3,figsize=(15,8.5),layout='constrained')
for ax,(key,label) in zip(axes.flat,fields):
 if key=='controller_cpu_one_core_pct':
  profiles=json.loads((ROOT/'profile_compute.json').read_text());by={r['candidate']:r for r in profiles};cs=list(colors)
  ax.bar(cs,[by[c]['p95_ms'] for c in cs],color=[colors[c] for c in cs],width=.6)
  for i,c in enumerate(cs):ax.text(i,by[c]['p95_ms']+1,f"{by[c]['p95_ms']:.2f}",ha='center',fontsize=10)
  ax.axhline(50,color='#64748b',linestyle=':',linewidth=1);ax.set_ylim(0,65);ax.set_xticks(range(4),[c.upper() for c in cs]);ax.set_title('Compute time p95 (ms) — same ARM64 host\nSupplemental straight-route profiling',fontsize=11);ax.grid(axis='y',alpha=.22);ax.spines[['top','right']].set_visible(False);continue
 for candidate,color in colors.items():
  valid=[r for r in rows if r['candidate']==candidate and r.get(key) is not None and math.isfinite(r[key])]
  if key in ['position_error_cm','yaw_error_deg']:valid=[r for r in valid if r.get('action_status')==4]
  if not valid:continue
  for success,marker in [(True,'o'),(False,'x')]:
   subset=[r for r in valid if (r.get('action_status')==4)==success]
   ax.scatter([r['goal_index']+1 for r in subset],[r[key] for r in subset],color=color,marker=marker,s=65,label=candidate.upper()+f" ({sum(r['action_status']==4 for r in rows if r['candidate']==candidate)}/5)" if success else None)
 ax.set_title(label,fontsize=11);ax.set_xticks(range(1,6),['Straight','Turn arrival','Chair gap','Far heading','Distant'],rotation=15,fontsize=9);ax.grid(axis='y',alpha=.22);ax.spines[['top','right']].set_visible(False)
 if key=='position_error_cm':ax.axhline(15,color='#64748b',linestyle=':',linewidth=1);ax.set_ylim(0,max(17,max(r['position_error_cm'] for r in rows if r.get('action_status')==4)*1.15))
 if key=='yaw_error_deg':ax.axhline(10,color='#64748b',linestyle=':',linewidth=1);ax.set_ylim(0,12)
handles,labels=axes[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,1.015),ncol=4,frameon=False)
fig.suptitle('Cleany Nav2 controller comparison — five shared routes\nRTF target 1 | wheel command cap 5 rad/s | goal tolerance 15 cm / 10°',fontsize=15,y=1.12)
fig.text(.02,-.02,'Accuracy panels: arrivals only. X elsewhere: aborted/timed out. Compute: supplemental straight-route profiling on one ARM64 host; scheduling effects included.',fontsize=9,color='#475569')
fig.savefig(ROOT/'comparison.png',dpi=160,bbox_inches='tight');fig.savefig(ROOT/'comparison.pdf',bbox_inches='tight')
print(ROOT/'comparison.png')
