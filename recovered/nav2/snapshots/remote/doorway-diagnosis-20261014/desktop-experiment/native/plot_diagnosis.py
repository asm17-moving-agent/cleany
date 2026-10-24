from pathlib import Path
import json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).resolve().parent
v=json.loads((r/'replay/diagnosis.json').read_text())['models']
keys=list(v);d=[v[k]['snapshots'][0] for k in keys]
fig,axs=plt.subplots(1,2,figsize=(10,4))
for i,(k,row) in enumerate(zip(keys,d)):
 lo,hi=row['candidate_yaw_min_deg'],row['candidate_yaw_max_deg'];axs[0].plot([lo,hi],[i,i],linewidth=8,solid_capstyle='butt');axs[0].text(hi+.7,i,f'{lo:.2f} to {hi:.2f} deg',va='center',fontsize=8)
axs[0].axvline(d[0]['required_yaw_deg'],color='red',linestyle='--',label='Required path yaw change')
axs[0].set_yticks(range(len(keys)),keys);axs[0].set_xlabel('End yaw change from initial body yaw (deg)');axs[0].set_xlim(-10,40);axs[0].set_ylim(-.5,1.5);axs[0].legend(loc='upper left',fontsize=8);axs[0].set_title('First sampled batch: yaw coverage')
base=[0.,0.]
for critic in ['CostCritic','PathAlignCritic','PathFollowCritic','TwirlingCritic']:
 vals=[row['best_costs'][critic] for row in d];axs[1].bar(keys,vals,bottom=base,label=critic);base=[a+b for a,b in zip(base,vals)]
axs[1].set_title('Lowest critic-total candidate');axs[1].set_ylabel('Cost before MPPI control regularization');axs[1].legend(fontsize=8)
fig.suptitle('Frozen native optimizer replay: fixed pose, zero measured speed, fixed Gaussian batch',fontsize=10)
fig.tight_layout();fig.savefig(r/'diagnosis.png',dpi=150)
