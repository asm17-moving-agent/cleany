#!/usr/bin/env python3
"""Plot the measured native target matrix and export its first-pass results."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def plot(directory: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    import numpy as np

    cases=json.loads((directory/'targets.json').read_text())
    rows=json.loads((directory/'rows.json').read_text())
    first={row['case_id']:row for row in rows if row['repeat']==0}
    lookup={case['case_id']:case for case in cases}
    states={'planned':('#18865b','IK + validated CPU plan'),
            'ik_only':('#2878b5','IK accepted, CPU plan rejected'),
            'rejected':('#c74f48','No accepted aligned IK')}
    fig,axes=plt.subplots(2,2,figsize=(12,10),layout='constrained')
    for column,arm in enumerate(('left','right')):
        for row_index,projection in enumerate(((0,1),(0,2))):
            ax=axes[row_index,column]
            for state,(color,label) in states.items():
                points=[]
                for key,result in first.items():
                    case=lookup[key]
                    if case['arm']!=arm or case['expected_rejection']:
                        continue
                    actual=('planned' if result.get('plan_passed') else
                            'ik_only' if result['ik_accepted'] else 'rejected')
                    if state==actual:
                        points.append(case['pose']['position'])
                if points:
                    values=np.asarray(points)
                    ax.scatter(values[:,projection[0]],values[:,projection[1]],
                               s=30,c=color,alpha=.65,label=label,edgecolors='none')
            ax.set_title(f'{arm.title()} arm: '+('XY' if row_index==0 else 'XZ'))
            ax.set_xlabel('base_link X (m)')
            ax.set_ylabel('base_link '+('Y' if row_index==0 else 'Z')+' (m)')
            ax.grid(alpha=.2)
            ax.set_aspect('equal',adjustable='datalim')
    handles=[Line2D([],[],marker='o',linestyle='',color=color,label=label)
             for color,label in states.values()]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,-.04),ncol=3)
    fig.suptitle('Native hybrid target survey: first-pass positions\n'
                 'Expected rejection controls omitted; orientation variants overlap',fontsize=14)
    fig.savefig(directory/'target_positions.png',dpi=170,bbox_inches='tight')
    fig.savefig(directory/'target_positions.svg',bbox_inches='tight')
    plt.close(fig)

    categories=['interior_fk','joint_boundary','random_valid_fk','cartesian_grid','orientation',
                'outside_reach','tcp_obstacle','base_obstacle']
    labels=['Interior FK','Joint boundary','Random valid FK','XYZ offsets','Orientation',
            'Outside reach','TCP obstacle','Base obstacle']
    counts=[]; accepted=[]; planned=[]
    for category in categories:
        ids=[case['case_id'] for case in cases if case['category']==category]
        selected=[first[key] for key in ids if key in first]
        counts.append(len(ids)); accepted.append(sum(r['ik_accepted'] for r in selected))
        planned.append(sum(r.get('plan_passed',False) for r in selected))
    x=np.arange(len(categories)); fig,ax=plt.subplots(figsize=(12,5),layout='constrained')
    ax.bar(x-.24,counts,width=.24,color='#b8bec5',label='Unique conditions')
    ax.bar(x,accepted,width=.24,color='#2878b5',label='IK accepted')
    ax.bar(x+.24,planned,width=.24,color='#18865b',label='CPU plan passed')
    for shift,values in ((-.24,counts),(0,accepted),(.24,planned)):
        for index,value in enumerate(values):
            ax.text(index+shift,value+.7,str(value),ha='center',fontsize=8)
    ax.set_xticks(x,labels,rotation=15,ha='right')
    ax.set_ylabel('First-pass target conditions')
    ax.set_title('Native target outcomes by condition; negative controls should be rejected')
    ax.legend(ncol=3); ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
    fig.savefig(directory/'category_results.png',dpi=170,bbox_inches='tight')
    fig.savefig(directory/'category_results.svg',bbox_inches='tight')
    plt.close(fig)

    fields=['case_id','arm','category','x_m','y_m','z_m','opening_rad','witness_valid',
            'ik_accepted','plan_passed','plan_seconds','failure']
    with (directory/'first_pass.csv').open('w',newline='') as output:
        writer=csv.DictWriter(output,fieldnames=fields); writer.writeheader()
        for case in cases:
            result=first.get(case['case_id'],{})
            position=case['pose']['position']
            writer.writerow(dict(case_id=case['case_id'],arm=case['arm'],category=case['category'],
                x_m=position[0],y_m=position[1],z_m=position[2],opening_rad=case['opening'],
                witness_valid=case['witness_valid'],ik_accepted=result.get('ik_accepted'),
                plan_passed=result.get('plan_passed'),plan_seconds=result.get('plan_seconds'),
                failure=result.get('failure','not evaluated')))


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    plot(parser.parse_args().directory)


if __name__=='__main__':
    main()
