"""Render offline sphere candidates and prepare CUDA witness queries."""
from pathlib import Path
import argparse
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/validation'))
from render_collision_visualization import load_geometry, moved, world_spheres, surface, sphere_overlay, axis, header, save, BG, INK, MUTED, OLD, BALL
import matplotlib.pyplot as plt
from matplotlib import font_manager

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--comparison', type=Path, required=True)
parser.add_argument('--review', type=Path, required=True)
args = parser.parse_args()
OUT = args.comparison.parent
report = json.loads(args.comparison.read_text())
parts, transforms, raw, active, model, provenance = load_geometry(Path(report['source']))
fits = np.load(OUT/'fits.npz', allow_pickle=False)
font = Path('/System/Library/Fonts/AppleSDGothicNeo.ttc')
font_manager.fontManager.addfont(font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(3, 3, figsize=(14, 12), facecolor=BG)
fig.subplots_adjust(left=.045, right=.97, top=.84, bottom=.13, hspace=.55, wspace=.16)
header(fig, '구 개수를 유지해도 더 작게 맞출 수 있습니다',
       '회색 = 원본 CAD   ·   각 행은 동일한 시점과 축척   ·   파랑/보라 = 형상에 직접 맞춘 비교안, 운영 미적용')
names = [('right_lower_arm_link','하완 + 모터'), ('right_moving_jaw_link','오른쪽 가동 집게'), ('top_base_link','마스트')]
for row,(name,title) in enumerate(names):
    entries=[]
    for part in parts:
        if part.owner == name and part.filename!='primitive':
            transform=np.linalg.inv(transforms[name])@part.transform
            entries.append((moved(part.original_vertices,transform),part.original_faces))
    vertices=np.vstack([v for v,f in entries])
    _,_,basis=np.linalg.svd(vertices-vertices.mean(0),full_matrices=False)
    view=basis[[0,2,1]]
    c,r=active[name]
    projected=c@view.T
    lo=(projected[:,:2]-r[:,None]).min(0);hi=(projected[:,:2]+r[:,None]).max(0)
    size=hi-lo
    for col,(label,color,caption) in enumerate([('current16',BALL,'현재 16개'),('direct16','#386de8','형상 맞춤 16개'),('direct32','#8b5cf6','형상 맞춤 32개')]):
        ax=axes[row,col]
        c,r=fits[f'{name}__{label}__centers'],fits[f'{name}__{label}__radii']
        surface(ax,entries,view,tint=OLD)
        sphere_overlay(ax,(c,r),view,tint=color,fill=.12)
        metric=report['links'][name]['variants'][label]
        axis(ax,(lo-size*.12,hi+size*.12),f'{title} · {caption}',f"외곽 초과 {metric['max_directional_cad_overreach_mm']:.1f} mm")
        length=.02 if row<2 else .05
        y=lo[1]-size[1]*.06
        ax.plot([lo[0],lo[0]+length],[y,y],color=INK,lw=2)
        ax.text(lo[0]+length/2,y-size[1]*.025,f'{length*1000:.0f} mm',ha='center',va='top',color=MUTED,fontsize=8)
fig.text(.045,.075,'현재와 16개 비교안은 모두 전체 240개입니다. 32개 비교안은 480개이며, 처리 속도 개선을 뜻하지 않습니다.',fontsize=11,color=INK)
fig.text(.045,.048,'외곽 초과는 648개 방향에서 측정했습니다. 부품별 볼록 조각을 분할하고 각 조각을 포함하는 구를 직접 계산합니다.',fontsize=10,color=MUTED)
fig.text(.045,.022,'메시 링크 15개 대상. base와 primitive는 제안용 구에 포함되지 않으며 최종 전체 형상 검사가 필요합니다.',fontsize=10,color=MUTED)
save(fig,OUT,'sphere-fit-comparison.png')
review=json.loads(args.review.read_text())
arrays={}
for label in ('current16','direct16','direct32'):
    values={name:(fits[f'{name}__{label}__centers'],fits[f'{name}__{label}__radii']) for name in active}
    c,r=world_spheres(values,transforms,owner=lambda name:not name.startswith('left_'))
    arrays[label]=np.column_stack((c,r)).astype(np.float32)
arrays['box_centers']=np.array([w['box_center_base_m'] for w in review['witnesses']])
arrays['box_dimensions']=np.array([w['box_dimensions_m'] for w in review['witnesses']])
np.savez_compressed(OUT/'queries.npz',**arrays)
(OUT/'review.json').write_text(json.dumps(review,indent=2)+'\n')
print('Saved comparison image and CUDA queries')
