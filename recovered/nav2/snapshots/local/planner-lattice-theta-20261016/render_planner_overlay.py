"""Re-render measured experiment plots with recorded Nav2 /plan messages."""
from pathlib import Path
import bisect
import csv
import json
import math
import sys
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'media/planner-overlay'
OUT.mkdir(exist_ok=True)
source=(ROOT/'render_results.py').read_text()
# Reuse the original map transforms, crop extents and GT interpolation exactly.
source=source[:source.rindex('\nfor i in INDICES:')]
namespace={'__file__':str(ROOT/'render_results.py'),'__name__':'overlay_render'}
sys.argv=[str(ROOT/'render_results.py')]
exec(compile(source,str(ROOT/'render_results.py'),'exec'),namespace)
W,H=namespace['W'],namespace['H']
PLANNERS=namespace['PLANNERS'];NAMES=namespace['NAMES']
DATA=namespace['DATA'];BACKGROUNDS=namespace['BACKGROUNDS']
FONT=namespace['F14'];PURPLE=(151,48,178)
PLANS={};provenance=[]
for planner in PLANNERS:
    for i,name in enumerate(NAMES):
        folder=ROOT/planner/f'{i}_{name}'
        plans=json.loads((folder/'plans.json').read_text())
        assert plans and all(p['frame']=='map' for p in plans)
        assert all(all(math.isfinite(v) for pose in p['poses'] for v in pose) for p in plans)
        result=DATA[planner,i][0]
        PLANS[planner,i]=plans,[p['sim_s'] for p in plans]
        provenance.append({'planner':planner,'goal':name,'source':str(folder/'plans.json'),
            'frame':'map','message_count':len(plans),'initial_pose_count':len(plans[0]['poses']),
            'initial_plan_elapsed_sim_s':plans[0]['sim_s']-result['navigation_sim_start_s'],
            'initial_plan_start':plans[0]['poses'][0],'initial_plan_end':plans[0]['poses'][-1]})

def dashed(draw,points,color=PURPLE,width=2):
    distance=0.
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b)
        if length<1e-9:continue
        consumed=0.
        while consumed<length-1e-9:
            phase=distance%10
            step=min(length-consumed,(6-phase) if phase<6 else (10-phase))
            if step<1e-8:step=min(length-consumed,1e-6)
            if phase<6:
                p0=tuple(a[k]+(b[k]-a[k])*consumed/length for k in [0,1])
                p1=tuple(a[k]+(b[k]-a[k])*(consumed+step)/length for k in [0,1])
                draw.line([p0,p1],fill=color,width=width)
            consumed+=step;distance+=step

def path_on(image,planner,i,plan):
    draw=ImageDraw.Draw(image);tr=BACKGROUNDS[i][1]
    dashed(draw,[tr(p[0],p[1]) for p in plan['poses']])

def add_gt(image,planner,i,until=None):
    result,rows,times,duration=DATA[planner,i];tr=BACKGROUNDS[i][1]
    selected=rows if until is None else rows[:bisect.bisect_right(times,until)]
    if len(selected)>1:
        points=[tr(r[1],r[2]) for r in selected[::3]]+[tr(*selected[-1][1:3])]
        ImageDraw.Draw(image).line(points,fill=namespace['COLORS'][planner],width=2)

def static_cell(planner,i,show_gt):
    image=namespace['cell'](planner,i,0)
    plans,_=PLANS[planner,i]
    path_on(image,planner,i,plans[0])
    if show_gt:add_gt(image,planner,i)
    draw=ImageDraw.Draw(image)
    draw.rectangle((1,25,W-2,93),fill=(249,249,249))
    result=DATA[planner,i][0]
    status={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(result['action_status'],'ENDED')
    draw.text((10,28),'Initial actual /plan'+(' + full recorded GT' if show_gt else ''),font=FONT,fill=(40,40,40))
    draw.text((10,49),'Purple dashed: plan'+(' | blue/orange: GT' if show_gt else ' | body: initial pose'),font=FONT,fill=(80,45,90))
    draw.text((10,70),f'Final {status}: {result["position_error_m"]*100:.1f}cm / {result["yaw_error_deg"]:.1f}deg',font=FONT,fill=(55,55,55))
    return image

for show_gt,name in [(False,'initial-planner-paths-2rows-8cols.png'),(True,'initial-plan-vs-actual-gt-2rows-8cols.png')]:
    image=Image.new('RGB',(W*8,H*2))
    for row,planner in enumerate(PLANNERS):
        for i in range(8):image.paste(static_cell(planner,i,show_gt),(W*i,H*row))
    image.save(OUT/name)
    print(name,flush=True)
    if show_gt:
        pair=Image.new('RGB',(W,H*2))
        for row,planner in enumerate(PLANNERS):pair.paste(static_cell(planner,7,True),(0,H*row))
        pair.save(OUT/'other-room-initial-plan-vs-gt.png')

# The animation shows the last /plan actually received by each replay time.
original_cell=namespace['cell']
def animated_cell(planner,i,time):
    image=original_cell(planner,i,time)
    result,rows,times,duration=DATA[planner,i]
    plans,stamps=PLANS[planner,i]
    index=bisect.bisect_right(stamps,result['navigation_sim_start_s']+min(time,duration))-1
    if index>=0:path_on(image,planner,i,plans[index])
    add_gt(image,planner,i,min(time,duration))
    draw=ImageDraw.Draw(image);draw.rectangle((1,69,W-2,92),fill=(249,249,249))
    text='Purple: latest actual /plan | solid: GT' if index>=0 else 'Waiting for first actual /plan'
    draw.text((10,70),text,font=FONT,fill=(80,45,90))
    return image
namespace['cell']=animated_cell
# Add purple to the global palette; original palettes are intentionally limited.
palette=namespace['palette'];colors=palette.getpalette();colors[246*3:247*3]=list(PURPLE);palette.putpalette(colors)
namespace['write_gif'](OUT/'live-planner-path-vs-gt-2rows-8cols-2x.gif',[(p,i) for p in PLANNERS for i in range(8)],8)
(OUT/'provenance.json').write_text(json.dumps({'static':'First recorded actual /plan, no replanning recomputation or hand-drawn route',
    'animation':'Latest /plan received at or before each simulation replay timestamp; no future messages shown',
    'layout':'Lattice top, Theta* bottom; original route order 1..8; original map transforms',
    'purple_dashed':'Actual global planner path centerline (XY), not footprint swept area',
    'blue_orange_solid':'Recorded Gazebo GT trajectory','sources':provenance},indent=2))
