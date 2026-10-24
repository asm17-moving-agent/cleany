"""Time-faithful 2x GIFs from measured Gazebo poses; no invented motion."""
from pathlib import Path
import bisect
import csv
import json
import math
import argparse
from PIL import Image, ImageDraw, ImageFont, ImageChops, GifImagePlugin
import yaml

ROOT = Path(__file__).resolve().parent
MEDIA = ROOT / 'media'
MEDIA.mkdir(exist_ok=True)
PLANNERS = ['dwb_lattice', 'dwb_theta', 'mppi_lattice', 'mppi_theta']
LABELS = {'dwb_lattice':'Lattice + DWB','dwb_theta':'Theta* + DWB','mppi_lattice':'Lattice + MPPI','mppi_theta':'Theta* + MPPI'}
COLORS = {'dwb_lattice':(142,68,173),'dwb_theta':(28,145,105),'mppi_lattice':(35,101,210),'mppi_theta':(213,111,27)}
W, H = 440, 320
FONT = '/usr/share/fonts/google-noto/NotoSans-Regular.ttf'
if not Path(FONT).exists():
    FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
if not Path(FONT).exists():
    FONT = '/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf'
F14, F17 = ImageFont.truetype(FONT, 14), ImageFont.truetype(FONT, 17)
NAMES = ['straight', 'turn_arrival', 'chair_gap', 'far_heading', 'distant', 'lateral', 'narrow_seat', 'other_room']
parser=argparse.ArgumentParser()
parser.add_argument('--goals',default='0,1,2,3,4,5,6,7')
INDICES=[int(i) for i in parser.parse_args().goals.split(',')]
DATA, BACKGROUNDS, PLAN_BACKGROUNDS = {}, {}, {}

for i in INDICES:
    name=NAMES[i]
    points = []
    for planner in PLANNERS:
        folder = ROOT / planner / f'{i}_{name}'
        result = json.loads((folder / 'result.json').read_text())
        start, end = result['navigation_sim_start_s'], result['navigation_sim_end_s']
        rows = []
        for row in csv.DictReader((folder / 'samples.csv').open()):
            t, x, y, yaw = [float(row[k]) for k in ['sim_s', 'gt_x', 'gt_y', 'gt_yaw']]
            if start - .2 <= t <= end + .2 and all(math.isfinite(v) for v in [t, x, y, yaw]):
                rows.append((t - start, x, y, yaw))
        rows.sort()
        if len(rows) < 2:
            raise ValueError(f'Insufficient recorded poses: {folder}')
        DATA[planner, i] = result, rows, [r[0] for r in rows], end - start
        points.extend(r[1:3] for r in rows)
    goal = result['goal_map']
    points.append(goal[:2])
    x0, x1 = min(p[0] for p in points) - .7, max(p[0] for p in points) + .7
    y0, y1 = min(p[1] for p in points) - .7, max(p[1] for p in points) + .7
    scale = min((W - 28) / (x1 - x0), (H - 107) / (y1 - y0))
    pw, ph = round((x1 - x0) * scale), round((y1 - y0) * scale)
    left, top = (W - pw) // 2, 96 + (H - 105 - ph) // 2
    def tr(x, y, x0=x0, y1=y1, scale=scale, left=left, top=top):
        return left + (x - x0) * scale, top + (y1 - y) * scale
    cfg = yaml.safe_load((ROOT / 'mppi_lattice' / f'{i}_{name}' / 'map.yaml').read_text())
    image = Image.open(ROOT / 'mppi_lattice' / f'{i}_{name}' / 'map.pgm').convert('L')
    mx, my, _ = cfg['origin']; res = cfg['resolution']
    crop = image.crop((round((x0 - mx) / res), round(image.height - (y1 - my) / res),
                       round((x1 - mx) / res), round(image.height - (y0 - my) / res)))
    crop = crop.point(lambda v: 55 if v < 100 else (220 if v < 240 else 249)).convert('RGB').resize((pw, ph), Image.Resampling.NEAREST)
    bg = Image.new('RGB', (W, H), (249, 249, 249)); bg.paste(crop, (left, top))
    draw = ImageDraw.Draw(bg); gx, gy = tr(*goal[:2]); radius = .3 * scale
    draw.ellipse((gx-radius, gy-radius, gx+radius, gy+radius), outline=(20, 150, 90), width=2)
    draw.line((gx-4, gy, gx+4, gy), fill=(20, 150, 90), width=2)
    draw.line((gx, gy-4, gx, gy+4), fill=(20, 150, 90), width=2)
    draw.rectangle((0, 0, W-1, H-1), outline=(180, 180, 180))
    BACKGROUNDS[i] = bg, tr, scale
    for planner in PLANNERS:
        plan_bg=bg.copy();pd=ImageDraw.Draw(plan_bg)
        plans=json.loads((ROOT/planner/f'{i}_{name}'/'plans.json').read_text())
        poses=plans[0]['poses'] if plans else []
        distance=0.
        for a,b in zip(poses,poses[1:]):
            aa,bb=tr(*a[:2]),tr(*b[:2]);length=math.dist(aa,bb);steps=max(1,math.ceil(length))
            for k in range(steps):
                if int(distance+k*length/steps)%10<6:
                    f=k/steps;g=min(1,(k+1)/steps)
                    pd.line((aa[0]+f*(bb[0]-aa[0]),aa[1]+f*(bb[1]-aa[1]),aa[0]+g*(bb[0]-aa[0]),aa[1]+g*(bb[1]-aa[1])),fill=(150,40,150),width=2)
            distance+=length
        PLAN_BACKGROUNDS[planner,i]=plan_bg

def cell(planner, i, time):
    result, rows, times, duration = DATA[planner, i]
    t = min(time, duration)
    j = min(max(1, bisect.bisect_left(times, t)), len(rows)-1)
    a, b = rows[j-1], rows[j]
    f = max(0, min(1, (t-a[0])/(b[0]-a[0]) if b[0] != a[0] else 0))
    x, y = a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2])
    yaw = a[3]+f*math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))
    bg, tr, scale = BACKGROUNDS[i]; image = PLAN_BACKGROUNDS[planner,i].copy(); draw = ImageDraw.Draw(image)
    color = COLORS[planner]
    state = 'RUNNING' if time < duration else {4:'NAV2 SUCCEEDED', 5:'CANCELED', 6:'ABORTED'}.get(result.get('action_status'), 'ENDED')
    if result.get('timeout') and time >= duration: state = 'TIME LIMIT'
    draw.text((10, 5), f'{LABELS[planner]} | {i+1}. {NAMES[i]}', font=F17, fill=color)
    draw.text((10, 28), f'{t:.1f}/{duration:.1f} sim s | {state}', font=F14, fill=(35,35,35))
    draw.text((10, 49), f'GT remaining {math.dist([x,y],result["goal_map"][:2]):.2f}m | 2x sim time', font=F14, fill=(55,55,55))
    if time >= duration:
        draw.text((10,70), f'Final GT {result["position_error_m"]*100:.1f}cm / {result["yaw_error_deg"]:.1f}deg',font=F14,fill=(55,55,55))
    else:
        draw.text((10,70),'GT footprint | dashed: first global plan',font=F14,fill=(75,75,75))
    trail = [tr(r[1], r[2]) for r in rows[:j:4]] + [tr(x,y)]
    if len(trail) > 1: draw.line(trail, fill=color, width=2)
    body = [tr(x+u*math.cos(yaw)-v*math.sin(yaw), y+u*math.sin(yaw)+v*math.cos(yaw))
            for u,v in [(-.238525,-.327373),(-.238525,.327427),(.238475,.327427),(.238475,-.327373)]]
    draw.polygon(body, fill=color, outline=(20,20,20))
    draw.line([tr(x,y),tr(x+.23*math.cos(yaw),y+.23*math.sin(yaw))], fill=(255,255,255),width=2)
    return image

colors = [(i,i,i) for i in range(240)] + [(249,249,249),(255,255,255),*COLORS.values(),(20,150,90),(150,40,150)]
colors += [(0,0,0)]*(256-len(colors)); palette=Image.new('P',(1,1));palette.putpalette([v for c in colors for v in c])

def write_gif(path, keys, cols):
    duration = max(DATA[k][3] for k in keys)
    count = math.ceil(duration/.4)+1
    previous=None
    with path.open('wb') as output:
        for k in range(count):
            canvas=Image.new('RGB',(W*cols,H*math.ceil(len(keys)/cols)))
            for j,key in enumerate(keys): canvas.paste(cell(*key,k*.4),((j%cols)*W,(j//cols)*H))
            if k==0:canvas.save(path.with_name(path.stem+'-start.png'))
            if k==count-1:canvas.save(path.with_name(path.stem+'-final.png'))
            indexed=canvas.quantize(palette=palette,dither=Image.Dither.NONE)
            if previous is None:
                for chunk in GifImagePlugin.getheader(indexed,info={'loop':0,'duration':200})[0]:output.write(chunk)
                box=(0,0,*indexed.size)
            else:box=ImageChops.difference(indexed,previous).getbbox() or (0,0,1,1)
            for chunk in GifImagePlugin.getdata(indexed.crop(box),offset=box[:2],duration=200,disposal=1):output.write(chunk)
            previous=indexed
        output.write(b';')
    image=Image.open(path); playback=0
    for i in range(image.n_frames):image.seek(i);playback+=image.info.get('duration',0)
    record={'file':path.name,'kind':'Actual Gazebo GT replay, not GUI screen recording','speedup':2,'sim_step_s':.4,
            'frame_ms':200,'duration_sim_s':duration,'playback_s':playback/1000,'frames':image.n_frames,
            'dimensions':image.size,'finished_runs_hold_final_pose':True,'failed_and_stationary_segments_preserved':True,'actual_first_global_plan_overlay':True}
    path.with_suffix('.json').write_text(json.dumps(record,indent=2));print(json.dumps(record),flush=True)

for i in INDICES:
    name=NAMES[i]
    write_gif(MEDIA/f'{i+1:02d}-{name}-four-combos-2x.gif',[(p,i) for p in PLANNERS],2)
write_gif(MEDIA/f'controller-planner-4rows-{len(INDICES)}cols-2x.gif',[(p,i) for p in PLANNERS for i in INDICES],len(INDICES))
