"""Select measured near/far/narrow/turn/error/guard cases from the full survey."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
from audit import assess


def cumulative_turn(angles: list[float]) -> float:
    return sum(abs(math.atan2(math.sin(b-a),math.cos(b-a))) for a,b in zip(angles,angles[1:]))


def select_cases(rows: list[dict], count: int=6) -> list[dict]:
    roles=[('near','distance_home_m',False),('far','distance_home_m',True),
           ('narrow_mapped_path','minimum_map_center_clearance_m',False),
           ('large_turn','cumulative_yaw_rad',True),('arrival_error','target_gt_error_m',True),
           ('guard_wait','guard_intervention_fraction',True)]
    chosen=[];used=set()
    for role,key,reverse in roles[:count]:
        candidates=[r for r in rows if r['passed'] and r['seat'] not in used and r.get(key) is not None]
        if not candidates:raise RuntimeError(f'No measured unique representative for {role}')
        best=sorted(candidates,key=lambda r:(r[key],r['seat']),reverse=reverse)[0]
        chosen.append({'role':role,**best});used.add(best['seat'])
    return chosen


def latest_attempts(rows: list[dict]) -> list[dict]:
    """Use explicit baseline/correction order while retaining all attempts."""
    latest={}
    for row in rows:latest[row['seat']]=row
    return list(latest.values())


class MapClearance:
    def __init__(self, map_file: Path):
        self.meta=yaml.safe_load(map_file.read_text());image=np.asarray(Image.open(map_file.parent/self.meta['image']).convert('L'))
        occupancy=image/255 if self.meta['negate'] else (255-image)/255
        known_free=occupancy<self.meta['free_thresh']
        self.distance=distance_transform_edt(known_free)*self.meta['resolution']
        self.height,self.width=image.shape

    def at(self,x: float,y: float) -> float:
        ox,oy,angle=self.meta['origin'];c,s=math.cos(angle),math.sin(angle)
        ix=math.floor((c*(x-ox)+s*(y-oy))/self.meta['resolution'])
        iy=math.floor((-s*(x-ox)+c*(y-oy))/self.meta['resolution'])
        if not 0<=ix<self.width or not 0<=iy<self.height:return 0.
        return float(self.distance[self.height-1-iy,ix])


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    parser.add_argument('--corrections',type=Path,help='Completed correction jobs, in authoritative queue order')
    args=parser.parse_args()
    directory=args.directory;fixture=directory/'fixture';targets=yaml.safe_load((fixture/'targets.yaml').read_text())
    clearance=MapClearance(fixture/'facility_integration.yaml');home=targets['home'];rows=[]
    jobs=json.loads((directory/'survey_jobs.json').read_text())
    if args.corrections:jobs+=json.loads(args.corrections.read_text())
    for job in jobs:
        for worker in sorted((directory/job['name']).glob('worker-*')):
            if not worker.is_dir():continue
            result=json.loads((worker/'result.json').read_text());shutdown=json.loads((worker/'shutdown.json').read_text())
            verdict=assess(result,shutdown);seat=result['world_manifest']['seat'];goal=targets['seats'][seat]
            trace=[json.loads(line) for line in (worker/'trace.jsonl').read_text().splitlines()]
            moving=[t for t in trace if t['snapshot']['state'] in ('NAVIGATE_TO_TARGET','RETURN_HOME') and 'ground_truth' in t['latest']]
            positions=[t['latest']['ground_truth'] for t in moving]
            decisions=[t['latest'].get('guard',{}).get('decision') for t in moving]
            mission=result.get('missions',[{}])[0] if result.get('missions') else {}
            pose=mission.get('target_ground_truth')
            rows.append({'seat':seat,'worker':str(worker),'passed':verdict['passed'],
              'distance_home_m':math.hypot(goal[0]-home[0],goal[1]-home[1]),
              'minimum_map_center_clearance_m':min((clearance.at(p['x'],p['y']) for p in positions),default=None),
              'cumulative_yaw_rad':cumulative_turn([p['yaw'] for p in positions]),
              'target_gt_error_m':math.hypot(pose['x']-goal[0],pose['y']-goal[1]) if pose else None,
              'guard_intervention_fraction':sum(d not in ('PASS','STOP_COMMAND') for d in decisions)/len(decisions) if decisions else None,
              'wall_seconds':mission.get('wall_seconds'),'rtf':mission.get('measured_rtf')})
    attempts=rows;rows=latest_attempts(attempts)
    inventory=json.loads((fixture/'seat_inventory.json').read_text())
    if {r['seat'] for r in rows}!=set(inventory['seats']):raise RuntimeError('Representative input lacks full seat coverage')
    selected=select_cases(rows)
    output={'selection':'six unique passing routes from complete survey, ranked by observed route descriptors',
      'map_clearance_scope':'sampled base centre to known mapped obstacle/unknown; excludes unrepresented live chairs; not footprint/contact certification',
      'selected':selected,'all_seats':rows,'attempts':attempts,
      'failed_seats':[r['seat'] for r in rows if not r['passed']]}
    (directory/'representatives.json').write_text(json.dumps(output,indent=2))
    print([(r['role'],r['seat']) for r in selected])


if __name__=='__main__':main()
