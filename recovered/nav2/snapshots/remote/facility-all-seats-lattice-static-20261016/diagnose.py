from pathlib import Path
import json,subprocess,math,os
import numpy as np
from PIL import Image
from scipy.ndimage import label
R=Path(__file__).resolve().parent;rows=json.loads((R/'results.json').read_text());goals=json.loads((R/'goals.json').read_text());fail=[x for x in rows if not x['path_generated']];first=R/'paths/seat-a1-01';meta=json.loads((first/'costmap-metadata.json').read_text());a=np.array(Image.open(R/'maps/all-chairs-present/map.pgm'))[::-1];components,_=label(a>200,np.ones((3,3)));ox,oy=meta['origin'];res=meta['resolution']
def cell(p):return int((p[1]-oy)/res),int((p[0]-ox)/res)
homecomponent=int(components[cell(goals['home'])]);out=[]
for row in fail:
 if row['zone']=='D-hub':continue
 folder=R/'paths'/row['seat'];inp={'nx':meta['size'][0],'ny':meta['size'][1],'resolution':res,'origin':[ox,oy,0],'binary':str(first/'costmap.bin'),'footprint':goals['footprint'],'mode':'snapshot','query_poses':[goals['home'],row['goal']]};p=folder/'endpoint-native.input.json';p.write_text(json.dumps(inp));q=folder/'endpoint-native.output.json'
 with (folder/'endpoint-native.log').open('w') as f:subprocess.run(['/home/donglee/workspace/cleany/lattice-straight-improvement-20261016/static-only-20261016/direct_check',str(p),str(q)],check=True,stdout=f,stderr=subprocess.STDOUT,env={**os.environ,'ROS_DOMAIN_ID':'185','ROS_LOCALHOST_ONLY':'1'})
 result=json.loads(q.read_text());component=int(components[cell(row['goal'])]);out.append({'seat':row['seat'],'start_footprint_collision':0 in result['collision_indices'],'goal_footprint_collision':1 in result['collision_indices'],'raw_map_point_connected_to_home_8way':component==homecomponent and component!=0,'planner_log':row['planner_diagnostics'],'meaning':'Point connectivity ignores footprint/orientation. Endpoint checks use existing native checker, 72-bin footprint cache. No alternative goal or obstacle removal performed.'})
(R/'failure-diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out),flush=True)
# Fix and enrich provenance without changing any input.
p=R/'provenance.json';d=json.loads(p.read_text());groups=[]
for g in goals['goals']:
 for group in groups:
  h=group[0]
  if math.dist(g['goal'][:2],h['goal'][:2])<1e-5 and abs(math.atan2(math.sin(g['goal'][2]-h['goal'][2]),math.cos(g['goal'][2]-h['goal'][2])))<1e-5:group.append(g);break
 else:groups.append([g])
d['duplicate_goal_groups']=[[x['seat'] for x in group] for group in groups if len(group)>1];d['distinct_geometric_goal_count']=len(groups);d['selected_zones']=['D-hub','A1','A2','A3','A4','M1','M2','M3'];p.write_text(json.dumps(d,indent=2))
