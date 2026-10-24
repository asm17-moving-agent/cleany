from pathlib import Path
import json,math,subprocess
R=Path(__file__).resolve().parent;folder=R/'recorded-start';summary=[]
for i,name in [(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')]:
 stem=folder/f'{i}-{name}-current';j=json.loads(stem.with_suffix('.input.json').read_text());p=json.loads(stem.with_suffix('.output.json').read_text())['poses'];a,b=p[0],p[-1];n=math.ceil(math.dist(a[:2],b[:2])/.01)
 poses=[[a[0]+(b[0]-a[0])*k/n,a[1]+(b[1]-a[1])*k/n,a[2]] for k in range(n+1)]
 delta=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));nr=max(1,math.ceil(abs(delta)/math.radians(2)))
 poses.extend([[b[0],b[1],a[2]+delta*k/nr] for k in range(nr+1)]);j.update(query_poses=poses,possible_inscribed_cost=0.)
 inp=R/f'{i}-direct.input.json';out=R/f'{i}-direct.output.json';inp.write_text(json.dumps(j));subprocess.run([str(R/'direct_check'),str(inp),str(out)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);d=json.loads(out.read_text());summary.append({'route':name,'route_index':i+1,**d});print(summary[-1],flush=True)
(R/'direct-check-summary.json').write_text(json.dumps({'method':'Synthetic straight XY reference holding initial yaw, then rotate at goal; sampled at <=1cm and <=2deg, checked by installed native GridCollisionChecker using full polygon checking. This is not a planner-generated path or continuous collision guarantee.','rows':summary},indent=2))
