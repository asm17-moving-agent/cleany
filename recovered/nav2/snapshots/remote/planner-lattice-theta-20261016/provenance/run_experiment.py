"""Owned Gazebo experiment using preserved laptop measurement definitions."""
import sys,os,json,time,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'laptop-runtime'))
import run_facility_gui as R
from prepare_experiment import setup,LOCK,COMMON
from std_srvs.srv import Empty
R.setup=setup;R.SNAPSHOT=COMMON
R.GOALS=[(g['id'],g['pose_map_world']) for g in LOCK['goals']]
R.GOAL_RECORDS=LOCK['goals']
class Probe(R.Probe):
 def __init__(self,out,scenario):
  super().__init__(out,scenario)
  self.nomotion=self.create_client(Empty,'/request_nomotion_update');self.pending=None
  self.create_timer(1.,self.update_amcl)
  self.contact_counts={};self.last_floor={}
 def update_amcl(self):
  if self.nomotion.service_is_ready() and (self.pending is None or self.pending.done()):self.pending=self.nomotion.call_async(Empty.Request())
 def contact(self,m,t):
  self.contact_counts[t]=self.contact_counts.get(t,0)+1
  # Preserve all non-floor/self contacts, sample positive-control floor data 1Hz.
  for c in m.contacts:
   names=[c.collision1.name,c.collision2.name]
   if 'ground_plane' in ' '.join(names) or all(n.startswith('cleany_mecanum::') for n in names):
    if self.now()-self.last_floor.get(t,-100)<1:return
    self.last_floor[t]=self.now()
  super().contact(m,t)
 def save(self):
  super().save();(self.output/'contact-topic-counts.json').write_text(json.dumps(self.contact_counts))
R.Probe=Probe
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--goal',type=int,default=7);p.add_argument('--domain',type=int,default=211);a=p.parse_args()
 a.output=a.output.resolve();os.environ['MPPI_TRACE_FILE']=str(a.output/'candidate_costs.csv')
 r=R.run(a.output,'mppi',a.goal,1.,a.domain)
 print(json.dumps({k:r.get(k) for k in ['completed','error','action_status','final_truth','navigation_sim_s','position_error_m','yaw_error_deg']}),flush=True)
