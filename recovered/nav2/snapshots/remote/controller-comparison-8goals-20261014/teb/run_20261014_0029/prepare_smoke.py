"""Offline proposed-template preparation only. No Gazebo launch or goal issue."""
import json,sys
from pathlib import Path
import runner
root=Path(__file__).resolve().parent
plan=json.loads((runner.COMMON/'plan.proposed.json').read_text())
goals=plan['goals']
for g in goals:
 g['world']='fixture/world.template.sdf';g['map_yaml']='fixture/facility_integration.yaml'
 # Chair removal belongs to DWB's canonical generation, not this offline check.
 g['remove_chair']=None
runner.LOCKED=plan
out=root/'preparation_only/0_template'
out.mkdir(parents=True,exist_ok=False)
scenario=runner.setup(out,'teb',1.)
print(json.dumps({'scope':'offline preparation; not a navigation measurement','world':scenario['world'],'initial_pose':scenario['initial_pose'],'contacts':len(scenario['contact_topics']),'map_sha256':scenario['map_sha256']}))
