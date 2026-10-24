"""Materialize portable common inputs without NumPy or repository dependencies."""
from pathlib import Path
import argparse,hashlib,json,shutil
ROOT=Path(__file__).resolve().parent

def resolve_text(text,common=ROOT):
 return text.replace('fixture://',str(Path(common).resolve()/'fixture')+'/').replace('common://',str(Path(common).resolve())+'/')
def materialize_goal(index,output,common=ROOT):
 common=Path(common);lock=json.loads((common/'goals.locked.json').read_text());g=lock['goals'][index];output=Path(output);output.mkdir(parents=True,exist_ok=True)
 for key,name in [('world','world.sdf'),('map','map.yaml')]:
  source=common/g[key];text=resolve_text(source.read_text(),common);(output/name).write_text(text)
 shutil.copy2((common/g['map']).parent/'map.pgm',output/'map.pgm');(output/'nav2_common.yaml').write_text(resolve_text((common/'fixture/config/nav2_amcl.yaml').read_text(),common));(output/'planner_params.yaml').write_text(resolve_text((common/'planner_params.yaml').read_text(),common));return g
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--goal-index',type=int,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--common',type=Path,default=ROOT);a=p.parse_args();print(json.dumps(materialize_goal(a.goal_index,a.output,a.common),indent=2))
