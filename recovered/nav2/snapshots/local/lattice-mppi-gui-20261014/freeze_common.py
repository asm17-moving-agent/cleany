#!/usr/bin/env python3
"""Read-only snapshot after shared READY; fail on a changing or incomplete lock."""
from pathlib import Path
import shutil,json,hashlib
ROOT=Path(__file__).resolve().parent
COMMON=Path('/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/common')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert (COMMON/'READY').is_file(),'READY absent; do not launch campaign'
 ready=sha(COMMON/'READY');locked_sha=sha(COMMON/'goals.locked.json')
 lock=json.loads((COMMON/'goals.locked.json').read_text());assert len(lock['goals'])==8
 assert [g['index'] for g in lock['goals']]==list(range(8))
 target=ROOT/'common_snapshot';target.mkdir(exist_ok=False)
 shutil.copytree(COMMON/'fixture',target/'fixture')
 for name in ['READY','goals.locked.json','plan.proposed.json','arrival_tolerance.yaml','world.full.template.sdf','preflight.static.json','preflight_planner.yaml','preflight_planner.py','prepare_common.py','materialize_common.py','planner_params.yaml','omni_lattice_2p5cm.json','CONSUMER_REQUIREMENTS.md','sha256.locked.json','preflight.planner.json']:
  if (COMMON/name).exists():shutil.copyfile(COMMON/name,target/name)
 import yaml
 planner_file=target/'preflight_planner.yaml'
 if planner_file.exists():
  planner=yaml.safe_load(planner_file.read_text())['planner_server']['ros__parameters']['GridBased']
  if planner.get('lattice_filepath','').startswith(str(COMMON)+'/'):
   src=Path(planner['lattice_filepath']);rel=src.relative_to(COMMON);dst=target/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
 for name in lock.get('preflight_evidence',[]):
  src=COMMON/name
  if src.is_file():
   dst=target/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
 for g in lock['goals']:
  for key in ['world','map']:
   path=Path(g[key]);assert not path.is_absolute() and '..' not in path.parts
   src=COMMON/path;dst=target/path;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
  import yaml
  src=COMMON/g['map'];meta=yaml.safe_load(src.read_text());image=Path(meta['image']);assert not image.is_absolute() and '..' not in image.parts
  shutil.copyfile(src.parent/image,(target/g['map']).parent/image)
  if (src.parent/'map-provenance.json').exists():shutil.copyfile(src.parent/'map-provenance.json',(target/g['map']).parent/'map-provenance.json')
  if g.get('planner_path') and (COMMON/g['planner_path']).is_file():
   dst=target/g['planner_path'];dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(COMMON/g['planner_path'],dst)
  assert sha(target/g['world'])==g['world_sha256']
  assert sha((target/g['map']).parent/image)==g['map_sha256']
 assert sha(COMMON/'READY')==ready and sha(COMMON/'goals.locked.json')==locked_sha
 hashes={str(p.relative_to(target)):sha(p) for p in target.rglob('*') if p.is_file()}
 expected=json.loads((target/'sha256.locked.json').read_text()) if (target/'sha256.locked.json').exists() else {}
 for name,digest in hashes.items():
  if name in expected:assert digest==expected[name],name
 (ROOT/'common_snapshot_manifest.json').write_text(json.dumps({'common_source':str(COMMON),'ready_sha256':ready,'locked_sha256':locked_sha,'files':hashes,'common_modified':False},indent=2))
 for p in target.rglob('*'):p.chmod(0o755 if p.is_dir() else 0o644)
 target.chmod(0o755)
 provenance=json.loads((ROOT/'provenance.json').read_text());provenance.update(common_setup_changes=lock.get('common_setup_changes'),common_locked_sha256=locked_sha,common_ready_sha256=ready,campaign_started=True);(ROOT/'provenance.json').write_text(json.dumps(provenance,indent=2))
 (ROOT/'STATUS.json').write_text(json.dumps({'phase':'launching_locked_8goal_campaign','campaign_started':True,'navigation_goals_sent':0,'locked_sha256':locked_sha,'output':str(ROOT)},indent=2))
 print('Locked 8-goal snapshot ready',target)
if __name__=='__main__':main()
