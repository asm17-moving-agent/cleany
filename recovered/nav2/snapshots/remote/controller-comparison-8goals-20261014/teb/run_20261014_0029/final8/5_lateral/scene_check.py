"""Save actual Gazebo scene and check all canonical model identities and meshes."""
import json,re,subprocess
from pathlib import Path
from urllib.parse import unquote,urlparse
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent

def flatten_models(models):
 for m in models:
  yield m
  yield from flatten_models(m.get('model',[]))
def values(data,key):
 if isinstance(data,dict):
  for k,v in data.items():
   if k==key:yield v
   yield from values(v,key)
 elif isinstance(data,list):
  for v in data:yield from values(v,key)

def verify(out,scenario):
 cmd=['ign','service','-s','/world/'+scenario['world']+'/scene/info','--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene','--timeout','10000','--req','']
 response=subprocess.run(cmd,capture_output=True,text=True,timeout=15)
 (out/'scene.raw.pbtxt').write_text(response.stdout)
 (out/'scene.service.log').write_text(response.stderr)
 if response.returncode:raise RuntimeError('Scene service failed '+response.stderr[:1000])
 parsed=subprocess.run([str(ROOT/'scene_to_json')],input=response.stdout,capture_output=True,text=True,timeout=10)
 if parsed.returncode:raise RuntimeError('Scene protobuf parse failed '+parsed.stderr[:1000])
 data=json.loads(parsed.stdout);(out/'scene.json').write_text(json.dumps(data,indent=2))
 models=list(flatten_models(data.get('model',[])));names={x['name'] for x in models}
 expected=set(scenario['expected_scene_models'])
 missing=sorted(expected-names)
 removed=scenario['destination_removed_chair']
 # Scene protobuf contains mesh filenames emitted by the running server.
 scene_files=set(str(x) for x in values(data,'filename'))
 sdf=ET.parse(out/'world.sdf')
 expected_visuals={x.text for x in sdf.findall('.//visual/geometry/mesh/uri')}
 def normalize(value):
  return str(Path(unquote(urlparse(value).path)).resolve()) if value.startswith('file://') else str(Path(unquote(value)).resolve())
 actual={normalize(v) for v in scene_files}
 absent_mesh=sorted(x for x in expected_visuals if normalize(x) not in actual)
 log=(out/'launch.log').read_text(errors='replace')
 errors=[line for line in log.splitlines() if re.search(r'(unable|failed|error|could not|cannot).*(mesh|\.dae|\.obj|\.stl)|(mesh|\.dae|\.obj|\.stl).*(unable|failed|error|could not|cannot)',line,re.I)]
 audit={'actual_scene_model_count':len(names),'expected_canonical_model_count':len(expected),'missing_models':missing,'removed_chair':removed,'removed_chair_absent':removed is None or removed not in names,'all_other_canonical_models_present':not missing,'expected_unique_visual_meshes':len(expected_visuals),'actual_unique_scene_meshes':len(actual),'missing_visual_meshes':absent_mesh,'server_mesh_load_error_lines':errors,'mesh_files':sorted(scene_files),'scope':'Actual scene service + local URI/hash checks + server load diagnostics; no GUI was opened.'}
 (out/'scene_validation.json').write_text(json.dumps(audit,indent=2))
 if missing or absent_mesh or errors or not audit['removed_chair_absent']:raise RuntimeError('Actual scene validation failed; see scene_validation.json')
 return audit
