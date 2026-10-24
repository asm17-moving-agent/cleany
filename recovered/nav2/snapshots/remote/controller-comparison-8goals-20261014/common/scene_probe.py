from pathlib import Path
import json,os,subprocess,time,signal,uuid,xml.etree.ElementTree as ET
from prepare_common import ROOT,materialize,FIX
out=ROOT/'scene_preflight';out.mkdir(exist_ok=False);materialize(ROOT/'goals/0_straight/world.canonical.sdf',out/'world.sdf');worldname=ET.parse(out/'world.sdf').find('world').get('name');partition='dwb_8goal_scene_'+uuid.uuid4().hex;os.environ.update(IGN_PARTITION=partition,GZ_PARTITION=partition,IGN_DISCOVERY_MSG_PORT='22551',IGN_DISCOVERY_SRV_PORT='22552',IGN_IP='127.0.0.1',GZ_IP='127.0.0.1');log=(out/'server.log').open('w');proc=subprocess.Popen(['ign','gazebo','-s','-r','--headless-rendering',str(out/'world.sdf')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(out/'session.json').write_text(json.dumps({'partition':partition,'server_pid':proc.pid,'supervisor_pid':os.getpid(),'world_name':worldname}));response=None
try:
 for _ in range(20):
  r=subprocess.run(['ign','service','-s','/world/'+worldname+'/scene/info','--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene','--timeout','3000','--req',''],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
  if r.returncode==0 and 'model {' in r.stdout:response=r.stdout;break
  if proc.poll() is not None:raise RuntimeError('Gazebo server exited')
  time.sleep(1)
 if response is None:raise RuntimeError('SceneInfo unavailable')
 (out/'scene.info.pbtxt').write_text(response);(out/'scene.request.stderr').write_text(r.stderr);time.sleep(2)
finally:
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
text=(out/'server.log').read_text();fail=[line for line in text.splitlines() if any(x in line.lower() for x in ['unable to find file','unable to find uri','failed to load mesh','could not find resource','unable to load mesh'])];v={'actual_scene_info_received':response is not None,'fixture_or_repo_uri_remaining_in_materialized_world':any(x in (out/'world.sdf').read_text() for x in ['fixture://','repo://']),'fixture_or_repo_uri_remaining_in_scene':any(x in (response or '') for x in ['fixture://','repo://']),'missing_mesh_errors':fail,'scene_model_name_count':(response or '').count('model {'),'scope':'headless actual Gazebo server scene metadata and log; no GUI or navigation; not physical proof','server_stopped':proc.poll() is not None};(out/'verification.json').write_text(json.dumps(v,indent=2));print(json.dumps(v))
