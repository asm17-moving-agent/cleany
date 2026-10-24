from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
records=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  cmd=(p/'cmdline').read_bytes().split(b'\0')
  if not cmd or not cmd[0].endswith(b'/nav2_planner/planner_server'):continue
  env=(p/'environ').read_bytes().split(b'\0');domain=next((e.split(b'=',1)[1].decode() for e in env if e.startswith(b'ROS_DOMAIN_ID=')),None)
  if domain not in ['150','151','153','155']:continue
  libs={line.split()[-1] for line in (p/'maps').read_text().splitlines() if 'nav2_smac_planner' in line and line.split()[-1].endswith('.so')}
  records.append({'pid':int(p.name),'domain':domain,'libraries':{f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in libs}})
 except (FileNotFoundError,PermissionError):continue
assert records,'No owned refined planner process to audit'
(root/'provenance/loaded-planner-libraries.json').write_text(json.dumps(records,indent=2));print(json.dumps(records))
