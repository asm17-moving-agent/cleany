from pathlib import Path
import os,signal,json
partition=b'cleany_comparison_e556ba5392a34938a74a4ac686c7c507'
remaining=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid():continue
 try:
  env=(p/'environ').read_bytes().split(b'\0')
  owned=any(e in [b'IGN_PARTITION='+partition,b'GZ_PARTITION='+partition] for e in env)
  stat=(p/'stat').read_text();tail=stat[stat.rfind(')')+2:].split()
  if owned or int(tail[2])==83286:
   remaining.append({'pid':int(p.name),'name':stat[stat.find('(')+1:stat.rfind(')')],'group':int(tail[2])})
   os.kill(int(p.name),signal.SIGTERM)
 except (OSError,ValueError):pass
print(json.dumps(remaining))
