from pathlib import Path,PurePosixPath
import subprocess,tarfile,shutil,json,time
root=Path('/home/donglee/workspace/cleany/cleany-data/recovery/nav2-tuning-20261024')
out=root/'selected';out.mkdir(parents=True,exist_ok=False)
selected=set(Path('/tmp/cleany-nav2-recovery-20261024-files.txt').read_text().splitlines());remaining=set(selected)
archive='/home/donglee/workspace/cleany/cleany-data/archives/workspace-cleanup/20261017-182827/workspace-data.tar.zst'
p=subprocess.Popen(['zstd','-dc',archive],stdout=subprocess.PIPE)
start=time.time();count=0
try:
 with tarfile.open(fileobj=p.stdout,mode='r|') as t:
  for m in t:
   name=m.name.removeprefix('./')
   if name not in remaining:continue
   assert m.isfile() and '..' not in PurePosixPath(name).parts and not name.startswith('/')
   dest=out/name;dest.parent.mkdir(parents=True,exist_ok=True)
   with t.extractfile(m) as src,dest.open('wb') as dst:shutil.copyfileobj(src,dst)
   dest.chmod(m.mode & 0o777);remaining.remove(name);count+=1
   if count%1000==0:print(json.dumps({'restored':count,'elapsed_s':time.time()-start}),flush=True)
   if not remaining:break
finally:
 p.terminate();p.wait()
assert not remaining, sorted(remaining)[:10]
subprocess.run(['tar','--zstd','-cf',str(root/'selected-code.tar.zst'),'-C',str(out),'.'],check=True)
print(json.dumps({'complete':True,'count':count,'elapsed_s':time.time()-start,'archive':str(root/'selected-code.tar.zst')}),flush=True)
