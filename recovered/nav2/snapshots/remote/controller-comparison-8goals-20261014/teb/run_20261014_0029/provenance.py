import hashlib,json,platform,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PRIOR=Path('/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01')
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(args):return subprocess.check_output(args,text=True).strip()
source=Path('/home/donglee/workspace/cleany/controller-optimization-20261013/teb/source')
info={'host':platform.node(),'platform':platform.platform(),'client_date':'2026-10-14','client_timezone':'Asia/Seoul','cpu':run(['lscpu']),'container':'cleany-ros-humble-cdi','gpu':run(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader']),'source_head':run(['git','-C',str(source),'rev-parse','HEAD']),'source_status':run(['git','-C',str(source),'status','--short']),'teb_head':'630a22e88dc9fd45be726a762edbb5b776bef231','prior_environment_reference':str(PRIOR/'environment.json'),'packages':run(['dpkg-query','-W','libignition-gazebo6','ros-humble-nav2-controller']),'permission_context':{'sandbox_mode':'danger-full-access','approval_policy':'never'},'domain':227,'localhost':True,'rtf_target':1,'physics_step':'Canonical world; record per goal','cross_host_compute_ranking_valid':False,'environment_repair':json.loads((ROOT/'environment-repair/repair.json').read_text()),'scripts_sha256':{p.name:digest(p) for p in ROOT.glob('*.py')},'wheel_limiter_prior_hash_match':digest(ROOT/'wheel_limiter.py')==digest(PRIOR/'wheel_limiter.py')}
(ROOT/'environment.json').write_text(json.dumps(info,indent=2))
for name in ['compatibility.patch','diagnostics.patch','final-teb-source.tar.gz','build-logs.tar.gz','original_compile_flags.txt','dedicated_compile_flags.txt']:
 import shutil
 if not (ROOT/name).exists():shutil.copy2(PRIOR/name,ROOT/name)
print(json.dumps({k:info[k] for k in ['host','gpu','source_head','source_status','wheel_limiter_prior_hash_match']}))
