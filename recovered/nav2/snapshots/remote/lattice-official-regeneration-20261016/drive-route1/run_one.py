"""Paired planner campaign using the preserved, waypoint-free MPPI runtime."""
from pathlib import Path
import argparse
import hashlib
import inspect
import json
import os
import sys
import yaml

HERE = Path(__file__).resolve().parent
BASE = Path('/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment')
sys.path.insert(0, str(BASE))
import run_experiment as experiment
import prepare_experiment as preparation

R = experiment.R
parser = argparse.ArgumentParser()
parser.add_argument('--planner', choices=['lattice', 'theta'], required=True)
parser.add_argument('--goal', type=int, required=True)
parser.add_argument('--variant', choices=['baseline','improved','refined'], required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
plugin = {'lattice': 'nav2_smac_planner/SmacPlannerLattice',
          'theta': 'nav2_theta_star_planner/ThetaStarPlanner'}[args.planner]

def setup(out, candidate, rtf, goal_index):
    scenario = preparation.setup(out, candidate, rtf, goal_index)
    params = yaml.safe_load((out / 'parameters.yaml').read_text())
    if args.planner == 'theta':
        params['planner_server']['ros__parameters']['GridBased'] = {
            'plugin': plugin, 'allow_unknown': False,
        }
    if args.variant != 'baseline':
        params['planner_server']['ros__parameters']['GridBased'].update(yaml.safe_load((HERE/('refined-planner-override.yaml' if args.variant=='refined' else 'planner-override.yaml')).read_text()))
    assert params['planner_server']['ros__parameters']['GridBased']['plugin'] == plugin
    assert not scenario['approach_stages'], 'Manual waypoints must stay disabled'
    (out / 'parameters.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    (out / 'planner_params.yaml').write_text(yaml.safe_dump({'planner_server': params['planner_server']}))
    params['controller_server']['ros__parameters']['FollowPath']['benchmark_timing_file'] = str(out / 'compute_timing.csv')
    (out / 'parameters.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    (out / 'config/comparison_motion.yaml').write_text(yaml.safe_dump({'controller_server': params['controller_server']}))
    # Persist the complete comparison contract before sending a navigation goal.
    contract = {'variant': args.variant, 'planner': args.planner, 'plugin': plugin, 'manual_waypoints': False,
                'mppi_install': os.environ['MPPI_INSTALL'],
                'controller': params['controller_server'],
                'bt_sha256': hashlib.sha256((out / 'behavior.xml').read_bytes()).hexdigest(),
                'map_sha256': hashlib.sha256((out / 'map.pgm').read_bytes()).hexdigest(),
                'world_sha256': scenario['canonical_world_sha256'],
                'requested_rtf': rtf}
    (out / 'comparison_contract.json').write_text(json.dumps(contract, indent=2))
    return scenario

R.setup = setup
source = inspect.getsource(R.run)
old = "assert result['verified_planner']=={'GridBased.plugin':'nav2_smac_planner/SmacPlannerLattice'}"
assert old in source
source = source.replace(old, "assert result['verified_planner']=={'GridBased.plugin':os.environ['COMPARISON_PLANNER_PLUGIN']}")
dump_line = "subprocess.run(['ros2','param','dump','/controller_server','--no-daemon','--spin-time','3','--output-dir',str(out)],check=True)"
assert dump_line in source
service_dump = """def _flatten_parameters(value, prefix=''):
   values={}
   for key,item in value.items():
    if isinstance(item,dict):values.update(_flatten_parameters(item,prefix+key+'.'))
    else:values[prefix+key]=item
   return values
  _names=['FollowPath.'+name for name in _flatten_parameters(f)]
  _actual=node.parameters('/controller_server',_names)
  _follow={}
  for name,value in _actual.items():
   _cursor=_follow;_parts=name.removeprefix('FollowPath.').split('.')
   for part in _parts[:-1]:_cursor=_cursor.setdefault(part,{})
   _cursor[_parts[-1]]=value
  (out/'controller_server.yaml').write_text(yaml.safe_dump({'/controller_server':{'ros__parameters':{'FollowPath':_follow}}}))"""
source = source.replace(dump_line, service_dump)
offset = 0 if args.planner == 'lattice' else 200
source = source.replace('23151+2*goal_index', f'{28151 + offset}+2*goal_index')
source = source.replace('23152+2*goal_index', f'{28152 + offset}+2*goal_index')
exec(compile(source, str(HERE / 'paired_run_function.py'), 'exec'), R.__dict__)
(HERE / f'runner_{args.planner}_sha256.txt').write_text(hashlib.sha256(source.encode()).hexdigest())
os.environ['COMPARISON_PLANNER_PLUGIN'] = plugin
os.environ['MPPI_TRACE_FILE'] = str(args.output.resolve() / 'candidate_costs.csv')
domain = {'baseline':180,'improved':181,'refined':182}[args.variant] + args.goal
result = R.run(args.output.resolve(), 'mppi', args.goal, 2., domain)
print(json.dumps({'planner': args.planner, **{k: result.get(k) for k in
    ['goal_name', 'navigation_goal_sent', 'action_status', 'position_error_m', 'yaw_error_deg', 'error']}}), flush=True)
raise SystemExit(0 if result.get('navigation_goal_sent') else 2)
