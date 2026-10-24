from pathlib import Path
import json,yaml,shutil,hashlib
R=Path(__file__).resolve().parent
S=R.parent/'facility-all-seats-lattice-static-20261016'
REMOTE=Path('/home/donglee/workspace/cleany/facility-planners-static-20261016')
common=dict(allow_unknown=False,tolerance=0.0)
plugins={
 'navfn':dict(common,plugin='nav2_navfn_planner/NavfnPlanner',use_astar=False,use_final_approach_orientation=False),
 'smac2d':dict(common,plugin='nav2_smac_planner/SmacPlanner2D',max_planning_time=80.0,max_iterations=-1,max_on_approach_iterations=1000,cost_travel_multiplier=2.0,use_final_approach_orientation=False,smooth_path=False),
 'hybrid':dict(common,plugin='nav2_smac_planner/SmacPlannerHybrid',max_planning_time=80.0,max_iterations=-1,max_on_approach_iterations=1000,motion_model_for_search='REEDS_SHEPP',minimum_turning_radius=.1,angle_quantization_bins=72,lookup_table_size=20.0,analytic_expansion_ratio=3.5,analytic_expansion_max_length=3.0,cache_obstacle_heuristic=False,smooth_path=False),
 'theta':dict(plugin='nav2_theta_star_planner/ThetaStarPlanner',allow_unknown=False,w_traversal_cost=5.0,how_many_corners=8),
}
for n,(name,params) in enumerate(plugins.items()):
 folder=R/name;folder.mkdir(exist_ok=True)
 doc=yaml.safe_load((S/'planner.yaml').read_text());doc['planner_server']['ros__parameters']['GridBased']=params
 (folder/'planner.yaml').write_text(yaml.safe_dump(doc));(folder/'planner.launch.py').write_text((S/'planner.launch.py').read_text());shutil.copy2(S/'goals.json',folder/'goals.json')
 s=(S/'run_plans.py').read_text().replace("ROS_DOMAIN_ID='184'",f"ROS_DOMAIN_ID='{190+n}'").replace("'domain':184",f"'domain':{190+n}")
 old="live={'planner':n.params('/planner_server',['GridBased.plugin','GridBased.lattice_filepath','GridBased.tolerance','GridBased.max_planning_time','GridBased.rotation_penalty','GridBased.cost_penalty','GridBased.non_straight_penalty','GridBased.lookup_table_size','GridBased.smooth_path','GridBased.allow_reverse_expansion'])"
 keys=['GridBased.'+k for k in params]
 new="live={'planner':n.params('/planner_server',"+repr(keys)+")"
 assert old in s;s=s.replace(old,new).replace("assert live['planner']['GridBased.plugin']=='nav2_smac_planner/SmacPlannerLattice'",f"assert live['planner']['GridBased.plugin']=={params['plugin']!r}")
 s=s.replace("'variant'", "'variant'")
 (folder/'run_plans.py').write_text(s)
(R/'campaign-config.json').write_text(json.dumps({'planners':plugins,'sources':str(S),'domain_ids':dict(zip(plugins,range(190,194))),'scope':'82 goals each, existing canonical maps and fixed targets reused byte-for-byte; no Gazebo/controller started','rationale':{'hybrid':'Reeds-Shepp preserves forward/reverse; radius0.1m matches new Lattice metadata. 72 heading bins and analytic expansions are native Hybrid settings, not Lattice primitives.','theta':'w_traversal_cost5 preserved from successful recent Theta experiment.','navfn':'Dijkstra mode, exact-goal tolerance0.','smac2d':'Cost multiplier2, smoothing disabled.','goal_handling':'Native positional/heading discretization differs. Theta has no tolerance parameter; exact requested endpoint is evaluated separately. No common fake timeout or physical feasibility guarantee.'}},indent=2))
print(R)
