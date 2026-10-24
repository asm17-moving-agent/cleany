#!/usr/bin/env python3
from pathlib import Path
from types import SimpleNamespace as NS
import io,json,importlib.util,math
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('facility_runner',ROOT/'run_facility.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
probe=NS(contact_count=0,contact_file=io.StringIO(),now=lambda:12.04)
contacts=[NS(collision1=NS(name='cleany_mecanum::base::collision',id=1),collision2=NS(name=n,id=2),depths=[.001],positions=[NS(x=1.,y=2.,z=3.)]) for n in ['ground_plane::link::collision','cleany_mecanum::wheel::collision','office_chair_28::leg::collision']]
msg=NS(header=NS(stamp=NS(sec=12,nanosec=20000000)),contacts=contacts)
m.Probe.contact(probe,msg,'/evaluation/contacts/base_link');saved=[json.loads(x) for x in probe.contact_file.getvalue().splitlines()]
assert probe.contact_count==3 and len(saved)==3
assert saved[2]['collision2']['name']=='office_chair_28::leg::collision' and saved[2]['depths']==[.001] and saved[2]['positions']==[[1.,2.,3.]]
assert saved[0]['stamp_s']==12.02 and saved[0]['received_sim_s']==12.04
vel=[[i*.05,0.,0.,.1] for i in range(80)];s=m.summarize_velocity(vel,0,4)
assert s['stationary_s']==0 and s['motion_start_count']==0
assert abs(max(map(abs,m.wheel_rates([.3175,0,0])))-5)<1e-10
(ROOT/'tool_validation.json').write_text(json.dumps({'contact_stream_keeps_ground_self_environment_and_depth_position_stamp':True,'rotation_only_is_not_stationary':True,'pure_straight_3175_maps_to_wheel5':True,'compile_syntax':True},indent=2));print('tool boundary checks passed')
