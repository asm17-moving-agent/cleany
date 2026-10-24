from pathlib import Path
import json,subprocess,glob,math,numpy as np
r=Path(__file__).parent;o=r/'actual-path-snapshot';d=json.loads((o/'snapshot.json').read_text());a=np.load(o/'local-costmap.npy');p=json.loads((o/'actual-path.json').read_text())['path'];dense=[x['odom_pose'] for x in json.loads((r/'stall-diagnosis/actual-sync-swept-vs-local.json').read_text())['rows']]
rotation=[[d['robot_pose'][0],d['robot_pose'][1],d['robot_pose'][2]+(p[0][2]-d['robot_pose'][2])*i/30] for i in range(31)]
translation=[[x,y,d['robot_pose'][2]] for x,y,yaw in dense]
paths=[d['robot_pose']]+p+dense+rotation+translation
cpp='''#include <iostream>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
int main(){
'''+f'nav2_costmap_2d::Costmap2D map({a.shape[1]},{a.shape[0]},{d["resolution"]},{d["origin"][0]},{d["origin"][1]},0);\n'
cpp+='unsigned char data[]={'+','.join(map(str,a.ravel()))+'};for(unsigned int y=0;y<'+str(a.shape[0])+';y++)for(unsigned int x=0;x<'+str(a.shape[1])+';x++)map.setCost(x,y,data[y*'+str(a.shape[1])+'+x]);\n'
cpp+='nav2_costmap_2d::Footprint fp;geometry_msgs::msg::Point b;'
for x,y in [[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]:cpp+=f'b.x={x};b.y={y};fp.push_back(b);'
cpp+='nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);'
cpp+='double poses[][3]={'+','.join('{'+','.join(map(str,x))+'}' for x in paths)+'};for(auto &p:poses)std::cout<<checker.footprintCostAtPose(p[0],p[1],p[2],fp)<<"\\n";}'
f=r/'stall-diagnosis/native_check.cpp';f.write_text(cpp);bin=r/'stall-diagnosis/native_check';args=['g++','-std=c++17',str(f),'-o',str(bin)]+['-I/opt/ros/humble/include','-I/usr/include/eigen3']+['-I'+p for p in glob.glob('/opt/ros/humble/include/*') if Path(p).is_dir()]+['-L/opt/ros/humble/lib','-Wl,-rpath,/opt/ros/humble/lib','-lnav2_costmap_2d_core'];compile_result=subprocess.run(args,capture_output=True,text=True);print(compile_result.stderr);assert compile_result.returncode==0
values=[float(x) for x in subprocess.check_output([str(bin)],text=True).splitlines()];result={'checker':'Installed Nav2 FootprintCollisionChecker<Costmap2D*> native library','current_cost':values[0],'sparse_collisions':sum(x==254 for x in values[1:1+len(p)]),'sparse_count':len(p),'sweep_collisions':sum(x==254 for x in values[1+len(p):1+len(p)+len(dense)]),'sweep_count':len(dense),'rotate_in_place_collisions':sum(x==254 for x in values[1+len(p)+len(dense):1+len(p)+len(dense)+len(rotation)]),'rotate_in_place_count':len(rotation),'translate_with_current_yaw_collisions':sum(x==254 for x in values[-len(translation):]),'translate_with_current_yaw_count':len(translation),'rotation_rows':[{'pose':p,'cost':c} for p,c in zip(rotation,values[1+len(p)+len(dense):1+len(p)+len(dense)+len(rotation)])],'translation_rows':[{'pose':p,'cost':c} for p,c in zip(translation,values[-len(translation):])],'dense_collision_poses':[p for p,c in zip(dense,values[1+len(p):]) if c==254]};(r/'stall-diagnosis/native-maneuver-check-result.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if not k.endswith('rows')})
