from pathlib import Path
import glob,subprocess
r=Path(__file__).parent
args=['g++','-O2','-std=c++17',str(r/'theta_probe.cpp'),'-o',str(r/'theta_probe'),'-I/opt/ros/humble/include','-I/usr/include/eigen3']+['-I'+p for p in glob.glob('/opt/ros/humble/include/*') if Path(p).is_dir()]+['-L/opt/ros/humble/lib','-L/opt/ros/humble/lib/x86_64-linux-gnu','-Wl,-rpath,/opt/ros/humble/lib','-Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu','-lnav2_smac_planner_lattice','-lnav2_smac_planner','-lnav2_costmap_2d_core','-lrclcpp','-lrclcpp_lifecycle','-lnav2_theta_star_planner','-lompl','-lrcutils']
subprocess.run(args,check=True)
