from pathlib import Path
import glob,subprocess
r=Path(__file__).parent
args=['g++','-shared','-fPIC','-std=c++17',str(r/'footprint_api.cpp'),'-o',str(r/'libfootprint.so'),'-I/opt/ros/humble/include','-I/usr/include/eigen3']+['-I'+p for p in glob.glob('/opt/ros/humble/include/*') if Path(p).is_dir()]+['-L/opt/ros/humble/lib','-Wl,-rpath,/opt/ros/humble/lib','-lnav2_costmap_2d_core']
subprocess.run(args,check=True)
