#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/feat-runtime-integration/install/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/install-navigation/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/measurement_ws/install/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-20261013/teb_ws/install/setup.bash
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export IGN_PARTITION=cleany_facility_gui_teb_b17f10d4bf4a42b384ea549d6e9df755 GZ_PARTITION=cleany_facility_gui_teb_b17f10d4bf4a42b384ea549d6e9df755
export IGN_DISCOVERY_MSG_PORT=22751 IGN_DISCOVERY_SRV_PORT=22752
export QT_QPA_PLATFORM=xcb DISPLAY=:0 GALLIUM_THREAD=0
export CLEANY_COMPARISON_REPO=/home/ehdrms/workspace/cleany/controller-opt-dwb
export CLEANY_BENCHMARK_TIMING=1
