#!/usr/bin/env bash
source /home/ehdrms/workspace/cleany/cleany-data/runs/controller-facility-gui-20261013-053337/teb/env.sh
exec ign gazebo -g -v 4 --render-engine-gui ogre2 --gui-config /home/ehdrms/workspace/cleany/cleany-data/runs/controller-facility-gui-20261013-053337/teb/gui.config
