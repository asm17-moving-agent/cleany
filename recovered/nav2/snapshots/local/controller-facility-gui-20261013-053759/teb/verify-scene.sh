#!/bin/bash
source /home/ehdrms/workspace/cleany/cleany-data/runs/controller-facility-gui-20261013-053759/teb/env.sh
ign service -s /world/cleany_study_cafe/scene/info --reqtype ignition.msgs.Empty --reptype ignition.msgs.Scene --timeout 5000 --req "" > /home/ehdrms/workspace/cleany/cleany-data/runs/controller-facility-gui-20261013-053759/teb/scene-proof.txt
