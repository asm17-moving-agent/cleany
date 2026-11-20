"""Compatibility import for the shared navigation estimator."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/cleany_navigation'))
from cleany_navigation import icp_convergence as _implementation
sys.modules[__name__] = _implementation
