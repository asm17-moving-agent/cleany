Import("env")

import importlib.util
from pathlib import Path

root = Path(env.subst("$PROJECT_DIR")).parent
spec = importlib.util.spec_from_file_location("imu_runtime_config", root / "tools/imu_runtime_config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
config.generate(config.SOURCE, Path(env.subst("$BUILD_DIR")) / "cleany_config/imu_startup_config.hpp")
