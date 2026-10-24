from pathlib import Path
import shlex,subprocess
root=Path(__file__).resolve().parent
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','ignition-msgs8'],text=True))
subprocess.run(['g++','-O2','-o',str(root/'scene_to_json'),str(root/'scene_to_json.cpp'),*flags],check=True)
