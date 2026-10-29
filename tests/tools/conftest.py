"""Import repository host tools without installing device dependencies."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/headcam'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
