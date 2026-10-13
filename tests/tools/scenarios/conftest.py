"""Import the standalone scenario tools while testing their pure logic."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools/validation/scenarios"))
