"""Bounded 30s polling without launching any navigation measurement."""
import json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
COMMON=ROOT.parents[1]/'common'
while not (COMMON/'READY').is_file():
 time.sleep(30)
print(json.dumps({'common_READY':(COMMON/'READY').read_text(),'locked_present':(COMMON/'goals.locked.json').exists(),'received_wall_epoch_s':time.time()}),flush=True)
