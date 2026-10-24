from pathlib import Path
import subprocess,time,json
ROOT=Path(__file__).resolve().parent
started=time.monotonic()
while not (ROOT/'baseline-completed.json').exists():
 if time.monotonic()-started>1800:raise TimeoutError('Baseline did not finish; inspect baseline-campaign.log')
 time.sleep(2)
with (ROOT/'improved-campaign.log').open('w') as log:
 code=subprocess.run(['python3',str(ROOT/'campaign.py'),'--variant','improved'],stdout=log,stderr=subprocess.STDOUT).returncode
if code:raise RuntimeError('Improved campaign failed: '+str(code))
print('Both campaigns completed',flush=True)
