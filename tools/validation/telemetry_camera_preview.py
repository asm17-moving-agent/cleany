"""Check fresh preview responses from one provider and its monitor consumer."""
import argparse
import json
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import urlopen

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--provider', default='http://127.0.0.1:8769')
parser.add_argument('--monitor', default='http://127.0.0.1:18768')
parser.add_argument('--topic', required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
results = {}
for name, origin in [('provider',args.provider),('monitor',args.monitor)]:
    samples = []
    deadline = time.monotonic()+15
    while len(samples)<3 and time.monotonic()<deadline:
        try:
            with urlopen(origin+'/api/camera?'+urlencode({'topic':args.topic}),timeout=3) as response:
                payload=response.read()
                if response.status==200:
                    assert response.headers.get_content_type()=='image/jpeg'
                    assert payload.startswith(b'\xff\xd8') and payload.endswith(b'\xff\xd9')
                    age=float(response.headers['X-Frame-Age'])
                    assert 0<=age<=2
                    samples.append({'age_seconds':age,'ros_stamp':response.headers['X-ROS-Stamp'],
                                    'bytes':len(payload)})
                    (args.output/f'{name}.jpg').write_bytes(payload)
        except HTTPError as error:
            assert error.code==400, f'{name}: HTTP {error.code}'
        except URLError:
            pass  # The server may still be binding during process startup.
        time.sleep(.3)
    assert len(samples)==3, f'{name}: no fresh preview'
    assert len({item['ros_stamp'] for item in samples})>1, 'Source time did not advance'
    results[name]=samples
results.update(topic=args.topic,motion_commands=False)
(args.output/'validation.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results))
