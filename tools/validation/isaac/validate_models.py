"""Check downloaded inference assets without substituting detector results."""
import json
from pathlib import Path
import torch
from local_perception import LocalImageClassifier
from download_models import sha256
from cleany_perception.adapters.yoloe_detector import YoloeDetector

torch.set_num_threads(4)
root = Path('/models')
yolo = YoloeDetector(str(root/'yoloe/yoloe-26l-seg.pt'),
    ['cup','computer mouse','crumpled tissue','lego brick'],device='cpu',require_masks=True,
    text_encoder_directory=str(root/'yoloe'))
yolo.prepare()
print('YOLOE_READY',flush=True)
model = LocalImageClassifier(str(root/'qwen3vl-4b'),Path('/output/evaluations/semantics'))
model.prepare()
print('LOCAL_VLM_READY',flush=True)
record = {str(p.relative_to(root)):sha256(p)
          for directory in ('yoloe', 'qwen3vl-4b') for p in (root/directory).rglob('*')
          if p.is_file() and '.cache' not in p.parts}
Path('/output/model-assets.json').write_text(json.dumps(record,indent=2))
