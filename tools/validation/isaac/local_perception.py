"""Run existing RGB-D inspection with downloaded YOLOE and a local image classifier.

This validation entrypoint changes the semantic inference provider only. Geometry,
tracking, selected-object reconstruction and ROS actions stay in cleany_perception.
"""
import json
from pathlib import Path
import io
import re
import time

import numpy as np
import torch
from PIL import Image
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration, BitsAndBytesConfig

from cleany_perception.adapters.yoloe_detector import YoloeDetector, YoloeMaskSegmenter
from cleany_perception.adapters.yoloe_gemini import YoloeGeminiDetector
from cleany_perception.adapters.gemini_detector import GeminiClassifier
from cleany_perception.inspection_node import InspectionNode
from cleany_perception.core.models import InspectionFailure, FailureKind
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from wrist_detector import WristDetector
from head_detector import HeadDetector


class LocalImageClassifier(GeminiClassifier):
    """Use the existing numbered-instance prompt and parser with a local VLM."""

    def __init__(self, model_path: str, evidence: Path):
        self.model_path, self.evidence = model_path, evidence
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.model = self.processor = None
        evidence.mkdir(parents=True, exist_ok=True)
        super().__init__(model=model_path, response_provider=self._infer)

    def prepare(self):
        if self.model is None:
            self.processor = AutoProcessor.from_pretrained(self.model_path, local_files_only=True)
            options = dict(local_files_only=True, dtype=torch.bfloat16, attn_implementation='sdpa')
            if self.device.startswith('cuda'):
                options.update(device_map={'':self.device}, quantization_config=BitsAndBytesConfig(
                    load_in_4bit=True, bnb_4bit_quant_type='nf4', bnb_4bit_use_double_quant=True,
                    bnb_4bit_compute_dtype=torch.bfloat16))
            self.model = Qwen3VLForConditionalGeneration.from_pretrained(self.model_path, **options).eval()
            # Warm CUDA kernels before readiness; this artificial input is never
            # returned as an observation or used for semantic classification.
            self._generate(Image.new('RGB',(640,480)), 'Describe the image.', 1)
            print(f'LOCAL_VLM_READY device={self.device}',flush=True)

    def _generate(self, visible, prompt, maximum_tokens=256):
        messages = [{'role':'user','content':[{'type':'image','image':visible},
            {'type':'text','text':prompt}]}]
        inputs = self.processor.apply_chat_template(messages, tokenize=True,
            add_generation_prompt=True, return_dict=True, return_tensors='pt').to(self.device)
        with torch.inference_mode():
            generated = self.model.generate(**inputs, max_new_tokens=maximum_tokens, do_sample=False)
        return self.processor.batch_decode(generated[:,inputs['input_ids'].shape[1]:],
            skip_special_tokens=True)[0].strip()

    def _infer(self, image_bytes, prompt, schema):
        self.prepare()
        started = time.monotonic()
        visible = Image.open(io.BytesIO(image_bytes)).convert('RGB')
        effective_prompt = (prompt + '\nReturn only the JSON data object, not a schema. '
            'The top-level key must be "objects", containing an array of classification records. '
            'Do not return "type" or "properties" keys. Validate the data against this schema: '
            + json.dumps(schema))
        answer = self._generate(visible,effective_prompt)
        identifier = str(time.time_ns())
        visible.save(self.evidence/f'{identifier}.png')
        (self.evidence/f'{identifier}.json').write_text(json.dumps(dict(
            model=self.model_path, device=self.device, prompt=prompt, effective_prompt=effective_prompt,
            quantization='nf4-double-bf16' if self.device.startswith('cuda') else 'bf16', schema=schema, answer=answer,
            elapsed_s=time.monotonic()-started),indent=2))
        fenced = re.fullmatch(r'```(?:json)?\s*\n(.*)\n```',answer,re.DOTALL)
        return fenced[1] if fenced else answer


class LocalDeskDetector(YoloeGeminiDetector):
    def detect(self, rgb, query):
        luminance = np.asarray(rgb).mean(axis=2)
        if np.percentile(luminance, 95) < 5. or np.std(luminance) < 1.:
            raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                'RGB image is dark or featureless; cannot establish an empty desk')
        return super().detect(rgb, query)


def main():
    torch.set_num_threads(4)
    rclpy.init()
    classifier = LocalImageClassifier('/models/qwen3vl-4b',Path('/output/evaluations/semantics'))
    camera = json.loads(Path('/output/config/room.json').read_text())['cleanup']
    yolo = YoloeDetector('/models/yoloe/yoloe-26l-seg.pt',
        classes=('cup','computer mouse','crumpled tissue','lego brick'), device='cpu',
        image_size=max(camera.get('camera_resolution',[640,480])),
        text_encoder_directory='/models/yoloe', require_masks=True)
    detector = LocalDeskDetector(HeadDetector(yolo,
        Path('/output/evaluations/head-detections')),classifier)
    node = InspectionNode(detector=detector,segmenter=YoloeMaskSegmenter(),wrist_detector=WristDetector(yolo),parameter_overrides=[
        Parameter('detector_type',value='yoloe_local'), Parameter('segmenter_type',value='yoloe_seg'),
        Parameter('maximum_mask_depth_jump_m',value=0.02),
        Parameter('preload_models',value=True), Parameter('use_sim_time',value=True),
        Parameter('depth_image_topic',value='/camera/aligned_depth_to_color/image_raw'),
        Parameter('depth_info_topic',value='/camera/aligned_depth_to_color/camera_info'),
        Parameter('enable_wrist_observation',value=True),
        Parameter('wrist_failure_image_directory',value='/output/evaluations/wrist-failures'),
        Parameter('snapshot_timeout_seconds',value=20.), Parameter('yoloe_device',value='cpu')])
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
