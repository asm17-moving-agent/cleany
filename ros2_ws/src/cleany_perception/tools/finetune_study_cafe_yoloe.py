"""스터디카페 시뮬레이션 데이터셋으로 YOLOE-seg를 미세 조정합니다.

오프라인으로 실행하며 로봇은 MuJoCo segmentation ID를 읽지 않습니다.
데이터셋과 결과 체크포인트는 구현 저장소 밖에 둡니다.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from ultralytics import YOLOE
from ultralytics.models.yolo.yoloe import YOLOEPESegTrainer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--text-encoder-directory', type=Path, required=True)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=8)
    parser.add_argument('--batch', type=int, default=4)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    dataset = args.dataset.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    encoder_dir = args.text_encoder_directory.expanduser().resolve()
    project = args.project.expanduser().resolve()
    for path in (dataset, checkpoint, encoder_dir / 'mobileclip2_b.ts'):
        if not path.is_file():
            raise FileNotFoundError(path)
    if args.epochs <= 0 or args.batch <= 0:
        raise ValueError('Epochs and batch must be positive')
    project.mkdir(parents=True, exist_ok=True)
    original_dir = Path.cwd()
    try:
        # Ultralytics는 작업 디렉터리에서 텍스트 encoder를 찾습니다.
        os.chdir(encoder_dir)
        model = YOLOE(str(checkpoint))
        model.train(
            data=str(dataset), trainer=YOLOEPESegTrainer,
            project=str(project), name='fit', exist_ok=False,
            device=args.device, epochs=args.epochs, batch=args.batch,
            imgsz=640, workers=0, seed=20260928,
            mosaic=0.2, scale=0.15, translate=0.05,
            hsv_h=0.01, hsv_s=0.2, hsv_v=0.2,
            plots=False, cache=False,
        )
    finally:
        os.chdir(original_dir)
    run = Path(model.trainer.save_dir)
    manifest = {
        'dataset': str(dataset), 'source_checkpoint': str(checkpoint),
        'trainer': 'YOLOEPESegTrainer', 'epochs': args.epochs,
        'batch': args.batch, 'device': args.device, 'image_size': 640,
        'best_checkpoint': str(run / 'weights/best.pt'),
    }
    (run / 'cleany_training_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8'
    )
    print(json.dumps(manifest, indent=2), flush=True)


if __name__ == '__main__':
    main()
