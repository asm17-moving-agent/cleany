"""Download pinned official inference assets into an explicit external directory."""
import argparse
import hashlib
from pathlib import Path
from urllib.request import urlretrieve

from huggingface_hub import snapshot_download


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    (root/'yoloe').mkdir(parents=True, exist_ok=True)
    assets = (
        ('v8.4.0', 'yoloe-26l-seg.pt', 'a612d2d505f24e14d87ec82d688b823b6cb600646664f16125ce6c84ce360da9'),
        ('v8.3.0', 'mobileclip2_b.ts', '35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f'),
    )
    for version, name, expected in assets:
        path = root/'yoloe'/name
        if not path.exists():
            partial = path.with_suffix(path.suffix+'.partial')
            urlretrieve(f'https://github.com/ultralytics/assets/releases/download/{version}/{name}', partial)
            if sha256(partial) != expected:
                raise ValueError(f'Model hash mismatch: {partial}')
            partial.rename(path)
        if sha256(path) != expected:
            raise ValueError(f'Existing model hash mismatch: {path}')
    snapshot_download('Qwen/Qwen3-VL-4B-Instruct',
        revision='ebb281ec70b05090aa6165b016eac8ec08e71b17', local_dir=root/'qwen3vl-4b',
        allow_patterns=['*.json', '*.safetensors', '*.txt', '*.jinja', 'README.md'], max_workers=4)


if __name__ == '__main__':
    main()
