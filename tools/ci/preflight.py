#!/usr/bin/env python3
"""빌드 전에 필수 도구와 이미지 생성 API를 실제로 호출합니다."""
import importlib.metadata
import io
import shutil


def main() -> None:
    for command in ('git', 'make', 'ros2', 'colcon', 'rosdep', 'xvfb-run', 'ccache', 'gdb'):
        if not shutil.which(command): raise RuntimeError(f'Missing CI tool: {command}')
    from PIL import Image
    # Ubuntu 22.04의 Pillow와 최신 Pillow에서 같은 보간 기능을 확인합니다.
    resampling = getattr(Image, 'Resampling', Image)
    picture = Image.new('RGB', (8, 8))
    picture.thumbnail((4, 4), resampling.LANCZOS)
    picture.save(io.BytesIO(), format='PNG')
    print('Image API available; Pillow ' + importlib.metadata.version('Pillow'))


if __name__ == '__main__':
    main()
