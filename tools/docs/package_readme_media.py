#!/usr/bin/env python3
"""Publish the six reviewed media sets and build their portable review gallery."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from build_media_catalog import build

ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    media = args.run/'media'
    assets = ROOT/'docs/assets/readme'
    entries = [
        ('perception', 'Detection · Segmentation · 3D', '인식', '같은 RGB-D 프레임의 처리 결과',
         ['perception-pipeline.png'], 'perception-pipeline.json',
         '저장된 RGB-D 한 프레임에 현재 YOLOE adapter와 geometry core를 적용한 결과. 라벨은 모델 예측값.'),
        ('gazebo', 'Gazebo 스터디카페', '시뮬레이션', '책상과 통로가 배치된 실내 환경',
         ['gazebo-study-cafe.png'], 'gazebo-study-cafe.json',
         '2026-09-11에 저장한 Gazebo 스터디카페 화면. 당시 로봇 모델을 사용한 기록.'),
        ('clips', 'MuJoCo · RViz', '시뮬레이션', '동작이 보이는 viewport만 추출',
         ['mujoco-work.gif', 'rviz-motion.gif'], 'clips.json',
         '기존 시뮬레이션 녹화에서 추출한 1배속 GIF. 새 실물 실행이나 전체 정리 성공의 근거가 아님.'),
        ('isaac', 'Isaac Sim 병렬 환경', '시뮬레이션', '16개 독립 환경의 새 캡처',
         ['isaac-parallel.png'], 'isaac-parallel.json',
         '실제 Isaac Sim에서 만든 문서용 병렬 환경. 학습된 policy 실행이나 RL 성능 평가가 아님.'),
        ('mecanum', 'Mecanum 운동학', '하드웨어', '전진 · 횡이동 · 회전',
         ['mecanum-motion.gif'], 'mecanum-motion.json',
         '실제 CAD와 base driver의 wheel_speeds를 사용한 운동학 애니메이션. 접촉 물리나 실물 주행 시험이 아님.'),
        ('cad', 'FreeCAD 작업 화면', '하드웨어', '모델 트리와 전체 조립체',
         ['freecad-workspace.png'], 'freecad-workspace.json',
         '원본 Cleany-final.FCStd를 실제 FreeCAD에서 열어 촬영한 작업 화면. 모델 형상은 수정하지 않음.'),
    ]
    manifest_path = assets/'sources.json'
    manifest = json.loads(manifest_path.read_text())
    new_names = {name for *_, names, metadata, scope in entries for name in names}
    manifest['assets'] = [x for x in manifest['assets'] if Path(x['asset']).name not in new_names]
    cards = []
    for identifier, title, group, note, names, metadata, scope in entries:
        json.loads((media/metadata).read_text())
        for name in names:
            source = media/name
            shutil.copyfile(source, assets/name)
            manifest['assets'].append({
                'asset': f'docs/assets/readme/{name}', 'kind': group,
                'source': str(source), 'evidence': str(media/metadata),
                'scope': scope, 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            })
        cards.append({'id':identifier, 'title':title, 'group':group, 'priority':'완성',
                      'note':note, 'scope':scope, 'evidence':str(media/metadata),
                      'media':[{'path':str(media/name), 'label':Path(name).stem} for name in names]})
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    config = {'title':'Cleany 시각자료 6종', 'mode':'completed', 'created':'2026-11-15',
              'description':'실제 CAD, 저장된 기록, 현재 코드로 만든 README 자료.',
              'repo':str(ROOT), 'commit':commit, 'items':cards, 'ideas':[], 'omitted':[]}
    (args.run/'gallery-input.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n')
    gallery = args.run/'gallery'
    build(config, gallery)
    # Keep the original capture formats and provenance beside the gallery.
    shutil.copytree(media, gallery/'downloads', dirs_exist_ok=True)
    (gallery/'downloads/index.html').write_text('<!doctype html><meta charset="utf-8"><h1>Cleany media</h1><ul>'+''.join(
        f'<li><a href="{p.name}">{p.name}</a></li>' for p in sorted(media.iterdir()) if p.is_file())+'</ul>')
    document = (gallery/'index.html').read_text()
    document = document.replace('<nav aria-label=', '<p><a href="downloads/">PNG · GIF · MP4 원본 모음</a></p><nav aria-label=')
    for _, _, _, _, _, metadata, _ in entries:
        document = document.replace((media/metadata).as_uri(), f'downloads/{metadata}')
    (gallery/'index.html').write_text(document)
    scripts = sorted(ROOT.glob('tools/docs/*readme*.py'))+[ROOT/'tools/docs/build_media_catalog.py', ROOT/'tools/docs/review_readme_media.mjs']
    source_dir = args.run/'provenance/source-snapshot'
    source_dir.mkdir(parents=True, exist_ok=True)
    for source in scripts:
        shutil.copyfile(source, source_dir/source.name)
    (args.run/'provenance/run.json').write_text(json.dumps({
        'base_commit':commit, 'working_tree':str(ROOT), 'uncommitted_visual_changes':True,
        'scripts':[{'path':str(p.relative_to(ROOT)), 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in scripts],
        'output_root':str(args.run), 'readme_languages':['en', 'ko'],
    }, indent=2)+'\n')
    print(f'Packaged {len(entries)} media sets in {gallery}')


if __name__ == '__main__':
    main()
