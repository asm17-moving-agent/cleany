"""Audit every configured seat against the restored facility chair inventory."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from worlds import prepare,restored_tree,chair_name


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('fixture',type=Path);args=parser.parse_args()
    fixture=args.fixture
    seats=yaml.safe_load((fixture/'targets.yaml').read_text())['seats']
    tree=restored_tree(fixture/'world.sdf',fixture/'world.study.sdf')
    world=tree.getroot().find('world')
    models=[m.get('name') for m in world.findall('model')]
    expected=[chair_name(seat) for seat in seats]
    if len(set(expected))!=len(expected):raise ValueError('Duplicate seat-chair mapping')
    if any(models.count(name)!=1 for name in expected):raise ValueError('Missing or duplicate chair')
    manifests=[]
    with tempfile.TemporaryDirectory(prefix='cleany-seat-audit-') as temp:
        for seat in seats:
            output=Path(temp)/(seat+'.sdf')
            evidence=prepare(fixture/'world.sdf',fixture/'world.study.sdf',seat,output)
            remaining=[m.get('name') for m in ET.parse(output).getroot().find('world').findall('model')]
            if any((name in remaining)!=(name!=chair_name(seat)) for name in expected):
                raise ValueError(f'Incorrect chair inventory for {seat}')
            manifests.append(evidence)
    report={'seat_count':len(seats),'chair_count':len(expected),'seats':list(seats),'manifests':manifests}
    (fixture/'seat_inventory.json').write_text(json.dumps(report,indent=2))
    print('Verified',len(seats),'seat-specific worlds; each removes exactly one chair.')


if __name__=='__main__':main()
