from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from worlds import chair_name, prepare


def test_seat_world_restores_other_chair_and_preserves_obstacles(tmp_path: Path):
    source = tmp_path/'source.sdf'
    source.write_text('<sdf><world><model name="wall"><static>true</static></model></world></sdf>')
    reference = tmp_path/'reference.sdf'
    reference.write_text('<sdf><world>'+''.join(f'<model name="office_chair_{i:02d}"/>' for i in range(1,49))+'</world></sdf>')
    output = tmp_path/'seat43.sdf'
    manifest = prepare(source, reference, 'seat-43', output)
    world = ET.parse(output).getroot().find('world')
    assert world.find("model[@name='office_chair_43']") is None
    assert world.find("model[@name='office_chair_13']") is not None
    assert ET.tostring(world.find("model[@name='wall']")) == ET.tostring(ET.parse(source).getroot().find('world/model'))
    assert manifest['model_count_before'] - manifest['model_count_after'] == 1


def test_missing_reference_fails_without_output(tmp_path: Path):
    source = tmp_path/'source.sdf'
    source.write_text('<sdf><world/></sdf>')
    with pytest.raises(ValueError):
        prepare(source, source, 'seat-a1-01', tmp_path/'out.sdf')
    assert not (tmp_path/'out.sdf').exists()


def test_room_mapping():
    assert chair_name('seat-m3-06') == 'facility_chair_M36'
    with pytest.raises(ValueError):
        chair_name('../chair')
