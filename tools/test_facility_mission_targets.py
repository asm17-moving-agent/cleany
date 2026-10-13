"""Regression coverage for facility/model IDs crossing the gateway boundary."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / 'facility'))
from generate_mission_targets import canonical_seat_id, generate


@pytest.mark.parametrize(('seat', 'expected'), [
    ({'seat_id': '43', 'zone_id': 'd-hub'}, 'seat-43'),
    ({'seat_id': 'A41', 'zone_id': 'space-a4'}, 'seat-a4-01'),
    ({'seat_id': 'M36', 'zone_id': 'space-m3'}, 'seat-m3-06'),
])
def test_model_seat_ids_match_gateway_inventory(seat: dict, expected: str) -> None:
    assert canonical_seat_id(seat) == expected


def test_room_mismatch_is_not_silently_remapped() -> None:
    with pytest.raises(ValueError, match='unrecognized room seat'):
        canonical_seat_id({'seat_id': 'A41', 'zone_id': 'space-m3'})


def test_removed_chair_keeps_explicit_front_direction_and_uncertainty_clearance(tmp_path):
    import json
    import math
    import yaml
    from PIL import Image, ImageDraw
    image = Image.new('L', (160, 160), 254)
    ImageDraw.Draw(image).rectangle((48, 72, 72, 87), fill=0)
    image.save(tmp_path/'map.pgm')
    map_path = tmp_path/'map.yaml'
    map_path.write_text(yaml.safe_dump({'image':'map.pgm', 'resolution':.05, 'origin':[0.,0.,0.]}))
    world = tmp_path/'removed.sdf'
    world.write_text('<sdf><world><model name="desk"><pose>3 4 0 0 0 0</pose></model></world></sdf>')
    world.with_suffix('.seats.json').write_text(json.dumps([
        {'seat_id':'43','zone_id':'d-hub','desk_model':'desk','chair_model':'chair','pose':[3,4,0,0,0,0]}]))
    reference = tmp_path/'original.sdf'
    reference.write_text('<sdf><world><model name="chair"><pose>3 3.7 0 0 0 0</pose></model></world></sdf>')
    with pytest.raises(ValueError, match='supply reference_world'):
        generate(world, map_path, [1.,1.,0.], tmp_path)
    override = {'seat-43':[3.,3.225,math.pi/2]}
    targets, evidence = generate(world, map_path, [1.,1.,0.], tmp_path, override,
                                 localization_margin=0., reference_world=reference)
    assert targets['seats']['seat-43'][2] == round(math.pi/2, 6)
    assert not evidence['seat-43']['chair_present']
    # A pose that fits nominal geometry is too close when uncertainty is included.
    with pytest.raises(ValueError, match='no safe desk-front goal'):
        generate(world, map_path, [1.,1.,0.], tmp_path, override,
                 localization_margin=.1, reference_world=reference)
