from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from assets import localize_and_hash


def scene(uri):
    return ET.ElementTree(ET.fromstring(f'<sdf><mesh><uri>{uri}</uri></mesh></sdf>'))


def test_missing_mesh_or_texture_prevents_scene_acceptance(tmp_path: Path):
    tree = scene('fixture://chair.obj')
    with pytest.raises(FileNotFoundError):
        localize_and_hash(tree, tmp_path, tmp_path)
    (tmp_path/'chair.obj').write_text('mtllib chair.mtl\nv 1 2 3\n')
    (tmp_path/'chair.mtl').write_text('newmtl chair\nmap_Kd chair.png\n')
    with pytest.raises(FileNotFoundError):
        localize_and_hash(tree, tmp_path, tmp_path)
    (tmp_path/'chair.png').write_bytes(b'image bytes')
    hashes = localize_and_hash(tree, tmp_path, tmp_path)
    assert set(hashes) == {str(tmp_path/name) for name in ('chair.obj', 'chair.mtl', 'chair.png')}
    assert tree.find('.//uri').text == (tmp_path/'chair.obj').as_uri()
    (tmp_path/'chair.png').write_bytes(b'')
    with pytest.raises(ValueError, match='Empty'):
        localize_and_hash(tree, tmp_path, tmp_path)


def test_cached_fuel_is_pinned_to_local_file_and_hashed(tmp_path: Path):
    mesh = tmp_path/'.ignition/fuel/fuel.gazebosim.org/openrobotics/models/chair/1/meshes/chair.obj'
    mesh.parent.mkdir(parents=True)
    mesh.write_text('v 1 2 3\n')
    tree = scene('https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair/1/files/meshes/chair.obj')
    hashes = localize_and_hash(tree, tmp_path, tmp_path/'repository')
    assert str(mesh) in hashes
    assert tree.find('.//uri').text == mesh.as_uri()


def test_collada_image_dependency_is_required(tmp_path: Path):
    mesh = tmp_path/'chair.dae'
    mesh.write_text('<COLLADA xmlns="urn:collada"><library_images><image><init_from>missing.png</init_from></image></library_images></COLLADA>')
    with pytest.raises(FileNotFoundError):
        localize_and_hash(scene(mesh.as_uri()), tmp_path, tmp_path)


def test_fortress_receives_byte_identical_ascii_mesh_path(tmp_path: Path):
    mesh = tmp_path/'robot_êäèë.stl'
    mesh.write_bytes(b'solid test\nendsolid test\n')
    tree = scene(mesh.as_uri())
    aliases = tmp_path/'assets'
    hashes = localize_and_hash(tree, tmp_path, tmp_path, aliases)
    target = tree.find('.//uri').text.removeprefix('file://')
    assert target.isascii()
    assert Path(target).parent == aliases
    assert Path(target).read_bytes() == mesh.read_bytes()
    assert hashes[target] == hashes[str(mesh)]
    assert str(mesh) in hashes


def test_repo_assets_relocate_and_cannot_escape_root(tmp_path):
    repo = tmp_path/'checkout'
    repo.mkdir()
    (repo/'robot.stl').write_bytes(b'solid robot\nendsolid robot')
    hashes = localize_and_hash(scene('repo://robot.stl'), tmp_path, repo)
    assert str(repo/'robot.stl') in hashes
    with pytest.raises(ValueError, match='escapes'):
        localize_and_hash(scene('repo://../outside.stl'), tmp_path, repo)
