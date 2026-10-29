from pathlib import Path

import numpy as np
import pytest

from cleany_skill_executor.curobo_mast import mast_cuboids
from cleany_skill_executor.curobo_model import CuRoboConfig


def description(kind='fixed', yaw='0'):
    return f'''<robot><link name="base_link"/><link name="top_base_link">
      <collision><origin xyz=".1 0 0"/><geometry><box size=".2 .1 .4"/></geometry></collision>
      <collision><geometry><box size=".1 .08 .1"/></geometry></collision>
    </link><joint name="mast" type="{kind}"><parent link="base_link"/>
      <child link="top_base_link"/><origin xyz="1 2 3" rpy="0 0 {yaw}"/>
    </joint></robot>'''


def test_fixed_mast_pose_padding_and_containment():
    import trimesh

    boxes = mast_cuboids(description(yaw=str(np.pi/2)), .0005)
    assert len(boxes) == 2
    box = next(iter(boxes.values()))
    assert box['dims'] == pytest.approx([.201, .101, .401])
    assert box['pose'][:3] == pytest.approx([1., 2.1, 3.])
    rotation = trimesh.transformations.quaternion_matrix(box['pose'][3:])[:3, :3]
    np.testing.assert_allclose(rotation @ [1., 0., 0.], [0., 1., 0.], atol=1e-12)


def test_no_mast_is_supported_but_movable_mast_fails():
    assert mast_cuboids('<robot><link name="base_link"/></robot>', .0005) == {}
    with pytest.raises(ValueError, match='fixed chain'):
        mast_cuboids(description(kind='revolute'), .0005)

def test_installed_mast_boxes_cover_every_triangle_with_padding():
    import trimesh
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from cleany_skill_executor.collision_sphere_model import mesh_path
    from cleany_skill_executor.core.urdf_fk import UrdfChain
    import xml.etree.ElementTree as ET

    package = Path(get_package_share_directory('cleany_description'))
    xml = xacro.process_file(str(package/'urdf/cleany.urdf.xacro'), mappings={
        'include_head_camera': 'true', 'include_wheel_joints': 'false'}).toxml()
    boxes = mast_cuboids(xml, .0005)
    assert len(boxes) == 5
    position, rotation = UrdfChain(xml, 'base_link', 'top_base_link').pose({})
    triangles = []
    for collision in ET.fromstring(xml).find("link[@name='top_base_link']").findall('collision'):
        mesh, origin = collision.find('geometry/mesh'), collision.find('origin')
        part = trimesh.load_mesh(mesh_path(mesh.get('filename')), process=True)
        part.apply_scale(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
        matrix = trimesh.transformations.euler_matrix(*np.fromstring(origin.get('rpy'), sep=' '))
        vertices = part.vertices @ matrix[:3, :3].T + np.fromstring(origin.get('xyz'), sep=' ')
        triangles.extend((vertices @ rotation.T + position)[part.faces])
    triangles = np.asarray(triangles)
    covered = np.zeros(len(triangles), dtype=bool)
    for box in boxes.values():
        inverse = trimesh.transformations.quaternion_matrix(box['pose'][3:])[:3, :3]
        local = (triangles - box['pose'][:3]) @ inverse
        covered |= (np.abs(local) <= np.array(box['dims'])/2 - .0005 + 1e-10).all(axis=(1, 2))
    assert covered.all(), 'Every complete triangle must fit inside a single padded box'
