from pathlib import Path

import numpy as np
import pytest

from cleany_handeye_calibration.offline_fk import UrdfOfflineFk


URDF = '''
<robot name="test">
  <link name="base_link"/>
  <link name="shoulder"/>
  <link name="slider"/>
  <link name="tip"/>
  <joint name="yaw" type="revolute">
    <parent link="base_link"/><child link="shoulder"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="extension" type="prismatic">
    <parent link="shoulder"/><child link="slider"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
  </joint>
  <joint name="tip_fixed" type="fixed">
    <parent link="slider"/><child link="tip"/>
    <origin xyz="0 1 0" rpy="0 0 0"/>
  </joint>
</robot>
'''.strip()


def _urdf(tmp_path: Path) -> Path:
    path = tmp_path / 'robot.urdf'
    path.write_text(URDF, encoding='utf-8')
    return path


def test_urdf_offline_fk_uses_feedback_joint_values(tmp_path):
    fk = UrdfOfflineFk(_urdf(tmp_path), tip_frame='tip')

    pose = fk.compute(('extension', 'yaw', 'extra'), (0.5, np.pi / 2.0, 9.0))

    assert fk.moving_joint_names == ('yaw', 'extension')
    np.testing.assert_allclose(
        pose.translation_m,
        (0.0, 1.5, 0.0),
        rtol=0.0,
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        pose.rotation_array(),
        np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))),
        rtol=0.0,
        atol=1.0e-12,
    )


def test_urdf_offline_fk_rejects_missing_joint_and_wrong_chain(tmp_path):
    fk = UrdfOfflineFk(_urdf(tmp_path), tip_frame='tip')
    with pytest.raises(ValueError, match='missing FK joints'):
        fk.compute(('yaw',), (0.0,))
    with pytest.raises(ValueError, match='no URDF chain'):
        UrdfOfflineFk(_urdf(tmp_path), tip_frame='missing')
