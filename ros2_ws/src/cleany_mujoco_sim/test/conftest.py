from pathlib import Path

import pytest

from cleany_mujoco_sim.scene_loader import load_model, materialize_scene

TINY_MJCF = """
<mujoco>
  <worldbody>
    <body name="chassis">
      <freejoint/>
      <site name="lidar_site" pos="0 0 0" size="0.01"/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="arm">
        <joint name="shoulder" type="hinge" axis="0 0 1" range="-1 1"/>
        <geom type="box" size="0.05 0.05 0.05" pos="0.2 0 0"/>
      </body>
    </body>
    <body name="scan_target" pos="1 0 0">
      <geom type="box" size="0.05 0.5 0.5"/>
    </body>
  </worldbody>
  <actuator>
    <position name="shoulder_position" joint="shoulder"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def scene_path(tmp_path: Path) -> Path:
    path = tmp_path / "scene.xml"
    path.write_text(TINY_MJCF)
    return path


@pytest.fixture
def model_data(scene_path: Path):
    return load_model(scene_path)


@pytest.fixture(scope='session')
def cleany_scene_path() -> Path:
    template_path = (
        Path(__file__).parents[1] / 'scenes' / 'default.xml.in'
    )
    return materialize_scene(template_path)


@pytest.fixture(scope='session')
def rgbd_pick_scene_path() -> Path:
    template_path = (
        Path(__file__).parents[1] / 'scenes' / 'rgbd_pick_demo.xml.in'
    )
    return materialize_scene(template_path)
