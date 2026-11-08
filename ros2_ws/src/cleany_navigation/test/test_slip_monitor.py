from pathlib import Path
import math
import numpy as np
import pytest
import yaml
from cleany_navigation.slip_estimation import Settings
from cleany_navigation.slip_monitor import ObservationHistory, ScanHistory, base_cloud
from cleany_navigation.traction_brake import BrakeRamp
from cleany_navigation.traction_config import configure_retry

ROOT = Path(__file__).resolve().parents[1]


def test_persistence_requires_new_comparisons_not_heartbeat_and_unknown_breaks():
    h = ObservationHistory(Settings())
    row = dict(t0=1.,t1=2.,status='suspect')
    assert h.update(row)['state']=='suspect'
    assert h.update(row)['state']=='suspect'
    assert h.update(dict(row,t0=1.5,t1=2.5))['state']=='confirmed'
    assert h.update(dict(row,t0=2.,t1=3.,status='unknown'))['state']=='unknown'
    assert h.update(dict(row,t0=2.5,t1=3.5))['state']=='suspect'


def test_stale_spaced_windows_do_not_accumulate():
    h = ObservationHistory(Settings())
    for t in [1.,2.,3.]:
        assert h.update(dict(t0=t,t1=t+1,status='suspect'))['state']=='suspect'


def test_pair_requires_bracketing_odometry_and_one_second_span():
    h = ScanHistory(Settings())
    h.add_odom([.9,0,0,0]);h.add_odom([1.5,0,0,0])
    h.add_scan(1.,.13,np.zeros((10,2)));h.add_scan(2.,.13,np.zeros((10,2)))
    assert h.pair() is None
    h.add_odom([2.1,0,0,0]);assert h.pair() is not None
    assert h.pair() is None


def test_brake_is_monotone_preserves_translation_direction_and_stale_is_zero():
    ramp = BrakeRamp()
    ramp.step((.1,.1,.2),False,.02)
    previous = np.array(ramp.output)
    def wheels(v):
        x,y,z=v;return np.array([x-y-.47*z,x+y+.47*z,x+y-.47*z,x-y+.47*z])/.0635
    for _ in range(100):
        out = np.array(ramp.step((.2,-.2,0),True,.02))
        assert np.all(abs(out)<=abs(previous)+1e-12)
        assert out[0]==pytest.approx(out[1])
        assert max(abs(wheels(out)-wheels(previous))) <= 3*.02+1e-12
        previous=out
    assert np.linalg.norm(previous)==0
    ramp.step((.1,0,0),False,.02)
    assert ramp.step((.1,0,0),False,.02,False)==(0,0,0)


def test_lidar_mount_rotation_and_far_subset():
    points=np.array([[1.,0.],[2.,0.]])
    cloud=base_cloud(points,np.array([.139275,0,math.pi]),[],Settings(),True)
    np.testing.assert_allclose(cloud,[[-1.860725,0]],atol=1e-12)


def test_retry_has_no_lateral_motion_and_preserves_omni_plugin():
    p=yaml.safe_load((ROOT/'config/nav2_amcl.yaml').read_text())
    from cleany_navigation.runtime_config import merge_parameters
    merge_parameters(p,yaml.safe_load((ROOT/'config/nav2_phone_omni.yaml').read_text()))
    original=p['controller_server']['ros__parameters']['FollowPath'].copy()
    retry=configure_retry(p,yaml.safe_load((ROOT/'config/nav2_phone_longitudinal.yaml').read_text()),'/tmp/diff.json')
    c=p['controller_server']['ros__parameters']
    assert c['FollowPath']==original
    assert c['Longitudinal']['motion_model']=='DiffDrive'
    assert c['Longitudinal']['vy_max']==c['Longitudinal']['vy_std']==0
    assert c['Longitudinal']['acceleration_limits'] is True
    assert c['Longitudinal']['wheel_max_rad_s']==4
    assert p['planner_server']['ros__parameters']['planner_plugins']==['GridBased']
    planner=retry['/traction_retry/planner_server']['ros__parameters']
    assert planner['planner_plugins']==['Longitudinal']
    assert planner['Longitudinal']['holonomic_direct_path'] is False
    assert planner['Longitudinal']['lattice_filepath']=='/tmp/diff.json'
    assert retry['/traction_retry/global_costmap/global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic']=='/scan'
