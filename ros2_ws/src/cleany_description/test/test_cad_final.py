"""Compare the default runtime model against saved CAD geometry, not just its twin."""
from collections import defaultdict
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull

PACKAGE = Path(__file__).resolve().parents[1]
MANIFEST = json.loads((PACKAGE/'config/geometry/cad_final.json').read_text())


@lru_cache(None)
def expanded(*args):
    return ET.fromstring(subprocess.check_output(['xacro', str(PACKAGE/'urdf/cleany.urdf.xacro'), *args]))


def fk(root, angles):
    result = {'base_link':np.eye(4)}
    joints = list(root.findall('joint'))
    while joints:
        ready = [j for j in joints if j.find('parent').get('link') in result]
        assert ready
        for j in ready:
            o = j.find('origin')
            t = np.eye(4)
            t[:3,3] = np.fromstring(o.get('xyz'),sep=' ')
            t[:3,:3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy'),sep=' ')).as_matrix()
            if j.get('type') != 'fixed':
                a = np.fromstring(j.find('axis').get('xyz'),sep=' ')
                t[:3,:3] = t[:3,:3] @ Rotation.from_rotvec(a*angles.get(j.get('name'),0)).as_matrix()
            result[j.find('child').get('link')] = result[j.find('parent').get('link')] @ t
            joints.remove(j)
    return result


def vertices(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84],'little')
    assert len(raw) == 84+50*count
    assert 0 < count <= 200000
    dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
    return np.frombuffer(raw,dtype=dtype,offset=84)['v'].reshape(-1,3)


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(PACKAGE/'mjcf/cleany.xml'))


def test_default_geometry_is_entire_latest_assembly():
    assert MANIFEST['source_sha256'] == '91c26ec2eb8d1227fe9a0a18ded1235fcde257d2ef94cf9d345d337511625ff0'
    assert len({c['name'] for c in MANIFEST['components']}) == 308
    root=expanded()
    visuals=root.findall('.//visual/geometry/mesh')
    assert len(visuals) == len(MANIFEST['components'])
    assert all('/cad_final/' in e.get('filename') for e in visuals)
    for c in MANIFEST['components']:
        assert hashlib.sha256((PACKAGE/c['mesh']).read_bytes()).hexdigest() == c['mesh_sha256']


def test_registered_runtime_geometry_reconstructs_cad_bounds():
    transforms=fk(expanded(), MANIFEST['registration_angles_rad'])
    cad_from_base=np.linalg.inv(np.asarray(MANIFEST['cad_to_base']))
    actual=defaultdict(list)
    for c in MANIFEST['components']:
        t=cad_from_base@transforms[c['link']]
        v=vertices(PACKAGE/c['mesh']) @ t[:3,:3].T + t[:3,3]
        actual[c['name']].append(v*1000)
    for c in MANIFEST['components']:
        v=np.concatenate(actual.pop(c['name'])) if c['name'] in actual else None
        if v is not None:
            # Tessellation may miss a curved extremum by the 0.15 mm deflection.
            np.testing.assert_allclose(np.r_[v.min(0),v.max(0)], c['cad_bounds_mm'],atol=.3,err_msg=c['name'])


@pytest.mark.parametrize('seed',range(8))
def test_all_movable_joint_frames_match_mujoco(model,seed):
    root=expanded()
    rng=np.random.default_rng(seed)
    angles={j.get('name'):float(rng.uniform(-.4,.4)) for j in root.findall('joint') if j.get('type')!='fixed'}
    for side in ['left','right']:
        angles[side+'_gripper_joint']=0
    frames=fk(root,angles)
    data=mujoco.MjData(model)
    for name,a in angles.items():
        data.qpos[model.joint(name).qposadr[0]]=a
    mujoco.mj_forward(model,data)
    base=data.body('chassis')
    for j in root.findall('joint'):
        if j.get('type')=='fixed':
            continue
        mj=model.joint(j.get('name'))
        child=j.find('child').get('link')
        point=base.xmat.reshape(3,3).T@(data.xanchor[mj.id]-base.xpos)
        np.testing.assert_allclose(point,frames[child][:3,3],atol=2e-6,err_msg=j.get('name'))
        axis=frames[child][:3,:3]@np.fromstring(j.find('axis').get('xyz'),sep=' ')
        np.testing.assert_allclose(base.xmat.reshape(3,3).T@data.xaxis[mj.id],axis,atol=2e-6)


@pytest.mark.parametrize('side',['left','right'])
def test_new_wrist_optics_follow_cad_lens(model,side):
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    jaw=data.body('Fixed_Jaw' if side=='left' else 'Fixed_Jaw_2')
    t=np.asarray(MANIFEST['wrist_cameras'][side])
    cam=model.camera(side+'_wrist_rgb').id
    np.testing.assert_allclose(data.cam_xpos[cam],jaw.xpos+jaw.xmat.reshape(3,3)@t[:3,3],atol=1e-10)
    np.testing.assert_allclose(data.cam_xmat[cam].reshape(3,3)@np.diag([1,-1,-1]),jaw.xmat.reshape(3,3)@t[:3,:3],atol=1e-10)


def test_measured_profile_cannot_be_applied_to_new_assembly():
    with pytest.raises(subprocess.CalledProcessError):
        expanded('calibration_profile:=jetson_20261009')
    old=expanded('model_profile:=jetson_20261009','calibration_profile:=jetson_20261009')
    assert old.find("joint[@name='left_wrist_camera_calibrated_joint']") is not None


def test_pincer_is_explicitly_locked_and_actuation_rejected(model):
    for side in ['left','right']:
        j=expanded().find(f"joint[@name='{side}_gripper_joint']")
        assert j.find('limit').get('lower') == j.find('limit').get('upper') == '0'
        np.testing.assert_allclose(model.joint(side+'_gripper_joint').range,0,atol=1e-9)
    with pytest.raises(subprocess.CalledProcessError):
        subprocess.check_output(['xacro',str(PACKAGE/'urdf/cleany_control.urdf.xacro'),'enable_gripper_command:=true'])


def test_short_physics_step_is_finite(model):
    data=mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model,data)
    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()
    assert not any(w.number for w in data.warning)


def test_collision_stl_is_closed_convex_and_encloses_visual():
    for c in MANIFEST['components']:
        path=PACKAGE/c['collision_mesh']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == c['collision_sha256']
        tri=vertices(path).reshape(-1,3,3).astype(float)
        points=np.unique(tri.reshape(-1,3),axis=0)
        normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
        normal/=np.linalg.norm(normal,axis=1)[:,None]
        assert np.all(np.einsum('ij,ij->i',normal,tri.mean(1)-points.mean(0)) > 0),c['name']
        visual=np.unique(vertices(PACKAGE/c['mesh']),axis=0)
        for start in range(0,len(normal),128):
            n=normal[start:start+128]
            d=np.einsum('ij,ij->i',n,tri[start:start+128,0])
            assert np.max(points@n.T-d) <= 1e-9,c['name']
            assert np.max(visual@n.T-d) <= 1e-7,c['name']
        # Each output vertex must remain within 0.25 mm of the source hull.
        # A point on the center-to-vertex ray supplies a conservative bound.
        source = ConvexHull(visual)
        center = visual.mean(0)
        delta = points-center
        normals, offsets = source.equations[:, :3], source.equations[:, 3]
        projection = delta@normals.T
        ratios = np.full(projection.shape, np.inf)
        np.divide(-(normals@center+offsets)[None, :], projection,
                  out=ratios, where=projection > 0)
        fraction = np.minimum(1., ratios.min(1))
        bound = np.linalg.norm(delta, axis=1)*(1.-fraction)
        assert bound.max() <= .00025, c['name']
        _,ids=np.unique(tri.reshape(-1,3),axis=0,return_inverse=True)
        faces=ids.reshape(-1,3)
        edges=np.sort(faces[:,[0,1,1,2,2,0]].reshape(-1,2),axis=1)
        _,count=np.unique(edges,axis=0,return_counts=True)
        assert np.all(count==2),c['name']


def test_mock_gripper_interfaces_are_read_only():
    path = PACKAGE.parent/'cleany_moveit_config/config/cleany_mock.urdf.xacro'
    root = ET.fromstring(subprocess.check_output(['xacro', str(path)]))
    for side in ['left', 'right']:
        joint = root.find(f"ros2_control/joint[@name='{side}_gripper_joint']")
        assert joint.find('command_interface') is None
        assert joint.find("state_interface[@name='position']") is not None
