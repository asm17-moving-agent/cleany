"""Reuse a verified CAD release and existing calibration for the grasp runtime."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

from cleany_description.cad_coordinates import gripper_touch_links, match_gripper_endpoints
from cleany_robot_interface.calibrated_cad_model import calibrated_cad_model
from cleany_robot_interface.calibrated_camera_pose import CameraPoseModel
from cleany_robot_interface.state_profile import prepare_state_profile


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_reference(path: Path) -> tuple[dict, str]:
    """Read the existing indexed cache without importing its private release."""
    selected = json.loads(path.read_text())
    if (selected.get('schema') != 'cleany.runtime_model/v1'
            or selected.get('mode') != 'reference_only'
            or selected.get('motion_enabled') is not False):
        raise ValueError('An unchanged reference-only CAD release is required')
    for filename, expected in selected['files_sha256'].items():
        if digest(Path(filename)) != expected:
            raise ValueError('CAD reference dependency changed: '+filename)
    cache = Path(selected['moveit_collision_cache'])
    manifest = json.loads((cache/'manifest.json').read_text())
    filename = 'cleany_freecad_kinematic.urdf'
    indexed = cache/filename
    if digest(indexed) != manifest['models'][filename]['indexed_sha256']:
        raise ValueError('Indexed CAD model changed')
    for filename, expected in manifest['assets'].items():
        if digest(cache/filename) != expected:
            raise ValueError('Indexed CAD asset changed: '+filename)
    tree = ET.fromstring(indexed.read_text())
    for mesh in tree.findall('.//mesh'):
        filename = mesh.get('filename')
        if not filename.startswith('package://'):
            asset = (cache/filename).resolve()
            if not asset.is_relative_to(cache.resolve()):
                raise ValueError('Indexed CAD asset escapes its cache')
            mesh.set('filename', asset.as_uri())
    return selected, ET.tostring(tree, encoding='unicode')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('repository','selected-model','base-profile','collection','reuse-inputs','gripper-endpoints','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args = parser.parse_args()
    root = args.repository.resolve()
    output = args.output.resolve()
    output.mkdir(exist_ok=False,parents=True)
    selected, source = load_reference(args.selected_model)
    original = ET.fromstring(source)
    profile = json.loads(args.base_profile.read_text())
    pose = Path(profile['camera_pose_config']).expanduser()
    inputs = json.loads(args.reuse_inputs.read_text())
    if digest(pose) != inputs['pose_config_sha256']:
        raise ValueError('The current camera/actuator correspondence differs from the reuse audit')
    camera = CameraPoseModel(pose)
    calibrated_source = Path(camera.config['model_file'])
    if not calibrated_source.is_absolute():
        calibrated_source = pose.parent/calibrated_source
    calibrated_joints = {j.get('name'):j for j in ET.parse(calibrated_source).getroot().findall('joint')}
    reused_origins = {}
    for joint in original.findall('joint'):
        calibrated = calibrated_joints.get(joint.get('name'))
        if calibrated is None:
            continue
        for tag in ('parent','child','axis'):
            a,b = joint.find(tag),calibrated.find(tag)
            if (a.attrib if a is not None else {}) != (b.attrib if b is not None else {}):
                raise ValueError('Frozen camera and CAD joint topology differ: '+joint.get('name'))
        a,b = joint.find('origin'),calibrated.find('origin')
        if a is not None and b is not None and a.attrib != b.attrib:
            reused_origins[joint.get('name')] = {'cached':dict(a.attrib),'calibrated':dict(b.attrib)}
            a.attrib.clear()
            a.attrib.update(b.attrib)
    package = Path(selected['description_package'])
    geometry = package/'config/geometry/freecad_kinematics.json'
    if digest(geometry) != inputs['cad_metadata']['source_kinematics_sha256']:
        raise ValueError('The selected CAD linkage differs from the confirmed source')
    endpoint_path = args.gripper_endpoints.resolve()
    endpoints = json.loads(endpoint_path.read_text())
    if endpoints['source_cad_sha256'] != json.loads(geometry.read_text())['source_sha256']:
        raise ValueError('CAD endpoints belong to another original assembly')
    config_dir = root/'configs/robot'
    description = root/'ros2_ws/src/cleany_description'
    mesh_dir = description/'meshes/cad_runtime_collision'
    mesh_dir.mkdir(exist_ok=True)
    copied = {}
    for mesh in original.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://cleany_description/'):
            relative = uri.split('package://cleany_description/',1)[1]
            source_mesh = package/relative
            destination = description/relative
            if not destination.is_file() or digest(destination) != digest(source_mesh):
                raise ValueError('Existing CAD visual asset differs from the release: '+relative)
            continue
        if not uri.startswith('file://'):
            raise ValueError('Unexpected verified CAD cache asset URI: '+uri)
        source_mesh = Path(unquote(urlsplit(uri).path))
        sha = digest(source_mesh)
        destination = mesh_dir/(sha+source_mesh.suffix)
        if destination.exists():
            if digest(destination) != sha:
                raise ValueError('CAD runtime cache asset changed: '+str(destination))
        else:
            shutil.copy2(source_mesh,destination)
        mesh.set('filename','package://cleany_description/meshes/cad_runtime_collision/'+destination.name)
        copied[str(destination.relative_to(root))] = {'source':str(source_mesh),'sha256':sha}
    exact_self_links = []
    for link in original.findall('link'):
        if not link.get('name', '').startswith(('cad_left_gripper_driving_', 'cad_right_gripper_driving_')):
            continue
        # Keep the cached solid enclosure for world obstacles. Self collision
        # needs the original concave rod surface; a convex hull fills its bend.
        for collision in link.findall('collision'):
            collision.set('name', 'cad_world__'+collision.get('name'))
        for index, visual in enumerate(link.findall('visual')):
            collision = ET.SubElement(link, 'collision',
                name='cad_self__source_'+link.get('name')+'_'+str(index))
            for tag in ('origin', 'geometry'):
                collision.append(deepcopy(visual.find(tag)))
        exact_self_links.append(link.get('name'))
    source_path = config_dir/'cad_grasp_source.urdf'
    source_path.write_text(ET.tostring(original,encoding='unicode'))
    primitive_source = Path(selected['curobo_primitives'])
    primitive_path = description/'config/geometry/cad_runtime_gpu_primitives.json'
    shutil.copy2(primitive_source,primitive_path)
    metadata = match_gripper_endpoints(inputs['cad_metadata'], endpoints)
    metadata.update(gpu_primitives_uri='package://cleany_description/config/geometry/cad_runtime_gpu_primitives.json',
                    gpu_primitives_sha256=digest(primitive_path),
                    source_model_sha256=digest(source_path))
    state = prepare_state_profile(args.base_profile,root,output/'limits-source')
    limits = {j.get('name'):{k:float(j.find('limit').get(k)) for k in ('lower','upper','effort','velocity')}
              for j in ET.parse(state.model).getroot().findall('joint') if j.get('type')=='revolute'}
    binding = {'schema':'cleany.cad_grasp_binding/v1','pose_config_sha256':digest(pose),
        'source_model_sha256':digest(source_path),'joint_coordinates':inputs['coordinates'],
        'cad_metadata':metadata,
        'joint_dynamics':{n:{k:v[k] for k in ('effort','velocity')} for n,v in limits.items()},
        'reused_release':str(package),'source_artifacts':selected['files_sha256'],
        'calibrated_geometry_model':str(calibrated_source),
        'calibrated_geometry_model_sha256':digest(calibrated_source),
        'gripper_endpoint_file':str(endpoint_path.relative_to(root)),
        'gripper_endpoint_sha256':digest(endpoint_path),
        'exact_self_collision_links':exact_self_links,
        'reused_calibrated_origins':reused_origins,
        'motion_validation':{'ready':False,'reason':'Full-opening native CAD collision verification is required'},
        'collision_assets':copied,'new_mesh_fit_performed':False}
    binding_path = config_dir/'cad_grasp_binding.json'
    binding_path.write_text(json.dumps(binding,indent=2)+'\n')
    profile.update(model_variant='freecad_calibrated',robot_model=str(source_path.relative_to(root)),
                   cad_binding_file=str(binding_path.relative_to(root)),
                   status='calibrated_cad_with_existing_actuator_coordinates')
    profile.pop('rigid_accessory_files',None)
    profile_path = config_dir/'cad_grasp.json'
    profile_path.write_text(json.dumps(profile,indent=2)+'\n')
    candidate, model_report = calibrated_cad_model(source_path.read_text(),binding['joint_coordinates'],
        limits,metadata,abs(profile['grasp_tcp_y_m']))
    (output/'candidate-cad.urdf').write_text(candidate)
    collection = json.loads(args.collection.read_text())
    for side in ('left','right'):
        collection['arms'][side]['touch_links'] = gripper_touch_links(candidate,side)
    collection_path = config_dir/'collection_cad.json'
    collection_path.write_text(json.dumps(collection,indent=2)+'\n')
    result={'state_profile':str(profile_path),'collection_profile':str(collection_path),
            'binding_file':str(binding_path),'model_report':model_report,
            'source_release':str(package),'copied_verified_meshes':len(copied),
            'new_collision_geometry_created':False,'camera_calibration_changed':False,
            'reused_calibrated_origins':reused_origins,
            'motor_calibration_changed':False,'gripper_width_control_changed':False,
            'operator_settings_activated':False,'motor_commands_sent':0}
    (output/'preparation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
