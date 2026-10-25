"""Add a physical cleanup fixture and derive a reachable approach on the selected desk."""
import json
import math
from pathlib import Path
import numpy as np
import yaml


def prepare_cleanup(output: Path, source: Path, *, at_desk: bool = False,
                    object_name: str = 'cup', profile: str = 'canonical'):
    if profile not in ('canonical', 'slender-cup-95mm'):
        raise ValueError('Unknown physical cleanup profile')
    preset = {}
    if profile != 'canonical':
        profiles = Path(__file__).parent/'profiles'
        preset = json.loads((profiles/f'{profile}.json').read_text())
        if object_name != preset['object_name']:
            raise ValueError('Cleanup profile requires its declared object')
    standoff = preset.get('approach_standoff_m', .55)
    path = output/'config/room.json'
    room = json.loads(path.read_text())
    if room['world'] != 'facility18':
        raise ValueError('Physical cleanup fixture requires Facility 18')
    target = np.asarray(room['target'])
    forward = np.array([math.cos(target[2]), math.sin(target[2])])
    left = np.array([-forward[1], forward[0]])
    geometry = json.loads((output/'config/world_geometry.json').read_text())
    desks = []
    for entry in geometry:
        if entry['kind'] != 'collision' or entry['name'] != 'desk_envelope_collision':
            continue
        transform = np.asarray(entry['transform'])
        delta = transform[:2,3]-target[:2]
        if delta@forward > 0 and abs(delta@left) < .6:
            desks.append((float(delta@forward), entry))
    if not desks:
        raise ValueError('No desk ahead of target')
    _, desk = min(desks, key=lambda value: value[0])
    transform = np.asarray(desk['transform'])
    half = np.asarray(desk['shape']['size'])/2
    # The original seat pose is a navigation stop behind the chair. Moving
    # straight toward the desk can intersect that chair. Select an accessible
    # edge using the unchanged facility occupancy map, before placing the fixture.
    from PIL import Image
    from scipy.ndimage import distance_transform_edt
    metadata = yaml.safe_load((output/room['map_file']).read_text())
    map_path = (output/room['map_file']).parent/Path(metadata['image']).name
    pixels = np.asarray(Image.open(map_path))
    clearance = distance_transform_edt(pixels > 250)*metadata['resolution']
    origin, resolution = metadata['origin'], metadata['resolution']
    rotation = transform[:2,:2]
    local_target = rotation.T@(target[:2]-transform[:2,3])
    candidates = []
    for axis in (0,1):
        other = 1-axis
        for sign in (-1,1):
            local = np.zeros(2)
            local[axis] = sign*(half[axis]+standoff)
            local[other] = np.clip(local_target[other], -half[other]+.22, half[other]-.22)
            point = transform[:2,3]+rotation@local
            column = int((point[0]-origin[0])/resolution)
            row = pixels.shape[0]-1-int((point[1]-origin[1])/resolution)
            if not (0 <= row < pixels.shape[0] and 0 <= column < pixels.shape[1]):
                continue
            margin = float(clearance[row,column])
            if margin < .50:
                continue
            direction = -sign*rotation[:,axis]
            yaw = math.atan2(direction[1],direction[0])
            candidates.append((float(np.linalg.norm(point-target[:2])),point,direction,yaw,margin))
    if not candidates:
        raise ValueError('No desk edge has sufficient navigation clearance for physical cleanup')
    _, approach, forward, yaw, margin = min(candidates,key=lambda value:value[0])
    left = np.array([-forward[1],forward[0]])
    # Keep the canonical cup fully supported near the working edge and within
    # the pose-constrained arm workspace.
    edge_inset = .05
    cup_xy = approach+forward*(standoff+edge_inset)-left*.10
    bins = yaml.safe_load((source/'ros2_ws/src/cleany_manipulation_bt/config/manipulation_bins.yaml').read_text())
    bins['simulation_ignore_mast_collision'] = False
    layout = yaml.safe_load((source/'ros2_ws/src/cleany_mujoco_sim/config/study_cafe_layout.yaml').read_text())
    payload = next(item for item in layout['mujoco']['tabletop_objects'] if item['name'] == object_name)
    if payload['visual_type'] not in ('paper_cup','crumpled_tissue'):
        raise ValueError('Unsupported physical cleanup fixture')
    original_height = payload['collision']['size_m'][-1]
    height = preset.get('cup_height_m', original_height)
    payload['collision']['size_m'][-1] = height
    room['park_joints']['head_tilt_joint'] = .15
    room['cleanup'] = {'desk_model':desk['model'], 'original_navigation_target':room['target'],
        'payload_position':[*map(float,cup_xy),float(transform[2,3]+half[2]+height/2+.002)],
        'payload':payload, 'camera_focal_length_mm':16., 'camera_resolution':[960,720],
        'payload_edge_inset_m':edge_inset,
        'bins':bins['bins'], 'camera_hz':5, 'approach_map_clearance_m':margin,
        'actuation':'unmodified ROS controllers; contact-only grasp; no object attachments'}
    if preset:
        room['physics_hz'] = preset['physics_hz']
        room['cleanup']['fixture_variant'] = {
            'name': profile, 'baseline_height_m': original_height,
            'scope': preset['scope']}
        room['cleanup']['physics'] = preset['physics']
        room['cleanup']['navigation_parameters'] = preset.get('navigation_parameters', {})
        room['cleanup']['launch_parameters'] = preset['launch_parameters']
        (output/'config/coordinator.yaml').write_text(
            (profiles/f'{profile}-coordinator.yaml').read_text())
    room['target'] = [*map(float,approach),yaw]
    if at_desk:
        room['spawn'] = [*map(float,approach),.38]
        room['spawn_yaw'] = yaw
    targets = yaml.safe_load((output/'config/targets.yaml').read_text())
    targets['seats'][room['target_id']] = room['target']
    (output/'config/targets.yaml').write_text(yaml.safe_dump(targets))
    path.write_text(json.dumps(room,indent=2)+'\n')
    (output/'config/cleanup_bins.yaml').write_text(yaml.safe_dump(bins))
    return room
