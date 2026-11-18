"""Reuse the deployed CAD primitive cache for GPU IK proposal costs."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from cleany_description.cad_coordinates import CadCoordinates

from .collision_sphere_model import mesh_path


def configure_cached_cad(model, description, semantic):
    """Full collision acceptance belongs to CalibratedCADFCL in hybrid mode."""
    cad = CadCoordinates.from_urdf(description)
    if cad is None:
        return False
    if model.config.sphere_model != 'cad_cached':
        raise ValueError('Calibrated CAD GPU IK requires its verified primitive cache')
    path = mesh_path(cad.config['gpu_primitives_uri'])
    if hashlib.sha256(path.read_bytes()).hexdigest() != cad.config['gpu_primitives_sha256']:
        raise ValueError('Cached CAD GPU primitives changed')
    profile = json.loads(path.read_text())
    if profile['source_kinematics_sha256'] != cad.config['source_kinematics_sha256']:
        raise ValueError('Cached GPU primitives and nonlinear CAD linkage differ')
    root = ET.fromstring(description)
    owners = {link.get('name') for link in root.findall('link')}
    if not set(profile['links']) <= owners:
        raise ValueError('Cached GPU primitive owner is missing from the CAD model')
    model.cad_coordinates = cad
    model.kinematic_links = [joint.find('child').get('link') for joint in root.findall('joint')
                             if joint.get('type') != 'fixed']
    model.limits, model.velocity_limits = {}, {}
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed' or joint.find('mimic') is not None or joint.get('name') in cad.dependent_names:
            continue
        limit = joint.find('limit')
        model.limits[joint.get('name')] = tuple(float(limit.get(k)) for k in ('lower', 'upper'))
        model.velocity_limits[joint.get('name')] = float(limit.get('velocity'))
    model.spheres = {name: [{'center': item['center'], 'radius': item['radius']} for item in data['spheres']]
                     for name, data in profile['links'].items() if data['spheres'] and name != 'base_link'}
    for values in model.spheres.values():
        for item in values:
            if len(item['center']) != 3 or not all(math.isfinite(v) for v in [*item['center'], item['radius']]) or item['radius'] <= 0:
                raise ValueError('Invalid cached CAD IK sphere')
    model.collision_links = list(model.spheres)
    model.bodies = []
    model.ignore = {name: [] for name in model.collision_links}
    for item in ET.fromstring(semantic).findall('disable_collisions'):
        a, b = item.get('link1'), item.get('link2')
        if a in model.ignore and b in model.ignore:
            model.ignore[a].append(b)
    model.mast_obstacles = {item['name']: {'pose': item['pose'], 'dims': item['dims']}
                            for item in profile['links']['base_link']['cuboids']}
    model.full_geometry_cache_hit = True
    model.sphere_model_report = {'model': 'cad_cached', 'sha256': cad.config['gpu_primitives_sha256'],
                                 'full_collision_acceptance': 'cleany/CalibratedCADFCL',
                                 'new_sphere_fit_performed': False}
    return True
