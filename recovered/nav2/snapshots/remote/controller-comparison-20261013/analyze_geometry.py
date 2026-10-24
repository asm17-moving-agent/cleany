#!/usr/bin/env python3
"""Conservative XY static-collision projection; never a Gazebo contact measurement.

Run using geometry-venv/bin/python analyze_geometry.py RUN [RUN ...].
The physical CAD footprint is used without safety/planning padding. All collision
volumes whose full z bounds intersect the robot's full z bounds are projected.
Convex mesh projections can fill openings; overlapping projections do not prove
3D contact. Geometry is evaluated at recorded samples, not swept between them.
"""
import argparse
from collections import Counter
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.geometry import MultiPoint, Polygon
from shapely.strtree import STRtree


def pose_matrix(element):
    p = element.find('pose')
    values = [float(x) for x in (p.text or '').split()] if p is not None else []
    if not values:
        return np.eye(4)
    t = np.eye(4)
    t[:3, 3] = values[:3]
    if p.get('rotation_format', 'euler_rpy') == 'quat_xyzw':
        x, y, z, w = values[3:7]
        norm = math.sqrt(x*x+y*y+z*z+w*w)
        if norm == 0:
            raise ValueError('zero quaternion')
        x, y, z, w = (v/norm for v in (x,y,z,w))
        t[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    else:
        r, pch, y = values[3:6]
        if p.get('degrees', 'false') == 'true':
            r, pch, y = map(math.radians, (r,pch,y))
        cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(pch),math.sin(pch),math.cos(y),math.sin(y)
        t[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                    [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    return t


class Frames:
    """Resolve named SDF model/link/joint/frame poses, with scoped relative_to."""
    def __init__(self, world):
        self.nodes = {'world': (None, '', '', 'world')}
        self.cache = {'world': np.eye(4)}
        self.collisions = []
        self.includes = []
        for frame in world.findall('frame'):
            self.add(frame.get('name'), frame, '', frame.get('attached_to', 'world'))
        for model in world.findall('model'):
            self.model(model, '', False)

    def add(self, key, element, scope, default):
        p = element.find('pose')
        relative = p.get('relative_to', default) if p is not None else default
        self.nodes[key] = (element, scope, relative, default)

    def model(self, model, parent, inherited_static):
        key = '::'.join(filter(None, (parent, model.get('name'))))
        self.add(key, model, parent, parent or 'world')
        static = inherited_static or model.findtext('static', 'false').strip() == 'true'
        for item in model:
            if item.tag in ('link', 'joint', 'frame'):
                item_key = key+'::'+item.get('name')
                default = key
                if item.tag == 'frame':
                    default = item.get('attached_to', '__model__')
                elif item.tag == 'joint':
                    default = item.findtext('child', '__model__')
                self.add(item_key, item, key, default)
                if item.tag == 'link':
                    for c in item.findall('collision'):
                        ck = item_key+'::@'+c.get('name', 'collision')
                        self.add(ck, c, key, item_key)
                        self.collisions.append((ck, key, c, static))
            elif item.tag == 'model':
                self.model(item, key, static)
            elif item.tag == 'include':
                self.includes.append({'model': key, 'uri': item.findtext('uri'), 'static_parent': static})

    def ref(self, name, scope):
        if name in ('world', '__world__'):
            return 'world'
        if name == '__model__':
            return scope or 'world'
        if name.startswith('::') and name[2:] in self.nodes:
            return name[2:]
        current = scope
        while current:
            candidate = current+'::'+name
            if candidate in self.nodes:
                return candidate
            current = current.rsplit('::',1)[0] if '::' in current else ''
        if name in self.nodes:
            return name
        raise ValueError(f'unresolved relative_to {name!r} in {scope!r}')

    def get(self, key, active=()):
        if key in self.cache:
            return self.cache[key]
        if key in active:
            raise ValueError('pose cycle: '+' -> '.join((*active,key)))
        element, scope, relative, _ = self.nodes[key]
        self.cache[key] = self.get(self.ref(relative,scope), (*active,key)) @ pose_matrix(element)
        return self.cache[key]


def resolve_mesh(uri, world_dir, cache_roots):
    parsed = urlparse(uri)
    if parsed.scheme in ('', 'file'):
        path = Path(unquote(parsed.path))
        return path if path.is_absolute() else world_dir/path
    if parsed.scheme in ('http','https'):
        parts = unquote(parsed.path).strip('/').split('/')
        # Fuel URL: 1.0/OWNER/models/NAME/VERSION/files/meshes/FILE.
        if len(parts)>=7 and parts[2]=='models' and parts[5]=='files':
            tail = Path(parsed.netloc.lower(),parts[1].lower(),'models',parts[3].lower(),parts[4],*parts[6:])
            for root in cache_roots:
                p = root/tail
                if p.exists():
                    return p
    raise ValueError('mesh URI unavailable in local cache: '+uri)


def mesh_vertices(path):
    if path.suffix.lower() == '.obj':
        points = []
        with path.open() as f:
            for line in f:
                if line.startswith('v '):
                    points.append([float(x) for x in line.split()[1:4]])
        if not points:
            raise ValueError('OBJ has no vertices')
        return np.asarray(points)
    if path.suffix.lower() == '.stl':
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0] if len(data)>=84 else 0
        if len(data)==84+50*count:
            return np.asarray([struct.unpack_from('<fff',data,84+50*i+12+12*j)
                               for i in range(count) for j in range(3)])
        points = [[float(x) for x in line.split()[1:4]] for line in data.decode().splitlines()
                  if line.strip().startswith('vertex ')]
        if points:
            return np.asarray(points)
    raise ValueError('only OBJ and STL mesh vertices supported: '+str(path))


def project_geometry(geometry, transform, world_dir, cache_roots, mesh_cache):
    kind = list(geometry)[0].tag if geometry is not None and len(geometry) else 'missing'
    shape = geometry.find(kind) if geometry is not None else None
    source = None
    if kind == 'box':
        size = np.asarray([float(x) for x in shape.findtext('size').split()])
        points = np.asarray(list(itertools.product(*[[-v/2,v/2] for v in size])))
    elif kind == 'cylinder':
        r, length = float(shape.findtext('radius')),float(shape.findtext('length'))
        n = 96
        # Circumscribed regular polygon, so the cylinder is contained, not shrunk.
        r /= math.cos(math.pi/n)
        points = np.asarray([[r*math.cos(a),r*math.sin(a),z] for a in np.arange(n)*2*math.pi/n
                             for z in (-length/2,length/2)])
    elif kind == 'sphere':
        r = float(shape.findtext('radius'))
        xy = transform[:2,3]
        n = 96
        poly = Polygon([(xy[0]+r/math.cos(math.pi/n)*math.cos(a),xy[1]+r/math.cos(math.pi/n)*math.sin(a))
                        for a in np.arange(n)*2*math.pi/n])
        return kind,poly,[float(transform[2,3]-r),float(transform[2,3]+r)],source
    elif kind == 'mesh':
        if shape.find('submesh') is not None:
            raise ValueError('mesh submesh selection unsupported')
        path = resolve_mesh(shape.findtext('uri'),world_dir,cache_roots)
        if str(path) not in mesh_cache:
            mesh_cache[str(path)] = mesh_vertices(path)
        scale = np.asarray([float(x) for x in shape.findtext('scale','1 1 1').split()])
        points = mesh_cache[str(path)]*scale
        source = {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                  'projection':'convex hull of all mesh vertices (openings filled conservatively)'}
    else:
        raise ValueError('unsupported geometry: '+kind)
    world = points @ transform[:3,:3].T + transform[:3,3]
    return kind, MultiPoint(world[:,:2]).convex_hull,[float(world[:,2].min()),float(world[:,2].max())],source


def analyze(run, cache_roots):
    scenario = json.loads((run/'scenario.json').read_text())
    metadata = json.loads((run/'world.model.json').read_text())
    physical = metadata.get('navigation_geometry',{}).get('physical')
    physical_source = 'world.model.json.navigation_geometry.physical'
    if physical is None:
        physical = scenario['model_metadata']['navigation_geometry']['physical']
        physical_source = 'scenario.json.model_metadata.navigation_geometry.physical'
    spawn = scenario['spawn_pose']
    if abs(spawn[3])+abs(spawn[4])>1e-8:
        raise ValueError('GT samples provide yaw only; nonzero spawn roll/pitch unsupported')
    bounds = metadata['collision_bounds']
    zrange = [spawn[2]+bounds[0][2],spawn[2]+bounds[1][2]]
    frames = Frames(ET.parse(run/'world.sdf').getroot().find('world'))
    obstacles, ignored, unsupported = [],Counter(),[]
    mesh_cache = {}
    for key, model, collision, static in frames.collisions:
        if not static:
            ignored['nonstatic_collision']+=1
            continue
        if model == 'ground_plane' or model.startswith('ground_plane::'):
            ignored['ground_plane_collision']+=1
            continue
        try:
            kind, poly, z, mesh = project_geometry(collision.find('geometry'),frames.get(key),run,cache_roots,mesh_cache)
            if z[1]<zrange[0] or z[0]>zrange[1]:
                ignored['outside_robot_z_range']+=1
                continue
            if poly.is_empty or poly.area<=0:
                raise ValueError('empty or degenerate XY projection')
            obstacles.append({'name':key,'geometry':kind,'polygon':poly,'z_range_m':z,'mesh':mesh})
        except (ValueError,OSError,TypeError) as e:
            unsupported.append({'name':key,'reason':str(e)})
    tree = STRtree([o['polygon'] for o in obstacles]) if obstacles else None
    rows = []
    rejected = 0
    c,s = math.cos(spawn[5]),math.sin(spawn[5])
    for row in csv.DictReader((run/'samples.csv').open()):
        try:
            t,x,y,yaw = (float(row[k]) for k in ('sim_s','gt_x','gt_y','gt_yaw'))
            if not all(map(math.isfinite,(t,x,y,yaw))):
                raise ValueError('nonfinite sample')
        except ValueError:
            rejected+=1
            continue
        wx,wy,wyaw = spawn[0]+c*x-s*y,spawn[1]+s*x+c*y,spawn[5]+yaw
        cy,sy = math.cos(wyaw),math.sin(wyaw)
        fp = Polygon([(wx+cy*a-sy*b,wy+sy*a+cy*b) for a,b in physical])
        nearest = int(tree.nearest(fp)) if tree is not None else None
        clearance = fp.distance(obstacles[nearest]['polygon']) if nearest is not None else None
        overlaps = [(int(i),fp.intersection(obstacles[int(i)]['polygon']).area)
                    for i in tree.query(fp,predicate='intersects')] if tree is not None else []
        positive = [(i,a) for i,a in overlaps if a>1e-12]
        rows.append({'sim_s':t,'phase':row.get('phase',''),'clearance_m':clearance,
                     'nearest':obstacles[nearest]['name'] if nearest is not None else None,
                     'overlap_area_m2':max((a for _,a in positive),default=0.),
                     'overlap_obstacles':[obstacles[i]['name'] for i,_ in positive],
                     'world_xy_yaw':[wx,wy,wyaw]})

    def summary(selected):
        finite = [r for r in selected if r['clearance_m'] is not None]
        minimum = min(finite,key=lambda r:r['clearance_m']) if finite else None
        overlap = [r for r in selected if r['overlap_area_m2']>0]
        return {'sample_count':len(selected),'min_projected_clearance_m':minimum['clearance_m'] if minimum else None,
                'minimum_sample':minimum,'projected_overlap_sample_count':len(overlap),
                'max_projected_overlap_area_m2':max((r['overlap_area_m2'] for r in selected),default=None),
                'first_projected_overlap_sample':overlap[0] if overlap else None,
                'overlap_obstacles':sorted({name for r in overlap for name in r['overlap_obstacles']})}
    output = {'metric':'conservative projected static collision geometry vs physical CAD footprint',
              'actual_contact':'unknown: no Gazebo contact instrumentation in this analysis',
              'completeness':'partial_unknown_geometry' if unsupported or frames.includes else 'all_static_collisions_supported',
              'footprint_source':physical_source,'physical_footprint_m':physical,'spawn_pose':spawn,
              'robot_world_z_range_m':zrange,'obstacle_count':len(obstacles),
              'supported_geometry_counts':dict(Counter(o['geometry'] for o in obstacles)),
              'ignored_counts':dict(ignored),'unsupported_geometry_count':len(unsupported),
              'unsupported_geometry':unsupported,'unexpanded_includes':frames.includes,
              'rejected_gt_sample_count':rejected,'all_phases':summary(rows),
              'navigation':summary([r for r in rows if r['phase']=='comparison_navigation']),
              'per_phase':{p:summary([r for r in rows if r['phase']==p]) for p in sorted({r['phase'] for r in rows})},
              'mesh_sources':list({o['mesh']['path']:o['mesh'] for o in obstacles if o['mesh']}.values()),
              'limitations':['XY projection and full-height robot bounding prism can overstate overlap at different heights.',
                             'Mesh convex hull fills concavities and gaps; cylinder/sphere use circumscribed 96-gons.',
                             'GT samples transformed from map to world using spawn yaw; fixed spawn z used (no live roll/pitch/z).',
                             'Static SDF poses only; dynamic obstacles, geometry changes, and inter-sample swept contact unknown.',
                             'Distance is nonnegative; projected overlap is reported as area, not signed penetration or contact.'],
              'versions':{'numpy':np.__version__,'shapely':shapely.__version__},
              'input_sha256':{name:hashlib.sha256((run/name).read_bytes()).hexdigest()
                              for name in ('world.sdf','world.model.json','scenario.json','samples.csv')}}
    (run/'geometry_summary.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    return output


def self_test():
    a = Polygon([(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)])
    b = Polygon([(1.5,-.5),(2.5,-.5),(2.5,.5),(1.5,.5)])
    assert abs(a.distance(b)-1.)<1e-12
    rot = np.asarray([[math.cos(math.pi/4),-math.sin(math.pi/4)],[math.sin(math.pi/4),math.cos(math.pi/4)]])
    square = Polygon(np.asarray(a.exterior.coords) @ rot.T)
    assert abs(square.distance(b)-(1.5-math.sqrt(.5)))<1e-12
    assert abs(a.intersection(Polygon([(0,-.5),(1,-.5),(1,.5),(0,.5)])).area-.5)<1e-12
    w = ET.fromstring('<world><model name="outer"><static>true</static><pose>1 2 0 0 0 1.5707963267948966</pose><frame name="f"><pose>2 0 0 0 0 0</pose></frame><model name="inner"><pose relative_to="f">0 1 0 0 0 0</pose><link name="body"><pose>0 0 1 0 0 0</pose><collision name="c"><pose relative_to="body">1 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></model></world>')
    frames = Frames(w)
    assert np.allclose(frames.get('outer::inner::body::@c')[:3,3],[0,5,1])
    # A sibling relative_to referenced from a nested scope must resolve outward.
    assert np.allclose(frames.get('outer::f')[:3,3],[1,4,0])
    g = ET.fromstring('<geometry><box><size>2 4 6</size></box></geometry>')
    _,p,z,_ = project_geometry(g,np.eye(4),Path('.'),[],{})
    assert abs(p.area-8)<1e-12 and z==[-3,3]
    print('self-test passed: rectangle gap, rotated polygon gap, overlap area, nested relative_to poses, box projection')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='*')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--fuel-cache',type=Path,action='append',default=[])
    args = parser.parse_args()
    if args.self_test:
        self_test()
    cache_roots = args.fuel_cache or [Path.home()/'.ignition/fuel',Path.home()/'.gz/fuel']
    for run in args.runs:
        result = analyze(run.resolve(),cache_roots)
        print(json.dumps({'run':str(run),'navigation':result['navigation'],
                          'obstacle_count':result['obstacle_count'],'unsupported':result['unsupported_geometry_count']}))


if __name__ == '__main__':
    main()
