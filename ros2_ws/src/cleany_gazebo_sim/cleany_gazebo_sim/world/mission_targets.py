"""Generate approximate desk-front simulation goals from a materialized facility.

Requires numpy, scipy, Pillow and PyYAML (available in the Humble environment).
The map must share the SDF world frame. Goals are not calibrated arm targets.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import binary_erosion, distance_transform_edt, label
from scipy.spatial import ConvexHull


def canonical_seat_id(seat: dict) -> str:
    """Match the control-plane room inventory without changing facility model names."""
    if seat["zone_id"] == "d-hub":
        return "seat-"+seat["seat_id"]
    room = seat["zone_id"].removeprefix("space-")
    local_number = seat["seat_id"].removeprefix(room.upper())
    if room not in {"a1", "a2", "a3", "a4", "m1", "m2", "m3"} or not local_number.isdigit():
        raise ValueError(f"unrecognized room seat: {seat}")
    return f"seat-{room}-{int(local_number):02d}"


def generate(world: Path, map_file: Path, home: list[float], mesh_root: Path, overrides: dict[str, list[float]] | None = None,
             half_extents: tuple[float, float] = (.3185, .4074),
             padding: float = .02, front_offset: float = 1.2,
             localization_margin: float = .10, reference_world: Path | None = None) -> tuple[dict, dict]:
    if (any(not math.isfinite(v) or v <= 0 for v in (*half_extents, front_offset))
            or any(not math.isfinite(v) or v < 0 for v in (padding, localization_margin))):
        raise ValueError('body dimensions and standoff must be positive; padding nonnegative')
    metadata = yaml.safe_load(map_file.read_text())
    resolution = float(metadata['resolution'])
    ox, oy, angle = metadata['origin']
    if angle != 0 or metadata.get('negate', 0):
        raise ValueError('requires a non-rotated, non-negated reference map')
    image = Image.open(map_file.parent / metadata['image']).convert('L')
    free = np.asarray(image)[::-1] > 200
    clearance = distance_transform_edt(free) * resolution
    components, _ = label(clearance > .35)
    hx, hy = int((home[0] - ox) / resolution), int((home[1] - oy) / resolution)
    if not (0 <= hy < free.shape[0] and 0 <= hx < free.shape[1]) or components[hy, hx] == 0:
        raise ValueError('home is not in a free component')
    reachable = components == components[hy, hx]
    models = {m.get('name'): m for m in ET.parse(world).findall('./world/model')}
    reference_models = ({m.get('name'): m for m in ET.parse(reference_world).findall('./world/model')}
                        if reference_world is not None else {})
    seats = json.loads(world.with_suffix('.seats.json').read_text())
    # The reference LiDAR-plane raster omits mesh chairs. Add their conservative
    # XY bounding polygons, including the seat/back above the LiDAR plane.
    draw = ImageDraw.Draw(image)
    for seat in seats:
        chair = models.get(seat['chair_model'])
        if chair is None:
            # Missing chairs contribute no collisions. The front direction
            # still needs an explicit layout reference, never an invented yaw.
            if seat['chair_model'] not in reference_models:
                raise ValueError(f'missing chair {seat["chair_model"]}; supply reference_world for its front direction')
            continue
        mp = list(map(float, chair.findtext('pose').split()))
        for collision in chair.findall('./link/collision'):
            mesh = collision.find('./geometry/mesh')
            box = collision.find('./geometry/box')
            cylinder = collision.find('./geometry/cylinder')
            if mesh is not None:
                uri = mesh.findtext('uri')
                obj = mesh_root / uri.split('/files/')[-1]
                if not obj.is_file():
                    raise ValueError(f'missing collision mesh: {obj}')
                vertices = np.array([list(map(float, line.split()[1:4]))
                                     for line in obj.read_text().splitlines() if line.startswith('v ')])
                vertices *= np.array(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                low, high = vertices.min(axis=0), vertices.max(axis=0)
            elif box is not None:
                high = np.array(list(map(float, box.findtext('size').split())))/2
                low = -high
            elif cylinder is not None:
                radius = float(cylinder.findtext('radius'))
                high = np.array([radius, radius, float(cylinder.findtext('length'))/2])
                low = -high
            else:
                raise ValueError('unsupported chair collision geometry')
            cp = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
            def matrix(pose: list[float]) -> np.ndarray:
                roll, pitch, yaw = pose[3:]
                cr, sr = math.cos(roll), math.sin(roll)
                ct, st = math.cos(pitch), math.sin(pitch)
                cy, sy = math.cos(yaw), math.sin(yaw)
                rotation = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array(
                    [[ct,0,st],[0,1,0],[-st,0,ct]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
                result = np.eye(4); result[:3,:3] = rotation; result[:3,3] = pose[:3]
                return result
            link_pose = list(map(float, chair.find('link').findtext('pose', '0 0 0 0 0 0').split()))
            transform = matrix(mp) @ matrix(link_pose) @ matrix(cp)
            corners = np.array([[x,y,z,1] for x in (low[0],high[0])
                                for y in (low[1],high[1]) for z in (low[2],high[2])])
            points = (corners @ transform.T)[:,:2]
            polygon = points[ConvexHull(points).vertices]
            draw.polygon([((x-ox)/resolution, image.height-(y-oy)/resolution)
                          for x,y in polygon], fill=0)
    free_with_chairs = np.asarray(image)[::-1] > 200
    yy, xx = np.indices(free.shape)
    xs, ys = ox+(xx+.5)*resolution, oy+(yy+.5)*resolution
    overrides = overrides or {}
    known_ids = {canonical_seat_id(seat) for seat in seats}
    if set(overrides) - known_ids:
        raise ValueError('override references an unknown seat')
    targets, evidence = {}, {}
    for seat in seats:
        dx, dy = seat['pose'][:2]
        chair = models.get(seat['chair_model'], reference_models.get(seat['chair_model']))
        chair_pose = list(map(float, chair.findtext('pose').split()))
        vx, vy = chair_pose[0]-dx, chair_pose[1]-dy
        length = math.hypot(vx, vy)
        if length < .01:
            raise ValueError(f'chair has no front direction: {seat["seat_id"]}')
        vx, vy = vx/length, vy/length
        yaw = math.atan2(-vy, -vx)
        # CAD half extents + planning padding + one raster-cell diagonal/2.
        # Erode with the entire oriented footprint, not just its center cell.
        margin = padding+localization_margin+resolution/math.sqrt(2)
        extent = math.ceil((max(half_extents)+margin)/resolution)+1
        grid = np.arange(-extent, extent+1)*resolution
        ky, kx = np.meshgrid(grid, grid, indexing='ij')
        c, s = math.cos(yaw), math.sin(yaw)
        footprint = (np.abs(c*kx+s*ky) <= half_extents[0]+margin) & (np.abs(-s*kx+c*ky) <= half_extents[1]+margin)
        valid = binary_erosion(free_with_chairs, structure=footprint, border_value=0) & reachable
        rx, ry = xs-dx, ys-dy
        distance = np.hypot(rx, ry)
        front, side = rx*vx+ry*vy, np.abs(rx*vy-ry*vx)
        score = distance+.6*side+8*np.maximum(0, front_offset-front)
        score[~valid | (distance > 3)] = np.inf
        seat_id = canonical_seat_id(seat)
        if seat_id in overrides:
            x, y, requested_yaw = overrides[seat_id]
            if not all(math.isfinite(v) for v in (x, y, requested_yaw)):
                raise ValueError('override must be finite')
            if abs(math.sin(requested_yaw-yaw)) > 1e-4:
                raise ValueError('override yaw must preserve the desk-normal footprint')
            index = (int((y-oy)/resolution), int((x-ox)/resolution))
            if not (0 <= index[0] < score.shape[0] and 0 <= index[1] < score.shape[1]):
                raise ValueError('override is outside the map')
        else:
            index = np.unravel_index(np.argmin(score), score.shape)
        if not np.isfinite(score[index]):
            raise ValueError(f'no safe desk-front goal within 3 m: {seat["seat_id"]}')
        seat_id = canonical_seat_id(seat)
        if seat_id in overrides:
            expected_yaw = math.atan2(vy, vx) if front[index] < 0 else yaw
            error = requested_yaw-expected_yaw
            if abs(math.atan2(math.sin(error), math.cos(error))) > 1e-4:
                raise ValueError('override must face the desk normal')
        # For an opposite-side fallback, turn toward the desk rather than away.
        if front[index] < 0:
            yaw = math.atan2(vy, vx)
        targets[seat_id] = [round(float(xs[index]), 4), round(float(ys[index]), 4), round(yaw, 6)]
        evidence[seat_id] = {'desk_distance_m': round(float(distance[index]), 3),
                             'front_offset_m': round(float(front[index]), 3),
                             'side_offset_m': round(float(side[index]), 3),
                             'localization_margin_m': localization_margin,
                             'chair_present': seat['chair_model'] in models}
    return dict(schema_version=1, map_id=map_file.stem, frame_id='map', home=home, seats=targets), evidence
