"""Explicit Gazebo/ROS test stimuli; never modify runtime safety parameters."""
from __future__ import annotations
import math
import json
import hashlib
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def add_obstacle(world_file: Path) -> str:
    tree=ET.parse(world_file);world=tree.getroot().find('world')
    name='validation_obstacle'
    model=ET.SubElement(world,'model',name=name)
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text='50 50 .4 0 0 0'
    link=ET.SubElement(model,'link',name='body')
    for kind in ('collision','visual'):
        shape=ET.SubElement(link,kind,name=kind)
        box=ET.SubElement(ET.SubElement(shape,'geometry'),'box')
        ET.SubElement(box,'size').text='.25 .65 .8'
    # Keep this controllable entity out of the immutable static scene grouping.
    ET.SubElement(world,'frame',name='validation_obstacle_frame',attached_to=name)
    tree.write(world_file,encoding='utf-8',xml_declaration=True)
    return world.get('name')


def move_obstacle(world: str, x: float, y: float, yaw: float) -> dict:
    req=f'name: "validation_obstacle" position: {{x: {x} y: {y} z: .4}} orientation: {{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}'
    command=['ign','service','-s',f'/world/{world}/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',req]
    response=subprocess.run(command,text=True,capture_output=True,timeout=6)
    if response.returncode or 'true' not in response.stdout:
        raise RuntimeError(f'Gazebo set_pose failed: {response.stdout} {response.stderr}')
    return dict(command=command,response=response.stdout)


def entity_request(world: str, service: str, request_type: str, request: str) -> dict:
    command=['ign','service','-s',f'/world/{world}/{service}','--reqtype',request_type,'--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request]
    response=subprocess.run(command,text=True,capture_output=True,timeout=6)
    if response.returncode or 'true' not in response.stdout:
        raise RuntimeError(f'Gazebo {service} failed: {response.stdout} {response.stderr}')
    return {'command':command,'response':response.stdout}


def switch_chair(world: str, restored_model: ET.Element, remove_name: str) -> list[dict]:
    sdf=ET.Element('sdf',version='1.8');sdf.append(restored_model)
    restore=entity_request(world,'create','ignition.msgs.EntityFactory',
                           'sdf: '+json.dumps(ET.tostring(sdf,encoding='unicode'))+' allow_renaming: false')
    remove=entity_request(world,'remove','ignition.msgs.Entity',f'name: "{remove_name}" type: MODEL')
    return [restore,remove]


def scene_model_names(text: str) -> set[str]:
    """Read top-level model names from transport's protobuf text response."""
    tokens=re.findall(r'"(?:\\.|[^"\\])*"|[A-Za-z_][A-Za-z_0-9]*|[{}:]',text)
    stack=[];names=set();models=0
    for index,token in enumerate(tokens):
        if token=='{':
            if index==0:raise ValueError('Malformed scene response')
            stack.append(tokens[index-1])
            if stack==['model']:models+=1
        elif token=='}':
            if not stack:raise ValueError('Malformed scene response')
            stack.pop()
        elif stack==['model'] and token=='name':
            if tokens[index+1:index+2]!=[':'] or index+2>=len(tokens):
                raise ValueError('Missing model name')
            name=json.loads(tokens[index+2])
            if not isinstance(name,str) or name in names:raise ValueError('Invalid or duplicate model name')
            names.add(name)
    if stack or not names or models!=len(names):raise ValueError('Incomplete scene response')
    return names


def verify_chair_models(world: str, all_chairs: set[str], removed: str,
                        output: Path | None=None) -> dict:
    command=['ign','service','-s',f'/world/{world}/scene/info',
             '--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene',
             '--timeout','3000','--req','']
    response=subprocess.run(command,text=True,capture_output=True,timeout=6)
    if response.returncode:raise RuntimeError('Gazebo scene query failed: '+response.stderr)
    if output:
        output.parent.mkdir(parents=True,exist_ok=True)
        with output.open('x') as stream:stream.write(response.stdout)
    models=scene_model_names(response.stdout)
    observed=models & all_chairs;expected=all_chairs-{removed}
    if observed!=expected:
        raise RuntimeError(f'Gazebo chair mismatch: missing={sorted(expected-observed)}, extra={sorted(observed-expected)}')
    return {'removed':removed,'observed_chairs':sorted(observed),'scene_file':str(output) if output else None,
            'model_count':len(models),'command':command,
            'scene_sha256':hashlib.sha256(response.stdout.encode()).hexdigest()}


def obstacle_ahead(current: dict, previous: dict, distance: float) -> list[float]:
    """Use observed world displacement; omni travel need not follow body yaw."""
    dx=current['x']-previous['x'];dy=current['y']-previous['y']
    if not all(math.isfinite(v) for v in (current['x'],current['y'],dx,dy,distance)) or distance<=0 or math.hypot(dx,dy)<=.001:
        raise ValueError('A finite observed motion direction is required')
    angle=math.atan2(dy,dx)
    return [current['x']+distance*math.cos(angle),current['y']+distance*math.sin(angle),angle]
