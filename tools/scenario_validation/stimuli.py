"""Explicit Gazebo/ROS test stimuli; never modify runtime safety parameters."""
from __future__ import annotations
import math
import json
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
