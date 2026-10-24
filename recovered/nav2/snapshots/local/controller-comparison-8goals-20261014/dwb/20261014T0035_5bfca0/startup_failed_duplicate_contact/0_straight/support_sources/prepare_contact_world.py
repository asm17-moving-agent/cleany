"""Add observation-only ContactSystem and one sensor per actual robot collision."""
import json,xml.etree.ElementTree as ET
from pathlib import Path

def instrument(out):
 path=out/'world.sdf';tree=ET.parse(path);world=tree.getroot().find('world')
 ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='gz::sim::systems::Contact')
 robot=world.find("model[@name='cleany_mecanum']");records=[]
 for link in robot.findall('link'):
  for collision in link.findall('collision'):
   index=len(records);topic='/benchmark/contacts/c'+str(index)
   sensor=ET.SubElement(link,'sensor',name='benchmark_contact_'+str(index),type='contact')
   contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text=collision.get('name');ET.SubElement(contact,'topic').text=topic
   ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='20'
   records.append({'topic':topic,'link':link.get('name'),'collision':collision.get('name'),'nominal_update_rate_hz':20})
 tree.write(path,encoding='unicode',xml_declaration=True)
 (out/'contact_instrumentation.json').write_text(json.dumps({'robot_model':'cleany_mecanum','sensors':records,'scope':'physics-engine contact reports for every robot collider; no collision shape/physics altered','ground_model':'ground_plane','sampling_note':'Sensor update_rate setting20Hz; new common physics timestep.001s; persistent contact expected1000Hz at physics timestep; floor-wheel JSON thinned20Hz only, raw bag unthrottled; DDS loss possible; no physical proof'},indent=2))
 return [x['topic'] for x in records]
