from pathlib import Path
import xml.etree.ElementTree as ET


def test_large_rgbd_profile_keeps_udp_and_bounded_shared_memory_without_qos_override():
    source = Path(__file__).parents[1] / 'config' / 'fastdds_rgbd.xml'
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(source).getroot()
    transports = root.findall('d:transport_descriptors/d:transport_descriptor', ns)
    by_type = {item.findtext('d:type', namespaces=ns): item for item in transports}
    assert set(by_type) == {'UDPv4', 'SHM'}
    assert int(by_type['SHM'].findtext('d:segment_size', namespaces=ns)) == 16*1024*1024
    participant = root.find('d:participant', ns)
    assert participant.attrib['is_default_profile'] == 'true'
    assert participant.findtext('d:rtps/d:useBuiltinTransports', namespaces=ns) == 'false'
    assert len(participant.findall('d:rtps/d:userTransports/d:transport_id', ns)) == 2
    assert not root.findall('.//d:reliability', ns) and not root.findall('.//d:history', ns)
