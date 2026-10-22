"""Check Linux socket evidence and transport profile isolation."""
import importlib.util
from pathlib import Path
import socket
import xml.etree.ElementTree as ET


PATH = Path(__file__).resolve().parents[2]/'tools/validation/dds_buffer_sweep.py'
SPEC = importlib.util.spec_from_file_location('dds_buffer_sweep', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_socket_evidence_matches_own_socket_and_leaves_it_open():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 212992)
        rows = MODULE.own_udp_sockets()
        row = next(r for r in rows if r['address'] == list(receiver.getsockname()))
        assert row['rcvbuf_bytes'] == receiver.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        assert row['drops'] == 0
        assert row['rx_queue_bytes'] == 0
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b'still open', receiver.getsockname())
        receiver.settimeout(1.)
        assert receiver.recv(64) == b'still open'


def test_profile_copy_preserves_network_and_source(tmp_path):
    source = PATH.parents[2]/'containers/vision/cyclonedds/host.xml'
    before = source.read_bytes()
    target = tmp_path/'receiver.xml'
    MODULE.receiver_profile(source, target, 212992)
    original, modified = ET.fromstring(before), ET.parse(target).getroot()
    tag = f'.//{{{MODULE.NS}}}SocketReceiveBufferSize'
    assert modified.find(tag).attrib == {'min': 'default', 'max': '212992B'}
    modified.find(tag).attrib = original.find(tag).attrib
    assert ET.tostring(modified) == ET.tostring(original)
    assert source.read_bytes() == before


def test_report_excludes_startup_windows_and_distinguishes_disabled_camera():
    from collections import defaultdict

    spec = importlib.util.spec_from_file_location('dds_buffer_report', PATH.with_name('dds_buffer_report.py'))
    report = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(report)
    manifest = dict(config=dict(window_s=1., streams=[
        dict(topic='/camera', label='camera', hz=15), dict(topic='/wrist', label='wrist', hz=15)]),
        finished=True, duration_s=5.01, writer=dict(complete=True),
        udp_before=dict(RcvbufErrors=100), udp_after=dict(RcvbufErrors=111))
    records = defaultdict(list)
    records['arrival'] = [dict(topic='/camera', t=t, gap_ms=gap, stamp_age_ms=10.)
                          for t, gap in [(1.1, None), (2.1, 1000.), (3., 900.), (4., 1000.)]]
    records['window'] = [dict(t=t, streams=[
        dict(topic='/camera', hz=hz, status='OBSERVED'), dict(topic='/wrist', hz=0., status='OFF')])
        for t, hz in [(2.5, 0.), (3.5, 15.), (4.5, 14.)]]
    records['udp_sockets'] = [dict(t=t, sockets=[dict(inode=3, drops=drops, rcvbuf_bytes=425984)])
                              for t, drops in [(0., 1), (2., 5), (5., 11)]]
    result = report.summarize(manifest, records, 2., 5.)
    assert result['metrics']['/camera']['minimum_rolling_hz'] == 14.
    assert result['metrics']['/camera']['mean_hz'] == 1.
    assert result['metrics']['/wrist']['status'] == 'OFF'
    assert result['metrics']['/wrist']['mean_hz'] is None
    assert result['metrics']['/wrist']['max_silence_ms'] is None
    assert result['socket_drops_full'] == 11
    assert result['socket_drops_after_warmup'] == 6
    assert result['complete'] and result['idle_observed']
    records['command'].append(dict(record=dict(owner_alive=True, state='ACTIVE')))
    manifest['writer']['complete'] = False
    result = report.summarize(manifest, records, 2., 5.)
    assert not result['idle_observed'] and not result['complete']
