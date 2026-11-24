"""Write the laptop viewer's Fast DDS profile for the route to the Jetson."""

import argparse
import os
import socket
import tempfile
from ipaddress import IPv4Address
from pathlib import Path


def viewer_profile(peer: IPv4Address, directory: Path) -> Path:
    # UDP connect asks the kernel for the source address without sending a packet.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
        route.connect((str(peer), 7400))
        local = IPv4Address(route.getsockname()[0])

    profile = directory / f"lan-{local}-{peer}.xml"
    xml = f"""<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>wifi</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>{local}</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="viewer" is_default_profile="true"><rtps>
    <builtin><initialPeersList>
      <locator><udpv4><address>{local}</address></udpv4></locator>
      <locator><udpv4><address>{peer}</address></udpv4></locator>
    </initialPeersList></builtin>
    <userTransports><transport_id>wifi</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>
"""
    with tempfile.NamedTemporaryFile(mode="w", dir=directory, delete=False) as staged:
        staged.write(xml)
    os.replace(staged.name, profile)
    return profile


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("peer", type=IPv4Address, help="Jetson LAN IPv4 address")
    parser.add_argument("directory", type=Path, help="existing viewer cache directory")
    args = parser.parse_args()
    print(viewer_profile(args.peer, args.directory))
