"""Read-only FULLTREE/STATUS replies from the actual C++ tree, without stage projection."""
import struct
import uuid


class LiveMonitor:
    def __init__(self) -> None:
        self.tree_id = uuid.uuid4().bytes
        self.snapshot: dict | None = None

    def update(self, snapshot: dict, *, new_execution: bool = False) -> None:
        if new_execution:
            self.tree_id = uuid.uuid4().bytes
        self.snapshot = snapshot

    def reply(self, frames: list[bytes]) -> list[bytes]:
        header = frames[0] if frames else b''
        payload = b''
        if self.snapshot is not None and len(frames) == 1 and len(header) == 6 and header[0] == 2:
            if header[1] == ord('T'):
                payload = self.snapshot['xml'].encode()
            elif header[1] == ord('S'):
                values = {'IDLE': 0, 'RUNNING': 1, 'SUCCESS': 2, 'FAILURE': 3, 'SKIPPED': 4}
                payload = b''.join(struct.pack('<HB', node['uid'], values[node['status']])
                                   for node in self.snapshot['nodes'])
        return [header + self.tree_id, payload]
