"""Bounded incremental manager-log fault detection for supervised bench recorders.

This supplements hardware protection; fresh JointState publication alone does not
prove the hardware values are fresh after a runtime fault.
"""
from pathlib import Path


class ControllerLogGuard:
    def __init__(self, path):
        self.path = Path(path)
        self.offset = 0
        self.tail = b''
        self.failure = None

    def poll(self):
        if self.failure:
            return self.failure
        if self.path.stat().st_size < self.offset:
            self.failure = 'manager log truncated during trial'
            return self.failure
        with self.path.open('rb') as stream:
            stream.seek(self.offset)
            chunk = stream.read(65536)
            self.offset = stream.tell()
        data = self.tail + chunk
        for line in data.splitlines():
            if b'[ERROR]' in line or b'[FATAL]' in line:
                self.failure = line.decode('utf-8', errors='replace')[:2048]
                return self.failure
        self.tail = data[-1024:]
        return None
