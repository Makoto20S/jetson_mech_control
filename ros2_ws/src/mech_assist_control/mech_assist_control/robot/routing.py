"""Canonical joint to controller routing with measured coordinate mapping."""
from dataclasses import dataclass
import math
from .contracts import CommandMode

@dataclass(frozen=True)
class ControllerRoute:
    joint: str
    mode: CommandMode
    topic: str
    scale: float = 1.0  # canonical angle = scale * actuator angle + offset
    max_output: float = 0.3


class CommandRouter:
    """Map canonical joints to controller topics, never CAN IDs or motor brands."""
    def __init__(self, names, mode, routes):
        self.names, self.mode, self.routes = tuple(names), mode, tuple(routes)
        if (tuple(r.joint for r in self.routes) != self.names or
                len(set(self.names)) != len(self.names) or
                any(r.mode != mode or not r.topic.startswith('/') or ' ' in r.topic
                    or not math.isfinite(r.scale) or r.scale == 0
                    or not math.isfinite(r.max_output) or r.max_output <= 0
                    for r in self.routes) or
                len({r.topic for r in self.routes}) != len(self.routes)):
            raise ValueError('invalid or conflicting controller routes')
        self.last_sequence = -1

    def route(self, envelope, now):
        if (not envelope.valid or envelope.schema_version != 1 or
                envelope.names != self.names or envelope.mode != self.mode or
                envelope.sequence <= self.last_sequence or
                not math.isfinite(now) or not math.isfinite(envelope.issued_at) or
                not math.isfinite(envelope.expires_at) or
                not envelope.issued_at <= now < envelope.expires_at or
                len(envelope.values) != len(self.routes) or
                not all(math.isfinite(v) for v in envelope.values)):
            raise ValueError('expired, replayed or incompatible envelope')
        mapped = tuple((r.topic, r.scale*v if self.mode == CommandMode.EFFORT
                        else v/r.scale) for r, v in zip(self.routes, envelope.values))
        if any(abs(v) > r.max_output for r, (_, v) in zip(self.routes, mapped)):
            raise ValueError('mapped controller command outside envelope')
        self.last_sequence = envelope.sequence
        return mapped
