"""Canonical joint to controller routing with measured coordinate mapping."""
from dataclasses import dataclass
import math
from .contracts import CommandMode, ImpedancePlan

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

    def _validate(self, envelope, now):
        if (not envelope.valid or envelope.schema_version != 1 or
                envelope.names != self.names or envelope.mode != self.mode or
                envelope.sequence <= self.last_sequence or
                not math.isfinite(now) or not math.isfinite(envelope.issued_at) or
                not math.isfinite(envelope.expires_at) or
                not envelope.issued_at <= now < envelope.expires_at or
                len(envelope.values) != len(self.routes) or
                not all(math.isfinite(v) for v in envelope.values)):
            raise ValueError('expired, replayed or incompatible envelope')

    def route_training(self, envelope, now):
        """Validate the whole plan before returning ANY per-axis wire payload.

        Current fixed-plan backend accepts shaft identity mapping only. A new
        transmission must define reference and gain transformations explicitly.
        """
        self._validate(envelope, now)
        plan = envelope.training_plan
        if (not isinstance(plan, ImpedancePlan) or plan.names != self.names or
                self.mode != CommandMode.EFFORT or any(envelope.values) or
                any(r.scale != 1.0 or not r.topic.endswith('/training_command') or
                    cap > r.max_output
                    for r, cap in zip(self.routes, plan.torque_caps))):
            raise ValueError('incompatible training plan or actuator mapping')
        frames = tuple((r.topic, (ref, plan.stiffness, plan.damping, cap))
                       for r, ref, cap in zip(self.routes, plan.reference, plan.torque_caps))
        self.last_sequence = envelope.sequence
        return frames

    def route(self, envelope, now):
        self._validate(envelope, now)
        if envelope.training_plan is not None:
            raise ValueError('training plan requires structured routing, never scalar effort')
        mapped = tuple((r.topic, r.scale*v if self.mode == CommandMode.EFFORT
                        else v/r.scale) for r, v in zip(self.routes, envelope.values))
        if any(abs(v) > r.max_output for r, (_, v) in zip(self.routes, mapped)):
            raise ValueError('mapped controller command outside envelope')
        self.last_sequence = envelope.sequence
        return mapped
