"""Brand-independent, non-RT whole-robot application contracts and supervisor.

Only a validated active decision permits command refresh. This is not a drive
watchdog, a motor enable operation, or a substitute for the C++ control layer.
"""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, Tuple


class CommandMode(str, Enum):
    EFFORT = 'effort'
    VELOCITY = 'velocity'


@dataclass(frozen=True)
class JointLimit:
    name: str
    minimum: float
    maximum: float
    max_velocity: float
    max_command: float
    max_slew: float

    def __post_init__(self):
        values = (self.minimum, self.maximum, self.max_velocity,
                  self.max_command, self.max_slew)
        if (not self.name or not all(math.isfinite(v) for v in values) or
                self.minimum >= self.maximum or
                min(self.max_velocity, self.max_command, self.max_slew) <= 0):
            raise ValueError('invalid joint limits')


@dataclass(frozen=True)
class JointObservation:
    name: str
    position: float
    velocity: float


@dataclass(frozen=True)
class ContactObservation:
    name: str
    in_contact: bool
    received_at: float
    verified: bool


@dataclass(frozen=True)
class RobotObservation:
    sequence: int
    received_at: float  # Host monotonic arrival; never ROS/source time.
    joints: Tuple[JointObservation, ...]
    healthy: bool
    health_received_at: float
    calibrated: bool
    # Future insole/state-estimator adapter must set this from actual evidence.
    contacts: Tuple[ContactObservation, ...] = ()
    coordinates: str = 'calibrated_joint'  # motor_shaft never implies anatomical calibration.


@dataclass(frozen=True)
class PolicyTarget:
    mode: CommandMode
    values: Tuple[float, ...]


@dataclass(frozen=True)
class CommandEnvelope:
    schema_version: int
    sequence: int
    observation_sequence: int
    issued_at: float
    expires_at: float
    policy: str
    mode: CommandMode
    names: Tuple[str, ...]
    values: Tuple[float, ...]
    valid: bool
    reason: str


class Policy(Protocol):
    name: str
    mode: CommandMode
    requires_contact: bool

    def start(self, observation: RobotObservation) -> None: ...
    def evaluate(self, observation: RobotObservation, dt: float) -> PolicyTarget: ...


