"""Pure policies; no ROS, device IDs or controller topics."""
import math
from .contracts import CommandMode, PolicyTarget

class TransparentPolicy:
    name, mode, requires_contact = 'transparent', CommandMode.EFFORT, False

    def start(self, observation):
        pass

    def evaluate(self, observation, dt):
        return PolicyTarget(self.mode, (0.0,) * len(observation.joints))


class DampingPolicy(TransparentPolicy):
    name = 'resistance'
    dissipative = True

    def __init__(self, damping=0.5, deadband=0.0):
        if (not math.isfinite(damping) or damping < 0 or
                not math.isfinite(deadband) or deadband < 0):
            raise ValueError('invalid damping')
        self.damping, self.deadband = damping, deadband

    def evaluate(self, observation, dt):
        return PolicyTarget(self.mode, tuple(-self.damping*math.copysign(max(0.0, abs(j.velocity)-self.deadband), j.velocity)
                                            for j in observation.joints))


class ResistancePermissionPolicy(TransparentPolicy):
    """No local torque law: the C++ controller computes encoder damping.

    The supervisor still checks observations and explicit arm/run duration.
    Its valid envelope authorizes a Boolean-equivalent heartbeat only.
    """
    name = 'resistance_training'


class ImpedanceTrainingPolicy(TransparentPolicy):
    """Supervised permission for FixedPositionImpedanceController.

    PD effort and activation anchor belong to the C++ update loop. Zero local
    values are placeholders, never a computed training torque or reference.
    """
    name = 'impedance_training'


class ResistanceTrainingPolicy(DampingPolicy):
    """STM32 viscous training law with time-based filtering and safe release.

    Uses shaft encoder velocity, not anatomical pose or user intent. The shared
    supervisor applies per-axis torque caps, slew, freshness and position bounds.
    """
    name = 'resistance_training'
    release_on_reversal = True
    release_on_deceleration = True

    def __init__(self, damping=(1.0, 1.0), deadband=0.02, filter_tau=0.05,
                 start_velocity=None):
        super().__init__(0.0, deadband)
        self.gains = tuple(damping)
        if (not 1 <= len(self.gains) <= 6 or
                not all(math.isfinite(v) and v >= 0 for v in self.gains) or
                not math.isfinite(filter_tau) or filter_tau <= 0):
            raise ValueError('invalid resistance training parameters')
        self.filter_tau = filter_tau
        self.start_velocity = deadband if start_velocity is None else start_velocity
        if (not math.isfinite(self.start_velocity) or self.start_velocity < deadband):
            raise ValueError('training start velocity must be at least the rest deadband')
        self.filtered = [0.0] * len(self.gains)
        self.moving = [False] * len(self.gains)

    def start(self, observation):
        if len(observation.joints) != len(self.gains):
            raise ValueError('one training damping gain required per joint')
        self.filtered = [0.0] * len(self.gains)
        self.moving = [False] * len(self.gains)

    def evaluate(self, observation, dt):
        if (len(observation.joints) != len(self.gains) or
                not math.isfinite(dt) or dt < 0):
            raise ValueError('invalid training observation or interval')
        alpha = -math.expm1(-dt / self.filter_tau)
        values = []
        for i, (joint, gain) in enumerate(zip(observation.joints, self.gains)):
            velocity = joint.velocity
            if not math.isfinite(velocity):
                raise ValueError('invalid training velocity')
            # The raw velocity decides release/direction: filter lag must never
            # leave an assistive residual at rest or after direction reversal.
            if abs(velocity) <= self.deadband or self.filtered[i] * velocity < 0:
                self.filtered[i] = 0.0
                self.moving[i] = False
                values.append(0.0)
                continue
            # Optional Schmitt gate: noise between stop/start thresholds cannot
            # repeatedly establish torque. Never delay unloading at raw rest
            # or retain torque through a measured direction reversal.
            if not self.moving[i] and abs(velocity) < self.start_velocity:
                values.append(0.0)
                continue
            self.moving[i] = True
            self.filtered[i] += alpha * (velocity - self.filtered[i])
            # Match the STM32 viscous law T=-B*vf. The raw-speed deadband is
            # an enable gate, not an offset subtracted from every moving sample.
            values.append(-gain * math.copysign(abs(self.filtered[i]), velocity))
        return PolicyTarget(self.mode, tuple(values))


class MotionAssistPolicy(TransparentPolicy):
    """Encoder motion amplification, not intent detection or gait assistance.

    Positive feedback can sustain motion after external effort stops. Bounds,
    taper, command-work budget and supervisor deadline are therefore mandatory.
    """
    name = 'motion_assist'
    motion_aligned = True

    def __init__(self, gain=0.3, deadband=0.04, taper_speed=0.3,
                 cutoff_speed=0.5, work_budget=0.05):
        values = (gain, deadband, taper_speed, cutoff_speed, work_budget)
        if (not all(math.isfinite(v) for v in values) or gain <= 0 or
                not 0 <= deadband < taper_speed < cutoff_speed or work_budget <= 0):
            raise ValueError('invalid motion assistance parameters')
        self.gain, self.deadband = gain, deadband
        self.taper_speed, self.cutoff_speed = taper_speed, cutoff_speed
        self.work_budget = work_budget

    def evaluate(self, observation, dt):
        values = []
        for joint in observation.joints:
            speed = abs(joint.velocity)
            if speed >= self.cutoff_speed:
                raise ValueError('assistance speed cutoff')
            taper = min(1.0, (self.cutoff_speed-speed)/
                        (self.cutoff_speed-self.taper_speed))
            values.append(self.gain*math.copysign(
                max(0.0, speed-self.deadband)*taper, joint.velocity))
        return PolicyTarget(self.mode, tuple(values))


class HoldVelocityPolicy(TransparentPolicy):
    """Experimental velocity reference toward the measured pose captured at arm."""
    name, mode = 'hold_velocity', CommandMode.VELOCITY

    def __init__(self, gain=0.8):
        if not math.isfinite(gain) or gain <= 0:
            raise ValueError('invalid hold gain')
        self.gain = gain
        self.reference = ()

    def start(self, observation):
        self.reference = tuple(j.position for j in observation.joints)

    def evaluate(self, observation, dt):
        return PolicyTarget(self.mode, tuple(self.gain*(ref-j.position)
                                            for ref, j in zip(self.reference, observation.joints)))


def make_policy(name):
    factories = {'transparent': TransparentPolicy, 'resistance': DampingPolicy,
                 'hold_velocity': HoldVelocityPolicy, 'motion_assist': MotionAssistPolicy,
                 'resistance_training': ResistanceTrainingPolicy,
                 'impedance_training': ImpedanceTrainingPolicy}
    if name not in factories:
        raise ValueError('unsupported policy; implement the Policy contract first')
    return factories[name]()


