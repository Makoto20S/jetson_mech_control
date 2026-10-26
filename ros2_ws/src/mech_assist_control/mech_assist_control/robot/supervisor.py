"""Whole-robot freshness, explicit arm and fault supervision."""
import math
from .contracts import CommandMode, CommandEnvelope, ImpedancePlan
from .safety import PositionGuard

class ExoskeletonSupervisor:
    """Explicit standby -> active -> standby/fault; faults require explicit reset.

    State and every policy result are checked before routing. A fault invalidates
    the envelope; the ROS compatibility adapter may emit ONE neutral, never a
    stream of fresh zero commands that conceals a dead policy/input source.
    """
    def __init__(self, limits, policy, sample_timeout=0.3, health_timeout=0.3,
                 command_ttl=0.1, max_tick_gap=0.1, run_limit=10.0,
                 soft_margin=0.1, max_excursion=None):
        self.limits = tuple(limits)
        if (not 1 <= len(self.limits) <= 6 or
                len({j.name for j in self.limits}) != len(self.limits)):
            raise ValueError('require 1..6 unique configured joints')
        if not isinstance(policy.mode, CommandMode):
            raise ValueError('policy must declare a command mode')
        timing = (sample_timeout, health_timeout, command_ttl, max_tick_gap, run_limit)
        if not all(math.isfinite(v) and v > 0 for v in timing):
            raise ValueError('invalid timing')
        self.position_guard = PositionGuard(self.limits, soft_margin, max_excursion)
        self.policy = policy
        self.sample_timeout, self.health_timeout = sample_timeout, health_timeout
        self.command_ttl, self.max_tick_gap, self.run_limit = command_ttl, max_tick_gap, run_limit
        self.names = tuple(j.name for j in self.limits)
        self.state, self.reason = 'standby', 'explicit arm required'
        self.observation = None
        self.last_observation_sequence = -1
        self.last_received_at = None
        self.command_sequence = 0
        self.previous = (0.0,) * len(self.limits)
        self.started = self.last_tick = None
        self.command_work = 0.0
        self.active_training_plan = None

    def fault(self, reason):
        self.state, self.reason = 'fault', reason
        self.previous = (0.0,) * len(self.limits)
        self.active_training_plan = None

    def observe(self, observation, now=None):
        if (type(observation.sequence) is not int or observation.sequence < 0 or
                observation.sequence <= self.last_observation_sequence or
                not math.isfinite(observation.received_at) or
                (self.last_received_at is not None and
                 observation.received_at <= self.last_received_at)):
            self.fault('replayed or invalid observation')
            return
        self.last_observation_sequence = observation.sequence
        self.last_received_at = observation.received_at
        self.observation = observation
        # Health and joint samples are independently timestamped. A newer
        # diagnostic is not future data merely because the joint sample is old.
        error = self._health_error(observation.received_at if now is None else now)
        if error and self.state == 'active':
            self.fault(error)

    def _health_error(self, now):
        o = self.observation
        if not math.isfinite(now) or o is None:
            return 'observation unavailable'
        if not 0 <= now-o.received_at <= self.sample_timeout:
            return 'joint observation stale or future'
        if (not math.isfinite(o.health_received_at) or
                not 0 <= now-o.health_received_at <= self.health_timeout or not o.healthy):
            return 'estimator health unavailable or stale'
        if o.coordinates not in ('calibrated_joint', 'motor_shaft'):
            return 'unknown observation coordinates'
        if o.coordinates == 'motor_shaft':
            if self.policy.name not in ('resistance', 'resistance_training', 'impedance_training', 'motion_assist') or o.calibrated:
                return 'shaft coordinates require an uncalibrated motor policy'
        elif not o.calibrated:
            return 'calibration unavailable'
        if tuple(j.name for j in o.joints) != self.names:
            return 'joint set/order mismatch'
        for j, limit in zip(o.joints, self.limits):
            if (not math.isfinite(j.position) or not math.isfinite(j.velocity) or
                    not limit.minimum <= j.position <= limit.maximum or
                    abs(j.velocity) > limit.max_velocity):
                return 'joint state outside envelope'
        position_error = self.position_guard.error(o.joints)
        if position_error:
            return position_error
        if self.policy.requires_contact:
            required = getattr(self.policy, 'contact_names', ('left_foot', 'right_foot'))
            contacts = {c.name: c for c in o.contacts}
            if (not required or len(contacts) != len(o.contacts) or
                    any(name not in contacts or not contacts[name].verified or
                        type(contacts[name].in_contact) is not bool or
                        not math.isfinite(contacts[name].received_at) or
                        not 0 <= now-contacts[name].received_at <= self.sample_timeout
                        for name in required)):
                return 'verified foot contact required'
        return ''

    def arm(self, now):
        error = self._health_error(now)
        if self.state != 'standby' or error:
            if self.state != 'fault':
                self.reason = error or 'already active'
            return False
        try:
            self.policy.start(self.observation)
        except Exception:
            self.fault('policy initialization failed')
            return False
        self.state, self.reason = 'active', ''
        self.previous = (0.0,) * len(self.limits)
        self.started = self.last_tick = now
        self.command_work = 0.0
        self.active_training_plan = None
        self.position_guard.arm(self.observation.joints)
        return True

    def disarm(self):
        if self.state != 'fault':
            self.state, self.reason = 'standby', 'explicitly disarmed'
            self.position_guard.reset()
        self.previous = (0.0,) * len(self.limits)
        self.active_training_plan = None

    def reset(self):
        if self.state == 'active':
            return False
        self.state, self.reason = 'standby', 'fresh observation and explicit arm required'
        self.position_guard.reset()
        self.observation = None  # Old data cannot re-arm a reset supervisor.
        self.previous = (0.0,) * len(self.limits)
        self.active_training_plan = None
        return True

    def _check_training_plan(self, target):
        plan = target.training_plan
        expected = getattr(self.policy, 'produces_training_plan', False)
        if not expected and plan is None:
            return
        if (not expected or not isinstance(plan, ImpedancePlan) or
                target.mode != CommandMode.EFFORT or plan.names != self.names or
                any(v != 0.0 for v in target.values) or
                any(not limit.minimum <= ref <= limit.maximum or cap > limit.max_command
                    for limit, ref, cap in zip(self.limits, plan.reference, plan.torque_caps))):
            raise ValueError('incompatible or out-of-envelope training plan')
        if self.active_training_plan is not None and plan != self.active_training_plan:
            raise ValueError('fixed training plan changed while active')
        self.active_training_plan = plan

    def tick(self, now):
        if self.state == 'active':
            dt = now-self.last_tick
            error = self._health_error(now)
            if not math.isfinite(dt) or not 0 <= dt <= self.max_tick_gap:
                error = 'supervisor tick gap'
            elif now-self.started >= self.run_limit:
                self.disarm()
                self.reason = 'run limit reached'
            if error:
                self.fault(error)
            if self.state == 'active':
                try:
                    target = self.policy.evaluate(self.observation, dt)
                    if (target.mode != self.policy.mode or len(target.values) != len(self.limits) or
                            not all(math.isfinite(v) for v in target.values)):
                        raise ValueError('invalid policy result')
                    self._check_training_plan(target)
                    if target.training_plan is not None:
                        # A plan is not an effort vector. Only C++ applies its
                        # spring/damping and instantaneous output constraints.
                        self.previous = (0.0,) * len(self.limits)
                    else:
                        values = []
                        for value, old, limit, joint in zip(
                                target.values, self.previous, self.limits, self.observation.joints):
                            if (getattr(self.policy, 'release_on_reversal', False) and
                                    old * joint.velocity > 0):
                                values.append(0.0)
                                continue
                            bounded = max(-limit.max_command, min(limit.max_command, value))
                            if (getattr(self.policy, 'release_on_deceleration', False) and
                                    abs(bounded) < abs(old)):
                                # Viscous resistance must unload as its target falls.
                                # Applying the buildup slew here retains excessive
                                # braking after a short stroke has already slowed.
                                values.append(bounded)
                                continue
                            step = limit.max_slew*dt
                            # Removing damping inside the deadband takes priority over slew.
                            # Otherwise residual torque would push a stationary shaft.
                            values.append(0.0 if (getattr(self.policy, 'dissipative', False) or
                                                 getattr(self.policy, 'motion_aligned', False))
                                          and bounded == 0.0 else
                                          old + max(-step, min(step, bounded-old)))
                        values = list(self.position_guard.constrain(values, self.observation.joints))
                        if getattr(self.policy, 'dissipative', False) and any(
                                v*j.velocity > 1e-12 for v, j in zip(values, self.observation.joints)):
                            raise ValueError('slew would turn damping into assistance')
                        if getattr(self.policy, 'motion_aligned', False):
                            # Reversal first withdraws the previous direction; restart
                            # from zero on the next tick rather than oppose new motion.
                            values = [0.0 if v*j.velocity < 0 else v
                                      for v, j in zip(values, self.observation.joints)]
                            # Endpoint-maximum commanded-power estimate. This is
                            # not a measured mechanical-energy or passivity bound.
                            work = dt*sum(max(abs(old), abs(v))*abs(j.velocity)
                                          for old, v, j in zip(self.previous, values,
                                                              self.observation.joints))
                            if self.command_work+work > self.policy.work_budget:
                                raise ValueError('assistance command-work budget exhausted')
                            self.command_work += work
                        self.previous = tuple(values)
                except ValueError as error:
                    self.fault('policy rejected: '+str(error))
                except Exception:
                    self.fault('policy evaluation failed or invalid result')
            self.last_tick = now
        self.command_sequence += 1
        valid = self.state == 'active'
        return CommandEnvelope(
            1, self.command_sequence, self.last_observation_sequence, now,
            now+self.command_ttl if valid else now, self.policy.name, self.policy.mode,
            self.names, self.previous if valid else (0.0,)*len(self.limits), valid, self.reason,
            self.active_training_plan if valid else None)


