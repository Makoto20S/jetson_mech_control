"""Pure shared position protection; no ROS, motor IDs or active braking."""
import math


class PositionGuard:
    def __init__(self, limits, soft_margin=0.1, max_excursion=None):
        self.limits = tuple(limits)
        if (not math.isfinite(soft_margin) or soft_margin <= 0 or
                any(2*soft_margin >= l.maximum-l.minimum for l in self.limits)):
            raise ValueError('soft margin must be positive and smaller than half the position range')
        if max_excursion is not None and (
                not math.isfinite(max_excursion) or max_excursion <= 0):
            raise ValueError('invalid excursion bound')
        self.margin, self.excursion = soft_margin, max_excursion
        self.reference = None

    def arm(self, joints):
        self.reference = tuple(j.position for j in joints)

    def reset(self):
        self.reference = None

    def bounds(self, index):
        limit = self.limits[index]
        lo, hi = limit.minimum, limit.maximum
        if self.reference is not None and self.excursion is not None:
            lo = max(lo, self.reference[index]-self.excursion)
            hi = min(hi, self.reference[index]+self.excursion)
        return lo, hi

    def error(self, joints):
        for i, joint in enumerate(joints):
            lo, hi = self.bounds(i)
            if not lo <= joint.position <= hi:
                return 'position/excursion bound exceeded: '+joint.name
        return ''

    def constrain(self, values, joints):
        """After slew: reduce outward commands, retain inward commands.

        This reduces commanded velocity/effort, not actual momentum. On effort
        axes outward velocity with inward braking torque remains permitted.
        """
        result = []
        for i, (value, joint, limit) in enumerate(zip(values, joints, self.limits)):
            lo, hi = self.bounds(i)
            margin = min(self.margin, (hi-lo)/2)
            distance = hi-joint.position if value > 0 else joint.position-lo
            cap = limit.max_command*max(0.0, min(1.0, distance/margin))
            result.append(math.copysign(min(abs(value), cap), value))
        return tuple(result)
