"""Fresh-source gating for the PH-EXO simulation impedance loop (no I/O)."""
import math


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class StateImpedance:
    def __init__(self, controller, timeout_s=.06):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Positive finite timeout required')
        self.controller = controller
        self.timeout_ns = int(timeout_s * 1e9)
        self.sample = None
        self.source_ns = 0
        self.received_ns = None
        self.sent_ns = 0
        self.active = False
        self.tripped = False
        self.reason = 'waiting_for_state'

    def stop(self, reason):
        self.reason = reason
        if self.active:
            self.tripped = True
        self.sample = None

    def update(self, state, now_ros, now_mono):
        if self.tripped:
            return
        source = state.sources[0]
        stamp = stamp_ns(source.source_stamp)
        valid = (source.valid and state.mapping_confirmed and state.q_valid
                 and state.q_velocity_valid and len(state.q_position) == 2
                 and len(state.q_velocity) == 2
                 and all(math.isfinite(x) for x in [*state.q_position, *state.q_velocity]))
        if not valid:
            self.stop('state_invalid')
        elif stamp <= 0 or not -5_000_000 <= now_ros-stamp <= self.timeout_ns:
            self.stop('source_expired')
        elif stamp < self.source_ns:
            self.stop('source_regressed')
        elif stamp > self.source_ns:
            self.source_ns = stamp
            self.received_ns = now_mono
            self.sample = (list(state.q_position), list(state.q_velocity))
        # Re-publication of the same measurement cannot extend its lease.

    def command(self, now_ros, now_mono):
        if self.tripped or self.sample is None:
            return None
        if (not 0 <= now_mono-self.received_ns <= self.timeout_ns or
                not -5_000_000 <= now_ros-self.source_ns <= self.timeout_ns):
            self.stop('feedback_timeout')
            return None
        if self.sent_ns == self.source_ns:
            return None
        try:
            result = self.controller.command(*self.sample)
        except (ValueError, RuntimeError) as error:
            self.stop('controller_invalid:' + str(error))
            return None
        self.sent_ns = self.source_ns
        self.active = True
        self.reason = 'active'
        return result
