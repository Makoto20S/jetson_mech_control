import copy,unittest
from ref01_stale_validation import analyze_stale_state
class StaleCause(unittest.TestCase):
    def data(self):
        # Synthetic steady-clock nanoseconds; keep source clocks unrenewed.
        trace=[dict(dropped=0),
            dict(stage="kinematic_read_end",ns=1010000000,position=.990,feedback=.990,generation=2,flags=1),
            dict(stage="kinematic_read_end",ns=1020000000,position=.990,feedback=1.020,generation=2,flags=1),
            dict(stage="kinematic_lease",ns=1035000000,flags=1),
            dict(stage="hardware_error_begin",ns=1040000000),
            dict(stage="hardware_error_revoked",ns=1040000100),
            dict(stage="hardware_error_end",ns=1040000200)]
        status=dict(mode=3,reason_name="feedback_expired",first_fault_timing=dict(
            at_s=1.055,manager_period_s=.005,update_gap_s=.005,feedback_age_s=[.065,.064]))
        return {104:copy.deepcopy(trace),105:copy.deepcopy(trace)},[status]
    def test_complete_cause(self):
        traces,status=self.data();self.assertTrue(analyze_stale_state(traces,status)["passed"])
    def test_secondary_period_fault_rejected(self):
        traces,status=self.data();status[0]["reason_name"]="update_period"
        status[0]["first_fault_timing"]["update_gap_s"]=.02
        self.assertFalse(analyze_stale_state(traces,status)["passed"])
    def test_not_really_frozen_rejected(self):
        traces,status=self.data();traces[104][2]["position"]=1.020
        self.assertFalse(analyze_stale_state(traces,status)["passed"])
    def test_no_raw_feedback_progress_rejected(self):
        traces,status=self.data();traces[105][2]["feedback"]=.990
        self.assertFalse(analyze_stale_state(traces,status)["passed"])
    def test_missing_or_dropped_evidence_rejected(self):
        for kind in ("missing","dropped","timing"):
            traces,status=self.data()
            if kind=="missing":traces[104]=[]
            elif kind=="dropped":traces[105][0]["dropped"]=1
            else:status[0].pop("first_fault_timing")
            self.assertFalse(analyze_stale_state(traces,status)["passed"])
    def test_wrong_order_rejected(self):
        traces,status=self.data();traces[104][-1]["ns"]=1060000000
        self.assertFalse(analyze_stale_state(traces,status)["passed"])
if __name__=="__main__":unittest.main()
