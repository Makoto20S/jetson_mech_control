uint8 INACTIVE=0
uint8 WAITING=1
uint8 FOLLOWING=2
uint8 FAULTED=3
uint64 session_id
uint64 accepted_sequence
uint8 mode
uint32 reason_code
string reason
float64 observed_at_monotonic_s
float64 paired_source_time_monotonic_s
float64[2] position_rad
float64[2] velocity_rad_s
float64[2] reference_rad
# Last accepted target velocity; zero for fixed-pose sources.
float64[2] dq_ref
bool dynamic
float64[2] motor_target_nm
bool state_valid
bool state_is_predicted
# A stop request is not proof of drive disable.
bool stop_requested
bool[2] disable_confirmed
