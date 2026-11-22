# Observation only. valid is a tracking gate, not anatomical phase acceptance.
std_msgs/Header header
builtin_interfaces/Time source_stamp
string state
string reason
bool valid
bool phase_reference_verified
float64 phase_rad
float64 frequency_hz
float64 amplitude
float64 residual
float64 source_age_ms
float64 receive_age_ms
float64 compute_us
uint64 accepted_samples
