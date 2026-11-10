std_msgs/Header header
# Order: joints, imu1, imu2. Each source keeps its own timestamps.
mech_exo_state/SourceStatus[3] sources
# Canonical order motor104_joint, motor105_joint (configurable names).
sensor_msgs/JointState joints
sensor_msgs/Imu[2] imus
bool[2] imu_orientation_valid
bool[2] imu_angular_velocity_valid
bool[2] imu_linear_acceleration_valid
# Model input order alpha1 (105), alpha2 (104); rad and rad/s.
float64[2] alpha_position
float64[2] alpha_velocity
bool alpha_valid
bool alpha_velocity_valid
# FK result in fixed REF01 frame: [psi, beta], radians.
# Derived only from joints/alpha; timestamp/quality source is sources[0].
# Invalid => NaN. No IMU dependency, velocity estimate or measured-truth claim.
float64[2] q_position
bool q_valid
string q_reason
# Maximum absolute MuJoCo connect-constraint position residual (metres).
float64 q_closure_residual_m
# Native MIT receive samples in fixed ID order [104,105].
mech_exo_state/MitMotorFeedback[2] mit_motors
bool mapping_confirmed
bool messages_aligned
float64 source_skew_ms
# Always false in v1: legacy sources do not expose verified sample clocks.
bool hardware_freshness_verified
