# Contract version 1. Same Orin host monotonic clock only.
# Reference and gains are immutable during a session.
uint32 schema_version
uint64 session_id
uint64 sequence
float64 issued_at_monotonic_s
bool enabled
# Task order [psi, beta].
float64[2] reference_rad
float64[2] stiffness_nm_per_rad
float64[2] damping_nm_s_per_rad
# Physical joint order [motor104_joint, motor105_joint], NOT alpha order.
float64[2] motor_torque_cap_nm
