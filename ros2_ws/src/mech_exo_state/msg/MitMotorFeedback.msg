# Latest 0x29 feedback, in device-native units. Position shaft is calibration-dependent.
uint8 drive_id
bool has_sample
uint64 rx_sequence
uint64 host_rx_us
float64 age_ms
float64 position_deg
float64 erpm
# Motor gearbox output, positive in the configured feedback direction. Candidate until confirmed.
float64 output_velocity_rad_s
bool output_velocity_valid
float64 iq_a
float64 temperature_c
uint8 fault_code
bool valid
string reason
