#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "mech_protocol_cubemars/ak30_servo_position_session.hpp"
namespace mech::mech_bringup {
std::string format_servo_feedback(std::int64_t monotonic_ns, std::uint64_t motor_frames,
    const std::vector<std::uint16_t>& ids,
    const std::vector<std::optional<mech_protocol_cubemars::ServoPositionSnapshot>>& samples);
}
