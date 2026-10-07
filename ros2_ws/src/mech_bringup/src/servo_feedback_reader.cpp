#include "mech_bringup/servo_feedback_reader.hpp"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
namespace mech::mech_bringup {
namespace {
using Availability = mech_protocol_cubemars::ServoPositionAvailability;
const char* availability_name(Availability value) {
  switch (value) {
    case Availability::Fresh: return "fresh";
    case Availability::Stale: return "stale";
    case Availability::Invalid: return "invalid";
    case Availability::Fault: return "fault";
    case Availability::DisableAcknowledged: return "disable_acknowledged";
    case Availability::Unknown: return "unknown";
  }
  return "unknown";
}
void number(std::ostream& out, double value) {
  if (std::isfinite(value)) out << value;
  else out << "null";
}
}
std::string format_servo_feedback(std::int64_t monotonic_ns, std::uint64_t motor_frames,
    const std::vector<std::uint16_t>& ids,
    const std::vector<std::optional<mech_protocol_cubemars::ServoPositionSnapshot>>& samples) {
  bool valid = !ids.empty() && samples.size() == ids.size() && motor_frames == 0;
  for (const auto& sample : samples) valid = valid && sample && sample->availability == Availability::Fresh;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << "{\"schema_version\":1,\"monotonic_ns\":" << monotonic_ns
      << ",\"motor_command_frames\":" << motor_frames << ",\"valid\":" << (valid ? "true" : "false") << ",\"motors\":[";
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) out << ',';
    const auto sample = i < samples.size() && samples[i] ? *samples[i] : mech_protocol_cubemars::ServoPositionSnapshot{};
    out << "{\"id\":" << ids[i] << ",\"position_rad\":"; number(out, sample.position_rad);
    out << ",\"feedback_position_deg\":"; number(out, sample.feedback_position_deg);
    out << ",\"electrical_speed_erpm\":"; number(out, sample.electrical_speed_erpm);
    out << ",\"current_iq_a\":"; number(out, sample.current_iq_a);
    out << ",\"temperature_c\":"; number(out, sample.temperature_c);
    out << ",\"raw_status\":" << static_cast<unsigned>(sample.raw_status)
        << ",\"availability\":\"" << availability_name(sample.availability) << "\",\"sequence\":" << sample.sequence << ",\"host_rx_ns\":";
    if (sample.host_rx_time) out << sample.host_rx_time->nanoseconds(); else out << "null";
    out << '}';
  }
  out << "]}";
  return out.str();
}
}
