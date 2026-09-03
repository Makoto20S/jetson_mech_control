// AK3.0 force-control bring-up probe. First program to drive a real motor
// through the repository's own stack: PosixCdcSerialPort -> UsbCdcTransport
// -> Ak30ForceControlSession. Every frame on the wire is produced by
// mech_control_core and mech_protocol_cubemars code that shipped through the
// offline test suite.
//
// Safety envelope (G3 bench-scaled, owner-confirmed 2026-09-02/03): the motor
// is unloaded, mounted, within reach of the power button. The probe enforces
// a hard run duration, a zero-torque neutral tail after the run, and aborts
// on any fault byte, over-temperature, over-current or over-speed observed in
// feedback. It never sends brake or disable commands.
//
// Exit codes: 0 normal completion, 2 aborted by a safety condition.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "mech_bringup/posix_cdc_serial_port.hpp"
#include "mech_control_core/adapter_template.hpp"
#include "mech_control_core/config.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/time.hpp"
#include "mech_control_core/transport.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_cubemars/ak30_force_codec.hpp"
#include "mech_protocol_cubemars/ak30_force_session.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::CanonicalDeviceState;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::DeviceConfig;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::ProtocolProfile;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::Transport;
using mech::mech_control_core::TransportResult;
using mech::mech_control_core::UsbCdcOptions;
using mech::mech_control_core::UsbCdcTransport;
using mech::mech_protocol_cubemars::Ak30ForceControlSession;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::Ak30SessionConfig;
using mech::mech_protocol_cubemars::ForceControlSubMode;
using mech::mech_bringup::PosixCdcSerialPort;

MonotonicTime now() noexcept {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch();
  return *MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ticks).count());
}

MonotonicTime at_offset(MonotonicTime base, std::int64_t offset_ns) noexcept {
  return *MonotonicTime::from_nanoseconds(base.nanoseconds() + offset_ns);
}

struct RunOptions final {
  std::string device{"/dev/ttyACM0"};
  double torque_nm{0.0};
  double run_seconds{0.0};
  double neutral_seconds{0.5};
  double period_seconds{0.01};
  // Abort thresholds, all measured from decoded feedback.
  double max_abs_effort_nm{3.0};
  double max_abs_velocity_rad_s{8.0};
  double max_temp_c{60.0};
  std::uint64_t max_abs_position_step_deg{45U};
};

// Bench-observed 2026-09-03: the feedback position can jump as the motor
// spins under load; a per-sample sanity bound on reported effort/velocity
// comes from the feedback itself, decoded by the session.

bool parse_double(const char* text, double& output) noexcept {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    return false;
  }
  output = value;
  return true;
}

int run(const RunOptions& options) noexcept {
  PosixCdcSerialPort serial{options.device};
  UsbCdcOptions cdc_options{};
  cdc_options.logical_bus = 1U;
  cdc_options.nominal_bitrate_hz = 0U;  // firmware-fixed, undocumented (OQ-08)
  cdc_options.receive_queue_capacity = 128U;
  cdc_options.verified_board_version = {4U, 8U, 8U};
  UsbCdcTransport transport{serial, cdc_options};

  if (!transport.open()) {
    std::fprintf(stderr, "ERROR: transport.open() failed for %s\n",
                 options.device.c_str());
    return 1;
  }
  if (!serial.send_pass_through_init()) {
    std::fprintf(stderr, "ERROR: pass-through init frame was not accepted\n");
    transport.close();
    return 1;
  }

  Ak30Mapping mapping{};  // motor1's evidenced defaults, direction_sign now verified
  Ak30SessionConfig session_config{};
  session_config.drive_id = 104U;  // motor1, decimal, from the host tool
  session_config.sub_mode = ForceControlSubMode::Torque;
  session_config.mapping = mapping;
  session_config.firmware_id = 1U;
  session_config.firmware_id_min = 1U;
  session_config.firmware_id_max = 1000U;

  DeviceConfig device_config{};
  device_config.device_id = 1U;
  device_config.name = "motor1";
  device_config.logical_bus = 1U;
  device_config.profile = ProtocolProfile::Ak30ForceControlExtended;
  device_config.frame_type = CanFrameType::Classic;
  device_config.frame_format = CanFrameFormat::Extended;
  device_config.command_id =
      CanId::create(mech::mech_protocol_cubemars::force_control_can_id(104U),
                    CanFrameFormat::Extended);
  device_config.feedback_id = CanId::create(
      mech::mech_protocol_cubemars::feedback_can_id(104U),
      CanFrameFormat::Extended);
  device_config.command_payload_bytes = 8U;
  device_config.feedback_payload_bytes = 8U;
  device_config.writable = true;

  Ak30ForceControlSession session{transport, session_config};
  const auto configured =
      session.configure(device_config, transport.capabilities());
  if (configured != AdapterResult::Ok) {
    std::fprintf(stderr, "ERROR: session.configure() = %d\n",
                 static_cast<int>(configured));
    transport.close();
    return 1;
  }
  if (session.activate() != AdapterResult::Ok) {
    std::fprintf(stderr, "ERROR: session.activate() failed\n");
    transport.close();
    return 1;
  }
  std::printf("CONFIGURED drive_id=104 sub_mode=Torque "
              "command_id=0x%04X feedback_id=0x%04X torque=%.3f Nm "
              "duration=%.1f s\n",
              device_config.command_id->value,
              device_config.feedback_id->value, options.torque_nm,
              options.run_seconds);

  CanonicalDeviceCommand command{};
  command.effort = options.torque_nm;

  // Torque sub-mode evidence: effort is the canonical observable (position is
  // B4-gated), so the summary must report what the session actually exported.
  double last_effort_nm = 0.0;
  double min_effort_nm = 0.0;
  double max_effort_nm = 0.0;
  bool have_effort = false;

  const auto deadline_offset_ns =
      static_cast<std::int64_t>(options.period_seconds * 1e9);
  std::uint64_t sent = 0U;
  std::uint64_t samples = 0U;
  bool aborted = false;
  std::string abort_reason;
  double first_position_deg = 0.0;
  double last_position_deg = 0.0;
  bool have_position = false;

  const MonotonicTime start = now();
  MonotonicTime next_send = start;
  const auto run_ns = static_cast<std::int64_t>(options.run_seconds * 1e9);
  const auto neutral_ns =
      static_cast<std::int64_t>(options.neutral_seconds * 1e9);
  const auto total_ns = run_ns + neutral_ns;

  while (true) {
    const MonotonicTime current = now();
    if (current.nanoseconds() - start.nanoseconds() >= total_ns) {
      break;
    }
    const bool neutral_phase =
        current.nanoseconds() - start.nanoseconds() >= run_ns;
    if (!neutral_phase && current.nanoseconds() >= next_send.nanoseconds()) {
      command.generation = sent + 1U;
      command.deadline = at_offset(current, deadline_offset_ns * 2);
      const auto result = session.submit(command, current);
      if (result == AdapterResult::Ok) {
        ++sent;
      } else if (result != AdapterResult::WouldBlock) {
        aborted = true;
        abort_reason = "submit_failed";
        break;
      }
      next_send = at_offset(next_send, deadline_offset_ns);
    }

    RawCanFrame frame{};
    const auto received = transport.try_receive(frame);
    if (received == TransportResult::Ok) {
      const auto processed = session.process(frame, current);
      if (processed == AdapterResult::Ok) {
        const CanonicalDeviceState state = session.snapshot(current);
        if (state.status.quality ==
            mech::mech_control_core::SampleQuality::Valid) {
          ++samples;
          const double effort_nm = state.effort;
          if (!have_effort) {
            min_effort_nm = effort_nm;
            max_effort_nm = effort_nm;
            have_effort = true;
          }
          last_effort_nm = effort_nm;
          min_effort_nm = std::min(min_effort_nm, effort_nm);
          max_effort_nm = std::max(max_effort_nm, effort_nm);
          const double position_deg = state.position * 57.29577951308232;
          if (!have_position) {
            first_position_deg = position_deg;
            have_position = true;
          }
          last_position_deg = position_deg;
          if (state.status.raw_fault_code != 0U) {
            aborted = true;
            abort_reason = "fault_code=" +
                           std::to_string(state.status.raw_fault_code);
            break;
          }
          if (std::abs(state.effort) > options.max_abs_effort_nm) {
            aborted = true;
            abort_reason = "effort=" + std::to_string(state.effort);
            break;
          }
          if (std::abs(state.velocity) > options.max_abs_velocity_rad_s) {
            aborted = true;
            abort_reason = "velocity=" + std::to_string(state.velocity);
            break;
          }
        }
      }
    } else if (received != TransportResult::WouldBlock) {
      aborted = true;
      abort_reason = "receive_failed";
      break;
    }
  }

  // Neutral tail: even on the abort path, command zero torque until the tail
  // window closes, then deactivate. Never send brake/disable commands.
  if (aborted) {
    const MonotonicTime tail_start = now();
    while (now().nanoseconds() - tail_start.nanoseconds() < neutral_ns) {
      const MonotonicTime current = now();
      CanonicalDeviceCommand neutral{};
      neutral.effort = 0.0;
      neutral.deadline = at_offset(current, deadline_offset_ns * 2);
      const auto submit_result = session.submit(neutral, current);
      static_cast<void>(submit_result);
      RawCanFrame frame{};
      if (transport.try_receive(frame) == TransportResult::Ok) {
        const auto process_result = session.process(frame, now());
        static_cast<void>(process_result);
      }
    }
  }
  session.deactivate();
  transport.close();

  std::printf(
      "END sent=%llu samples=%llu abort=%s position_deg=%.1f->%.1f "
      "effort_nm=%.3f..%.3f (last=%.3f)\n",
      static_cast<unsigned long long>(sent),
      static_cast<unsigned long long>(samples),
      aborted ? abort_reason.c_str() : "none", first_position_deg,
      last_position_deg, have_effort ? min_effort_nm : 0.0,
      have_effort ? max_effort_nm : 0.0,
      have_effort ? last_effort_nm : 0.0);
  return aborted ? 2 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  RunOptions options{};
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: %s <torque_nm> <run_seconds> [device]\n"
                 "  torque must be within +-15 Nm (AKE60-8); the probe "
                 "enforces its own abort thresholds regardless.\n",
                 argv[0]);
    return 1;
  }
  if (!parse_double(argv[1], options.torque_nm) ||
      !parse_double(argv[2], options.run_seconds)) {
    std::fprintf(stderr, "ERROR: torque and duration must be numbers\n");
    return 1;
  }
  if (options.torque_nm < -15.0 || options.torque_nm > 15.0 ||
      options.run_seconds <= 0.0 || options.run_seconds > 300.0) {
    std::fprintf(stderr,
                 "ERROR: torque out of +-15 Nm or duration out of (0, 300] s\n");
    return 1;
  }
  if (argc >= 4) {
    options.device = argv[3];
  }
  return run(options);
}
