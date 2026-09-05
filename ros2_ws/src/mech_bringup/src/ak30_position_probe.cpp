// Controlled AK3.0 position-mode bench probe.
// This is an opt-in bring-up tool, not the production ros2_control plugin.
//
// Two modes:
//   - "hold": capture the first valid feedback position, then command exactly
//     that position for the run window. The zero-offset and direction sign
//     cancel in the canonical<->device round trip, so a mapping error cannot
//     create displacement; this is the progressive route's step ③ shape.
//   - numeric target: command the given canonical position (step ④ shape).
//
// Safety envelope: fail-closed listen phase (hold mode sends nothing unless a
// live position was captured), software aborts on displacement/velocity/
// effort/fault, and a tail that holds the last observed position instead of
// commanding the calibrated zero.
//
// Exit codes: 0 normal completion, 1 setup failure, 2 aborted by a safety
// condition.

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
#include "mech_protocol_cubemars/ak30_force_session.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

namespace {
using namespace mech::mech_control_core;
using mech::mech_bringup::PosixCdcSerialPort;
using mech::mech_protocol_cubemars::Ak30ForceControlSession;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::Ak30SessionConfig;
using mech::mech_protocol_cubemars::ForceControlGains;
using mech::mech_protocol_cubemars::ForceControlSubMode;

// Software abort thresholds, all measured from decoded feedback. The
// displacement bound is generous (5 deg) so normal sensor noise never trips
// it, while a genuine wrong-direction crawl is caught within a few samples.
constexpr double kMaxDisplacementDeg = 5.0;
constexpr double kMaxAbsVelocityRadS = 1.0;
constexpr double kMaxAbsEffortNm = 1.0;
// Hold mode must capture a live position within this window or send nothing.
constexpr double kListenTimeoutSeconds = 1.0;
constexpr double kRadToDeg = 180.0 / mech::mech_protocol_cubemars::kPi;

MonotonicTime now() noexcept {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch();
  return *MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ticks).count());
}

MonotonicTime offset(MonotonicTime base, std::int64_t ns) noexcept {
  return *MonotonicTime::from_nanoseconds(base.nanoseconds() + ns);
}

bool number(const char* text, double& value) noexcept {
  char* end = nullptr;
  value = std::strtod(text, &end);
  return end != text && *end == '\0' && std::isfinite(value);
}

struct RunOptions final {
  std::string device{"/dev/ttyACM0"};
  bool hold_current{false};
  // Step-4 delta applied on top of the captured live position (hold mode
  // only). Bounded to +-45 deg (a quarter turn); larger moves are not
  // what this probe is for.
  double hold_delta_rad{0.0};
  double target_rad{0.0};
  double kp{0.0};
  double kd{0.0};
  double seconds{0.0};
};

// Step-4 owner-directed test: 30 deg. Bounded to +-45 deg so the move stays
// inside a quarter turn; larger moves are not what this probe is for.
constexpr double kMaxHoldDeltaRad = 45.0 * mech::mech_protocol_cubemars::kPi / 180.0;

int run(const RunOptions& options) noexcept {
  PosixCdcSerialPort serial{options.device};
  UsbCdcOptions cdc_options{};
  cdc_options.logical_bus = 1U;
  cdc_options.nominal_bitrate_hz = 0U;
  cdc_options.receive_queue_capacity = 128U;
  cdc_options.verified_board_version = {4U, 8U, 8U};
  UsbCdcTransport transport{serial, cdc_options};
  if (!transport.open() || !serial.send_pass_through_init()) {
    std::fprintf(stderr, "ERROR: CDC transport setup failed for %s\n",
                 options.device.c_str());
    transport.close();
    return 1;
  }

  Ak30Mapping mapping{};
  Ak30SessionConfig session_config{};
  session_config.drive_id = 104U;
  session_config.sub_mode = ForceControlSubMode::Position;
  session_config.mapping = mapping;
  session_config.gains = ForceControlGains{options.kp, options.kd};
  session_config.firmware_id = 1U;
  session_config.firmware_id_min = 1U;
  session_config.firmware_id_max = 1000U;

  DeviceConfig config{};
  config.device_id = 1U;
  config.name = "motor1";
  config.logical_bus = 1U;
  config.profile = ProtocolProfile::Ak30ForceControlExtended;
  config.frame_type = CanFrameType::Classic;
  config.frame_format = CanFrameFormat::Extended;
  config.command_id = CanId::create(
      mech::mech_protocol_cubemars::force_control_can_id(104U),
      CanFrameFormat::Extended);
  config.feedback_id = CanId::create(
      mech::mech_protocol_cubemars::feedback_can_id(104U),
      CanFrameFormat::Extended);
  config.command_payload_bytes = 8U;
  config.feedback_payload_bytes = 8U;
  config.writable = true;

  Ak30ForceControlSession session{transport, session_config};
  if (session.configure(config, transport.capabilities()) != AdapterResult::Ok ||
      session.activate() != AdapterResult::Ok) {
    std::fprintf(stderr, "ERROR: Position session configuration failed\n");
    transport.close();
    return 1;
  }

  const auto period_ns = static_cast<std::int64_t>(10e6);
  const auto run_ns = static_cast<std::int64_t>(options.seconds * 1e9);
  const auto tail_ns = static_cast<std::int64_t>(500e6);
  const auto listen_ns =
      static_cast<std::int64_t>(kListenTimeoutSeconds * 1e9);

  // Listen phase (hold mode): capture the live position before commanding.
  // Fail closed - if no valid feedback arrives, no position command is ever
  // sent, because one computed from a guessed position could move the motor.
  double command_position_rad = options.target_rad;
  if (options.hold_current) {
    const auto listen_start = now();
    bool captured = false;
    while (now().nanoseconds() - listen_start.nanoseconds() < listen_ns) {
      RawCanFrame frame{};
      if (transport.try_receive(frame) == TransportResult::Ok &&
          session.process(frame, now()) == AdapterResult::Ok) {
        const auto state = session.snapshot(now());
        if (state.status.quality == SampleQuality::Valid) {
          command_position_rad = state.position;
          captured = true;
          break;
        }
      }
    }
    if (!captured) {
      std::fprintf(stderr,
                   "ERROR: hold mode captured no feedback within %.1f s; "
                   "no command was sent\n",
                   kListenTimeoutSeconds);
      session.deactivate();
      transport.close();
      return 1;
    }
    // Step-4 shape: a small delta on top of the captured live position, so
    // the commanded displacement is exactly delta regardless of the mapping.
    command_position_rad += options.hold_delta_rad;
    std::printf("HOLD captured=%.3frad (%.1fdeg) delta=%.4frad (%.2fdeg) "
                "target=%.3frad (%.1fdeg) zero_offset=%.1fdeg\n",
                command_position_rad - options.hold_delta_rad,
                (command_position_rad - options.hold_delta_rad) * kRadToDeg,
                options.hold_delta_rad, options.hold_delta_rad * kRadToDeg,
                command_position_rad, command_position_rad * kRadToDeg,
                mapping.zero_offset_rad.value * kRadToDeg);
  }

  std::printf("CONFIGURED mode=%s target=%.3frad kp=%.3f kd=%.3f seconds=%.1f "
              "aborts: target_miss<%.0fdeg velocity<%.0frad_s effort<%.0fNm\n",
              options.hold_current ? "hold" : "target", command_position_rad,
              options.kp, options.kd, options.seconds,
              std::max(kMaxDisplacementDeg,
                       std::abs(options.hold_delta_rad * kRadToDeg) +
                           kMaxDisplacementDeg),
              kMaxAbsVelocityRadS, kMaxAbsEffortNm);

  CanonicalDeviceCommand command{};
  command.position = command_position_rad;
  command.deadline = offset(now(), period_ns * 2);
  std::uint64_t sent = 0U;
  std::uint64_t samples = 0U;
  bool aborted = false;
  std::string reason;
  double hold_position_rad = command_position_rad;
  double first_position_deg = 0.0;
  double last_position_deg = 0.0;
  double min_effort_nm = 0.0;
  double max_effort_nm = 0.0;
  bool have_effort = false;
  bool have_position = false;
  const auto start = now();
  auto next = start;
  while (now().nanoseconds() - start.nanoseconds() < run_ns) {
    const auto current = now();
    if (current.nanoseconds() >= next.nanoseconds()) {
      command.generation = ++sent;
      command.deadline = offset(current, period_ns * 2);
      const auto result = session.submit(command, current);
      if (result != AdapterResult::Ok && result != AdapterResult::WouldBlock) {
        aborted = true;
        reason = "submit_failed";
        break;
      }
      next = offset(next, period_ns);
    }
    RawCanFrame frame{};
    const auto received = transport.try_receive(frame);
    if (received == TransportResult::Ok) {
      if (session.process(frame, current) == AdapterResult::Ok) {
        const auto state = session.snapshot(current);
        if (state.status.quality == SampleQuality::Valid) {
          ++samples;
          hold_position_rad = state.position;
          const double position_deg = state.position * kRadToDeg;
          if (!have_position) {
            first_position_deg = position_deg;
            have_position = true;
          }
          last_position_deg = position_deg;
          const double effort_nm = state.effort;
          if (!have_effort) {
            min_effort_nm = effort_nm;
            max_effort_nm = effort_nm;
            have_effort = true;
          }
          if (effort_nm < min_effort_nm) {
            min_effort_nm = effort_nm;
          }
          if (effort_nm > max_effort_nm) {
            max_effort_nm = effort_nm;
          }
          std::printf("sample=%llu pos=%.3fdeg vel=%.3frad_s effort=%.3fNm "
                      "temp=unavailable fault=%u\n",
                      static_cast<unsigned long long>(samples),
                      position_deg, state.velocity, state.effort,
                      state.status.raw_fault_code);
          if (state.status.raw_fault_code != 0U) {
            aborted = true;
            reason = "fault=" +
                     std::to_string(state.status.raw_fault_code);
            break;
          }
          // Displacement bound: measured against the commanded target, not the
          // starting position. A hold+delta run is EXPECTED to travel the
          // delta, so its allowed displacement is delta + a small overshoot
          // margin; a plain hold or numeric-target run stays at the fixed
          // kMaxDisplacementDeg. Travel beyond that is an uncommanded move.
          const double allowed_deg = std::max(
              kMaxDisplacementDeg,
              std::abs(options.hold_delta_rad * kRadToDeg) +
                  kMaxDisplacementDeg);
          if (std::abs(position_deg - command_position_rad * kRadToDeg) >
              allowed_deg) {
            aborted = true;
            reason = "displaced_to=" + std::to_string(position_deg) + "deg";
            break;
          }
          if (std::abs(state.velocity) > kMaxAbsVelocityRadS) {
            aborted = true;
            reason = "velocity=" + std::to_string(state.velocity);
            break;
          }
          if (std::abs(state.effort) > kMaxAbsEffortNm) {
            aborted = true;
            reason = "effort=" + std::to_string(state.effort);
            break;
          }
        }
      }
    } else if (received != TransportResult::WouldBlock) {
      aborted = true;
      reason = "receive_failed";
      break;
    }
  }

  // Position-mode tail: the session's configured Kp/Kd remain active, so a
  // zero-valued CanonicalDeviceCommand would command the calibrated zero,
  // potentially causing an unexpected move. Hold the last observed position
  // (or the commanded target if no feedback arrived) with zero feed-forward.
  const auto tail_start = now();
  CanonicalDeviceCommand neutral{};
  auto next_tail_send = tail_start;
  while (now().nanoseconds() - tail_start.nanoseconds() < tail_ns) {
    const auto current = now();
    if (current.nanoseconds() >= next_tail_send.nanoseconds()) {
      neutral.position = hold_position_rad;
      neutral.generation = ++sent;
      neutral.deadline = offset(current, period_ns * 2);
      static_cast<void>(session.submit(neutral, current));
      next_tail_send = offset(next_tail_send, period_ns);
    }
    RawCanFrame frame{};
    if (transport.try_receive(frame) == TransportResult::Ok) {
      if (session.process(frame, current) == AdapterResult::Ok) {
        const auto state = session.snapshot(current);
        if (state.status.quality == SampleQuality::Valid) {
          hold_position_rad = state.position;
        }
      }
    }
  }
  session.deactivate();
  transport.close();
  std::printf("END sent=%llu samples=%llu abort=%s mode=%s "
              "target=%.3frad kp=%.3f kd=%.3f pos=%.1f->%.1fdeg "
              "effort_nm=%.3f..%.3f\n",
              static_cast<unsigned long long>(sent),
              static_cast<unsigned long long>(samples),
              aborted ? reason.c_str() : "none",
              options.hold_current ? "hold" : "target", command_position_rad,
              options.kp, options.kd,
              have_position ? first_position_deg : 0.0,
              have_position ? last_position_deg : 0.0,
              have_effort ? min_effort_nm : 0.0,
              have_effort ? max_effort_nm : 0.0);
  return aborted ? 2 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 5 || argc > 7) {
    std::fprintf(stderr,
                 "usage: %s <target_rad|hold> <kp> <kd> <seconds> [device] or\n"
                 "       %s hold <delta_rad> <kp> <kd> <seconds> [device]\n"
                 "  target_rad: canonical target within +-0.5 rad, or 'hold' "
                 "to capture the live position and command it back (zero "
                 "displacement by construction)\n"
                 "  delta_rad: optional hold-mode offset within +-45 deg "
                 "(step-4 small-move shape)\n"
                 "  kp/kd are bounded to 20/5; seconds to 10\n",
                 argv[0], argv[0]);
    return 1;
  }
  RunOptions options{};
  const char* target_text = argv[1];
  if (std::strcmp(target_text, "hold") == 0) {
    options.hold_current = true;
    options.target_rad = 0.0;
  } else if (!number(target_text, options.target_rad) ||
             std::abs(options.target_rad) > 0.5) {
    std::fprintf(stderr, "ERROR: target must be 'hold' or within +-0.5 rad\n");
    return 1;
  }
  int arg_index = 2;
  if (options.hold_current && argc >= 7) {
    // hold + delta form: the only way to reach argv[6..] is with a delta.
    if (!number(argv[2], options.hold_delta_rad) ||
        std::abs(options.hold_delta_rad) > kMaxHoldDeltaRad) {
      std::fprintf(stderr,
                   "ERROR: hold delta must be a number within +-45 deg\n");
      return 1;
    }
    arg_index = 3;
  }
  if (!number(argv[arg_index], options.kp) ||
      !number(argv[arg_index + 1], options.kd) ||
      !number(argv[arg_index + 2], options.seconds) || options.kp < 0.0 ||
      options.kp > 20.0 || options.kd < 0.0 || options.kd > 5.0 ||
      options.seconds <= 0.0 || options.seconds > 10.0) {
    std::fprintf(stderr, "ERROR: limits kp<=20 kd<=5 seconds<=10\n");
    return 1;
  }
  if (argc >= arg_index + 4) {
    options.device = argv[arg_index + 3];
  }
  return run(options);
}
