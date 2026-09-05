// Controlled AK3.0 position-mode bench probe.
// This is an opt-in bring-up tool, not the production ros2_control plugin.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

int run(const char* device, double target_rad, double kp, double kd,
        double seconds) noexcept {
  PosixCdcSerialPort serial{device};
  UsbCdcOptions options{};
  options.logical_bus = 1U;
  options.nominal_bitrate_hz = 0U;
  options.receive_queue_capacity = 128U;
  options.verified_board_version = {4U, 8U, 8U};
  UsbCdcTransport transport{serial, options};
  if (!transport.open() || !serial.send_pass_through_init()) {
    std::fprintf(stderr, "ERROR: CDC transport setup failed for %s\n", device);
    transport.close();
    return 1;
  }

  Ak30Mapping mapping{};
  Ak30SessionConfig session_config{};
  session_config.drive_id = 104U;
  session_config.sub_mode = ForceControlSubMode::Position;
  session_config.mapping = mapping;
  session_config.gains = ForceControlGains{kp, kd};
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
  const auto run_ns = static_cast<std::int64_t>(seconds * 1e9);
  const auto tail_ns = static_cast<std::int64_t>(500e6);
  CanonicalDeviceCommand command{};
  command.position = target_rad;
  command.deadline = offset(now(), period_ns * 2);
  std::uint64_t sent = 0U;
  std::uint64_t samples = 0U;
  bool aborted = false;
  std::string reason;
  double hold_position_rad = target_rad;
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
          std::printf("sample=%llu pos=%.3fdeg vel=%.3frad_s effort=%.3fNm "
                      "temp=unavailable fault=%u\n",
                      static_cast<unsigned long long>(samples),
                      state.position * 180.0 / mech::mech_protocol_cubemars::kPi,
                      state.velocity, state.effort,
                      state.status.raw_fault_code);
          if (state.status.raw_fault_code != 0U) {
            aborted = true;
            reason = "fault";
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
  // (or the requested target if no feedback arrived) with zero feed-forward.
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
  std::printf("END sent=%llu samples=%llu abort=%s target=%.3frad kp=%.3f kd=%.3f\n",
              static_cast<unsigned long long>(sent),
              static_cast<unsigned long long>(samples),
              aborted ? reason.c_str() : "none", target_rad, kp, kd);
  return aborted ? 2 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 5 || argc > 6) {
    std::fprintf(stderr,
                 "usage: %s <target_rad> <kp> <kd> <seconds> [device]\n",
                 argv[0]);
    return 1;
  }
  double target = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double seconds = 0.0;
  if (!number(argv[1], target) || !number(argv[2], kp) || !number(argv[3], kd) ||
      !number(argv[4], seconds) || std::abs(target) > 0.5 || kp < 0.0 ||
      kp > 20.0 || kd < 0.0 || kd > 5.0 || seconds <= 0.0 || seconds > 10.0) {
    std::fprintf(stderr, "ERROR: limits target<=0.5rad kp<=20 kd<=5 seconds<=10\n");
    return 1;
  }
  return run(argc == 6 ? argv[5] : "/dev/ttyACM0", target, kp, kd, seconds);
}
