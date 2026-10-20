#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <thread>

#include <poll.h>
#include <unistd.h>

#include "mech_control_core/posix_cdc_serial_port.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

namespace {

using namespace mech::mech_control_core;
using namespace mech::mech_protocol_damiao;

constexpr std::uint16_t kBus = 1U;
constexpr std::uint32_t kMotorCanId = 0x02U;
constexpr std::uint32_t kFeedbackCanId = 0x00U;
constexpr double kHighProbeVelocityRadS = 1.5;
constexpr double kHighProbeKd = 3.2;
volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) { stop_requested = 1; }

RawCanFrame command(std::uint32_t id,
                    const std::array<std::uint8_t, 8U>& bytes) {
  return *RawCanFrame::create(
      kBus, *CanId::create(id, CanFrameFormat::Standard),
      CanFrameType::Classic, FrameDirection::Tx, 8U,
      std::array<std::uint8_t, kMaxCanPayloadBytes>{
          bytes[0], bytes[1], bytes[2], bytes[3],
          bytes[4], bytes[5], bytes[6], bytes[7]},
      *MonotonicTime::from_nanoseconds(0));
}

bool send_bounded(Transport& transport, const RawCanFrame& frame) {
  for (int attempt = 0; attempt < 20; ++attempt) {
    const auto result = transport.try_send(frame);
    if (result == TransportResult::Ok) return true;
    if (result != TransportResult::WouldBlock &&
        result != TransportResult::QueueFull) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

bool wait_for_feedback(Transport& transport, std::chrono::milliseconds timeout,
                       MitFeedback& feedback) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
    RawCanFrame frame{};
    const auto result = transport.try_receive(frame);
    if (result == TransportResult::WouldBlock || result == TransportResult::Invalid) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    if (result != TransportResult::Ok) return false;
    if (decode_mit_feedback(frame, kBus, kFeedbackCanId, 2U,
                            kSdkDm4340Limits, feedback)) {
      std::printf("Feedback: pos=%.3f rad vel=%.3f rad/s torque=%.3f Nm status=%u mos=%u C rotor=%u C\n",
                  feedback.position_rad, feedback.velocity_rad_s,
                  feedback.torque_nm, feedback.raw_fault,
                  feedback.raw_byte_6, feedback.raw_byte_7);
      return true;
    }
    std::printf("Other frame: id=0x%X dlc=%u\n", frame.id.value,
                frame.payload_size);
  }
  return false;
}

bool disable(Transport& transport) {
  const auto frame = command(kMotorCanId,
                             {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                              0xFFU, 0xFFU, 0xFFU, 0xFDU});
  int sent = 0;
  for (int i = 0; i < 3; ++i) {
    sent += send_bounded(transport, frame) ? 1 : 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::printf("Disable frames accepted by gateway: %d/3\n", sent);
  if (sent != 3) {
    std::fprintf(stderr, "Disable send incomplete: cut motor power now.\n");
  }
  return sent == 3;
}

bool jog(Transport& transport, double start_position) {
  std::array<std::uint8_t, 8U> payload{};
  if (!encode_mit({start_position + 0.1, 0.0, 2.0, 0.1, 0.0},
                  kSdkDm4340Limits, payload)) {
    std::fprintf(stderr, "Jog target outside configured MIT range.\n");
    return false;
  }
  const auto frame = command(kMotorCanId, payload);
  double last_position = start_position;
  for (int i = 0; i < 20 && !stop_requested; ++i) {
    if (!send_bounded(transport, frame)) {
      std::fprintf(stderr, "Jog command send failed.\n");
      return false;
    }
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(50), feedback)) {
      std::fprintf(stderr, "Jog feedback timeout.\n");
      return false;
    }
    const double displacement = feedback.position_rad - start_position;
    if (!feedback_motor_enabled(feedback.raw_fault) ||
        displacement < -0.05 || displacement > 0.2 ||
        std::abs(feedback.velocity_rad_s) > 2.0 ||
        std::abs(feedback.torque_nm) > 0.5) {
      std::fprintf(stderr, "Jog safety bound exceeded; disabling.\n");
      return false;
    }
    last_position = feedback.position_rad;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::printf("Jog displacement: %.3f rad at motor shaft.\n",
              last_position - start_position);
  return !stop_requested && last_position - start_position >= 0.01;
}

bool slow_spin(Transport& transport, double start_position) {
  std::array<std::uint8_t, 8U> payload{};
  if (!encode_mit({start_position, 0.1, 0.0, 3.0, 0.0},
                  kSdkDm4340Limits, payload)) {
    return false;
  }
  const auto drive_frame = command(kMotorCanId, payload);
  const auto started = std::chrono::steady_clock::now();
  double last_position = start_position;
  double max_displacement = 0.0;
  for (int i = 0; i < 50 && !stop_requested; ++i) {
    if (!send_bounded(transport, drive_frame)) {
      std::fprintf(stderr, "Low-speed command send failed.\n");
      return false;
    }
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(60), feedback)) {
      std::fprintf(stderr, "Low-speed feedback timeout.\n");
      return false;
    }
    const double displacement = feedback.position_rad - start_position;
    if (!feedback_motor_enabled(feedback.raw_fault) ||
        displacement < -0.1 || displacement > 0.7 ||
        std::abs(feedback.velocity_rad_s) > 0.3 ||
        std::abs(feedback.torque_nm) > 0.5) {
      std::fprintf(stderr, "Low-speed safety bound exceeded.\n");
      return false;
    }
    last_position = feedback.position_rad;
    max_displacement = std::max(max_displacement, displacement);
    std::this_thread::sleep_until(started + std::chrono::milliseconds((i + 1) * 100));
  }
  if (stop_requested) return false;

  if (!encode_mit({last_position, 0.0, 0.0, 1.0, 0.0},
                  kSdkDm4340Limits, payload)) {
    return false;
  }
  const auto neutral_frame = command(kMotorCanId, payload);
  bool stopped = false;
  for (int i = 0; i < 5 && !stop_requested; ++i) {
    if (!send_bounded(transport, neutral_frame)) break;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(60), feedback)) break;
    if (!feedback_motor_enabled(feedback.raw_fault) ||
        std::abs(feedback.position_rad - start_position) > 0.7 ||
        std::abs(feedback.torque_nm) > 0.5) break;
    stopped = std::abs(feedback.velocity_rad_s) < 0.03;
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  std::printf("Low-speed max displacement: %.3f rad; final position: %.3f rad.\n",
              max_displacement, last_position);
  return stopped && !stop_requested;
}

bool read_uint_parameter(Transport& transport, std::uint8_t register_id,
                         std::uint32_t& value) {
  const auto request = command(0x7FFU, {0x02U, 0x00U, 0x33U, register_id,
                                        0U, 0U, 0U, 0U});
  if (!send_bounded(transport, request)) return false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(1200);
  while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
    RawCanFrame frame{};
    const auto result = transport.try_receive(frame);
    if (result == TransportResult::WouldBlock || result == TransportResult::Invalid) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    if (result != TransportResult::Ok) return false;
    if (frame.logical_bus != kBus ||
        frame.id.format != CanFrameFormat::Standard ||
        frame.id.value != kFeedbackCanId ||
        frame.direction != FrameDirection::Rx || frame.payload_size != 8U ||
        frame.payload[0] != 0x02U || frame.payload[1] != 0U ||
        frame.payload[2] != 0x33U || frame.payload[3] != register_id) {
      continue;
    }
    value = static_cast<std::uint32_t>(frame.payload[4]) |
            (static_cast<std::uint32_t>(frame.payload[5]) << 8) |
            (static_cast<std::uint32_t>(frame.payload[6]) << 16) |
            (static_cast<std::uint32_t>(frame.payload[7]) << 24);
    return true;
  }
  std::fprintf(stderr, "Register %u readback timed out.\n", register_id);
  return false;
}

bool set_timeout_200ms(Transport& transport) {
  constexpr std::uint32_t kTimeoutTicks = 4000U;  // 50 us per tick.
  std::uint32_t before = 0U;
  if (!read_uint_parameter(transport, 9U, before)) return false;
  std::printf("TIMEOUT before: %u ticks\n", before);
  const auto write = command(0x7FFU, {0x02U, 0x00U, 0x55U, 9U,
                                       0xA0U, 0x0FU, 0x00U, 0x00U});
  if (!send_bounded(transport, write)) return false;
  std::uint32_t after = 0U;
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (read_uint_parameter(transport, 9U, after)) {
      if (after == kTimeoutTicks) {
        std::printf("TIMEOUT readback: %u ticks (nominal 200 ms, RAM only)\n", after);
        return true;
      }
      std::fprintf(stderr, "TIMEOUT readback mismatch: %u ticks.\n", after);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::fprintf(stderr, "TIMEOUT readback unavailable; do not enable motor.\n");
  return false;
}

bool verify_timeout_200ms(Transport& transport) {
  std::uint32_t ticks = 0U;
  if (!read_uint_parameter(transport, 9U, ticks) || ticks != 4000U) {
    std::fprintf(stderr, "TIMEOUT is not 4000 ticks; refusing enable.\n");
    return false;
  }
  std::uint32_t mode = 0U;
  if (!read_uint_parameter(transport, 10U, mode) || mode != 1U) {
    std::fprintf(stderr, "MIT mode not confirmed; refusing enable.\n");
    return false;
  }
  const auto enable = command(kMotorCanId,
      {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU});
  bool observed_timeout = false;
  if (send_bounded(transport, enable)) {
    MitFeedback enabled{};
    if (wait_for_feedback(transport, std::chrono::milliseconds(500), enabled) &&
        enabled.raw_fault == 1U) {
      std::array<std::uint8_t, 8U> payload{};
      if (encode_mit({0.0, 0.0, 0.0, 0.0, 0.0}, kSdkDm4340Limits, payload) &&
          send_bounded(transport, command(kMotorCanId, payload))) {
        std::printf("Zero MIT frame sent; withholding all TX for 450 ms.\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(450));
        for (int i = 0; i < 128; ++i) {
          RawCanFrame stale{};
          if (transport.try_receive(stale) != TransportResult::Ok) break;
        }
        const auto query = command(0x7FFU,
            {0x02U, 0x00U, 0xCCU, 0U, 0U, 0U, 0U, 0U});
        MitFeedback after{};
        if (send_bounded(transport, query) &&
            wait_for_feedback(transport, std::chrono::milliseconds(500), after)) {
          observed_timeout = after.raw_fault == 0xDU &&
              std::abs(after.velocity_rad_s) < 0.05;
          std::printf("After silence: status=%u speed=%.3f rad/s; timeout verified=%s\n",
                      after.raw_fault, after.velocity_rad_s,
                      observed_timeout ? "yes" : "no");
        }
      }
    }
  }
  const bool disabled = disable(transport);
  return observed_timeout && disabled;
}

bool read_mode(Transport& transport, std::uint32_t& mode) {
  if (!read_uint_parameter(transport, 10U, mode)) return false;
  std::printf("Control-mode register 10: %u (1=MIT, 2=position-velocity, 3=velocity, 4=position-force)\n",
              mode);
  return mode >= 1U && mode <= 4U;
}

bool monitor_feedback(Transport& transport) {
  const auto query = command(0x7FFU, {0x02U, 0x00U, 0xCCU, 0U,
                                      0U, 0U, 0U, 0U});
  const auto started = std::chrono::steady_clock::now();
  int consecutive_timeouts = 0;
  for (int i = 0; i < 300 && !stop_requested; ++i) {
    if (!send_bounded(transport, query)) {
      std::fprintf(stderr, "Status request failed.\n");
      return false;
    }
    std::printf("[%5.1f s] ",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    MitFeedback feedback{};
    if (wait_for_feedback(transport, std::chrono::milliseconds(120), feedback)) {
      consecutive_timeouts = 0;
    } else {
      std::printf("feedback timeout\n");
      if (++consecutive_timeouts >= 3) return false;
    }
    std::fflush(stdout);
    std::this_thread::sleep_until(started + std::chrono::milliseconds((i + 1) * 200));
  }
  return !stop_requested;
}

bool torque_step(Transport& transport, double start_position) {
  std::array<std::uint8_t, 8U> payload{};
  if (!encode_mit({start_position, 1.0, 0.0, 0.5, 0.0},
                  kSdkDm4340Limits, payload)) return false;
  const auto drive_frame = command(kMotorCanId, payload);
  const auto started = std::chrono::steady_clock::now();
  double last_position = start_position;
  double peak_torque = 0.0;
  for (int i = 0; i < 10 && !stop_requested; ++i) {
    if (!send_bounded(transport, drive_frame)) return false;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(60), feedback)) return false;
    const double displacement = feedback.position_rad - start_position;
    if (!feedback_motor_enabled(feedback.raw_fault) ||
        displacement < -0.05 || displacement > 0.1 ||
        std::abs(feedback.velocity_rad_s) > 0.3 ||
        std::abs(feedback.torque_nm) > 0.8) {
      std::fprintf(stderr, "Torque-step safety bound exceeded.\n");
      return false;
    }
    last_position = feedback.position_rad;
    peak_torque = std::max(peak_torque, std::abs(feedback.torque_nm));
    std::this_thread::sleep_until(started + std::chrono::milliseconds((i + 1) * 50));
  }
  if (stop_requested) return false;

  if (!encode_mit({last_position, 0.0, 0.0, 1.0, 0.0},
                  kSdkDm4340Limits, payload)) return false;
  const auto neutral_frame = command(kMotorCanId, payload);
  bool stopped = false;
  for (int i = 0; i < 5 && !stop_requested; ++i) {
    if (!send_bounded(transport, neutral_frame)) return false;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(60), feedback) ||
        !feedback_motor_enabled(feedback.raw_fault) ||
        std::abs(feedback.position_rad - start_position) > 0.1 ||
        std::abs(feedback.velocity_rad_s) > 0.3 ||
        std::abs(feedback.torque_nm) > 0.8) return false;
    stopped = std::abs(feedback.velocity_rad_s) < 0.03;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::printf("Torque-step peak feedback: %.3f Nm; displacement: %.3f rad; zero-speed settled: %s.\n",
              peak_torque, last_position - start_position, stopped ? "yes" : "no");
  return stopped && !stop_requested;
}

bool gui_profile(Transport& transport, double start_position,
                 std::chrono::seconds duration, double max_displacement,
                 double kd = 1.0, double feedforward_nm = 0.0) {
  std::array<std::uint8_t, 8U> payload{};
  if (!encode_mit({0.0, 0.8, 0.0, kd, feedforward_nm},
                  kSdkDm4340Limits, payload)) return false;
  const auto drive_frame = command(kMotorCanId, payload);
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + duration;
  double last_position = start_position;
  double peak_torque = 0.0;
  double steady_velocity_sum = 0.0;
  int steady_samples = 0;
  int samples = 0;
  for (int i = 0; i < duration.count() * 100 && !stop_requested &&
                  std::chrono::steady_clock::now() < deadline; ++i) {
    if (!send_bounded(transport, drive_frame)) return false;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(20), feedback)) return false;
    const double displacement = feedback.position_rad - start_position;
    if (!feedback_motor_enabled(feedback.raw_fault) ||
        displacement < -0.1 || displacement > max_displacement ||
        std::abs(feedback.velocity_rad_s) > 1.2 ||
        std::abs(feedback.torque_nm) > (kd > 1.0 ? 2.2 : 1.2)) {
      std::fprintf(stderr, "GUI-profile safety bound exceeded.\n");
      return false;
    }
    last_position = feedback.position_rad;
    peak_torque = std::max(peak_torque, std::abs(feedback.torque_nm));
    if (std::chrono::steady_clock::now() - started >= std::chrono::seconds(1)) {
      steady_velocity_sum += feedback.velocity_rad_s;
      ++steady_samples;
    }
    ++samples;
    std::this_thread::sleep_until(started + std::chrono::milliseconds((i + 1) * 10));
  }
  if (stop_requested || samples == 0) return false;

  if (!encode_mit({0.0, 0.0, 0.0, 1.0, 0.0},
                  kSdkDm4340Limits, payload)) return false;
  const auto neutral_frame = command(kMotorCanId, payload);
  bool stopped = false;
  for (int i = 0; i < 25 && !stop_requested; ++i) {
    if (!send_bounded(transport, neutral_frame)) return false;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(30), feedback) ||
        !feedback_motor_enabled(feedback.raw_fault) ||
        feedback.position_rad - start_position < -0.1 ||
        feedback.position_rad - start_position > max_displacement ||
        std::abs(feedback.velocity_rad_s) > 1.0 ||
        std::abs(feedback.torque_nm) > 1.2) return false;
    stopped = std::abs(feedback.velocity_rad_s) < 0.03;
    if (stopped) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::printf("GUI-profile target_velocity=0.8 rad/s kd=%.1f feedforward=%.2f Nm steady_velocity=%.3f rad/s samples=%d elapsed=%.3f s peak_torque=%.3f Nm displacement=%.3f rad settled=%s\n",
              kd, feedforward_nm,
              steady_samples ? steady_velocity_sum / steady_samples : 0.0,
              samples, std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
              peak_torque, last_position - start_position, stopped ? "yes" : "no");
  return stopped && !stop_requested;
}

bool gui_profile_1ms(Transport& transport, double start_position,
                     bool high_torque_probe) {
  using Clock = std::chrono::steady_clock;
  const int max_ticks = high_torque_probe ? 200 : 2000;
  const double max_displacement = high_torque_probe ? 0.3 : 1.0;
  const double max_velocity = high_torque_probe ? 2.0 : 1.0;
  const double max_feedback_torque = high_torque_probe ? 5.0 : 1.2;
  std::array<std::uint8_t, 8U> payload{};
  const MitCommand baseline_target{0.0, 0.8, 0.0, 1.0, 0.0};
  if (!high_torque_probe &&
      !encode_mit(baseline_target, kSdkDm4340Limits, payload)) return false;
  const auto baseline_frame = command(kMotorCanId, payload);
  const auto started = Clock::now();
  auto last_feedback = started;
  auto previous_send = started;
  double max_send_gap_ms = 0.0;
  double max_feedback_gap_ms = 0.0;
  double last_position = start_position;
  double peak_torque = 0.0;
  double peak_velocity = 0.0;
  int late_sends = 0;
  int feedback_count = 0;
  int sent_count = 0;

  for (int tick = 0; tick < max_ticks && !stop_requested; ++tick) {
    const auto deadline = started + std::chrono::milliseconds(tick);
    std::this_thread::sleep_until(deadline);
    const auto now = Clock::now();
    const double lateness_ms = std::chrono::duration<double, std::milli>(now - deadline).count();
    if (lateness_ms > 3.0) {
      std::fprintf(stderr, "1 ms send deadline missed by %.3f ms.\n", lateness_ms);
      return false;
    }
    if (lateness_ms > 0.5) ++late_sends;
    if (tick > 0) {
      max_send_gap_ms = std::max(max_send_gap_ms,
          std::chrono::duration<double, std::milli>(now - previous_send).count());
    }
    auto drive_frame = baseline_frame;
    if (high_torque_probe) {
      const double progress = std::min(1.0, static_cast<double>(tick) / 50.0);
      const double ramp = progress * progress * (3.0 - 2.0 * progress);
      if (!encode_mit({0.0, kHighProbeVelocityRadS * ramp, 0.0,
                       1.0 + (kHighProbeKd - 1.0) * ramp, 0.0},
                      kSdkDm4340Limits, payload)) return false;
      drive_frame = command(kMotorCanId, payload);
    }
    if (transport.try_send(drive_frame) != TransportResult::Ok) {
      std::fprintf(stderr, "1 ms command write failed.\n");
      return false;
    }
    previous_send = now;
    ++sent_count;

    for (int read = 0; read < 32; ++read) {
      RawCanFrame frame{};
      const auto result = transport.try_receive(frame);
      if (result == TransportResult::WouldBlock) break;
      if (result != TransportResult::Ok) {
        std::fprintf(stderr, "1 ms feedback receive failed.\n");
        return false;
      }
      MitFeedback feedback{};
      if (!decode_mit_feedback(frame, kBus, kFeedbackCanId, 2U,
                               kSdkDm4340Limits, feedback)) continue;
      const auto received_at = Clock::now();
      max_feedback_gap_ms = std::max(max_feedback_gap_ms,
          std::chrono::duration<double, std::milli>(received_at - last_feedback).count());
      last_feedback = received_at;
      ++feedback_count;
      const double displacement = feedback.position_rad - start_position;
      if (!feedback_motor_enabled(feedback.raw_fault) ||
          displacement < -0.1 || displacement > max_displacement ||
          std::abs(feedback.velocity_rad_s) > max_velocity ||
          std::abs(feedback.torque_nm) > max_feedback_torque) {
        std::fprintf(stderr, "1 ms profile safety bound exceeded.\n");
        return false;
      }
      last_position = feedback.position_rad;
      peak_torque = std::max(peak_torque, std::abs(feedback.torque_nm));
      peak_velocity = std::max(peak_velocity, std::abs(feedback.velocity_rad_s));
    }
    if (Clock::now() - last_feedback > std::chrono::milliseconds(40)) {
      std::fprintf(stderr, "1 ms profile feedback stale for over 40 ms.\n");
      return false;
    }
    if (high_torque_probe && tick >= 100 &&
        std::abs(last_position - start_position) < 0.01) {
      std::fprintf(stderr, "High-torque probe did not move within 100 ms.\n");
      return false;
    }
    if (high_torque_probe && last_position - start_position >= 0.2) break;
  }
  if (stop_requested || sent_count == 0 || feedback_count == 0 ||
      (!high_torque_probe && sent_count != max_ticks)) return false;

  if (!encode_mit({0.0, 0.0, 0.0, 1.0, 0.0}, kSdkDm4340Limits, payload)) return false;
  const auto neutral_frame = command(kMotorCanId, payload);
  bool settled = false;
  for (int i = 0; i < 25 && !stop_requested; ++i) {
    if (!send_bounded(transport, neutral_frame)) return false;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(30), feedback) ||
        !feedback_motor_enabled(feedback.raw_fault) ||
        feedback.position_rad - start_position < -0.1 ||
        feedback.position_rad - start_position > max_displacement ||
        std::abs(feedback.velocity_rad_s) > max_velocity ||
        std::abs(feedback.torque_nm) > max_feedback_torque) return false;
    last_position = feedback.position_rad;
    peak_torque = std::max(peak_torque, std::abs(feedback.torque_nm));
    peak_velocity = std::max(peak_velocity, std::abs(feedback.velocity_rad_s));
    settled = std::abs(feedback.velocity_rad_s) < 0.03 &&
              std::abs(feedback.torque_nm) < 0.1;
    if (settled) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::printf("1 ms profile sent=%d feedback=%d elapsed=%.3f s late>0.5ms=%d max_send_gap=%.3f ms max_feedback_gap=%.3f ms peak_velocity=%.3f rad/s peak_torque=%.3f Nm final_displacement=%.3f rad settled=%s\n",
              sent_count, feedback_count,
              std::chrono::duration<double>(Clock::now() - started).count(),
              late_sends, max_send_gap_ms, max_feedback_gap_ms,
              peak_velocity, peak_torque, last_position - start_position,
              settled ? "yes" : "no");
  return settled && !stop_requested;
}

bool interactive_mit(Transport& transport) {
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  auto last_feedback = started;
  std::int64_t tick = 0;
  std::uint64_t feedback_count = 0;
  bool manual_stop = false;
  bool healthy = true;
  std::array<std::uint8_t, 8U> payload{};

  std::printf("Interactive MIT: v=%.1f rad/s Kd=%.1f. Press Enter to stop.\n",
              kHighProbeVelocityRadS, kHighProbeKd);
  std::fflush(stdout);
  while (!stop_requested && !manual_stop && healthy) {
    pollfd input{STDIN_FILENO, static_cast<short>(POLLIN | POLLHUP | POLLERR), 0};
    const int ready = poll(&input, 1, 0);
    if (ready < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "Terminal polling failed.\n");
      healthy = false;
      break;
    }
    if (ready > 0) {
      if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        std::fprintf(stderr, "Terminal disconnected.\n");
        healthy = false;
        break;
      }
      if (input.revents & POLLIN) {
        char key[32]{};
        const auto count = read(STDIN_FILENO, key, sizeof(key));
        if (count <= 0) {
          std::fprintf(stderr, "Terminal input closed.\n");
          healthy = false;
        } else {
          manual_stop = true;
        }
        break;
      }
    }

    const auto deadline = started + std::chrono::milliseconds(tick);
    std::this_thread::sleep_until(deadline);
    const auto now = Clock::now();
    if (now - deadline > std::chrono::milliseconds(3)) {
      std::fprintf(stderr, "Interactive send deadline exceeded.\n");
      healthy = false;
      break;
    }
    const double progress = std::min(1.0, static_cast<double>(tick) / 50.0);
    const double ramp = progress * progress * (3.0 - 2.0 * progress);
    if (!encode_mit({0.0, kHighProbeVelocityRadS * ramp, 0.0,
                     1.0 + (kHighProbeKd - 1.0) * ramp, 0.0},
                    kSdkDm4340Limits, payload) ||
        transport.try_send(command(kMotorCanId, payload)) != TransportResult::Ok) {
      std::fprintf(stderr, "Interactive command send failed.\n");
      healthy = false;
      break;
    }
    ++tick;
    for (int read_count = 0; read_count < 32; ++read_count) {
      RawCanFrame frame{};
      const auto result = transport.try_receive(frame);
      if (result == TransportResult::WouldBlock) break;
      if (result != TransportResult::Ok) {
        std::fprintf(stderr, "Interactive feedback receive failed.\n");
        healthy = false;
        break;
      }
      MitFeedback feedback{};
      if (!decode_mit_feedback(frame, kBus, kFeedbackCanId, 2U,
                               kSdkDm4340Limits, feedback)) continue;
      last_feedback = Clock::now();
      ++feedback_count;
      if (!feedback_motor_enabled(feedback.raw_fault) ||
          std::abs(feedback.velocity_rad_s) > 2.0 ||
          std::abs(feedback.torque_nm) > 5.0) {
        std::fprintf(stderr, "Interactive feedback bound exceeded.\n");
        healthy = false;
        break;
      }
      if (feedback_count % 100 == 0) {
        std::printf("pos=%.3f rad vel=%.3f rad/s torque=%.3f Nm status=%u\n",
                    feedback.position_rad, feedback.velocity_rad_s,
                    feedback.torque_nm, feedback.raw_fault);
        std::fflush(stdout);
      }
    }
    if (Clock::now() - last_feedback > std::chrono::milliseconds(40)) {
      std::fprintf(stderr, "Interactive feedback stale for over 40 ms.\n");
      healthy = false;
    }
  }

  if (!healthy || stop_requested || !manual_stop) return false;
  if (!encode_mit({0.0, 0.0, 0.0, 1.0, 0.0}, kSdkDm4340Limits, payload)) return false;
  const auto neutral = command(kMotorCanId, payload);
  for (int i = 0; i < 25; ++i) {
    if (!send_bounded(transport, neutral)) return false;
    MitFeedback feedback{};
    if (!wait_for_feedback(transport, std::chrono::milliseconds(30), feedback) ||
        !feedback_motor_enabled(feedback.raw_fault) ||
        std::abs(feedback.velocity_rad_s) > 2.0 ||
        std::abs(feedback.torque_nm) > 5.0) return false;
    if (std::abs(feedback.velocity_rad_s) < 0.03 &&
        std::abs(feedback.torque_nm) < 0.1) {
      std::printf("Interactive MIT stopped after %lld commands and %llu feedback frames; motor remains enabled.\n",
                  static_cast<long long>(tick),
                  static_cast<unsigned long long>(feedback_count));
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::fprintf(stderr, "Motor did not settle after manual stop.\n");
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string_view(argv[1]) != "/dev/ttyACM1" ||
      (std::string_view(argv[2]) != "--status-only" &&
       std::string_view(argv[2]) != "--disable-only" &&
       std::string_view(argv[2]) != "--enable" &&
       std::string_view(argv[2]) != "--enable-only" &&
       std::string_view(argv[2]) != "--jog" &&
       std::string_view(argv[2]) != "--slow-5s" &&
       std::string_view(argv[2]) != "--read-mode" &&
       std::string_view(argv[2]) != "--read-gear-ratio" &&
       std::string_view(argv[2]) != "--read-timeout" &&
       std::string_view(argv[2]) != "--set-timeout-200ms" &&
       std::string_view(argv[2]) != "--verify-timeout-200ms" &&
       std::string_view(argv[2]) != "--torque-step" &&
       std::string_view(argv[2]) != "--monitor-60s" &&
       std::string_view(argv[2]) != "--gui-2s" &&
       std::string_view(argv[2]) != "--gui-kd2-2s" &&
       std::string_view(argv[2]) != "--gui-ff04-2s" &&
       std::string_view(argv[2]) != "--gui-5s" &&
       std::string_view(argv[2]) != "--gui-1ms-2s" &&
       std::string_view(argv[2]) != "--high-torque-probe" &&
       std::string_view(argv[2]) != "--interactive-mit")) {
    std::fprintf(stderr, "Usage: %s /dev/ttyACM1 --status-only|--disable-only|--read-mode|--read-gear-ratio|--read-timeout|--enable|--enable-only|--jog|--slow-5s|--set-timeout-200ms|--verify-timeout-200ms|--torque-step|--monitor-60s|--gui-2s|--gui-kd2-2s|--gui-ff04-2s|--gui-5s|--gui-1ms-2s|--high-torque-probe|--interactive-mit\n",
                 argv[0]);
    return 2;
  }
  const bool interactive_requested = std::string_view(argv[2]) == "--interactive-mit";
  if (interactive_requested && !isatty(STDIN_FILENO)) {
    std::fprintf(stderr, "Interactive MIT requires a foreground terminal.\n");
    return 2;
  }
  const bool slow_requested = std::string_view(argv[2]) == "--slow-5s";
  const bool disable_requested = std::string_view(argv[2]) == "--disable-only";
  const bool mode_requested = std::string_view(argv[2]) == "--read-mode";
  const bool ratio_requested = std::string_view(argv[2]) == "--read-gear-ratio";
  const bool timeout_requested = std::string_view(argv[2]) == "--read-timeout";
  const bool set_timeout_requested =
      std::string_view(argv[2]) == "--set-timeout-200ms";
  const bool verify_timeout_requested =
      std::string_view(argv[2]) == "--verify-timeout-200ms";
  const bool step_requested = std::string_view(argv[2]) == "--torque-step";
  const bool monitor_requested = std::string_view(argv[2]) == "--monitor-60s";
  const bool gui_five_seconds = std::string_view(argv[2]) == "--gui-5s";
  const bool gui_kd2 = std::string_view(argv[2]) == "--gui-kd2-2s";
  const bool gui_ff04 = std::string_view(argv[2]) == "--gui-ff04-2s";
  const bool gui_one_ms = std::string_view(argv[2]) == "--gui-1ms-2s";
  const bool high_torque_probe = std::string_view(argv[2]) == "--high-torque-probe";
  const bool gui_requested = std::string_view(argv[2]) == "--gui-2s" || gui_kd2 || gui_ff04 || gui_five_seconds ||
                             gui_one_ms || high_torque_probe || interactive_requested;
  const bool enable_requested = std::string_view(argv[2]) != "--status-only" &&
                                !disable_requested &&
                                !slow_requested && !mode_requested && !step_requested &&
                                !monitor_requested && !gui_requested && !timeout_requested &&
                                !ratio_requested &&
                                !set_timeout_requested && !verify_timeout_requested;
  const bool keep_enabled = std::string_view(argv[2]) == "--enable-only";
  const bool jog_requested = std::string_view(argv[2]) == "--jog";
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGHUP, on_signal);

  PosixCdcSerialPort serial{argv[1]};
  UsbCdcTransport transport{serial, UsbCdcOptions{
      kBus, 1000000U, 64U, 1024U, CdcProtocolVersion{4U, 8U, 8U}}};
  if (!transport.open()) {
    std::fprintf(stderr, "Cannot open exclusive gateway port.\n");
    return 1;
  }
  if (!serial.send_pass_through_init()) {
    std::fprintf(stderr, "Gateway receive-pass init failed.\n");
    transport.close();
    return 1;
  }
  const auto query = command(0x7FFU, {0x02U, 0x00U, 0xCCU, 0U,
                                      0U, 0U, 0U, 0U});
  std::printf("Requesting DM4340 status without enabling.\n");
  if (!send_bounded(transport, query)) {
    std::fprintf(stderr, "Status query send failed.\n");
    transport.close();
    return 1;
  }
  MitFeedback feedback{};
  if (!wait_for_feedback(transport, std::chrono::milliseconds(1200), feedback) ||
      !feedback_status_ok(feedback.raw_fault) ||
      (enable_requested && feedback.raw_fault != 0U) || stop_requested) {
    const auto stats = transport.stats();
    std::fprintf(stderr, "Gateway stats: tx=%llu rx=%llu dropped=%llu errors=%llu\n",
                 static_cast<unsigned long long>(stats.tx_frames),
                 static_cast<unsigned long long>(stats.rx_frames),
                 static_cast<unsigned long long>(stats.rx_dropped),
                 static_cast<unsigned long long>(stats.errors));
    std::fprintf(stderr, "No healthy, matching feedback; motor was NOT enabled.\n");
    transport.close();
    return 1;
  }
  if (!enable_requested) {
    if (disable_requested) {
      const bool sent = disable(transport);
      if (!sent || !send_bounded(transport, query)) {
        transport.close();
        return 1;
      }
      MitFeedback after{};
      const bool disabled = wait_for_feedback(
          transport, std::chrono::milliseconds(500), after) &&
          after.raw_fault == 0U;
      transport.close();
      return disabled ? 0 : 1;
    }
    if (monitor_requested) {
      const bool ok = monitor_feedback(transport);
      transport.close();
      return ok ? 0 : 1;
    }
    if (mode_requested) {
      std::uint32_t mode = 0U;
      const bool ok = read_mode(transport, mode);
      transport.close();
      return ok ? 0 : 1;
    }
    if (ratio_requested) {
      std::uint32_t raw = 0U;
      const bool ok = read_uint_parameter(transport, 20U, raw);
      if (ok) {
        float ratio = 0.0F;
        static_assert(sizeof(ratio) == sizeof(raw));
        std::memcpy(&ratio, &raw, sizeof(ratio));
        std::printf("Gear ratio register 20: raw=0x%08X float=%.3f\n", raw, ratio);
      }
      transport.close();
      return ok ? 0 : 1;
    }
    if (timeout_requested) {
      std::uint32_t timeout = 0U;
      const bool ok = read_uint_parameter(transport, 9U, timeout);
      if (ok) std::printf("Motor communication-timeout register 9: %u (raw value; units not verified)\n", timeout);
      transport.close();
      return ok ? 0 : 1;
    }
    if (set_timeout_requested) {
      if (feedback.raw_fault != 0U) {
        std::fprintf(stderr, "TIMEOUT write requires disabled status 0.\n");
        transport.close();
        return 1;
      }
      const bool ok = set_timeout_200ms(transport);
      transport.close();
      return ok ? 0 : 1;
    }
    if (verify_timeout_requested) {
      if (feedback.raw_fault != 0U) {
        std::fprintf(stderr, "Timeout verification requires disabled status 0.\n");
        transport.close();
        return 1;
      }
      const bool ok = verify_timeout_200ms(transport);
      transport.close();
      return ok ? 0 : 1;
    }
    if (gui_requested) {
      std::uint32_t mode = 0U;
      if (!read_mode(transport, mode) || mode != 1U) {
        std::fprintf(stderr, "MIT mode not confirmed; no motion command sent.\n");
        transport.close();
        return 1;
      }
      if (feedback.raw_fault == 0U) {
        const auto enable_frame = command(kMotorCanId,
            {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU});
        MitFeedback enabled_feedback{};
        if (!send_bounded(transport, enable_frame) ||
            !wait_for_feedback(transport, std::chrono::milliseconds(500), enabled_feedback) ||
            !feedback_motor_enabled(enabled_feedback.raw_fault)) {
          std::fprintf(stderr, "Enable confirmation failed; attempting disable.\n");
          disable(transport);
          transport.close();
          return 1;
        }
        feedback = enabled_feedback;
      }
      for (int i = 0; i < 64; ++i) {
        RawCanFrame stale{};
        if (transport.try_receive(stale) != TransportResult::Ok) break;
      }
      const bool completed = interactive_requested
          ? interactive_mit(transport)
          : (gui_one_ms || high_torque_probe
              ? gui_profile_1ms(transport, feedback.position_rad, high_torque_probe)
              : gui_profile(transport, feedback.position_rad,
                            gui_five_seconds ? std::chrono::seconds(5) : std::chrono::seconds(2),
                            gui_five_seconds ? 2.5 : (gui_ff04 ? 2.0 : (gui_kd2 ? 1.8 : 1.0)),
                            gui_kd2 || gui_ff04 ? 2.0 : 1.0,
                            gui_ff04 ? 0.4 : 0.0));
      if ((gui_kd2 || gui_ff04) && completed) {
        const bool disabled = disable(transport);
        MitFeedback after{};
        const bool confirmed = disabled && send_bounded(transport, query) &&
            wait_for_feedback(transport, std::chrono::milliseconds(500), after) &&
            after.raw_fault == 0U;
        transport.close();
        return confirmed ? 0 : 1;
      }
      if (!completed) {
        std::fprintf(stderr, "GUI-profile test failed; attempting emergency disable.\n");
        disable(transport);
      }
      transport.close();
      return completed ? 0 : 1;
    }
    if (step_requested) {
      if (!feedback_motor_enabled(feedback.raw_fault)) {
        std::fprintf(stderr, "Torque-step requires an already enabled motor.\n");
        transport.close();
        return 1;
      }
      std::uint32_t mode = 0U;
      if (!read_mode(transport, mode) || mode != 1U) {
        std::fprintf(stderr, "MIT mode not confirmed; no motion command sent.\n");
        transport.close();
        return 1;
      }
      for (int i = 0; i < 64; ++i) {
        RawCanFrame stale{};
        if (transport.try_receive(stale) != TransportResult::Ok) break;
      }
      const bool completed = torque_step(transport, feedback.position_rad);
      if (!completed) {
        std::fprintf(stderr, "Torque-step failed; attempting emergency disable.\n");
        disable(transport);
      }
      transport.close();
      return completed ? 0 : 1;
    }
    if (slow_requested) {
      if (!feedback_motor_enabled(feedback.raw_fault)) {
        std::fprintf(stderr, "Low-speed test requires an already enabled motor.\n");
        transport.close();
        return 1;
      }
      for (int i = 0; i < 64; ++i) {
        RawCanFrame stale{};
        if (transport.try_receive(stale) != TransportResult::Ok) break;
      }
      const bool completed = slow_spin(transport, feedback.position_rad);
      if (!completed) {
        std::fprintf(stderr, "Low-speed test failed; attempting disable.\n");
        disable(transport);
      }
      transport.close();
      return completed ? 0 : 1;
    }
    transport.close();
    return 0;
  }

  // Discard any response queued by the preflight query before enabling.
  for (int i = 0; i < 64; ++i) {
    RawCanFrame stale{};
    if (transport.try_receive(stale) != TransportResult::Ok) break;
  }

  const auto enable_frame = command(kMotorCanId,
                                    {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                                     0xFFU, 0xFFU, 0xFFU, 0xFCU});
  std::printf("Sending one enable frame, no motion target.\n");
  const bool enable_sent = send_bounded(transport, enable_frame);
  if (!enable_sent) {
    std::fprintf(stderr, "Enable send uncertain; attempting disable.\n");
  }
  MitFeedback after_enable{};
  const bool got_feedback = enable_sent &&
      wait_for_feedback(transport, std::chrono::milliseconds(500), after_enable);
  const bool enabled = got_feedback &&
      feedback_motor_enabled(after_enable.raw_fault);
  if (keep_enabled && enabled && !stop_requested) {
    std::printf("Motor enabled (status=1); no disable or motion frame sent.\n");
    transport.close();
    return 0;
  }
  const bool jog_ok = !jog_requested ||
      (enabled && jog(transport, after_enable.position_rad));
  const bool disable_sent = disable(transport);
  const auto stats = transport.stats();
  transport.close();
  std::printf("Gateway TX=%llu RX=%llu; disable delivery to motor is NOT confirmed.\n",
              static_cast<unsigned long long>(stats.tx_frames),
              static_cast<unsigned long long>(stats.rx_frames));
  return enable_sent && enabled && jog_ok && disable_sent ? 0 : 1;
}
