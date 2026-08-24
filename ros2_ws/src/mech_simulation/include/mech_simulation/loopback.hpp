#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "mech_control_core/frame.hpp"

namespace mech::mech_simulation {

struct LoopbackCommand final {
  std::uint16_t device_id{0U};
  std::int32_t target_milli{0};
  std::uint8_t sequence{0U};
};

struct LoopbackFeedback final {
  std::uint16_t device_id{0U};
  std::int32_t position_milli{0};
  std::uint8_t fault_code{0U};
  std::uint8_t sequence{0U};
};

class LoopbackCodec final {
 public:
  [[nodiscard]] static std::optional<mech_control_core::RawCanFrame>
  encode_command(std::uint16_t logical_bus, std::uint16_t device_id,
                 std::int32_t target_milli, std::uint8_t sequence,
                 mech_control_core::MonotonicTime now) noexcept;

  [[nodiscard]] static std::optional<LoopbackCommand> decode_command(
      const mech_control_core::RawCanFrame& frame) noexcept;

  [[nodiscard]] static std::optional<mech_control_core::RawCanFrame>
  encode_feedback(std::uint16_t logical_bus, std::uint16_t device_id,
                  std::int32_t position_milli, std::uint8_t fault_code,
                  std::uint8_t sequence,
                  mech_control_core::MonotonicTime now) noexcept;

  [[nodiscard]] static std::optional<LoopbackFeedback> decode_feedback(
      const mech_control_core::RawCanFrame& frame) noexcept;
};

class SimulatedDevice final {
 public:
  SimulatedDevice(std::uint16_t logical_bus, std::uint16_t device_id,
                  std::int32_t max_step_milli = 100);

  [[nodiscard]] std::uint16_t device_id() const noexcept { return device_id_; }
  [[nodiscard]] std::int32_t position_milli() const noexcept { return position_milli_; }
  [[nodiscard]] std::int32_t target_milli() const noexcept { return target_milli_; }
  [[nodiscard]] std::uint8_t fault_code() const noexcept { return fault_code_; }

  [[nodiscard]] bool accept_command(
      const mech_control_core::RawCanFrame& frame) noexcept;
  [[nodiscard]] std::optional<mech_control_core::RawCanFrame> step(
      mech_control_core::MonotonicTime now) noexcept;
  void inject_fault(std::uint8_t fault_code) noexcept { fault_code_ = fault_code; }
  void clear_fault() noexcept { fault_code_ = 0U; }

 private:
  std::uint16_t logical_bus_;
  std::uint16_t device_id_;
  std::int32_t max_step_milli_;
  std::int32_t position_milli_{0};
  std::int32_t target_milli_{0};
  std::uint8_t fault_code_{0U};
  std::uint8_t sequence_{0U};
};

}  // namespace mech::mech_simulation
