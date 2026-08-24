#pragma once

#include <cstddef>
#include <cstdint>

#include "mech_control_core/config.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/status.hpp"
#include "mech_control_core/transport.hpp"

namespace mech::mech_control_core {

enum class AdapterResult : std::uint8_t {
  Ok,
  WouldBlock,
  InvalidConfiguration,
  InvalidCommand,
  Stale,
  Disconnected,
  Fault,
};

struct CanonicalDeviceCommand final {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint64_t generation{0U};
  MonotonicTime deadline{};
};

struct CanonicalDeviceState final {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  StatusSnapshot status{};
};

// Implement in a device package. Codec has no I/O or session state.
class DeviceCodec {
 public:
  virtual ~DeviceCodec() = default;
  [[nodiscard]] virtual ProtocolProfile profile() const noexcept = 0;
  [[nodiscard]] virtual AdapterResult encode(
      const CanonicalDeviceCommand& command, std::uint16_t logical_bus,
      MonotonicTime now, RawCanFrame& output) const noexcept = 0;
  [[nodiscard]] virtual AdapterResult decode(
      const RawCanFrame& frame, CanonicalDeviceState& output) const noexcept = 0;
};

// Session owns device semantics but borrows transport. It may never create or
// discover a physical channel.
class DeviceSession {
 public:
  explicit DeviceSession(Transport& transport) noexcept : transport_(transport) {}
  virtual ~DeviceSession() = default;
  DeviceSession(const DeviceSession&) = delete;
  DeviceSession& operator=(const DeviceSession&) = delete;
  [[nodiscard]] virtual AdapterResult configure(
      const DeviceConfig& config,
      const TransportCapabilities& capabilities) noexcept = 0;
  [[nodiscard]] virtual AdapterResult activate() noexcept = 0;
  virtual void deactivate() noexcept = 0;
  [[nodiscard]] virtual AdapterResult submit(
      const CanonicalDeviceCommand& command, MonotonicTime now) noexcept = 0;
  [[nodiscard]] virtual AdapterResult process(
      const RawCanFrame& frame, MonotonicTime now) noexcept = 0;
  [[nodiscard]] virtual CanonicalDeviceState snapshot(
      MonotonicTime now) const noexcept = 0;

 protected:
  [[nodiscard]] Transport& transport() noexcept { return transport_; }
  [[nodiscard]] const Transport& transport() const noexcept { return transport_; }

 private:
  Transport& transport_;
};

}  // namespace mech::mech_control_core
