#include "mech_control_core/adapter_template.hpp"

#include <type_traits>

#include <gtest/gtest.h>

namespace mech::mech_control_core {
namespace {

class ContractTransport final : public Transport {
 public:
  [[nodiscard]] TransportKind kind() const noexcept override {
    return TransportKind::Fake;
  }
  [[nodiscard]] const TransportCapabilities& capabilities()
      const noexcept override {
    return capabilities_;
  }
  [[nodiscard]] bool is_open() const noexcept override { return true; }
  bool open() noexcept override { return true; }
  void close() noexcept override {}
  [[nodiscard]] TransportResult try_receive(RawCanFrame&) noexcept override {
    return TransportResult::WouldBlock;
  }
  [[nodiscard]] TransportResult try_send(const RawCanFrame&) noexcept override {
    return TransportResult::Ok;
  }
  [[nodiscard]] TransportStats stats() const noexcept override { return {}; }

 private:
  TransportCapabilities capabilities_{};
};

class ContractSession final : public DeviceSession {
 public:
  explicit ContractSession(Transport& transport) noexcept
      : DeviceSession(transport) {}

  [[nodiscard]] AdapterResult configure(
      const DeviceConfig&, const TransportCapabilities&) noexcept override {
    return AdapterResult::Ok;
  }
  [[nodiscard]] AdapterResult activate() noexcept override {
    return AdapterResult::Ok;
  }
  void deactivate() noexcept override {}
  [[nodiscard]] AdapterResult submit(
      const CanonicalDeviceCommand&, MonotonicTime) noexcept override {
    return AdapterResult::Ok;
  }
  [[nodiscard]] AdapterResult process(
      const RawCanFrame&, MonotonicTime) noexcept override {
    return AdapterResult::Ok;
  }
  [[nodiscard]] CanonicalDeviceState snapshot(
      MonotonicTime) const noexcept override {
    return {};
  }

  [[nodiscard]] bool owns(const Transport& transport) const noexcept {
    return &this->transport() == &transport;
  }
};

TEST(AdapterContractV1, KeepsCodecAndSessionAbstractAndTransportInjected) {
  EXPECT_TRUE(std::is_abstract_v<DeviceCodec>);
  EXPECT_TRUE(std::is_abstract_v<DeviceSession>);
  EXPECT_TRUE(std::has_virtual_destructor_v<DeviceCodec>);
  EXPECT_TRUE(std::has_virtual_destructor_v<DeviceSession>);
  EXPECT_FALSE(std::is_default_constructible_v<ContractSession>);
  EXPECT_TRUE((std::is_constructible_v<ContractSession, Transport&>));
  ContractTransport transport;
  const ContractSession session{transport};
  EXPECT_TRUE(session.owns(transport));
}

}  // namespace
}  // namespace mech::mech_control_core
