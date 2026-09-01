#include "mech_control_core/config.hpp"
#include "mech_control_core/router.hpp"

#include <array>

#include <gtest/gtest.h>

namespace mech::mech_control_core {
namespace {

TransportCapabilities valid_capabilities() {
  TransportCapabilities capabilities;
  capabilities.supports_classic_can = true;
  capabilities.supports_can_fd = true;
  capabilities.supports_brs = true;
  capabilities.supports_standard_frames = true;
  capabilities.supports_extended_frames = true;
  capabilities.supports_filters = true;
  capabilities.supports_error_frames = true;
  capabilities.supports_timestamps = false;
  capabilities.supports_non_blocking_io = true;
  capabilities.supports_remote_frames = true;
  capabilities.nominal_bitrate_configurable = true;
  capabilities.nominal_bitrate_hz = 1000000U;
  capabilities.nominal_bitrate_verified = true;
  capabilities.max_payload_bytes = 64U;
  capabilities.queue_capacity = 16U;
  capabilities.queue_capacity_verified = true;
  return capabilities;
}

DeviceConfig simulation_device(std::uint16_t device_id = 1U,
                               std::uint16_t logical_bus = 1U) {
  return DeviceConfig{device_id,
                      "sim_motor_" + std::to_string(device_id),
                      logical_bus,
                      ProtocolProfile::LoopbackV1,
                      CanFrameType::Classic,
                      CanFrameFormat::Standard,
                      CanId::create(0x100U + device_id, CanFrameFormat::Standard),
                      CanId::create(0x180U + device_id, CanFrameFormat::Standard),
                      8U,
                      8U,
                      true};
}

DeploymentConfig valid_configuration() {
  return DeploymentConfig{
      kSchemaV1,
      {BusConfig{1U, "fake0", TransportKind::Fake, valid_capabilities()}},
      {simulation_device()}};
}

TEST(ConfigValidation, AcceptsValidSimulationConfiguration) {
  const auto result = validate_deployment(valid_configuration());
  EXPECT_TRUE(result.valid());
}

TEST(ConfigValidation, AcceptsReadOnlyDeviceWithoutCommandRoute) {
  auto config = valid_configuration();
  auto sensor = simulation_device(2U);
  sensor.name = "sim_sensor";
  sensor.command_id.reset();
  sensor.command_payload_bytes = 0U;
  sensor.writable = false;
  sensor.feedback_id = CanId::create(0x280U, CanFrameFormat::Standard);
  sensor.feedback_payload_bytes = 8U;
  config.devices.push_back(sensor);
  EXPECT_TRUE(validate_deployment(config).valid());
}

TEST(ConfigValidation, RejectsSchemaMismatch) {
  auto config = valid_configuration();
  config.schema_version.major = 2U;
  const auto result = validate_deployment(config);
  EXPECT_FALSE(result.valid());
  EXPECT_TRUE(result.has(ConfigErrorCode::SchemaMismatch));
}

TEST(ConfigValidation, RejectsMissingFieldsAndUnknownProfile) {
  auto config = valid_configuration();
  config.buses.front().physical_channel.clear();
  config.devices.front().profile = ProtocolProfile::Unknown;
  config.devices.front().command_id.reset();
  const auto result = validate_deployment(config);
  EXPECT_TRUE(result.has(ConfigErrorCode::MissingField));
  EXPECT_TRUE(result.has(ConfigErrorCode::UnknownProfile));
}

TEST(ConfigValidation, RejectsDuplicateIdsAndChannels) {
  auto config = valid_configuration();
  config.buses.push_back(
      BusConfig{1U, "fake0", TransportKind::Fake, valid_capabilities()});
  config.devices.push_back(simulation_device(1U));
  const auto result = validate_deployment(config);
  EXPECT_TRUE(result.has(ConfigErrorCode::DuplicateBus));
  EXPECT_TRUE(result.has(ConfigErrorCode::DuplicatePhysicalChannel));
  EXPECT_TRUE(result.has(ConfigErrorCode::DuplicateDevice));
  EXPECT_TRUE(result.has(ConfigErrorCode::DuplicateDeviceName));
  EXPECT_TRUE(result.has(ConfigErrorCode::DuplicateRouteId));
}

TEST(ConfigValidation, RejectsProfileAndCapabilityMismatch) {
  auto config = valid_configuration();
  config.devices.front().profile = ProtocolProfile::Ak30ServoExtended;
  config.devices.front().frame_format = CanFrameFormat::Standard;
  config.buses.front().capabilities.supports_standard_frames = false;
  const auto result = validate_deployment(config);
  EXPECT_TRUE(result.has(ConfigErrorCode::IncompatibleFrameFormat));
  EXPECT_TRUE(result.has(ConfigErrorCode::IncompatibleCapability));
}

// ADR-013: force control is a 29-bit extended frame, not the 11-bit standard
// frame the superseded AK2.0 MIT profile used. Declaring it as Standard must
// be rejected -- under the old enum this configuration would have been
// accepted, which is the concrete defect the baseline switch removes.
TEST(ConfigValidation, RejectsForceControlDeclaredAsStandardFrame) {
  auto config = valid_configuration();
  config.devices.front().profile = ProtocolProfile::Ak30ForceControlExtended;
  config.devices.front().frame_format = CanFrameFormat::Standard;
  const auto result = validate_deployment(config);
  EXPECT_TRUE(result.has(ConfigErrorCode::IncompatibleFrameFormat));
}

TEST(ConfigValidation, AcceptsForceControlAsExtendedFrame) {
  auto config = valid_configuration();
  config.devices.front().profile = ProtocolProfile::Ak30ForceControlExtended;
  config.devices.front().frame_format = CanFrameFormat::Extended;
  const auto result = validate_deployment(config);
  EXPECT_FALSE(result.has(ConfigErrorCode::IncompatibleFrameFormat));
}

TEST(ConfigValidation, RejectsInvalidCapabilitiesAndPayload) {
  auto config = valid_configuration();
  config.buses.front().capabilities.max_payload_bytes = 4U;
  config.devices.front().command_payload_bytes = 9U;
  const auto result = validate_deployment(config);
  EXPECT_TRUE(result.has(ConfigErrorCode::InvalidCapability));
  EXPECT_TRUE(result.has(ConfigErrorCode::InvalidPayloadSize));
}

TEST(FrameFilter, SeparatesStandardAndExtendedFrames) {
  const FrameFilter standard{CanFrameFormat::Standard, 0x100U, 0x7FFU,
                             std::nullopt};
  const FrameFilter extended{CanFrameFormat::Extended, 0x100U, 0x1FFFFFFFU,
                             std::nullopt};
  EXPECT_FALSE(standard.overlaps(extended));
}

TEST(FrameRouter, RejectsOverlappingFiltersUnlessExplicitFanout) {
  FrameRouter router;
  EXPECT_FALSE(router.add_route(FrameRoute{
      1U, FrameFilter{CanFrameFormat::Standard, 0x100U, 0x7FFU,
                      CanFrameType::Classic},
      9U}));
  EXPECT_EQ(router.add_route(FrameRoute{
                2U, FrameFilter{CanFrameFormat::Standard, 0x100U, 0x7FFU,
                                CanFrameType::Classic},
                0U}),
            RouterError::OverlappingFilter);
  EXPECT_FALSE(router.add_route(FrameRoute{
      3U, FrameFilter{CanFrameFormat::Standard, 0x100U, 0x7FFU,
                      CanFrameType::Classic},
      9U}));
  EXPECT_EQ(router.size(), 2U);
}

}  // namespace
}  // namespace mech::mech_control_core
