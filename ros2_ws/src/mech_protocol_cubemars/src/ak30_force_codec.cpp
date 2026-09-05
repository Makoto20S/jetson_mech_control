#include "mech_protocol_cubemars/ak30_force_codec.hpp"

#include <array>

namespace mech::mech_protocol_cubemars {

using mech_control_core::AdapterResult;
using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::CanId;
using mech_control_core::DeviceState;
using mech_control_core::FrameDirection;
using mech_control_core::ProtocolProfile;
using mech_control_core::SampleQuality;
using mech_control_core::StatusSnapshot;

Ak30ForceControlCodec::Ak30ForceControlCodec(std::uint8_t drive_id,
                                             ForceControlSubMode sub_mode,
                                             Ak30Mapping mapping,
                                             ForceControlGains gains) noexcept
    : drive_id_(drive_id),
      sub_mode_(sub_mode),
      mapping_(mapping),
      gains_(gains) {}

ProtocolProfile Ak30ForceControlCodec::profile() const noexcept {
  return ProtocolProfile::Ak30ForceControlExtended;
}

AdapterResult Ak30ForceControlCodec::encode(
    const mech_control_core::CanonicalDeviceCommand& command,
    std::uint16_t logical_bus, mech_control_core::MonotonicTime now,
    mech_control_core::RawCanFrame& output) const noexcept {
  ForceControlCommand wire{};
  to_device_command(mapping_, sub_mode_, gains_, command, wire);

  ForceControlPayload packed{};
  if (!encode_force_control(wire, mapping_.ranges, packed)) {
    return AdapterResult::InvalidCommand;
  }

  const auto id = CanId::create(force_control_can_id(drive_id_),
                                CanFrameFormat::Extended);
  if (!id.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }

  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  for (std::size_t index = 0; index < packed.size(); ++index) {
    payload[index] = packed[index];
  }

  const auto frame = mech_control_core::RawCanFrame::create(
      logical_bus, id.value(), CanFrameType::Classic, FrameDirection::Tx,
      static_cast<std::uint8_t>(packed.size()), payload, now);
  if (!frame.has_value()) {
    return AdapterResult::InvalidCommand;
  }
  output = frame.value();
  return AdapterResult::Ok;
}

AdapterResult Ak30ForceControlCodec::decode(
    const mech_control_core::RawCanFrame& frame,
    mech_control_core::CanonicalDeviceState& output) const noexcept {
  // Reject rather than degrade: a frame that is not exactly this device's
  // Classic extended 8-byte feedback is not this device's feedback. Direction
  // is part of that shape, not a redundant belt-and-braces check: nothing
  // upstream filters direction on the inbound path (every direction check in
  // the repository guards outbound Tx submission instead), so a Tx-tagged
  // frame that otherwise matches this ID would sail through unless rejected
  // here. Do not remove this on the assumption the transport already did it.
  if (frame.id.format != CanFrameFormat::Extended ||
      frame.id.value != feedback_can_id(drive_id_) ||
      frame.type != CanFrameType::Classic ||
      frame.direction != FrameDirection::Rx || frame.error_frame ||
      frame.remote_request || frame.bitrate_switch ||
      frame.payload_size != kForceControlPayloadBytes) {
    return AdapterResult::InvalidCommand;
  }

  ForceControlPayload packed{};
  for (std::size_t index = 0; index < packed.size(); ++index) {
    packed[index] = frame.payload[index];
  }

  ForceControlFeedback feedback{};
  decode_feedback(packed, feedback);
  to_canonical_state(mapping_, sub_mode_, feedback, output);

  const StatusMeaning meaning = classify_status(feedback.raw_status);
  // The codec reports only what the frame itself proves. The session owns the
  // device state machine and overwrites device_state with its own view.
  const DeviceState device_state = meaning == StatusMeaning::Fault
                                       ? DeviceState::Fault
                                       : DeviceState::Unknown;
  const SampleQuality quality = meaning == StatusMeaning::Unknown
                                    ? SampleQuality::Degraded
                                    : SampleQuality::Valid;

  const auto status = StatusSnapshot::create(
      quality, device_state, static_cast<std::uint32_t>(feedback.raw_status),
      0U, frame.host_arrival, frame.source_timestamp);
  if (!status.has_value()) {
    return AdapterResult::Fault;
  }
  output.status = status.value();
  return AdapterResult::Ok;
}

}  // namespace mech::mech_protocol_cubemars
