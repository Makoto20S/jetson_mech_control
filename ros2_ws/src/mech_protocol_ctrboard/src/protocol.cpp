#include "mech_protocol_ctrboard/protocol.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace mech::mech_protocol_ctrboard {
namespace {

constexpr std::size_t kHeaderSize = 4U;
constexpr std::size_t kCrcSize = 2U;
constexpr std::size_t kMaximumBufferedBytes = 1024U;
constexpr std::array<std::uint8_t, 2U> kHeader{kHeader1, kHeader2};

}  // namespace

std::uint16_t crc16_ccitt(const std::uint8_t* data,
                          std::size_t size) noexcept {
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0U; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
    for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<Packet> StreamDecoder::feed(
    const mech_control_core::RawCanFrame& frame) {
  std::vector<Packet> packets;
  if (!frame.is_valid() ||
      frame.direction != mech_control_core::FrameDirection::Rx ||
      frame.id.format != mech_control_core::CanFrameFormat::Standard ||
      frame.id.value != kMcuToHostCanId ||
      frame.type != mech_control_core::CanFrameType::Classic ||
      frame.error_frame || frame.remote_request) {
    return packets;
  }

  buffer_.insert(buffer_.end(), frame.payload.begin(),
                 frame.payload.begin() + frame.payload_size);
  if (buffer_.size() > kMaximumBufferedBytes) {
    discarded_bytes_ += buffer_.size();
    buffer_.clear();
    return packets;
  }

  while (buffer_.size() >= kHeaderSize) {
    const auto header = std::search(buffer_.begin(), buffer_.end(),
                                    kHeader.begin(), kHeader.end());
    if (header != buffer_.begin()) {
      const auto keep_header_prefix =
          header == buffer_.end() && buffer_.back() == kHeader1;
      const auto discarded = header == buffer_.end()
                                 ? buffer_.size() -
                                       (keep_header_prefix ? 1U : 0U)
                                 : static_cast<std::size_t>(header -
                                                            buffer_.begin());
      discarded_bytes_ += discarded;
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(discarded));
      if (buffer_.size() < kHeaderSize) {
        break;
      }
    }

    const auto payload_size = static_cast<std::size_t>(buffer_[3]);
    const auto packet_size = kHeaderSize + payload_size + kCrcSize;
    if (buffer_.size() < packet_size) {
      break;
    }

    const auto calculated =
        crc16_ccitt(buffer_.data(), kHeaderSize + payload_size);
    const auto received =
        static_cast<std::uint16_t>(buffer_[packet_size - 2U]) |
        (static_cast<std::uint16_t>(buffer_[packet_size - 1U]) << 8U);
    if (calculated != received) {
      ++crc_errors_;
      ++discarded_bytes_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    Packet packet{static_cast<MessageId>(buffer_[2]), {}};
    packet.payload.assign(
        buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
        buffer_.begin() +
            static_cast<std::ptrdiff_t>(kHeaderSize + payload_size));
    packets.push_back(std::move(packet));
    buffer_.erase(buffer_.begin(),
                  buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
  }
  return packets;
}

std::uint64_t StreamDecoder::crc_errors() const noexcept {
  return crc_errors_;
}

std::uint64_t StreamDecoder::discarded_bytes() const noexcept {
  return discarded_bytes_;
}

void StreamDecoder::reset() noexcept {
  buffer_.clear();
  crc_errors_ = 0U;
  discarded_bytes_ = 0U;
}

bool decode_sensor_state(const Packet& packet,
                         SensorTelemetryWire& telemetry) noexcept {
  if (packet.message_id != MessageId::SensorState ||
      packet.payload.size() != sizeof(telemetry)) {
    return false;
  }
  std::memcpy(&telemetry, packet.payload.data(), sizeof(telemetry));
  return true;
}

}  // namespace mech::mech_protocol_ctrboard
