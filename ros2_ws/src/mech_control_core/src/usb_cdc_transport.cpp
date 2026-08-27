#include "mech_control_core/usb_cdc_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace mech::mech_control_core {
namespace {

std::uint8_t crc8(const std::uint8_t* data, std::size_t size) noexcept {
  std::uint8_t crc = 0xFFU;
  while (size-- > 0U) {
    crc ^= *data++;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<std::uint8_t>((crc >> 1U) ^ 0x8CU)
                             : static_cast<std::uint8_t>(crc >> 1U);
    }
  }
  return crc;
}

std::uint16_t crc16(const std::uint8_t* data, std::size_t size) noexcept {
  std::uint16_t crc = 0xFFFFU;
  while (size-- > 0U) {
    crc ^= *data++;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                             : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

std::size_t fdcan_length(std::size_t size, CanFrameType type) noexcept {
  if (type == CanFrameType::Classic) {
    return size;
  }
  if (size <= 8U) return size;
  if (size <= 12U) return 12U;
  if (size <= 16U) return 16U;
  if (size <= 20U) return 20U;
  if (size <= 24U) return 24U;
  if (size <= 32U) return 32U;
  if (size <= 48U) return 48U;
  return 64U;
}

void put_u16(std::uint8_t* data, std::uint16_t value) noexcept {
  data[0] = static_cast<std::uint8_t>(value & 0xFFU);
  data[1] = static_cast<std::uint8_t>(value >> 8U);
}

void put_u32(std::uint8_t* data, std::uint32_t value) noexcept {
  for (int index = 0; index < 4; ++index) {
    data[index] = static_cast<std::uint8_t>(value >> (8U * index));
  }
}

std::uint16_t get_u16(const std::uint8_t* data) noexcept {
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(data[1] << 8U);
}

std::uint32_t get_u32(const std::uint8_t* data) noexcept {
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

MonotonicTime current_time() noexcept {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return *MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

}  // namespace

bool UsbCdcCodec::encode(const RawCanFrame& frame,
                         std::array<std::uint8_t, 528U>& output,
                         std::size_t& size) noexcept {
  if (frame.direction != FrameDirection::Tx || frame.error_frame ||
      !frame.is_valid()) {
    return false;
  }
  const auto data_length = fdcan_length(frame.payload_size, frame.type);
  const auto payload_length = 6U + data_length;
  if (payload_length > kMaxPayload || output.size() < payload_length + 7U) {
    return false;
  }
  output[0] = kHeader;
  output[1] = kPassCommand;
  put_u16(output.data() + 2U, static_cast<std::uint16_t>(payload_length));
  output[4] = crc8(output.data() + 1U, 3U);
  put_u32(output.data() + 7U, frame.id.value);
  output[11] = static_cast<std::uint8_t>(
      (frame.bitrate_switch ? 0x01U : 0U) |
      (frame.type == CanFrameType::FlexibleDataRate ? 0x02U : 0U) |
      (frame.id.format == CanFrameFormat::Extended ? 0x04U : 0U) | 0x08U);
  output[12] = static_cast<std::uint8_t>(data_length);
  std::copy_n(frame.payload.begin(), frame.payload_size, output.begin() + 13U);
  std::fill(output.begin() + 13U + frame.payload_size,
            output.begin() + 13U + data_length, 0U);
  put_u16(output.data() + 5U, crc16(output.data() + 7U, payload_length));
  size = payload_length + 7U;
  return true;
}

bool UsbCdcCodec::decode(const std::uint8_t* data, std::size_t size,
                         std::uint16_t logical_bus, MonotonicTime host_arrival,
                         CdcFrameBatch& output) noexcept {
  if (data == nullptr || size < 7U || data[0] != kHeader ||
      data[1] != kPassCommand || get_u16(data + 2U) != size - 7U ||
      data[4] != crc8(data + 1U, 3U) ||
      get_u16(data + 5U) != crc16(data + 7U, size - 7U)) {
    return false;
  }
  output.size = 0U;
  std::size_t offset = 7U;
  while (offset < size) {
    if (size - offset < 6U || output.size >= output.frames.size()) {
      return false;
    }
    const auto id_value = get_u32(data + offset);
    const auto flags = data[offset + 4U];
    const auto length = data[offset + 5U];
    const auto frame_type = (flags & 0x02U) != 0U
                                ? CanFrameType::FlexibleDataRate
                                : CanFrameType::Classic;
    const auto padded = fdcan_length(length, frame_type);
    if (length > (frame_type == CanFrameType::Classic ? 8U : 64U) ||
        padded != length || padded > size - offset - 6U ||
        ((flags & 0x01U) != 0U &&
         frame_type != CanFrameType::FlexibleDataRate) ||
        (flags & 0xF0U) != 0U) {
      return false;
    }
    const auto frame_id = CanId::create(
        id_value, (flags & 0x04U) != 0U ? CanFrameFormat::Extended
                                       : CanFrameFormat::Standard);
    if (!frame_id.has_value()) {
      return false;
    }
    std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
    std::copy_n(data + offset + 6U, padded, payload.begin());
    const auto frame = RawCanFrame::create(
        logical_bus, *frame_id, frame_type, FrameDirection::Rx,
        static_cast<std::uint8_t>(padded), payload, host_arrival, std::nullopt,
        (flags & 0x01U) != 0U);
    if (!frame.has_value()) {
      return false;
    }
    output.frames[output.size++] = *frame;
    offset += 6U + padded;
  }
  return offset == size && output.size > 0U;
}

bool UsbCdcCodec::supports_version(CdcProtocolVersion version) noexcept {
  if (version.major != kMinimumBoardVersion.major) {
    return version.major > kMinimumBoardVersion.major;
  }
  if (version.minor != kMinimumBoardVersion.minor) {
    return version.minor > kMinimumBoardVersion.minor;
  }
  return version.patch >= kMinimumBoardVersion.patch;
}

UsbCdcTransport::UsbCdcTransport(CdcSerialPort& serial, UsbCdcOptions options)
    : serial_(serial),
      options_(options),
      rx_(options_.receive_queue_capacity) {
  // Named assignment on purpose: see the note in SocketCanTransport.
  capabilities_.supports_classic_can = true;
  capabilities_.supports_can_fd = true;
  capabilities_.supports_brs = true;
  capabilities_.supports_standard_frames = true;
  capabilities_.supports_extended_frames = true;
  capabilities_.supports_filters = false;
  capabilities_.supports_error_frames = false;
  capabilities_.supports_timestamps = false;
  capabilities_.supports_non_blocking_io = true;
  // The documented pass-through command has no RTR bit.
  capabilities_.supports_remote_frames = false;
  capabilities_.nominal_bitrate_configurable = false;
  // The board's bus bitrate is firmware-fixed and the vendor does not document
  // it, and the protocol offers no way to read it back. It is therefore only
  // ever operator-declared, and zero legitimately means "unknown".
  capabilities_.nominal_bitrate_hz = options_.nominal_bitrate_hz;
  capabilities_.nominal_bitrate_verified = false;
  capabilities_.max_payload_bytes = 64U;
  capabilities_.queue_capacity = static_cast<std::uint16_t>(
      std::min<std::size_t>(options_.receive_queue_capacity, 65535U));
  // This is our own software queue, so its capacity is a fact we control.
  capabilities_.queue_capacity_verified = true;
}

TransportKind UsbCdcTransport::kind() const noexcept {
  return TransportKind::HighTorqueUsbCdc;
}

const TransportCapabilities& UsbCdcTransport::capabilities() const noexcept {
  return capabilities_;
}

bool UsbCdcTransport::is_open() const noexcept { return serial_.is_open(); }

bool UsbCdcTransport::open() noexcept {
  if (options_.logical_bus == 0U || !capabilities_.is_valid()) {
    return false;
  }
  if (!options_.verified_board_version.has_value() ||
      !UsbCdcCodec::supports_version(*options_.verified_board_version)) {
    return false;
  }
  return serial_.open();
}

void UsbCdcTransport::close() noexcept {
  serial_.close();
  pending_size_ = 0U;
  rx_.clear();
}

TransportResult UsbCdcTransport::fill_rx() noexcept {
  if (!is_open()) return TransportResult::Disconnected;
  std::size_t size = 0U;
  const auto read_capacity =
      std::min(input_.size(), options_.serial_read_capacity);
  const auto result = serial_.read_some(input_.data(), read_capacity, size);
  if (result == TransportResult::WouldBlock) return result;
  if (result != TransportResult::Ok) return result;
  if (size == 0U) {
    // A successful read of zero bytes is "no data right now", not a fault.
    return TransportResult::WouldBlock;
  }
  if (size > read_capacity || pending_size_ + size > pending_.size()) {
    // A genuine overflow: the byte stream can no longer be trusted to be
    // frame-aligned, so drop everything buffered so far rather than leaving
    // pending_size_ stuck at a value that will keep overflowing forever.
    pending_size_ = 0U;
    ++stats_.rx_dropped;
    return TransportResult::Invalid;
  }
  std::copy_n(input_.begin(), size, pending_.begin() + pending_size_);
  pending_size_ += size;
  while (pending_size_ >= 7U) {
    auto start = std::find(pending_.begin(), pending_.begin() + pending_size_,
                           UsbCdcCodec::kHeader);
    if (start == pending_.begin() + pending_size_) {
      pending_size_ = 0U;
      ++stats_.rx_dropped;
      break;
    }
    const auto offset = static_cast<std::size_t>(start - pending_.begin());
    if (offset > 0U) {
      std::move(pending_.begin() + offset, pending_.begin() + pending_size_, pending_.begin());
      pending_size_ -= offset;
    }
    if (pending_size_ < 7U) break;
    const auto payload_length = get_u16(pending_.data() + 2U);
    const auto total = payload_length + 7U;
    if (payload_length > UsbCdcCodec::kMaxPayload || total > pending_.size()) {
      std::move(pending_.begin() + 1U, pending_.begin() + pending_size_, pending_.begin());
      --pending_size_;
      ++stats_.rx_dropped;
      continue;
    }
    if (pending_size_ < total) break;
    CdcFrameBatch batch;
    if (!UsbCdcCodec::decode(pending_.data(), total, options_.logical_bus,
                             current_time(), batch)) {
      std::move(pending_.begin() + 1U, pending_.begin() + pending_size_, pending_.begin());
      --pending_size_;
      ++stats_.rx_dropped;
      continue;
    }
    for (std::size_t index = 0U; index < batch.size; ++index) {
      if (!rx_.push_back(batch.frames[index])) {
        ++stats_.rx_dropped;
        ++stats_.queue_full;
        continue;
      }
      ++stats_.rx_frames;
    }
    std::move(pending_.begin() + total, pending_.begin() + pending_size_, pending_.begin());
    pending_size_ -= total;
  }
  return TransportResult::Ok;
}

TransportResult UsbCdcTransport::try_receive(RawCanFrame& frame) noexcept {
  if (!is_open()) return TransportResult::Disconnected;
  if (rx_.empty()) {
    const auto result = fill_rx();
    if (result != TransportResult::Ok && result != TransportResult::WouldBlock) {
      return result;
    }
  }
  if (rx_.empty()) return TransportResult::WouldBlock;
  frame = rx_.front();
  rx_.pop_front();
  return TransportResult::Ok;
}

TransportResult UsbCdcTransport::try_send(const RawCanFrame& frame) noexcept {
  if (!is_open()) return TransportResult::Disconnected;
  if (frame.logical_bus != options_.logical_bus ||
      frame.direction != FrameDirection::Tx || frame.error_frame ||
      frame.remote_request || !frame.is_valid()) {
    ++stats_.errors;
    return TransportResult::Invalid;
  }
  std::array<std::uint8_t, 528U> encoded{};
  std::size_t size = 0U;
  if (!UsbCdcCodec::encode(frame, encoded, size)) {
    ++stats_.errors;
    return TransportResult::Invalid;
  }
  const auto result = serial_.write_all(encoded.data(), size);
  if (result == TransportResult::Ok) ++stats_.tx_frames;
  else if (result == TransportResult::QueueFull) ++stats_.queue_full;
  else if (result != TransportResult::WouldBlock) ++stats_.errors;
  return result;
}

TransportStats UsbCdcTransport::stats() const noexcept { return stats_; }

}  // namespace mech::mech_control_core
