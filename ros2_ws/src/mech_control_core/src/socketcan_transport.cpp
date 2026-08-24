#include "mech_control_core/socketcan_transport.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace mech::mech_control_core {
namespace {

MonotonicTime host_now() noexcept {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  return *MonotonicTime::from_nanoseconds(ticks.count());
}

#ifdef __linux__
std::uint32_t to_can_id(const CanId& id) noexcept {
  return id.value | (id.format == CanFrameFormat::Extended ? CAN_EFF_FLAG : 0U);
}

CanId from_can_id(canid_t raw, bool& error) noexcept {
  error = (raw & CAN_ERR_FLAG) != 0U;
  if (error) {
    return *CanId::create(0U, CanFrameFormat::Standard);
  }
  const auto extended = (raw & CAN_EFF_FLAG) != 0U;
  return *CanId::create(raw & (extended ? CAN_EFF_MASK : CAN_SFF_MASK),
                        extended ? CanFrameFormat::Extended
                                  : CanFrameFormat::Standard);
}
#endif

}  // namespace

SocketCanTransport::SocketCanTransport(SocketCanOptions options)
    : options_(std::move(options)), capabilities_(capabilities_for(options_)) {}

SocketCanTransport::~SocketCanTransport() { close(); }

TransportCapabilities SocketCanTransport::capabilities_for(
    const SocketCanOptions& options) noexcept {
  return TransportCapabilities{true,
                               options.enable_can_fd,
                               options.enable_can_fd,
                               true,
                               true,
                               true,
                               options.enable_error_frames,
                               true,
                               true,
                               false,
                               options.nominal_bitrate_hz,
                               static_cast<std::uint8_t>(
                                   options.enable_can_fd ? 64U : 8U),
                               static_cast<std::uint16_t>(std::min<std::size_t>(
                                   options.receive_queue_capacity, 65535U))};
}

TransportKind SocketCanTransport::kind() const noexcept {
  return TransportKind::SocketCan;
}

const TransportCapabilities& SocketCanTransport::capabilities() const noexcept {
  return capabilities_;
}

bool SocketCanTransport::is_open() const noexcept { return socket_fd_ >= 0; }

bool SocketCanTransport::open() noexcept {
  if (is_open() || options_.interface_name.empty() ||
      options_.logical_bus == 0U || !capabilities_.is_valid()) {
    return false;
  }
#ifdef __linux__
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd < 0) {
    return false;
  }
  int enable_fd = options_.enable_can_fd ? 1 : 0;
  if (options_.enable_can_fd &&
      ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                   sizeof(enable_fd)) < 0) {
    ::close(fd);
    return false;
  }
  if (options_.enable_error_frames) {
    const can_err_mask_t mask = CAN_ERR_MASK;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, sizeof(mask)) <
        0) {
      ::close(fd);
      return false;
    }
  }
  if (!options_.filters.empty()) {
    std::array<can_filter, 32U> filters{};
    if (options_.filters.size() > filters.size()) {
      ::close(fd);
      return false;
    }
    for (std::size_t index = 0U; index < options_.filters.size(); ++index) {
      const auto& filter = options_.filters[index];
      if (!filter.is_valid() || filter.frame_type.has_value()) {
        ::close(fd);
        return false;
      }
      const auto format_flag = filter.format == CanFrameFormat::Extended
                                   ? CAN_EFF_FLAG
                                   : 0U;
      filters[index].can_id = filter.value | format_flag;
      filters[index].can_mask = filter.mask |
                                (filter.format == CanFrameFormat::Extended
                                     ? CAN_EFF_FLAG
                                     : CAN_EFF_FLAG);
    }
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(options_.filters.size() *
                                            sizeof(can_filter))) < 0) {
      ::close(fd);
      return false;
    }
  }
  int timestamp = 1;
  if (::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &timestamp,
                   sizeof(timestamp)) < 0) {
    ::close(fd);
    return false;
  }
  const auto interface_index = if_nametoindex(options_.interface_name.c_str());
  if (interface_index == 0U) {
    ::close(fd);
    return false;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(interface_index);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    ::close(fd);
    return false;
  }
  socket_fd_ = fd;
  return true;
#else
  return false;
#endif
}

void SocketCanTransport::close() noexcept {
#ifdef __linux__
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
#endif
  socket_fd_ = -1;
}

TransportResult SocketCanTransport::try_receive(RawCanFrame& frame) noexcept {
#ifdef __linux__
  if (!is_open()) {
    return TransportResult::Disconnected;
  }
  std::array<std::uint8_t, CANFD_MTU> data{};
  std::array<std::uint8_t, 128U> control{};
  iovec vector{data.data(), data.size()};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  const auto received = ::recvmsg(socket_fd_, &message, MSG_DONTWAIT);
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return TransportResult::WouldBlock;
    }
    ++stats_.errors;
    return errno == ENETDOWN || errno == ENODEV ? TransportResult::Disconnected
                                                : TransportResult::Fault;
  }
  if ((message.msg_flags & MSG_TRUNC) != 0 ||
      (received != CAN_MTU && received != CANFD_MTU)) {
    ++stats_.rx_dropped;
    ++stats_.errors;
    return TransportResult::Invalid;
  }
  const bool is_fd = received == CANFD_MTU;
  const auto* can_id = reinterpret_cast<const canid_t*>(data.data());
  bool error = false;
  const auto id = from_can_id(*can_id, error);
  const auto* classic = reinterpret_cast<const can_frame*>(data.data());
  const auto* fd = reinterpret_cast<const canfd_frame*>(data.data());
  const auto* payload = is_fd ? fd->data : classic->data;
  const auto length = is_fd ? fd->len : classic->can_dlc;
  std::optional<SourceTimestamp> source;
  for (cmsghdr* header = CMSG_FIRSTHDR(&message); header != nullptr;
       header = CMSG_NXTHDR(&message, header)) {
    if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SCM_TIMESTAMPNS) {
      const auto* timestamp = reinterpret_cast<const timespec*>(CMSG_DATA(header));
      const auto ticks = static_cast<std::uint64_t>(timestamp->tv_sec) *
                             1000000000ULL + static_cast<std::uint64_t>(timestamp->tv_nsec);
      source = SourceTimestamp{SourceClockDomain::Transport, ticks};
    }
  }
  std::array<std::uint8_t, kMaxCanPayloadBytes> bytes{};
  if (length > bytes.size()) {
    ++stats_.errors;
    return TransportResult::Invalid;
  }
  std::copy_n(payload, length, bytes.begin());
  const auto created = RawCanFrame::create(
      options_.logical_bus, id,
      is_fd ? CanFrameType::FlexibleDataRate : CanFrameType::Classic,
      FrameDirection::Rx, length, bytes, host_now(), source,
      is_fd && (fd->flags & CANFD_BRS) != 0U);
  if (!created.has_value()) {
    ++stats_.errors;
    return TransportResult::Invalid;
  }
  frame = *created;
  frame.error_frame = error;
  ++stats_.rx_frames;
  return TransportResult::Ok;
#else
  (void)frame;
  return TransportResult::Fault;
#endif
}

TransportResult SocketCanTransport::try_send(const RawCanFrame& frame) noexcept {
#ifdef __linux__
  if (!is_open()) {
    return TransportResult::Disconnected;
  }
  if (frame.logical_bus != options_.logical_bus ||
      frame.direction != FrameDirection::Tx || frame.error_frame ||
      !frame.is_valid()) {
    ++stats_.errors;
    return TransportResult::Invalid;
  }
  std::array<std::uint8_t, CANFD_MTU> data{};
  std::size_t size = 0U;
  if (frame.type == CanFrameType::Classic) {
    auto* output = reinterpret_cast<can_frame*>(data.data());
    output->can_id = to_can_id(frame.id);
    output->can_dlc = frame.payload_size;
    std::copy_n(frame.payload.begin(), frame.payload_size, output->data);
    size = CAN_MTU;
  } else {
    auto* output = reinterpret_cast<canfd_frame*>(data.data());
    output->can_id = to_can_id(frame.id);
    output->len = frame.payload_size;
    output->flags = static_cast<__u8>(CANFD_FDF |
        (frame.bitrate_switch ? CANFD_BRS : 0U));
    std::copy_n(frame.payload.begin(), frame.payload_size, output->data);
    size = CANFD_MTU;
  }
  const auto sent = ::send(socket_fd_, data.data(), size, MSG_DONTWAIT);
  if (sent < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ++stats_.queue_full;
      ++stats_.tx_dropped;
      return TransportResult::QueueFull;
    }
    ++stats_.errors;
    return errno == ENETDOWN || errno == ENODEV ? TransportResult::Disconnected
                                                : TransportResult::Fault;
  }
  if (static_cast<std::size_t>(sent) != size) {
    ++stats_.errors;
    return TransportResult::Fault;
  }
  ++stats_.tx_frames;
  return TransportResult::Ok;
#else
  (void)frame;
  return TransportResult::Fault;
#endif
}

TransportStats SocketCanTransport::stats() const noexcept { return stats_; }

}  // namespace mech::mech_control_core
