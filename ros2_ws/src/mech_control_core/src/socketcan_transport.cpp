#include "mech_control_core/socketcan_transport.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/net_tstamp.h>
#include <linux/rtnetlink.h>
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
std::uint32_t to_can_id(const CanId& id, bool remote_request) noexcept {
  return id.value | (id.format == CanFrameFormat::Extended ? CAN_EFF_FLAG : 0U) |
         (remote_request ? CAN_RTR_FLAG : 0U);
}

// Decodes a non-error can_id word into a CanId plus the RTR bit. Must not be
// called for a frame with CAN_ERR_FLAG set -- see error-frame handling in
// try_receive(), which uses a completely different encoding (documented in
// the header) instead of going through this path.
CanId from_can_id(canid_t raw, bool& remote_request) noexcept {
  remote_request = (raw & CAN_RTR_FLAG) != 0U;
  const auto extended = (raw & CAN_EFF_FLAG) != 0U;
  return *CanId::create(raw & (extended ? CAN_EFF_MASK : CAN_SFF_MASK),
                        extended ? CanFrameFormat::Extended
                                  : CanFrameFormat::Standard);
}

// Reads the interface's real configured bitrate via an RTM_GETLINK netlink
// query: IFLA_LINKINFO -> IFLA_INFO_DATA -> IFLA_CAN_BITTIMING ->
// can_bittiming.bitrate. Returns true and sets `bitrate_hz` only when the
// kernel actually reports one. Failing -- including "this is not a CAN
// link", "the driver hasn't configured bit-timing yet", or "this is a vcan
// interface, which has no bitrate at all" -- is the normal, expected outcome
// for many interfaces and must not be treated as an error by the caller.
bool read_interface_bitrate(const std::string& interface_name,
                            std::uint32_t& bitrate_hz) noexcept {
  const int fd = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (fd < 0) {
    return false;
  }
  struct Request {
    nlmsghdr header;
    ifinfomsg info;
  };
  Request request{};
  request.header.nlmsg_len = sizeof(Request);
  request.header.nlmsg_type = RTM_GETLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST;
  request.header.nlmsg_pid = 0U;
  request.header.nlmsg_seq = 1U;
  request.info.ifi_family = AF_UNSPEC;
  request.info.ifi_index =
      static_cast<int>(if_nametoindex(interface_name.c_str()));
  if (request.info.ifi_index == 0) {
    ::close(fd);
    return false;
  }
  // open() must never be able to block forever on a kernel that does not
  // answer. This socket is only ever used for this one request/reply.
  timeval timeout{};
  timeout.tv_sec = 1;
  if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    ::close(fd);
    return false;
  }
  if (::send(fd, &request, sizeof(request), 0) < 0) {
    ::close(fd);
    return false;
  }
  std::array<std::uint8_t, 8192U> reply{};
  const auto received = ::recv(fd, reply.data(), reply.size(), 0);
  ::close(fd);
  if (received < static_cast<ssize_t>(sizeof(nlmsghdr))) {
    return false;
  }
  const auto* header = reinterpret_cast<const nlmsghdr*>(reply.data());
  // NLMSG_OK also enforces nlmsg_len <= received, so a datagram truncated by
  // our fixed-size buffer is rejected here rather than walked past the end.
  if (!NLMSG_OK(header, received) || header->nlmsg_type != RTM_NEWLINK) {
    return false;
  }
  // IFLA_PAYLOAD subtracts in unsigned arithmetic, so a short/malformed header
  // would wrap to a huge value. Reject it before it becomes a walk length.
  if (header->nlmsg_len < NLMSG_LENGTH(sizeof(ifinfomsg))) {
    return false;
  }
  const auto* info = reinterpret_cast<const ifinfomsg*>(NLMSG_DATA(header));
  auto attribute_len = static_cast<int>(IFLA_PAYLOAD(header));
  for (const auto* attribute = IFLA_RTA(info); RTA_OK(attribute, attribute_len);
       attribute = RTA_NEXT(attribute, attribute_len)) {
    if (attribute->rta_type != IFLA_LINKINFO) {
      continue;
    }
    auto link_info_len =
        static_cast<int>(RTA_PAYLOAD(attribute));
    for (const auto* link_attribute =
             reinterpret_cast<const rtattr*>(RTA_DATA(attribute));
         RTA_OK(link_attribute, link_info_len);
         link_attribute = RTA_NEXT(link_attribute, link_info_len)) {
      if (link_attribute->rta_type != IFLA_INFO_DATA) {
        continue;
      }
      auto data_len = static_cast<int>(RTA_PAYLOAD(link_attribute));
      for (const auto* data_attribute =
               reinterpret_cast<const rtattr*>(RTA_DATA(link_attribute));
           RTA_OK(data_attribute, data_len);
           data_attribute = RTA_NEXT(data_attribute, data_len)) {
        if (data_attribute->rta_type != IFLA_CAN_BITTIMING) {
          continue;
        }
        if (RTA_PAYLOAD(data_attribute) < sizeof(can_bittiming)) {
          return false;
        }
        const auto* timing = reinterpret_cast<const can_bittiming*>(
            RTA_DATA(data_attribute));
        if (timing->bitrate == 0U) {
          return false;
        }
        bitrate_hz = timing->bitrate;
        return true;
      }
    }
  }
  return false;
}
#endif

}  // namespace

SocketCanTransport::SocketCanTransport(SocketCanOptions options)
    : options_(std::move(options)), capabilities_(capabilities_for(options_)) {}

SocketCanTransport::~SocketCanTransport() { close(); }

TransportCapabilities SocketCanTransport::capabilities_for(
    const SocketCanOptions& options) noexcept {
  // Named assignment on purpose: this struct is mostly booleans, so positional
  // aggregate initialization would silently shift every later field the moment
  // one is inserted.
  TransportCapabilities capabilities;
  capabilities.supports_classic_can = true;
  capabilities.supports_can_fd = options.enable_can_fd;
  capabilities.supports_brs = options.enable_can_fd;
  capabilities.supports_standard_frames = true;
  capabilities.supports_extended_frames = true;
  capabilities.supports_filters = true;
  capabilities.supports_error_frames = options.enable_error_frames;
  capabilities.supports_timestamps = true;
  capabilities.supports_non_blocking_io = true;
  capabilities.supports_remote_frames = true;
  capabilities.nominal_bitrate_configurable = false;
  // Declared until open() reads the real value back from netlink.
  capabilities.nominal_bitrate_hz = options.nominal_bitrate_hz;
  capabilities.nominal_bitrate_verified = false;
  capabilities.max_payload_bytes =
      static_cast<std::uint8_t>(options.enable_can_fd ? 64U : 8U);
  capabilities.queue_capacity = static_cast<std::uint16_t>(
      std::min<std::size_t>(options.receive_queue_capacity, 65535U));
  capabilities.queue_capacity_verified = false;
  return capabilities;
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
    // SocketCAN hardware/kernel filters match on can_id and can_mask only;
    // they cannot express `frame_type` (Classic vs FD). A filter that
    // requests frame_type is still installed as a kernel id/mask filter (so
    // the kernel does the bulk of the work), and `has_software_frame_type_filters_`
    // is set so try_receive() re-checks the *complete* filter (including
    // frame_type) against every frame it decodes before returning it. That
    // keeps `supports_filters == true` honest instead of open() silently
    // failing for a filter set the hardware can only partially apply.
    for (std::size_t index = 0U; index < options_.filters.size(); ++index) {
      const auto& filter = options_.filters[index];
      if (!filter.is_valid()) {
        ::close(fd);
        return false;
      }
      if (filter.frame_type.has_value()) {
        has_software_frame_type_filters_ = true;
      }
      const auto format_flag = filter.format == CanFrameFormat::Extended
                                   ? CAN_EFF_FLAG
                                   : 0U;
      filters[index].can_id = filter.value | format_flag;
      // The mask must always include the EFF bit, regardless of this
      // filter's own format: without it a standard-frame filter (EFF bit
      // clear in can_id, and thus also clear in the low bits actually
      // compared) would still match an extended frame whose low 11 bits
      // happen to line up, and vice versa. So CAN_EFF_FLAG is unconditional
      // here, not a per-branch choice.
      filters[index].can_mask = filter.mask | CAN_EFF_FLAG;
    }
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(options_.filters.size() *
                                            sizeof(can_filter))) < 0) {
      ::close(fd);
      return false;
    }
  }
  // Prefer CLOCK_MONOTONIC-based receive timestamps (SO_TIMESTAMPING with
  // SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE) so timestamps
  // are directly comparable to MonotonicTime. NOTE: kernel documentation
  // (Documentation/networking/timestamping.rst) states that the general
  // software timestamp path (ts[0] in `struct scm_timestamping`, which is
  // what SOF_TIMESTAMPING_SOFTWARE reports) is sourced from
  // ktime_get_real(), i.e. CLOCK_REALTIME, not CLOCK_MONOTONIC -- so in
  // practice this path is NOT expected to be any more monotonic than
  // SO_TIMESTAMPNS. It is attempted first anyway because it is the
  // kernel-recommended modern API and because CAN_RAW's actual behaviour
  // could not be verified against real hardware in this change; if it turns
  // out to be realtime-based here too, the honesty contract is unaffected
  // because the domain is documented as "wall-clock, do not trust as
  // monotonic" in both cases (see the "Timestamp domain" note in the
  // header). SO_TIMESTAMPNS remains the fallback for kernels/drivers that
  // reject SO_TIMESTAMPING outright.
  std::uint32_t timestamping_flags = SOF_TIMESTAMPING_RX_SOFTWARE |
                                     SOF_TIMESTAMPING_SOFTWARE;
  if (::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPING, &timestamping_flags,
                   sizeof(timestamping_flags)) != 0) {
    int timestamp = 1;
    if (::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &timestamp,
                     sizeof(timestamp)) < 0) {
      ::close(fd);
      return false;
    }
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
  // Make the advertised receive queue capacity real: ask the kernel for the
  // requested SO_RCVBUF, then read back what it actually granted (the
  // kernel doubles the requested value and clamps to
  // net.core.rmem_max/rmem_default). Only the read-back value is trusted.
  const auto requested_rcvbuf = static_cast<int>(
      std::min<std::size_t>(options_.receive_queue_capacity, 65535U));
  if (::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &requested_rcvbuf,
                   sizeof(requested_rcvbuf)) == 0) {
    int actual_rcvbuf = 0;
    socklen_t actual_len = sizeof(actual_rcvbuf);
    if (::getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &actual_rcvbuf,
                     &actual_len) == 0 &&
        actual_rcvbuf > 0) {
      capabilities_.queue_capacity = static_cast<std::uint16_t>(
          std::min<int>(actual_rcvbuf, 65535));
      capabilities_.queue_capacity_verified = true;
    }
  }
  // Read the interface's real bitrate over netlink. Failure -- including
  // "this is vcan, which genuinely has none" -- must not fail open().
  std::uint32_t bitrate_hz = 0U;
  if (read_interface_bitrate(options_.interface_name, bitrate_hz)) {
    capabilities_.nominal_bitrate_hz = bitrate_hz;
    capabilities_.nominal_bitrate_verified = true;
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
  has_software_frame_type_filters_ = false;
}

TransportResult SocketCanTransport::try_receive(RawCanFrame& frame) noexcept {
#ifdef __linux__
  if (!is_open()) {
    return TransportResult::Disconnected;
  }
  // Bounded so a run of software-filtered frames (see
  // has_software_frame_type_filters_) cannot spin forever inside a single
  // call; the caller's own poll loop already re-invokes try_receive() on the
  // next budget slot, so returning WouldBlock after this many skips just
  // yields back rather than genuinely meaning "socket is empty".
  constexpr int kMaxSoftwareFilterSkips = 64;
  for (int skipped = 0; skipped < kMaxSoftwareFilterSkips; ++skipped) {
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
      return errno == ENETDOWN || errno == ENODEV
                 ? TransportResult::Disconnected
                 : TransportResult::Fault;
    }
    if ((message.msg_flags & MSG_TRUNC) != 0 ||
        (received != CAN_MTU && received != CANFD_MTU)) {
      ++stats_.rx_dropped;
      ++stats_.errors;
      return TransportResult::Invalid;
    }
    const bool is_fd = received == CANFD_MTU;
    const auto* can_id_word = reinterpret_cast<const canid_t*>(data.data());
    const bool is_error_frame = (*can_id_word & CAN_ERR_FLAG) != 0U;
    const auto* classic = reinterpret_cast<const can_frame*>(data.data());
    const auto* fd = reinterpret_cast<const canfd_frame*>(data.data());
    const auto* payload = is_fd ? fd->data : classic->data;
    const auto length = is_fd ? fd->len : classic->can_dlc;
    std::optional<SourceTimestamp> source;
    for (cmsghdr* header = CMSG_FIRSTHDR(&message); header != nullptr;
         header = CMSG_NXTHDR(&message, header)) {
      if (header->cmsg_level != SOL_SOCKET) {
        continue;
      }
      if (header->cmsg_type == SCM_TIMESTAMPING) {
        // struct scm_timestamping { struct timespec ts[3]; }; ts[0] is the
        // software timestamp (see the "Timestamp domain" note in the header
        // for which wall-clock/monotonic domain this actually is).
        const auto* timestamps =
            reinterpret_cast<const timespec*>(CMSG_DATA(header));
        const auto& timestamp = timestamps[0];
        const auto ticks = static_cast<std::uint64_t>(timestamp.tv_sec) *
                               1000000000ULL +
                           static_cast<std::uint64_t>(timestamp.tv_nsec);
        source = SourceTimestamp{SourceClockDomain::Transport, ticks};
      } else if (header->cmsg_type == SCM_TIMESTAMPNS && !source.has_value()) {
        const auto* timestamp =
            reinterpret_cast<const timespec*>(CMSG_DATA(header));
        const auto ticks = static_cast<std::uint64_t>(timestamp->tv_sec) *
                               1000000000ULL +
                           static_cast<std::uint64_t>(timestamp->tv_nsec);
        source = SourceTimestamp{SourceClockDomain::Transport, ticks};
      }
    }
    std::array<std::uint8_t, kMaxCanPayloadBytes> bytes{};
    if (length > bytes.size()) {
      ++stats_.errors;
      return TransportResult::Invalid;
    }

    if (is_error_frame) {
      // Error-frame encoding: see the class-level comment in the header.
      // CAN_ERR_MASK is 29 bits wide, wider than a standard 11-bit CanId can
      // hold, so the class bitmask is stored using CanFrameFormat::Extended
      // (kMaxExtendedCanId == CAN_ERR_MASK) purely so CanId::create() always
      // accepts it; the format flag itself carries no meaning for an error
      // frame, as documented in the header.
      const auto error_class = *can_id_word & CAN_ERR_MASK;
      const auto error_id =
          CanId::create(error_class, CanFrameFormat::Extended);
      if (!error_id.has_value()) {
        ++stats_.errors;
        return TransportResult::Invalid;
      }
      std::copy_n(classic->data, CAN_ERR_DLC, bytes.begin());
      const auto created = RawCanFrame::create(
          options_.logical_bus, *error_id, CanFrameType::Classic,
          FrameDirection::Rx, CAN_ERR_DLC, bytes, host_now(), source);
      if (!created.has_value()) {
        ++stats_.errors;
        return TransportResult::Invalid;
      }
      frame = *created;
      frame.error_frame = true;
      ++stats_.rx_frames;
      return TransportResult::Ok;
    }

    bool remote_request = false;
    const auto id = from_can_id(*can_id_word, remote_request);
    std::copy_n(payload, length, bytes.begin());
    const auto created = RawCanFrame::create(
        options_.logical_bus, id,
        is_fd ? CanFrameType::FlexibleDataRate : CanFrameType::Classic,
        FrameDirection::Rx, length, bytes, host_now(), source,
        is_fd && (fd->flags & CANFD_BRS) != 0U, remote_request && !is_fd);
    if (!created.has_value()) {
      ++stats_.errors;
      return TransportResult::Invalid;
    }
    if (has_software_frame_type_filters_ && !options_.filters.empty()) {
      bool matched_any = false;
      for (const auto& filter : options_.filters) {
        if (filter.matches(*created)) {
          matched_any = true;
          break;
        }
      }
      if (!matched_any) {
        // The kernel already matched id/mask (or this frame would never
        // have been delivered at all); only frame_type disagreed. Not an
        // error and not "no data" -- skip and keep draining the socket
        // within this call's budget instead of reporting WouldBlock, which
        // would wrongly tell the caller's poll loop to stop early even
        // though more frames may be queued.
        ++stats_.rx_dropped;
        continue;
      }
    }
    frame = *created;
    ++stats_.rx_frames;
    return TransportResult::Ok;
  }
  return TransportResult::WouldBlock;
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
    output->can_id = to_can_id(frame.id, frame.remote_request);
    output->can_dlc = frame.payload_size;
    std::copy_n(frame.payload.begin(), frame.payload_size, output->data);
    size = CAN_MTU;
  } else {
    // frame.is_valid() above already rejects remote_request combined with
    // any type other than Classic, so remote_request is guaranteed false
    // here -- CAN FD has no RTR and to_can_id() is not given the chance to
    // set CAN_RTR_FLAG on an FD frame.
    auto* output = reinterpret_cast<canfd_frame*>(data.data());
    output->can_id = to_can_id(frame.id, false);
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
