#include "mech_ctrboard_bridge/posix_cdc_serial_port.hpp"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <sys/file.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <utility>

namespace mech::mech_ctrboard_bridge {
namespace {

bool configure_raw_nonblocking(int fd) noexcept {
  termios attrs{};
  if (tcgetattr(fd, &attrs) != 0) {
    return false;
  }
  if (cfsetispeed(&attrs, B4000000) != 0 ||
      cfsetospeed(&attrs, B4000000) != 0) {
    return false;
  }
  cfmakeraw(&attrs);
  attrs.c_cflag &= ~CRTSCTS;
  attrs.c_cflag |= CLOCAL | CREAD;
  attrs.c_iflag &= ~(IXOFF | IXANY);
  attrs.c_cc[VMIN] = 0;
  attrs.c_cc[VTIME] = 0;
  return tcsetattr(fd, TCSANOW, &attrs) == 0;
}

}  // namespace

PosixCdcSerialPort::PosixCdcSerialPort(std::string device_path) noexcept
    : device_path_(std::move(device_path)) {}

PosixCdcSerialPort::~PosixCdcSerialPort() { close(); }

bool PosixCdcSerialPort::is_open() const noexcept { return fd_ >= 0; }

bool PosixCdcSerialPort::open() noexcept {
  if (fd_ >= 0) {
    return true;
  }
  const int fd = ::open(device_path_.c_str(),
                        O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  // Lock the opened inode before changing termios or flushing buffers. This
  // also rejects a second process that reaches the same device through a
  // symlink alias.
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd);
    return false;
  }
  if (!configure_raw_nonblocking(fd)) {
    ::close(fd);
    return false;
  }
  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  return true;
}

void PosixCdcSerialPort::close() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

TransportResult PosixCdcSerialPort::read_some(
    std::uint8_t* data, std::size_t capacity, std::size_t& size) noexcept {
  size = 0U;
  if (fd_ < 0) {
    return TransportResult::Disconnected;
  }
  const ssize_t received = ::read(fd_, data, capacity);
  if (received > 0) {
    if (read_timing_enabled_) {
      timespec stamp{};
      if (::clock_gettime(CLOCK_MONOTONIC, &stamp) == 0) {
        last_read_end_ns_ = static_cast<std::int64_t>(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec;
      } else {
        last_read_end_ns_ = 0;
        ++read_clock_errors_;
      }
    }
    size = static_cast<std::size_t>(received);
    return TransportResult::Ok;
  }
  if (received == 0) {
    return TransportResult::WouldBlock;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return TransportResult::WouldBlock;
  }
  return TransportResult::Fault;
}

TransportResult PosixCdcSerialPort::write_all(
    const std::uint8_t* data, std::size_t size) noexcept {
  if (fd_ < 0) {
    return TransportResult::Disconnected;
  }
  std::size_t written = 0U;
  while (written < size) {
    const std::int64_t begin =
        trace_sink_ != nullptr ? trace_sink_->begin_write() : 0;
    const ssize_t sent = ::write(fd_, data + written, size - written);
    const int saved_errno = sent < 0 ? errno : 0;
    if (trace_sink_ != nullptr) {
      trace_sink_->record_write(
          data + written, sent > 0 ? static_cast<std::size_t>(sent) : 0U,
          sent, size - written, saved_errno, begin);
    }
    if (sent < 0) errno = saved_errno;
    if (sent > 0) {
      written += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent < 0 && errno == EINTR) {
      continue;
    }
    if ((sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) || sent == 0) {
      // Retrying is safe only when no byte from this packet entered the
      // stream. A partial prefix followed by backpressure must fault the bus
      // epoch, otherwise replaying the packet corrupts vendor framing.
      return written == 0U ? TransportResult::WouldBlock
                           : TransportResult::Fault;
    }
    return TransportResult::Fault;
  }
  return TransportResult::Ok;
}

bool PosixCdcSerialPort::send_pass_through_init() noexcept {
  // Fixed vendor receive-forwarding setup; send_flag is zero.
  // Kept private to this addon so the deployed core library is unchanged.
  constexpr std::uint8_t bytes[] = {
      0xF7, 0x12, 0x06, 0x00, 0x7D, 0x70, 0x08,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!is_open()) return false;
  TransportResult result = TransportResult::WouldBlock;
  for (unsigned attempt = 0; attempt < 100U; ++attempt) {
    result = write_all(bytes, sizeof(bytes));
    if (result != TransportResult::WouldBlock) break;
  }
  return result == TransportResult::Ok;
}

}  // namespace mech::mech_ctrboard_bridge
