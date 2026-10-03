#include "mech_bringup/posix_cdc_serial_port.hpp"

#include <cerrno>
#include "mech_bringup/command_trace.hpp"
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include "mech_bringup/pass_through_init.hpp"

namespace mech::mech_bringup {
namespace {

// The board enumerates as CDC-ACM and speaks its own framing on top; the
// vendor stack and bench receiver set the host line coding to 4,000,000 baud
// with raw, non-blocking I/O. This is independent of the CAN bus bitrates.
bool configure_raw_nonblocking(int fd) noexcept {
  termios attrs{};
  if (tcgetattr(fd, &attrs) != 0) {
    return false;
  }
  if (cfsetispeed(&attrs, B4000000) != 0 ||
      cfsetospeed(&attrs, B4000000) != 0) {
    return false;
  }
  attrs.c_cflag &= ~(CSIZE | PARENB);
  attrs.c_cflag |= CS8 | CLOCAL | CREAD;
  attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
  attrs.c_iflag &= ~(INLCR | ICRNL | IGNCR | IXON | IXOFF | IXANY | ISTRIP);
  attrs.c_oflag &= ~OPOST;
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
  // Lock the opened inode before termios/flush touches it. Both production
  // plugins use this port, so a symlink alias or separate process must not
  // configure the same cooperative CDC channel concurrently.
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

mech_control_core::TransportResult PosixCdcSerialPort::read_some(
    std::uint8_t* data, std::size_t capacity, std::size_t& size) noexcept {
  size = 0U;
  if (fd_ < 0) {
    return mech_control_core::TransportResult::Disconnected;
  }
  const ssize_t received = ::read(fd_, data, capacity);
  if (received > 0) {
    size = static_cast<std::size_t>(received);
    return mech_control_core::TransportResult::Ok;
  }
  if (received == 0) {
    return mech_control_core::TransportResult::WouldBlock;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return mech_control_core::TransportResult::WouldBlock;
  }
  return mech_control_core::TransportResult::Fault;
}

mech_control_core::TransportResult PosixCdcSerialPort::write_all(
    const std::uint8_t* data, std::size_t size) noexcept {
  if (fd_ < 0) {
    return mech_control_core::TransportResult::Disconnected;
  }
  std::size_t written = 0U;
  while (written < size) {
    const auto begin = trace_ ? CommandTrace::now() : 0;
    const ssize_t sent = ::write(fd_, data + written, size - written);
    const int saved_errno = sent < 0 ? errno : 0;
    if (trace_) trace_->bytes("syscall_write", data + written,
        sent > 0 ? static_cast<std::size_t>(sent) : 0U, sent, size - written,
        saved_errno, begin);
    if (sent < 0) errno = saved_errno;
    if (sent > 0) {
      written += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent < 0 && errno == EINTR) {
      continue;
    }
    if ((sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) || sent == 0) {
      // Retry is safe only when no byte of this packet entered the stream.
      // After a partial write, callers cannot resume from our local offset;
      // retrying the whole packet or sending another route would corrupt the
      // framing. Fault the bus epoch instead of blocking or replaying it.
      return written == 0U ? mech_control_core::TransportResult::WouldBlock
                           : mech_control_core::TransportResult::Fault;
    }
    return mech_control_core::TransportResult::Fault;
  }
  return mech_control_core::TransportResult::Ok;
}

bool PosixCdcSerialPort::send_pass_through_init() noexcept {
  if (fd_ < 0) {
    return false;
  }
  return mech::mech_bringup::send_pass_through_init(*this);
}

}  // namespace mech::mech_bringup
