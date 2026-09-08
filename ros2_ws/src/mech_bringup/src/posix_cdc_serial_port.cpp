#include "mech_bringup/posix_cdc_serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "mech_bringup/pass_through_init.hpp"

namespace mech::mech_bringup {
namespace {

// The board enumerates as CDC-ACM and speaks its own framing on top; the
// vendor stack opens the device raw (no baud rate is negotiated for ACM) with
// non-blocking I/O, which is also what the bring-up probes used.
bool configure_raw_nonblocking(int fd) noexcept {
  termios attrs{};
  if (tcgetattr(fd, &attrs) != 0) {
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
  const int fd = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
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
    const ssize_t sent = ::write(fd_, data + written, size - written);
    if (sent > 0) {
      written += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent < 0 && errno == EINTR) {
      continue;
    }
    if ((sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) || sent == 0) {
      // A real non-blocking CDC write that cannot proceed right now. This is
      // exactly the transient case the RC defect permanently faulted on; the
      // caller retries the next cycle.
      return mech_control_core::TransportResult::WouldBlock;
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
