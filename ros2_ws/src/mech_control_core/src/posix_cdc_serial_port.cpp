#include "mech_control_core/posix_cdc_serial_port.hpp"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <utility>

namespace mech::mech_control_core {
namespace {

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

TransportResult PosixCdcSerialPort::read_some(
    std::uint8_t* data, std::size_t capacity, std::size_t& size) noexcept {
  size = 0U;
  if (fd_ < 0) {
    return TransportResult::Disconnected;
  }
  const ssize_t received = ::read(fd_, data, capacity);
  if (received > 0) {
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
    const ssize_t sent = ::write(fd_, data + written, size - written);
    if (sent > 0) {
      written += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent < 0 && errno == EINTR) {
      continue;
    }
    if ((sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) || sent == 0) {
      return TransportResult::WouldBlock;
    }
    return TransportResult::Fault;
  }
  return TransportResult::Ok;
}

bool PosixCdcSerialPort::send_pass_through_init() noexcept {
  return initialize_usb_cdc_pass_through(*this);
}

}  // namespace mech::mech_control_core
