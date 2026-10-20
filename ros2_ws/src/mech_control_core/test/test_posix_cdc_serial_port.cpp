#include "mech_control_core/posix_cdc_serial_port.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace mech::mech_control_core {
namespace {

// A pseudo-terminal exercises the real Linux descriptor path without hardware.
class PosixCdcSerialPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    master_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(grantpt(master_), 0);
    ASSERT_EQ(unlockpt(master_), 0);
    const char* path = ptsname(master_);
    ASSERT_NE(path, nullptr);
    path_ = path;
  }

  void TearDown() override {
    if (master_ >= 0) ::close(master_);
  }

  int master_{-1};
  std::string path_;
};

TEST_F(PosixCdcSerialPortTest, SecondOwnerCannotFlushLiveStreamAndLockIsReleased) {
  PosixCdcSerialPort first{path_};
  PosixCdcSerialPort second{path_};
  ASSERT_TRUE(first.open());
  ASSERT_TRUE(first.open());
  const std::array<std::uint8_t, 4U> packet{0x00U, 0x11U, 0x13U, 0xFFU};
  ASSERT_EQ(::write(master_, packet.data(), packet.size()),
            static_cast<ssize_t>(packet.size()));
  EXPECT_FALSE(second.open());
  EXPECT_FALSE(second.is_open());
  std::array<std::uint8_t, 4U> received{};
  std::size_t count = 0U;
  TransportResult result = TransportResult::WouldBlock;
  for (int retry = 0; retry < 100 && result == TransportResult::WouldBlock; ++retry) {
    result = first.read_some(received.data(), received.size(), count);
    if (result == TransportResult::WouldBlock) usleep(1000);
  }
  ASSERT_EQ(result, TransportResult::Ok);
  EXPECT_EQ(count, packet.size());
  EXPECT_EQ(received, packet);
  first.close();
  EXPECT_TRUE(second.open());
}

TEST_F(PosixCdcSerialPortTest, PartialPacketBackpressureIsFaultNotRetryable) {
  PosixCdcSerialPort port{path_};
  ASSERT_TRUE(port.open());
  // Larger than the PTY output queue: some bytes are accepted, then EAGAIN.
  const std::vector<std::uint8_t> packet(1024U * 1024U, 0x55U);
  EXPECT_EQ(port.write_all(packet.data(), packet.size()), TransportResult::Fault);
}

TEST_F(PosixCdcSerialPortTest, DestructorReleasesOwnership) {
  {
    PosixCdcSerialPort port{path_};
    ASSERT_TRUE(port.open());
  }
  PosixCdcSerialPort next{path_};
  EXPECT_TRUE(next.open());
}

}  // namespace
}  // namespace mech::mech_control_core
