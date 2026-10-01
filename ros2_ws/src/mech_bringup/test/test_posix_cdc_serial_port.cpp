#include "mech_bringup/posix_cdc_serial_port.hpp"

#include <array>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <gtest/gtest.h>

namespace {
using mech::mech_bringup::PosixCdcSerialPort;

class PosixCdcPortLockTest : public ::testing::Test {
 protected:
  void SetUp() override {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(::grantpt(master_), 0);
    ASSERT_EQ(::unlockpt(master_), 0);
    std::array<char, 256U> name{};
    ASSERT_EQ(::ptsname_r(master_, name.data(), name.size()), 0);
    path_ = name.data();
    ASSERT_EQ(path_.rfind("/dev/pts/", 0U), 0U);
  }

  void TearDown() override {
    if (master_ >= 0) ::close(master_);
  }

  int master_{-1};
  std::string path_;
};

TEST_F(PosixCdcPortLockTest, OpenSetsVendorBaudAfterPreviouslyConfigured9600) {
  termios attrs{};
  ASSERT_EQ(::tcgetattr(master_, &attrs), 0);
  ASSERT_EQ(::cfsetispeed(&attrs, B9600), 0);
  ASSERT_EQ(::cfsetospeed(&attrs, B9600), 0);
  ASSERT_EQ(::tcsetattr(master_, TCSANOW, &attrs), 0);
  ASSERT_EQ(::tcgetattr(master_, &attrs), 0);
  ASSERT_EQ(::cfgetispeed(&attrs), B9600);
  ASSERT_EQ(::cfgetospeed(&attrs), B9600);

  PosixCdcSerialPort serial(path_);
  ASSERT_TRUE(serial.open());
  ASSERT_EQ(::tcgetattr(master_, &attrs), 0);
  EXPECT_EQ(::cfgetispeed(&attrs), B4000000);
  EXPECT_EQ(::cfgetospeed(&attrs), B4000000);
}

TEST_F(PosixCdcPortLockTest, SecondInstanceCannotOpenSamePtyUntilFirstCloses) {
  PosixCdcSerialPort first(path_);
  PosixCdcSerialPort second(path_);
  ASSERT_TRUE(first.open());
  EXPECT_FALSE(second.open());
  EXPECT_TRUE(first.is_open());
  EXPECT_FALSE(second.is_open());
  first.close();
  EXPECT_TRUE(second.open());
  second.close();
  EXPECT_TRUE(first.open());
}

TEST_F(PosixCdcPortLockTest, SymlinkAliasSharesUnderlyingPtyLock) {
  char directory[] = "/tmp/servo-port-lock-XXXXXX";
  ASSERT_NE(::mkdtemp(directory), nullptr);
  const std::string alias = std::string(directory) + "/alias";
  ASSERT_EQ(::symlink(path_.c_str(), alias.c_str()), 0);
  {
    PosixCdcSerialPort first(path_);
    PosixCdcSerialPort second(alias);
    ASSERT_TRUE(first.open());
    EXPECT_FALSE(second.open());
    first.close();
    EXPECT_TRUE(second.open());
  }
  EXPECT_EQ(::unlink(alias.c_str()), 0);
  EXPECT_EQ(::rmdir(directory), 0);
}

TEST_F(PosixCdcPortLockTest, DestructorReleasesPtyLock) {
  PosixCdcSerialPort second(path_);
  {
    PosixCdcSerialPort first(path_);
    ASSERT_TRUE(first.open());
    EXPECT_FALSE(second.open());
  }
  EXPECT_TRUE(second.open());
}

TEST_F(PosixCdcPortLockTest, FailedOpenDoesNotPoisonLaterPtyOpen) {
  char directory[] = "/tmp/servo-port-lock-XXXXXX";
  ASSERT_NE(::mkdtemp(directory), nullptr);
  const std::string alias = std::string(directory) + "/alias";
  ASSERT_EQ(::symlink("/dev/pts/no-such-port", alias.c_str()), 0);
  PosixCdcSerialPort first(alias);
  EXPECT_FALSE(first.open());
  EXPECT_FALSE(first.is_open());
  ASSERT_EQ(::unlink(alias.c_str()), 0);
  ASSERT_EQ(::symlink(path_.c_str(), alias.c_str()), 0);
  EXPECT_TRUE(first.open());
  PosixCdcSerialPort second(path_);
  EXPECT_FALSE(second.open());
  first.close();
  EXPECT_TRUE(second.open());
  EXPECT_EQ(::unlink(alias.c_str()), 0);
  EXPECT_EQ(::rmdir(directory), 0);
}

TEST_F(PosixCdcPortLockTest, TerminalConfigurationFailureReleasesFileLock) {
  char regular_file[] = "/tmp/servo-port-not-tty-XXXXXX";
  const int creator = ::mkstemp(regular_file);
  ASSERT_GE(creator, 0);
  ASSERT_EQ(::close(creator), 0);

  PosixCdcSerialPort invalid_terminal(regular_file);
  EXPECT_FALSE(invalid_terminal.open());  // tcgetattr rejects a regular file
  const int probe = ::open(regular_file, O_RDWR);
  ASSERT_GE(probe, 0);
  EXPECT_EQ(::flock(probe, LOCK_EX | LOCK_NB), 0)
      << "failed termios configuration must close its locked descriptor";
  EXPECT_EQ(::close(probe), 0);
  EXPECT_EQ(::unlink(regular_file), 0);
}
}  // namespace

#include <sstream>
#include "mech_bringup/command_trace.hpp"
#include "mech_bringup/serial_trace.hpp"

TEST_F(PosixCdcPortLockTest, RecordsActualKernelWriteAndUnchangedPeerBytes) {
  auto trace = std::make_shared<mech::mech_bringup::CommandTrace>(16);
  auto port = std::make_shared<PosixCdcSerialPort>(path_);
  port->set_command_trace(trace.get());
  mech::mech_bringup::SerialTrace serial(port, 16, trace.get());
  ASSERT_TRUE(serial.open());
  const std::uint8_t packet[]{0xf7, 0x12, 0x00, 0xff, 0x0a, 0x0d};
  ASSERT_EQ(serial.write_all(packet, sizeof(packet)), mech::mech_control_core::TransportResult::Ok);
  std::array<std::uint8_t, 6> received{};
  ASSERT_EQ(::read(master_, received.data(), received.size()), 6);
  EXPECT_EQ(received, (std::array<std::uint8_t, 6>{0xf7, 0x12, 0, 0xff, 0x0a, 0x0d}));
  std::ostringstream out;
  trace->dump(out);
  EXPECT_NE(out.str().find("\"stage\":\"syscall_write\""), std::string::npos);
  EXPECT_NE(out.str().find("\"result\":6,\"errno\":0,\"requested\":6"), std::string::npos);
  EXPECT_NE(out.str().find("\"hex\":\"f71200ff0a0d\""), std::string::npos);
}

TEST_F(PosixCdcPortLockTest, BackpressureDoesNotClaimUnacceptedBytesWereWritten) {
  auto trace = std::make_shared<mech::mech_bringup::CommandTrace>(16);
  PosixCdcSerialPort port(path_);
  port.set_command_trace(trace.get());
  ASSERT_TRUE(port.open());
  // Fill a real nonblocking PTY without draining the peer. This forces a
  // partial prefix and EAGAIN, unlike FakeSerial's all-or-nothing interface.
  std::vector<std::uint8_t> bytes(1024U * 1024U, 0x5a);
  EXPECT_EQ(port.write_all(bytes.data(), bytes.size()),
            mech::mech_control_core::TransportResult::WouldBlock);
  std::ostringstream out; trace->dump(out);
  EXPECT_NE(out.str().find("\"result\":-1,\"errno\":11"), std::string::npos);
  EXPECT_EQ(out.str().find("\"result\":1048576"), std::string::npos);
}
