#include <gtest/gtest.h>
#include <array>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include "mech_bringup/command_trace.hpp"
#include "mech_bringup/serial_trace.hpp"
#include "mech_simulation/fake_serial.hpp"

namespace {
using mech::mech_bringup::TraceSnapshotHeader;
class SnapshotDirectory final {
 public:
  SnapshotDirectory() {
    char path[] = "/dev/shm/mech-trace-test-XXXXXX";
    const auto created = ::mkdtemp(path);
    if (!created) throw std::runtime_error("cannot create test tmpfs directory");
    path_ = created;
  }
  ~SnapshotDirectory() {
    ::unlink((path_ + "/chain-bus-1.snapshot").c_str());
    ::unlink((path_ + "/chain-bus-2.snapshot").c_str());
    ::unlink((path_ + "/chain.snapshot").c_str());
    ::unlink((path_ + "/serial.snapshot").c_str());
    ::unlink((path_ + "/alias").c_str());
    ::rmdir(path_.c_str());
  }
  std::string file(const char* name) const { return path_ + "/" + name; }
  const std::string& path() const noexcept { return path_; }
 private:
  std::string path_;
};
TraceSnapshotHeader header(const std::string& path) {
  TraceSnapshotHeader value{};
  std::ifstream input(path, std::ios::binary);
  input.read(reinterpret_cast<char*>(&value), sizeof(value));
  if (!input) throw std::runtime_error("snapshot header read failed");
  return value;
}

TEST(TraceSnapshot, NamedBusCapturesAreIndependentAndExclusive) {
  SnapshotDirectory directory;
  mech::mech_bringup::CommandTrace one(2, directory.file("chain-bus-1.snapshot"));
  mech::mech_bringup::CommandTrace two(2, directory.file("chain-bus-2.snapshot"));
  one.record("one"); two.record("two"); two.record("second");
  EXPECT_EQ(header(directory.file("chain-bus-1.snapshot")).total, 1U);
  EXPECT_EQ(header(directory.file("chain-bus-2.snapshot")).total, 2U);
  EXPECT_THROW(mech::mech_bringup::CommandTrace(2, directory.file("chain-bus-1.snapshot")), std::runtime_error);
  EXPECT_THROW(mech::mech_bringup::CommandTrace(2, directory.file("chain-.snapshot")), std::invalid_argument);
  EXPECT_THROW(mech::mech_bringup::CommandTrace(2, directory.file("serial-bus-1.snapshot")), std::invalid_argument);
}

TEST(TraceSnapshot, SurvivesObjectDestructionAndPreservesFirstRecords) {
  SnapshotDirectory directory;
  mech::mech_bringup::admit_trace_snapshots(directory.path(), 128 + 2 * 216);
  {
    mech::mech_bringup::CommandTrace trace(2, directory.file("chain.snapshot"));
    trace.record("first", 1, 42, 1.25, -3.0);
    trace.record("second");
    trace.record("omitted");
    EXPECT_EQ(header(directory.file("chain.snapshot")).closed, 0U);
    trace.seal_snapshot();
  }
  const auto h = header(directory.file("chain.snapshot"));
  EXPECT_EQ(std::string(h.magic, 8), "MCHTRC01");
  EXPECT_EQ(h.version, 1U); EXPECT_EQ(h.kind, 1U);
  EXPECT_EQ(h.record_stride, 216U); EXPECT_EQ(h.header_bytes, 128U);
  EXPECT_EQ(h.capacity, 2U); EXPECT_EQ(h.total, 3U);
  EXPECT_EQ(h.committed, 2U); EXPECT_EQ(h.dropped, 1U);
  EXPECT_EQ(h.overwritten, 0U); EXPECT_EQ(h.closed, 1U);
  std::ifstream input(directory.file("chain.snapshot"), std::ios::binary);
  input.seekg(128);
  std::array<char, 32> stage{};
  input.read(stage.data(), stage.size());
  EXPECT_EQ(std::string(stage.data()), "first");
  input.seekg(128 + 216);
  input.read(stage.data(), stage.size());
  EXPECT_EQ(std::string(stage.data()), "second");
  input.seekg(0, std::ios::end);
  EXPECT_EQ(input.tellg(), std::streampos(128 + 2 * 216));
}

TEST(TraceSnapshot, RawRingPublishesCompletedIoAndOverwriteCounts) {
  SnapshotDirectory directory;
  auto port = std::make_shared<mech::mech_simulation::FakeSerial>();
  {
    mech::mech_bringup::SerialTrace trace(port, 2, nullptr, directory.file("serial.snapshot"));
    ASSERT_TRUE(trace.open());
    for (const std::uint8_t byte : {0x11, 0x22, 0x33})
      ASSERT_EQ(trace.write_all(&byte, 1), mech::mech_control_core::TransportResult::Ok);
    trace.close(); trace.seal_snapshot();
  }
  const auto h = header(directory.file("serial.snapshot"));
  EXPECT_EQ(h.kind, 2U); EXPECT_EQ(h.record_stride, 1064U);
  EXPECT_EQ(h.total, 3U); EXPECT_EQ(h.committed, 2U);
  EXPECT_EQ(h.dropped, 0U); EXPECT_EQ(h.overwritten, 1U); EXPECT_EQ(h.closed, 1U);
  std::ifstream input(directory.file("serial.snapshot"), std::ios::binary);
  input.seekg(128 + 34); char byte = 0; input.get(byte);
  EXPECT_EQ(byte, 0x33);  // newest logical sequence 2 occupies slot 0
  input.seekg(128 + 1064 + 34); input.get(byte);
  EXPECT_EQ(byte, 0x22);
}

TEST(TraceSnapshot, AbruptProcessExitRetainsUnclosedCommittedPrefix) {
  SnapshotDirectory directory;
  const auto child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    try {
      mech::mech_bringup::CommandTrace trace(4, directory.file("chain.snapshot"));
      trace.record("committed", 1, 7, 0.75);
      ::_exit(0);  // no destructors, exactly the missing-seal crash condition
    } catch (...) { ::_exit(1); }
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status)); ASSERT_EQ(WEXITSTATUS(status), 0);
  const auto h = header(directory.file("chain.snapshot"));
  EXPECT_EQ(h.total, 1U); EXPECT_EQ(h.committed, 1U); EXPECT_EQ(h.closed, 0U);
}

TEST(TraceSnapshot, ExistingFileAndSymlinkTraversalAreRejectedWithoutClobber) {
  SnapshotDirectory directory;
  { std::ofstream original(directory.file("chain.snapshot")); original << "existing"; }
  EXPECT_THROW(mech::mech_bringup::CommandTrace(1, directory.file("chain.snapshot")),
               std::runtime_error);
  std::ifstream original(directory.file("chain.snapshot"));
  std::string text; original >> text; EXPECT_EQ(text, "existing");
  ASSERT_EQ(::symlink(directory.path().c_str(), directory.file("alias").c_str()), 0);
  EXPECT_THROW(mech::mech_bringup::admit_trace_snapshots(directory.file("alias"), 1024),
               std::runtime_error);
  EXPECT_THROW(mech::mech_bringup::admit_trace_snapshots("/tmp", 1024),
               std::invalid_argument);
  EXPECT_THROW(mech::mech_bringup::admit_trace_snapshots(directory.path(),
      mech::mech_bringup::kTraceSnapshotBudget + 1), std::invalid_argument);
}
}  // namespace
