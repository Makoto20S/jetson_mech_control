#include <gtest/gtest.h>
#include <sstream>
#include "mech_bringup/command_trace.hpp"

TEST(CommandTraceTest, OverflowIsExplicitAndPreservesActivationEvidence) {
  mech::mech_bringup::CommandTrace trace(2);
  trace.record("interface", 0, 1, 1.25);
  trace.record("dispatch", 0, 1, 1.25);
  trace.record("prepared", 0, 1, 1.25);
  std::ostringstream out;
  trace.dump(out);
  EXPECT_NE(out.str().find("\"dropped\":1"), std::string::npos);
  EXPECT_NE(out.str().find("\"stage\":\"interface\""), std::string::npos);
  EXPECT_EQ(out.str().find("\"stage\":\"prepared\""), std::string::npos);
}

TEST(CommandTraceTest, CopiesBytesAtBoundaryRatherThanRetainingMutableBuffer) {
  mech::mech_bringup::CommandTrace trace(8);
  std::uint8_t bytes[]{0xf7, 0x12, 0x68};
  trace.bytes("serial_request", bytes, 3, 0, 3);
  bytes[2] = 0x69;
  trace.bytes("syscall_write", bytes + 2, 1, 1, 1);
  std::ostringstream out;
  trace.dump(out);
  EXPECT_NE(out.str().find("\"hex\":\"f71268\""), std::string::npos);
  EXPECT_NE(out.str().find("\"hex\":\"69\""), std::string::npos);
}
