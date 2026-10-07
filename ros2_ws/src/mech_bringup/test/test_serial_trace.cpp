#include <gtest/gtest.h>
#include <sstream>
#include "mech_bringup/serial_trace.hpp"
#include "mech_simulation/fake_serial.hpp"
TEST(SerialTraceTest, PreservesIoAndReportsRingOverwrite) {
 using namespace mech::mech_control_core;
 auto serial=std::make_shared<mech::mech_simulation::FakeSerial>();
 mech::mech_bringup::SerialTrace trace(serial, 2);
 ASSERT_TRUE(trace.open());
 const std::uint8_t bytes[]={1,2,3};
 ASSERT_TRUE(trace.write_all(bytes,3)==TransportResult::Ok);
 ASSERT_TRUE(serial->take_tx()==std::vector<std::uint8_t>({1,2,3}));
 serial->force_next_write(TransportResult::Disconnected);
 ASSERT_TRUE(trace.write_all(bytes,3)==TransportResult::Disconnected);
 ASSERT_TRUE(serial->inject_rx({4,5})); std::uint8_t data[8];std::size_t n=0;
 ASSERT_TRUE(trace.read_some(data,8,n)==TransportResult::Ok && n==2 && data[0]==4);
 ASSERT_TRUE(trace.read_some(data,8,n)==TransportResult::WouldBlock);
 trace.close(); ASSERT_TRUE(!serial->is_open());
 std::ostringstream out;trace.dump(out);const auto s=out.str();
 ASSERT_TRUE(s.find("\"overwritten\":1")!=std::string::npos);
 ASSERT_TRUE(s.find("\"hex\":\"0405\"")!=std::string::npos);
 ASSERT_TRUE(s.find("\"direction\":\"tx\"")!=std::string::npos);
 ASSERT_TRUE(s.find("\"direction\":\"rx\"")!=std::string::npos);
}
