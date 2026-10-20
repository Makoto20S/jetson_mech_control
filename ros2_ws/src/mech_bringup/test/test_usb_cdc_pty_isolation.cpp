// Compile this target with PosixCdcSerialPort's source and -Wl,--wrap=write.
// The syscall seam is test-only: successful writes still reach a real PTY.
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <fcntl.h>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include "mech_bringup/command_trace.hpp"
#include "mech_bringup/serial_trace.hpp"
#include "mech_bringup/posix_cdc_serial_port.hpp"
#include "mech_control_core/runtime.hpp"

namespace {
std::deque<int> write_actions;
}
extern "C" ssize_t __real_write(int, const void*, size_t);
extern "C" ssize_t __wrap_write(int fd, const void* bytes, size_t size) {
  if (fd <= 2 || write_actions.empty()) return __real_write(fd, bytes, size);
  const int action = write_actions.front(); write_actions.pop_front();
  if (action < 0) { errno = -action; return -1; }
  if (action == 0) return 0;
  return __real_write(fd, bytes, std::min(size, static_cast<size_t>(action)));
}

namespace {
using namespace mech::mech_control_core;
using namespace mech::mech_bringup;
// Captured 082015 final targets: 104=83.6deg, 105=71.8deg. Neither golden
// uses the encoder under test. RX literals are separate actual board replies.
const std::vector<uint8_t> tx104{0xf7,0x12,0x0e,0,0x0b,0x91,0x60,0x68,6,0,0,0x0c,8,0,0x0c,0xc1,0xa0,0,0x0a,0,0x0a};
const std::vector<uint8_t> tx105{0xf7,0x12,0x0e,0,0x0b,0xde,0x7e,0x69,6,0,0,0x0c,8,0,0x0a,0xf0,0xc8,0,0x0a,0,0x0a};
const std::vector<uint8_t> rx104{0xf7,0x12,0x0e,0,0x0b,0x93,0xf6,0x68,0x29,0,0,0x0c,8,3,0x44,0,0,0xff,0xfa,0x2c,0};
const std::vector<uint8_t> rx105{0xf7,0x12,0x0e,0,0x0b,0x32,0x62,0x69,0x29,0,0,4,8,2,0xcd,0,0,0xff,0xd2,0x2c,0};
RawCanFrame command(uint32_t id, const std::vector<uint8_t>& wire) {
  std::array<uint8_t,kMaxCanPayloadBytes> payload{};
  std::copy_n(wire.begin()+13,8,payload.begin());
  return *RawCanFrame::create(1,*CanId::create(id,CanFrameFormat::Extended),
      CanFrameType::Classic,FrameDirection::Tx,8,payload,
      *MonotonicTime::from_nanoseconds(1));
}
class UsbCdcPtyIsolation : public ::testing::Test {
 protected:
  void SetUp() override {
    master = ::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
    ASSERT_GE(master,0); ASSERT_EQ(::grantpt(master),0); ASSERT_EQ(::unlockpt(master),0);
    std::array<char,256> path{}; ASSERT_EQ(::ptsname_r(master,path.data(),path.size()),0);
    port=std::make_shared<PosixCdcSerialPort>(path.data());
    trace=std::make_shared<CommandTrace>(256); port->set_write_trace_sink(trace.get());
    serial=std::make_unique<SerialTrace>(port,256,trace.get());
    transport=std::make_unique<UsbCdcTransport>(*serial,UsbCdcOptions{1,1000000,64,1024,CdcProtocolVersion{4,8,8}});
    ASSERT_TRUE(transport->open());
  }
  void TearDown() override { write_actions.clear(); transport->close(); ::close(master); }
  std::vector<uint8_t> drain(size_t wanted) {
    std::vector<uint8_t> output; std::array<uint8_t,1024> bytes{};
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(output.size()<wanted && std::chrono::steady_clock::now()<deadline) {
      const auto count=::read(master,bytes.data(),std::min(bytes.size(),wanted-output.size()));
      if(count>0) output.insert(output.end(),bytes.begin(),bytes.begin()+count);
      else if(count<0 && errno!=EAGAIN && errno!=EINTR) break;
      else std::this_thread::yield();
    }
    return output;
  }
  int master{-1}; std::shared_ptr<PosixCdcSerialPort> port;
  std::shared_ptr<CommandTrace> trace; std::unique_ptr<SerialTrace> serial;
  std::unique_ptr<UsbCdcTransport> transport;
};
TEST_F(UsbCdcPtyIsolation, PartialPrefixThenEagainFaultsInsteadOfAllowingPacketReplay) {
  write_actions={5,-EAGAIN};
  EXPECT_EQ(transport->try_send(command(0x668,tx104)),TransportResult::Fault);
  EXPECT_EQ(drain(5),std::vector<uint8_t>(tx104.begin(),tx104.begin()+5));
  EXPECT_EQ(transport->stats().tx_frames,0U);
  std::ostringstream chain;trace->dump(chain);
  EXPECT_NE(chain.str().find("\"result\":5,\"errno\":0,\"requested\":21"),std::string::npos);
  EXPECT_NE(chain.str().find("\"result\":-1,\"errno\":11,\"requested\":16"),std::string::npos);
  std::ostringstream raw;serial->dump(raw);
  // Raw TX is the full requested packet even though the peer received five bytes.
  EXPECT_NE(raw.str().find("f7120e000b9160680600000c08000cc1a0000a000a"),std::string::npos);
}
TEST_F(UsbCdcPtyIsolation, PartialPrefixThenZeroFaultsInsteadOfAllowingPacketReplay) {
  write_actions={5,0};
  EXPECT_EQ(transport->try_send(command(0x668,tx104)),TransportResult::Fault);
  EXPECT_EQ(drain(5),std::vector<uint8_t>(tx104.begin(),tx104.begin()+5));
}
TEST_F(UsbCdcPtyIsolation, NoProgressEagainAndZeroRemainRetryable) {
  for(const int transient:{-EAGAIN,0}) {
    write_actions={transient};
    ASSERT_EQ(transport->try_send(command(0x668,tx104)),TransportResult::WouldBlock);
    std::array<uint8_t,32> bytes{};
    EXPECT_EQ(::read(master,bytes.data(),bytes.size()),-1); EXPECT_EQ(errno,EAGAIN);
    ASSERT_EQ(transport->try_send(command(0x668,tx104)),TransportResult::Ok);
    EXPECT_EQ(drain(tx104.size()),tx104);
  }
}
TEST_F(UsbCdcPtyIsolation, SuccessfulPartialWritesAndEintrKeepExactOffsets) {
  write_actions={-EINTR,5,-EINTR,3,13};
  ASSERT_EQ(transport->try_send(command(0x668,tx104)),TransportResult::Ok);
  EXPECT_EQ(drain(tx104.size()),tx104);
  ASSERT_EQ(transport->try_send(command(0x669,tx105)),TransportResult::Ok);
  EXPECT_EQ(drain(tx105.size()),tx105);
}
TEST_F(UsbCdcPtyIsolation, RuntimePartialWriteFaultStopsBeforeSendingPeerRoute) {
  FrameRouter router(2); BusOwnershipRegistry ownership;
  ASSERT_FALSE(router.add_route({1,{CanFrameFormat::Extended,0x2968,0x1fffffff,CanFrameType::Classic},0}));
  ASSERT_FALSE(router.add_route({2,{CanFrameFormat::Extended,0x2969,0x1fffffff,CanFrameType::Classic},0}));
  BusRuntime bus(1,"pty",*transport,router,ownership,2,2);ASSERT_EQ(bus.start(),RuntimeResult::Ok);
  auto now=*MonotonicTime::from_nanoseconds(1),deadline=*MonotonicTime::from_nanoseconds(1000000);
  ASSERT_EQ(bus.submit(*CommandLease::create(1,1,command(0x668,tx104),now,deadline)),RuntimeResult::Ok);
  ASSERT_EQ(bus.submit(*CommandLease::create(2,1,command(0x669,tx105),now,deadline)),RuntimeResult::Ok);
  write_actions={5,-EAGAIN};
  EXPECT_EQ(bus.transmit(now),RuntimeResult::Fault); EXPECT_EQ(bus.state(),RuntimeState::Fault);
  EXPECT_FALSE(bus.was_sent(1,1)); EXPECT_FALSE(bus.was_sent(2,1));
  EXPECT_EQ(drain(5),std::vector<uint8_t>(tx104.begin(),tx104.begin()+5));
  std::array<uint8_t,128> bytes{};EXPECT_EQ(::read(master,bytes.data(),bytes.size()),-1); EXPECT_EQ(errno,EAGAIN);
  EXPECT_EQ(bus.transmit(now),RuntimeResult::NotRunning);
}
TEST_F(UsbCdcPtyIsolation, HundredThousandSlotReplacementsKeepBothPeerPacketsExact) {
  FrameRouter router(2);BusOwnershipRegistry ownership;
  ASSERT_FALSE(router.add_route({1,{CanFrameFormat::Extended,0x2968,0x1fffffff,CanFrameType::Classic},0}));
  ASSERT_FALSE(router.add_route({2,{CanFrameFormat::Extended,0x2969,0x1fffffff,CanFrameType::Classic},0}));
  BusRuntime bus(1,"pty",*transport,router,ownership,2,2);ASSERT_EQ(bus.start(),RuntimeResult::Ok);
  const auto first=command(0x668,tx104),second=command(0x669,tx105);
  for(uint64_t cycle=1;cycle<=100000;++cycle) {
    const auto now=*MonotonicTime::from_nanoseconds(cycle),deadline=*MonotonicTime::from_nanoseconds(cycle+1000000);
    for(const uint16_t route: cycle%2 ? std::array<uint16_t,2>{1,2} : std::array<uint16_t,2>{2,1}) {
      auto frame=route==1?first:second;auto poison=frame;poison.payload[0]=0x7f;
      ASSERT_EQ(bus.submit(*CommandLease::create(route,cycle*2-1,poison,now,deadline)),RuntimeResult::Ok);
      ASSERT_EQ(bus.submit(*CommandLease::create(route,cycle*2,frame,now,deadline)),RuntimeResult::Ok);
    }
    ASSERT_EQ(bus.transmit(now),RuntimeResult::Ok)<<cycle;
    auto expected=cycle%2?tx104:tx105;const auto& tail=cycle%2?tx105:tx104;
    expected.insert(expected.end(),tail.begin(),tail.end());
    ASSERT_EQ(drain(42),expected)<<cycle;
    ASSERT_TRUE(bus.was_sent(1,cycle*2));ASSERT_TRUE(bus.was_sent(2,cycle*2));
  }
  EXPECT_EQ(transport->stats().tx_frames,200000U); EXPECT_EQ(transport->stats().errors,0U);
}
TEST_F(UsbCdcPtyIsolation, FragmentedAndCoalescedFeedbackKeepIdsWithTheirOwnPayloads) {
  uint64_t received_count=0;
  for(size_t round=0;round<5000;++round) {
    auto pair=rx104;pair.insert(pair.end(),rx105.begin(),rx105.end());
    // Fragment at every possible location over successive pairs, including
    // single-byte arrivals and a complete two-packet kernel read.
    const size_t fragment=1+round%pair.size();
    size_t offset=0,index=0;
    while(offset<pair.size()) {
      const size_t length=std::min(fragment,pair.size()-offset);
      ASSERT_EQ(::write(master,pair.data()+offset,length),static_cast<ssize_t>(length));offset+=length;
      for(int attempts=0;attempts<100;++attempts) {
        RawCanFrame frame{};const auto result=transport->try_receive(frame);
        if(result==TransportResult::WouldBlock) break;
        ASSERT_EQ(result,TransportResult::Ok);
        ASSERT_LT(index,2U);const auto& want=index==0?rx104:rx105;
        EXPECT_EQ(frame.id.value,index==0?0x2968U:0x2969U);
        ASSERT_TRUE(std::equal(frame.payload.begin(),frame.payload.begin()+8,want.begin()+13));
        ++index;++received_count;
      }
    }
    // PTY delivery can become visible just after a nonblocking read.
    for(int attempts=0;index<2 && attempts<1000;++attempts) {
      RawCanFrame frame{};const auto result=transport->try_receive(frame);
      if(result==TransportResult::WouldBlock) {std::this_thread::yield();continue;}
      ASSERT_EQ(result,TransportResult::Ok);const auto& want=index==0?rx104:rx105;
      EXPECT_EQ(frame.id.value,index==0?0x2968U:0x2969U);
      ASSERT_TRUE(std::equal(frame.payload.begin(),frame.payload.begin()+8,want.begin()+13));++index;++received_count;
    }
    ASSERT_EQ(index,2U)<<round;
    // Mix both directions on the same actual port; RX storage must not leak
    // into the next stack-local TX encoding.
    ASSERT_EQ(transport->try_send(command(0x669,tx105)),TransportResult::Ok);
    ASSERT_EQ(drain(tx105.size()),tx105);
  }
  EXPECT_EQ(received_count,10000U);EXPECT_EQ(transport->stats().rx_dropped,0U);
}
TEST_F(UsbCdcPtyIsolation, ReopenClearsPartialReceiveAndDoesNotReuseOldPayload) {
  ASSERT_EQ(::write(master,rx104.data(),10),10);
  RawCanFrame frame{};
  for(int attempt=0;attempt<10;++attempt) ASSERT_EQ(transport->try_receive(frame),TransportResult::WouldBlock);
  transport->close();ASSERT_TRUE(transport->open());
  ASSERT_EQ(::write(master,rx105.data(),rx105.size()),static_cast<ssize_t>(rx105.size()));
  bool received=false;
  for(int attempt=0;attempt<1000 && !received;++attempt) {
    const auto result=transport->try_receive(frame);
    if(result==TransportResult::Ok) received=true;
    else {ASSERT_EQ(result,TransportResult::WouldBlock);std::this_thread::yield();}
  }
  ASSERT_TRUE(received);EXPECT_EQ(frame.id.value,0x2969U);
  ASSERT_TRUE(std::equal(frame.payload.begin(),frame.payload.begin()+8,rx105.begin()+13));
  ASSERT_EQ(transport->try_send(command(0x668,tx104)),TransportResult::Ok);EXPECT_EQ(drain(tx104.size()),tx104);
}
}  // namespace
