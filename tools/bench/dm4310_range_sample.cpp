// Standalone, bounded read-only endpoint sampler. Never enables a motor.
#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "mech_control_core/posix_cdc_serial_port.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

using namespace mech::mech_control_core;
using namespace mech::mech_protocol_damiao;

int main(int argc, char** argv) {
  if (argc != 4 || std::string(argv[1]) != "--confirmed-readonly" ||
      std::string(argv[2]).rfind("/dev/ttyACM", 0) != 0 ||
      (std::string(argv[3]) != "30" && std::string(argv[3]) != "50")) {
    std::fprintf(stderr, "Usage: %s --confirmed-readonly /dev/ttyACMx 30|50\n"
        "Requires current onsite authorization: ordinary DM4310 IDs1/2, "
        "feedback4/5, declared gateway4.8.8/CAN1Mbps, motors disabled.\n", argv[0]);
    return 2;
  }
  const double velocity_range = std::string(argv[3]) == "50" ? 50.0 : 30.0;
  PosixCdcSerialPort serial{argv[2]};
  UsbCdcTransport transport{serial, UsbCdcOptions{1U,1000000U,64U,1024U,
                                               CdcProtocolVersion{4,8,8}}};
  if (!transport.open()) return 3;
  if (!serial.send_pass_through_init()) return 4;
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  std::array<unsigned, 2> counts{};
  // Twenty sequential rounds, 50ms per device: 2 seconds, 10Hz per axis.
  for (unsigned round = 0; round < 20; ++round) {
    for (unsigned id : {1U, 2U}) {
      std::array<std::uint8_t,kMaxCanPayloadBytes> bytes{};
      bytes[0] = id;
      bytes[2] = 0xCC;  // Query status only; no other motor opcode exists here.
      auto query = RawCanFrame::create(1U,
          *CanId::create(0x7FFU,CanFrameFormat::Standard),
          CanFrameType::Classic,FrameDirection::Tx,8U,bytes,
          *MonotonicTime::from_nanoseconds(0));
      if (transport.try_send(*query) != TransportResult::Ok) return 5;
      const auto deadline = Clock::now() + std::chrono::milliseconds(50);
      bool received = false;
      while (Clock::now() < deadline) {
        RawCanFrame rx{};
        const auto result = transport.try_receive(rx);
        if (result == TransportResult::WouldBlock || result == TransportResult::Invalid) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        if (result != TransportResult::Ok) return 6;
        MitFeedback fb{};
        if (!decode_mit_feedback(rx,1U,id+3U,id,MitLimits{12.5,velocity_range,10.0},fb)) {
          std::fprintf(stderr,"Unexpected frame can=%u; abort endpoint capture\n",rx.id.value);
          return 7;
        }
        // No queue duplicates or late frames may count as a new response.
        if (received || fb.raw_fault != 0) return 8;
        received = true;
        ++counts[id-1];
        const double t = std::chrono::duration<double>(Clock::now()-start).count();
        std::printf("{\"t_s\":%.9f,\"motor_id\":%u,\"feedback_can_id\":%u,"
                    "\"status\":%u,\"position_rad\":%.12f,"
                    "\"velocity_rad_s\":%.12f,\"raw\":\"",
                    t,id,rx.id.value,fb.raw_fault,fb.position_rad,fb.velocity_rad_s);
        for (unsigned j=0;j<rx.payload_size;++j) std::printf("%02X",rx.payload[j]);
        std::puts("\"}");
        std::fflush(stdout);
      }
      if (!received) return 9;
    }
  }
  transport.close();
  return counts[0] == 20 && counts[1] == 20 ? 0 : 10;
}
