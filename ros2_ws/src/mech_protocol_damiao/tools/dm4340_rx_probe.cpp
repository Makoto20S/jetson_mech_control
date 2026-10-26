#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string_view>
#include <thread>

#include "mech_control_core/posix_cdc_serial_port.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

namespace {

bool parse_number(std::string_view text, std::uint32_t& value) {
  int base = 10;
  if (text.substr(0U, 2U) == "0x" || text.substr(0U, 2U) == "0X") {
    text.remove_prefix(2U);
    base = 16;
  }
  if (text.empty()) return false;
  const auto result = std::from_chars(text.data(), text.data() + text.size(),
                                      value, base);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

bool parse_version(std::string_view text,
                   mech::mech_control_core::CdcProtocolVersion& version) {
  const auto first = text.find('.');
  const auto second = first == text.npos ? text.npos : text.find('.', first + 1U);
  if (first == text.npos || second == text.npos) return false;
  std::uint32_t major = 0U, minor = 0U, patch = 0U;
  if (!parse_number(text.substr(0U, first), major) ||
      !parse_number(text.substr(first + 1U, second - first - 1U), minor) ||
      !parse_number(text.substr(second + 1U), patch) || major > 255U ||
      minor > 255U || patch > 255U) {
    return false;
  }
  version = {static_cast<std::uint8_t>(major),
             static_cast<std::uint8_t>(minor),
             static_cast<std::uint8_t>(patch)};
  return true;
}

void usage(const char* program) {
  std::fprintf(stderr,
               "Usage: %s /dev/ttyACMx seconds(1..30) board_version "
               "[feedback_can_id motor_id]\n"
               "Raw receive works without IDs; decode uses SDK reference "
               "ranges only. No motor CAN frame is sent.\n",
               program);
}

}  // namespace

int main(int argc, char** argv) {
  using namespace mech::mech_control_core;
  using namespace mech::mech_protocol_damiao;
  if (argc != 4 && argc != 6) {
    usage(argv[0]);
    return 2;
  }
  const std::string_view path{argv[1]};
  std::uint32_t seconds = 0U, feedback_id = 0U, motor_id = 0U;
  CdcProtocolVersion version{};
  if (path.substr(0U, 11U) != "/dev/ttyACM" ||
      !parse_number(argv[2], seconds) || seconds < 1U || seconds > 30U ||
      !parse_version(argv[3], version) ||
      !UsbCdcCodec::supports_version(version) ||
      (argc == 6 && (!parse_number(argv[4], feedback_id) ||
                     feedback_id > kMaxStandardCanId ||
                     !parse_number(argv[5], motor_id) || motor_id > 15U))) {
    usage(argv[0]);
    return 2;
  }

  PosixCdcSerialPort serial{argv[1]};
  UsbCdcTransport transport{serial, UsbCdcOptions{
                                        1U, 0U, 64U, 1024U, version}};
  if (!transport.open()) {
    std::fprintf(stderr, "Cannot open %s (permissions, busy, or missing).\n",
                 argv[1]);
    return 1;
  }
  if (!serial.send_pass_through_init()) {
    std::fprintf(stderr, "Gateway receive-pass init failed.\n");
    transport.close();
    return 1;
  }

  std::printf("Listening on %s for %u s; gateway init only, no motor TX.\n",
              argv[1], seconds);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(seconds);
  std::uint64_t decoded = 0U;
  while (std::chrono::steady_clock::now() < deadline) {
    RawCanFrame frame{};
    const auto result = transport.try_receive(frame);
    if (result == TransportResult::WouldBlock) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    if (result == TransportResult::Invalid) continue;
    if (result != TransportResult::Ok) {
      std::fprintf(stderr, "Gateway receive failed (result=%u).\n",
                   static_cast<unsigned>(result));
      transport.close();
      return 1;
    }
    std::printf("RX %s id=0x%X %s dlc=%u data=",
                frame.id.format == CanFrameFormat::Standard ? "std" : "ext",
                frame.id.value,
                frame.type == CanFrameType::Classic ? "classic" : "fd",
                frame.payload_size);
    for (std::uint8_t i = 0U; i < frame.payload_size; ++i) {
      std::printf("%02X", frame.payload[i]);
    }
    if (argc == 6) {
      MitFeedback feedback{};
      if (decode_mit_feedback(frame, 1U, feedback_id,
                              static_cast<std::uint8_t>(motor_id),
                              kSdkDm4340Limits, feedback)) {
        ++decoded;
        std::printf(" pos=%.3f rad vel=%.3f rad/s torque=%.3f Nm "
                    "fault=0x%X", feedback.position_rad,
                    feedback.velocity_rad_s, feedback.torque_nm,
                    feedback.raw_fault);
      }
    }
    std::putchar('\n');
  }
  const auto stats = transport.stats();
  transport.close();
  std::printf("Summary: raw=%llu decoded=%llu dropped=%llu motor_tx=%llu\n",
              static_cast<unsigned long long>(stats.rx_frames),
              static_cast<unsigned long long>(decoded),
              static_cast<unsigned long long>(stats.rx_dropped),
              static_cast<unsigned long long>(stats.tx_frames));
  return 0;
}
