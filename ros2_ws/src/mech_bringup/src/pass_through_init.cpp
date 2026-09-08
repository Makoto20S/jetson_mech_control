#include "mech_bringup/pass_through_init.hpp"

namespace mech::mech_bringup {

bool send_pass_through_init(
    mech::mech_control_core::CdcSerialPort& serial) noexcept {
  if (!serial.is_open()) {
    return false;
  }
  mech::mech_control_core::TransportResult result;
  std::size_t attempts = 0U;
  do {
    result = serial.write_all(kPassThroughInitFrame.data(),
                              kPassThroughInitFrame.size());
    ++attempts;
  } while (result == mech::mech_control_core::TransportResult::WouldBlock &&
           attempts < 100U);
  return result == mech::mech_control_core::TransportResult::Ok;
}

}  // namespace mech::mech_bringup
