#include "mech_bringup/pass_through_init.hpp"

namespace mech::mech_bringup {

bool send_pass_through_init(
    mech::mech_control_core::CdcSerialPort& serial) noexcept {
  return mech::mech_control_core::initialize_usb_cdc_pass_through(serial);
}

}  // namespace mech::mech_bringup
