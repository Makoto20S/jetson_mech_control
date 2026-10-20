#pragma once

#include <cstdint>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_bringup {

inline mech_control_core::UsbCdcOptions seven_channel_gateway_options(
    std::uint16_t logical_bus, std::uint32_t nominal_bitrate_hz) noexcept {
  mech_control_core::UsbCdcOptions options{};
  options.logical_bus = logical_bus;
  options.nominal_bitrate_hz = nominal_bitrate_hz;
  options.receive_queue_capacity = 128U;
  options.verified_board_version = {4U, 8U, 8U};
  return options;
}

}  // namespace mech::mech_bringup
