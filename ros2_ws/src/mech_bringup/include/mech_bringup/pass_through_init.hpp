#pragma once

#include <array>
#include <cstdint>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_bringup {

// The vendor pass-through init frame: MODE_FDCAN_PASS (0x12) with a
// six-byte zero payload (send_flag=0). Bench-verified on the real board
// (2026-09-02) and identical to what the vendor's own fdcan_init() emits;
// cfg=0x00 vs 0x07 measured identical across all seven channels. The bytes
// are a fixed literal - the CRCs are never recomputed at runtime or in
// tests (recomputing goldens is exactly how the 0.2 N*m golden was
// corrupted once; the literal is the evidence).
inline constexpr std::array<std::uint8_t, 13U> kPassThroughInitFrame{
    0xF7, 0x12, 0x06, 0x00, 0x7D, 0x70, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00};

// Writes the pass-through init frame over an already-open serial port,
// retrying transient WouldBlock up to a bounded number of attempts (the
// same policy PosixCdcSerialPort::send_pass_through_init uses). Must be
// called after open() and before the first try_send/try_receive; the
// transport itself does not send it.
[[nodiscard]] bool send_pass_through_init(
    mech::mech_control_core::CdcSerialPort& serial) noexcept;

}  // namespace mech::mech_bringup
