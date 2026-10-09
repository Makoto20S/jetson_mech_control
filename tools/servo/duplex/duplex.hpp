#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "../observer/observer.hpp"
#include "replay_schedule.hpp"

namespace duplex {
using observer::Bytes;
using observer::Capture;  // Two independent full schema-2 streams; each port byte is 0.
using observer::CdcSerialPort;
using observer::Clock;
using observer::RawCanFrame;

struct Config {
  ReplaySchedule replay;
  bool feedback_sequence{false};
  bool mixed_receive{false};  // Replay family1 feedback also leaves A; B sends no CAN.
  unsigned seconds{60}, hz{500}, feedback_hz{0}, log_mib{64};
  unsigned quiet_ms{2000}, drain_ms{1000};
  std::uint32_t nonce{0};  // Run metadata only; fixed CAN payloads have no nonce.
  bool feedback_reverse{false};  // B sends 105 before 104; counters stay ID ordered.
  unsigned feedback_phase_ms{0};  // B epoch offset: 0 or 5; disabled feedback requires 0.
  unsigned rx_gate_ms{0};  // 0 or 6; 6 is restricted to 2s, 10/10Hz, forward.
  void validate() const;
  unsigned command_ticks() const { return hz * seconds; }
  unsigned feedback_ticks() const { return feedback_hz * seconds; }
};

struct Content {
  std::array<std::uint64_t, 2> received_per_id{};
  std::uint64_t unexpected_ids{0}, payload_mismatches{0}, format_mismatches{0};
};

struct Result {
  observer::Result a, b;  // role="A"/"B", all I/O and all received CAN IDs.
  Content content_a, content_b, content_b_feedback;
  bool mixed_receive{false}, feedback_sequence{false}, feedback_sequence_match{false};
  std::string error;
  bool finished{false}, closed{false}, content_match{false};
  std::uint64_t quiet_begin_ns{0}, quiet_end_ns{0}, epoch_ns{0}, transmit_end_ns{0};
  std::uint64_t quiet_rx_a{0}, quiet_rx_b{0};
  std::array<std::uint64_t, 2> sent_a{}, sent_b{};
  bool complete() const { return finished && closed && error.empty(); }
};

// Exact frames: A sends production mode6 0x668=84.1deg,0x669=95.9deg,
// speed/acceleration=100. B sends explicitly synthetic classic feedback:
// 0x2968 payload03490000ffff2a00 (84.1deg,0ERPM,-.01A,42C,status0),
// 0x2969 payload03be000000252a00 (95.8deg,0ERPM,.37A,42C,status0).
RawCanFrame frame(bool feedback, unsigned lane);
RawCanFrame sequenced_feedback(unsigned lane, unsigned ordinal);
std::array<Bytes, 2> packets(bool feedback);

// Caller has opened both ports. Exactly one fixed initialization write per
// port, then 2s quiet (any CAN RX aborts), then ready callback and common epoch.
// All exits close both ports. Both captures must be full, independently bounded.
// CRC/IO/capacity/timing/stop faults abort; valid CAN content mismatches are
// preserved and collection continues. Complete collection != matching content.
Result run(CdcSerialPort& a, CdcSerialPort& b, const Config&,
           Capture& capture_a, Capture& capture_b,
           const std::function<bool()>& stop,
           const std::function<void()>& ready = {});
std::string plan_json(const Config&);
std::string result_json(const Result&, std::size_t capture_a_bytes,
                        std::size_t capture_b_bytes);
}  // namespace duplex
