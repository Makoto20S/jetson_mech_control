#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "../dual_board/diagnostic.hpp"

namespace observer {
using dual_board::Bytes;
using dual_board::Clock;
using mech::mech_control_core::CdcSerialPort;
using mech::mech_control_core::RawCanFrame;

struct Config {
  std::string role{"observe"}, profile{"sequence"};
  unsigned hz{10}, seconds{5}, drain_ms{200}, log_mib{16}, lanes{1};
  unsigned frames_per_id{0};  // 0: hz*seconds, or 4 for four-id.
  std::uint32_t nonce{1};
  bool count_only{false};
  void validate() const;
  unsigned count() const;
  unsigned id_count() const;
};

// Capture schema 2 reuses dual_board::Capture's 16-byte outer record:
// kind:u8, port:u8(0), result:u8, reserved:u8, payload_size:u32le,
// end_absolute_steady_ns:u64le. Payload: begin_absolute_steady_ns:u64le,
// requested_bytes:u32le, returned_bytes:u32le, then raw bytes in full mode.
// Count-only keeps all I/O metadata and write bytes, but omits read bytes.
// Every read_some call,
// including WouldBlock, is recorded. Write records contain requested bytes;
// the production interface cannot reveal partial-write byte counts.
class Capture {
 public:
  explicit Capture(std::size_t limit, bool count_only = false);
  void reserve_record(std::size_t raw_size, std::uint8_t kind = 2) const;
  void append(std::uint8_t kind, std::uint8_t result,
              std::size_t requested, std::size_t returned,
              const std::uint8_t* raw, std::size_t raw_size,
              std::uint64_t begin_ns, std::uint64_t end_ns);
  const Bytes& bytes() const { return storage_.bytes(); }
  bool count_only() const { return count_only_; }
 private:
  bool count_only_;
  dual_board::Capture storage_;
};

struct IdCount {
  std::uint32_t id{0};
  bool extended{false};
  std::uint64_t frames{0};
};

struct Result {
  std::string role, error;
  bool finished{false}, closed{false}, count_only{false};
  std::uint64_t start_ns{0}, ready_ns{0}, end_ns{0};
  std::uint64_t first_frame_ns{0}, last_frame_ns{0};
  std::uint64_t read_calls{0}, write_calls{0}, rx_bytes{0}, rx_frames{0}, tx_frames{0};
  std::uint64_t max_lateness_ns{0}, min_send_interval_ns{0};
  std::vector<IdCount> ids;
  bool complete() const { return finished && closed && error.empty(); }
  bool raw_complete() const { return complete() && !count_only; }
};

std::uint64_t now_ns();
// Fixture frames only. four-id lanes 0..3 = 0x668,0x669,0x2968,0x2969.
// The latter two are synthetic feedback, never device measurements.
RawCanFrame fixture_frame(const Config&, unsigned lane, unsigned sequence);
std::vector<Bytes> fixture_packets(const Config&, unsigned sequence);
// Caller has opened the port. Both roles initialize exactly once. Observe
// then only reads; fixture is an isolated emitter. All exits close the port.
// ready is called after successful initialization, before further I/O.
Result run(CdcSerialPort&, const Config&, Capture&,
           const std::function<bool()>& stop,
           const std::function<void()>& ready = {});
std::string plan_json(const Config&);
std::string result_json(const Result&, std::size_t capture_bytes);
}  // namespace observer
