#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace dual_board {
using namespace mech::mech_control_core;
using Bytes = std::vector<std::uint8_t>;
using Clock = std::chrono::steady_clock;

struct Config {
  unsigned hz{10}, seconds{2}, lanes{1}, drain_ms{1000}, log_mib{16};
  std::uint32_t nonce{1};
  std::string profile{"sequence"}, packing{"separate"};
  void validate() const;
  unsigned count() const { return hz * seconds; }
};

RawCanFrame frame(const Config&, unsigned lane, unsigned sequence);
Bytes encode(const RawCanFrame&);
// Batch packing is diagnostic-only. Individual record bytes come from the
// production encoder; only the new outer header/CRCs are constructed here.
Bytes envelope(const Bytes& records);
std::vector<Bytes> packets(const Config&, unsigned sequence);
std::string hex(const std::uint8_t*, std::size_t);

// Strict bounded stream splitter: fail on corruption, never silently resync.
// Payload decoding, including observed firmware prefix, uses UsbCdcCodec.
class Parser {
 public:
  void feed(const std::uint8_t*, std::size_t,
            const std::function<void(const RawCanFrame&)>&);
  void finish() const;
 private:
  std::array<std::uint8_t, UsbCdcCodec::kMaxPayload + 7> pending_{};
  std::size_t used_{0}, target_{7};
  bool failed_{false};
};

struct Lane {
  unsigned sent{0}, received{0}, matched{0}, duplicate{0}, out_of_order{0};
  unsigned mismatch{0}, highest{0};
  bool has_highest{false};
  std::vector<bool> seen;
};

class Matcher {
 public:
  explicit Matcher(Config);
  void sent(unsigned lane);
  // Returns an error category, or empty on a matching sample.
  std::string receive(const RawCanFrame&);
  unsigned missing(unsigned lane) const;
  bool complete() const;
  Config config;
  std::array<Lane, 2> lanes;
  unsigned unexpected_id{0};
};

// In-memory binary evidence, capped including record headers. Export only
// after ports are closed. Every record: kind:u8, port:u8, result:u8,
// reserved:u8, payload_size:u32le, elapsed_ns:u64le, payload bytes.
class Capture {
 public:
  explicit Capture(std::size_t limit);
  void append(std::uint8_t kind, std::uint8_t port, std::uint8_t result,
              const std::uint8_t*, std::size_t, std::uint64_t ns);
  void reserve_record(std::size_t payload) const;
  const Bytes& bytes() const { return bytes_; }
 private:
  std::size_t limit_;
  Bytes bytes_;
};

struct Result {
  explicit Result(const Config& c) : matcher(c) {}
  Matcher matcher;
  std::string error;
  bool finished{false};
  std::uint64_t max_lateness_ns{0};
  std::uint64_t min_send_interval_ns{0}, elapsed_ns{0};
  bool complete() const { return finished && error.empty() && matcher.complete(); }
};

// Ports are already opened by the caller. They are ALWAYS closed before
// return, including capacity, interruption, I/O, decode and timing failures.
Result run(CdcSerialPort& tx, CdcSerialPort& rx, const Config&, Capture&,
           const std::function<bool()>& stop);
std::string plan_json(const Config&);
std::string result_json(const Result&, std::size_t capture_bytes);
}  // namespace dual_board
