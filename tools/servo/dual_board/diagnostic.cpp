#include "diagnostic.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "mech_bringup/pass_through_init.hpp"
#include "mech_protocol_cubemars/ak30_servo_wire.hpp"

namespace dual_board {
namespace {
std::uint8_t crc8(const std::uint8_t* p, std::size_t n) {
  std::uint8_t c = 0xff;
  while (n--) {
    c ^= *p++;
    for (unsigned b = 0; b < 8; ++b) c = (c >> 1) ^ ((c & 1) ? 0x8c : 0);
  }
  return c;
}
std::uint16_t crc16(const Bytes& p) {
  std::uint16_t c = 0xffff;
  for (auto v : p) {
    c ^= v;
    for (unsigned b = 0; b < 8; ++b) c = (c >> 1) ^ ((c & 1) ? 0x8408 : 0);
  }
  return c;
}
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
std::uint64_t elapsed(Clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now()-start).count();
}
std::string json_string(const std::string& value) {
  std::string out{"\""};
  const char* alphabet = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') { out += '\\'; out += static_cast<char>(c); }
    else if (c < 0x20) {
      out += "\\u00"; out += alphabet[c >> 4]; out += alphabet[c & 15];
    } else out += static_cast<char>(c);
  }
  return out + '"';
}
}  // namespace

void Config::validate() const {
  require(hz >= 1 && hz <= 500 && seconds >= 1 && seconds <= 120,
          "rate/duration outside 1..500 Hz and 1..120 seconds");
  require(lanes == 1 || lanes == 2, "lanes must be 1 or 2");
  require(drain_ms >= 200 && drain_ms <= 5000, "drain outside 200..5000 ms");
  require(log_mib >= 1 && log_mib <= 64, "capture outside 1..64 MiB");
  require(profile == "sequence" || profile == "constant" || profile == "mode6",
          "unknown profile");
  require(packing == "separate" || packing == "joined" || packing == "batch",
          "unknown packing");
  require(lanes == 2 || packing == "separate", "packing comparison requires two IDs");
}

RawCanFrame frame(const Config& c, unsigned lane, unsigned sequence) {
  c.validate();
  require(lane < c.lanes && sequence < c.count(), "frame index out of range");
  auto now = *MonotonicTime::from_nanoseconds(1);
  RawCanFrame f{};
  if (c.profile == "mode6") {
    // Fixed real mode6 targets, not sequence markers disguised as positions.
    require(mech::mech_protocol_cubemars::encode_servo_position_speed(
        104 + static_cast<int>(lane), {lane == 0 ? 84.1 : 90.9, 100, 100}, 1, now, f),
        "mode6 encode failed");
    return f;
  }
  std::array<std::uint8_t, kMaxCanPayloadBytes> data{};
  for (unsigned i = 0; i < 4; ++i) data[i] = static_cast<std::uint8_t>(c.nonce >> (8*i));
  const auto seq = c.profile == "sequence" ? sequence : 0;
  for (unsigned i = 0; i < 3; ++i) data[4+i] = static_cast<std::uint8_t>(seq >> (8*i));
  data[7] = lane == 0 ? 0xa4 : 0xb5;
  return *RawCanFrame::create(1, *CanId::create(0x1fff0101U + lane,
      CanFrameFormat::Extended), CanFrameType::Classic, FrameDirection::Tx, 8, data, now);
}

Bytes encode(const RawCanFrame& f) {
  std::array<std::uint8_t, 528> bytes{};
  std::size_t size = 0;
  require(UsbCdcCodec::encode(f, bytes, size), "production encode failed");
  return Bytes(bytes.begin(), bytes.begin()+size);
}

Bytes envelope(const Bytes& records) {
  require(!records.empty() && records.size() <= UsbCdcCodec::kMaxPayload,
          "batch payload outside codec limit");
  // Allocate the complete envelope once. GCC 11 can diagnose the equivalent
  // seven-byte vector initializer followed by insert as an overread in its
  // inlined vector relocation path.
  Bytes out(records.size()+7, 0);
  out[0] = 0xf7;
  out[1] = 0x12;
  out[2] = static_cast<std::uint8_t>(records.size());
  out[3] = static_cast<std::uint8_t>(records.size() >> 8);
  out[4] = crc8(out.data()+1, 3);
  const auto crc = crc16(records);
  out[5] = static_cast<std::uint8_t>(crc);
  out[6] = static_cast<std::uint8_t>(crc >> 8);
  std::copy(records.begin(), records.end(), out.begin()+7);
  return out;
}

std::vector<Bytes> packets(const Config& c, unsigned sequence) {
  std::vector<Bytes> out{encode(frame(c, 0, sequence))};
  if (c.lanes == 1) return out;
  auto second = encode(frame(c, 1, sequence));
  if (c.packing == "separate") out.push_back(second);
  else if (c.packing == "joined") out[0].insert(out[0].end(), second.begin(), second.end());
  else {
    Bytes payload(out[0].begin()+7, out[0].end());
    payload.insert(payload.end(), second.begin()+7, second.end());
    out[0] = envelope(payload);
  }
  return out;
}

void Parser::feed(const std::uint8_t* data, std::size_t size,
                  const std::function<void(const RawCanFrame&)>& consume) {
  require(!failed_, "parser_failed");
  try {
    require(size == 0 || data != nullptr, "parser_null_input");
    for (std::size_t i = 0; i < size; ++i) {
      require(used_ < pending_.size(), "parser_capacity");
      pending_[used_++] = data[i];
      if (used_ == 7) {
        require(pending_[0] == 0xf7 && pending_[1] == 0x12, "unexpected_envelope");
        require(pending_[4] == crc8(pending_.data()+1, 3), "header_crc");
        const auto n = pending_[2] | (static_cast<unsigned>(pending_[3]) << 8);
        require(n >= 6 && n <= UsbCdcCodec::kMaxPayload, "payload_length");
        target_ = n+7;
      }
      if (used_ == target_ && used_ > 7) {
        CdcFrameBatch batch;
        require(UsbCdcCodec::decode(pending_.data(), used_, 1,
            *MonotonicTime::from_nanoseconds(1), batch), "payload_crc_or_frame_shape");
        used_ = 0; target_ = 7;
        for (std::size_t k = 0; k < batch.size; ++k) consume(batch.frames[k]);
      }
    }
  } catch (...) {
    failed_ = true;
    throw;
  }
}
void Parser::finish() const {
  require(!failed_, "parser_failed");
  require(used_ == 0, "truncated_packet");
}

Matcher::Matcher(Config c) : config(c) {
  c.validate();
  for (unsigned i = 0; i < c.lanes; ++i) lanes[i].seen.resize(c.count(), false);
}
void Matcher::sent(unsigned lane) {
  require(lane < config.lanes && lanes[lane].sent < config.count(), "send_count");
  ++lanes[lane].sent;
}
std::string Matcher::receive(const RawCanFrame& f) {
  unsigned lane = 0;
  for (; lane < config.lanes; ++lane) if (f.id.value == frame(config, lane, 0).id.value) break;
  if (lane == config.lanes) { ++unexpected_id; return "unexpected_id"; }
  auto& s = lanes[lane]; ++s.received;
  auto expected = frame(config, lane, 0);
  if (f.logical_bus != expected.logical_bus || f.id.format != expected.id.format ||
      f.type != expected.type || f.direction != FrameDirection::Rx ||
      f.remote_request || f.error_frame || f.bitrate_switch || f.payload_size != 8) {
    ++s.mismatch; return "data_or_frame_mismatch";
  }
  unsigned sequence = config.profile == "sequence" ?
      (f.payload[4] | (static_cast<unsigned>(f.payload[5]) << 8) |
       (static_cast<unsigned>(f.payload[6]) << 16)) : 0;
  if (sequence >= s.sent) { ++s.mismatch; return "unsent_or_invalid_sequence"; }
  expected = frame(config, lane, sequence);
  if (!std::equal(expected.payload.begin(), expected.payload.begin()+8, f.payload.begin())) {
    ++s.mismatch; return "data_or_frame_mismatch";
  }
  if (config.profile == "sequence") {
    if (s.seen[sequence]) { ++s.duplicate; return "duplicate"; }
    s.seen[sequence] = true; ++s.matched;
    if (s.has_highest && sequence < s.highest) { ++s.out_of_order; return "out_of_order"; }
    s.highest = sequence; s.has_highest = true;
  } else {
    if (s.matched >= s.sent) { ++s.duplicate; return "excess_fixed_payload"; }
    ++s.matched;
  }
  return {};
}
unsigned Matcher::missing(unsigned lane) const {
  require(lane < config.lanes, "lane_index");
  return lanes[lane].sent - lanes[lane].matched;
}
bool Matcher::complete() const {
  if (unexpected_id) return false;
  for (unsigned i = 0; i < config.lanes; ++i) {
    const auto& s = lanes[i];
    if (s.sent != config.count() || missing(i) || s.duplicate || s.mismatch || s.out_of_order)
      return false;
  }
  return true;
}

Capture::Capture(std::size_t limit) : limit_(limit) {
  require(limit <= 64U*1024U*1024U, "capture_limit");
  bytes_.reserve(limit);
}
void Capture::reserve_record(std::size_t payload) const {
  require(payload <= 1024 && bytes_.size() <= limit_ &&
      payload + 16 <= limit_ - bytes_.size(), "capture_capacity");
}
void Capture::append(std::uint8_t kind, std::uint8_t port, std::uint8_t result,
                     const std::uint8_t* data, std::size_t size, std::uint64_t ns) {
  reserve_record(size);
  require(size == 0 || data != nullptr, "capture_null_input");
  bytes_.insert(bytes_.end(), {kind, port, result, 0});
  for (unsigned i = 0; i < 4; ++i) bytes_.push_back(static_cast<std::uint8_t>(size >> (i*8)));
  for (unsigned i = 0; i < 8; ++i) bytes_.push_back(static_cast<std::uint8_t>(ns >> (i*8)));
  if (size) bytes_.insert(bytes_.end(), data, data+size);
}

Result run(CdcSerialPort& tx, CdcSerialPort& rx, const Config& c, Capture& capture,
           const std::function<bool()>& stop) {
  struct Close { CdcSerialPort& a; CdcSerialPort& b; ~Close() { a.close(); b.close(); } } close{tx,rx};
  Result result(c);
  const auto start = Clock::now();
  try {
    require(&tx != &rx, "ports_must_be_distinct");
    require(tx.is_open() && rx.is_open(), "ports_not_open");
    auto write = [&](CdcSerialPort& port, unsigned index, const Bytes& bytes) {
      require(!stop || !stop(), "interrupted");
      capture.reserve_record(bytes.size());
      const auto outcome = port.write_all(bytes.data(), bytes.size());
      capture.append(1, index, static_cast<std::uint8_t>(outcome), bytes.data(), bytes.size(), elapsed(start));
      require(outcome == TransportResult::Ok, "write_incomplete_no_retry");
    };
    const auto& init = mech::mech_bringup::kPassThroughInitFrame;
    const Bytes init_bytes(init.begin(), init.end());
    write(rx, 1, init_bytes); write(tx, 0, init_bytes);
    Parser parser;
    auto pump = [&] {
      require(!stop || !stop(), "interrupted");
      std::array<std::uint8_t, 1024> bytes{};
      // Bounded reads prevent a continuously busy peer from starving TX/timeout.
      for (unsigned index = 0; index < 2; ++index) {
        auto& port = index == 0 ? tx : rx;
        for (unsigned k = 0; k < 8; ++k) {
          capture.reserve_record(bytes.size());
          std::size_t n = 0;
          const auto outcome = port.read_some(bytes.data(), bytes.size(), n);
          require(n <= bytes.size(), "read_size");
          if (outcome == TransportResult::WouldBlock && n != 0) {
            capture.append(2, index, static_cast<std::uint8_t>(outcome), bytes.data(), n, elapsed(start));
            throw std::runtime_error("read_would_block_with_bytes");
          }
          if (outcome == TransportResult::WouldBlock || (outcome == TransportResult::Ok && n == 0)) break;
          capture.append(2, index, static_cast<std::uint8_t>(outcome), bytes.data(), n, elapsed(start));
          require(outcome == TransportResult::Ok, "read_fault");
          require(index == 1, "unexpected_sender_rx");
          parser.feed(bytes.data(), n, [&](const RawCanFrame& f) {
            const auto error = result.matcher.receive(f);
            if (!error.empty()) throw std::runtime_error(error);
          });
        }
      }
    };
    // Receive-only baseline after init; unexpected traffic aborts before CAN TX.
    const auto quiet_end = Clock::now()+std::chrono::milliseconds(100);
    while (Clock::now() < quiet_end) { pump(); std::this_thread::sleep_for(std::chrono::microseconds(100)); }
    parser.finish();
    const auto epoch = Clock::now();
    const auto period = std::chrono::nanoseconds(1000000000LL/c.hz);
    Clock::time_point last_send{};
    for (unsigned sequence = 0; sequence < c.count(); ++sequence) {
      auto group = packets(c, sequence);
      const auto deadline = epoch+period*sequence;
      while (Clock::now() < deadline) { pump(); std::this_thread::sleep_for(std::chrono::microseconds(100)); }
      const auto send_start = Clock::now();
      const auto late = std::chrono::duration_cast<std::chrono::nanoseconds>(send_start-deadline).count();
      result.max_lateness_ns = std::max(result.max_lateness_ns, static_cast<std::uint64_t>(late));
      // Small jitter follows the fixed epoch. Never compensate for a missed
      // half-period by issuing a shortened burst of queued samples.
      require(late < period.count()/2, "scheduler_overrun_no_catchup");
      if (sequence != 0) {
        const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(send_start-last_send).count();
        result.min_send_interval_ns = result.min_send_interval_ns == 0 ?
            static_cast<std::uint64_t>(interval) : std::min(result.min_send_interval_ns,
            static_cast<std::uint64_t>(interval));
        require(interval >= period.count()/2, "scheduler_overrun_no_catchup");
      }
      last_send = send_start;
      for (unsigned i = 0; i < group.size(); ++i) {
        write(tx, 0, group[i]);
        if (c.packing == "separate") result.matcher.sent(i);
        else for (unsigned lane = 0; lane < c.lanes; ++lane) result.matcher.sent(lane);
      }
      pump();
    }
    const auto drain_end = epoch+std::chrono::seconds(c.seconds)+std::chrono::milliseconds(c.drain_ms);
    while (Clock::now() < drain_end) { pump(); std::this_thread::sleep_for(std::chrono::microseconds(100)); }
    parser.finish();
    result.finished = true;
    if (!result.matcher.complete()) result.error = "missing_frames";
  } catch (const std::exception& e) { result.error = e.what(); }
  result.elapsed_ns = elapsed(start);
  return result;
}

std::string hex(const std::uint8_t* p, std::size_t n) {
  require(n == 0 || p != nullptr, "hex_null_input");
  const char* alphabet = "0123456789abcdef";
  std::string out; out.reserve(2*n);
  for (std::size_t i = 0; i < n; ++i) { out += alphabet[p[i] >> 4]; out += alphabet[p[i] & 15]; }
  return out;
}
std::string plan_json(const Config& c) {
  c.validate();
  std::ostringstream out;
  out << "{\"schema\":1,\"profile\":\"" << c.profile << "\",\"packing\":\"" << c.packing
      << "\",\"hz_per_id\":" << c.hz << ",\"seconds\":" << c.seconds
      << ",\"frames_per_id\":" << c.count() << ",\"drain_ms\":" << c.drain_ms
      << ",\"capture_limit_bytes\":" << c.log_mib*1024U*1024U
      << ",\"nonce\":" << c.nonce << ",\"firmware_verified\":false,\"bitrate_verified\":false"
      << ",\"sequence_observable\":" << (c.profile == "sequence" ? "true" : "false")
      << ",\"fixed_payload_limitation\":" << (c.profile == "sequence" ? "null" :
          "\"balanced_loss_duplicate_and_order_unobservable\"")
      << ",\"ids\":[";
  for (unsigned i = 0; i < c.lanes; ++i) out << (i ? "," : "") << frame(c,i,0).id.value;
  out << "],\"first_writes_hex\":[";
  auto group = packets(c,0);
  for (unsigned i = 0; i < group.size(); ++i) out << (i ? "," : "") << '"' << hex(group[i].data(),group[i].size()) << '"';
  out << "]}"; return out.str();
}
std::string result_json(const Result& r, std::size_t bytes) {
  std::ostringstream out;
  out << "{\"complete\":" << (r.complete() ? "true" : "false")
      << ",\"error\":" << json_string(r.error) << ",\"capture_bytes\":" << bytes
      << ",\"max_lateness_ns\":" << r.max_lateness_ns
      << ",\"min_send_interval_ns\":" << r.min_send_interval_ns
      << ",\"elapsed_ns\":" << r.elapsed_ns
      << ",\"sequence_observable\":" << (r.matcher.config.profile == "sequence" ? "true" : "false")
      << ",\"fixed_payload_limitation\":" << (r.matcher.config.profile == "sequence" ? "null" :
          "\"balanced_loss_duplicate_and_order_unobservable\"")
      << ",\"unexpected_id\":" << r.matcher.unexpected_id << ",\"lanes\":[";
  for (unsigned i = 0; i < r.matcher.config.lanes; ++i) {
    const auto& s = r.matcher.lanes[i];
    out << (i ? "," : "") << "{\"sent\":" << s.sent << ",\"received\":" << s.received
        << ",\"matched\":" << s.matched << ",\"missing\":" << r.matcher.missing(i)
        << ",\"duplicate\":" << s.duplicate << ",\"out_of_order\":" << s.out_of_order
        << ",\"mismatch\":" << s.mismatch << '}';
  }
  out << "]}"; return out.str();
}
}  // namespace dual_board
