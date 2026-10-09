// Standalone offline semantic tests. No ROS, physical serial port or CAN access.
#include "duplex.hpp"
#include <algorithm>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "mech_bringup/pass_through_init.hpp"

namespace {
using namespace duplex;
using namespace mech::mech_control_core;
unsigned checks = 0;
void check(bool ok, const char* text, int line) {
  ++checks;
  if (!ok) throw std::runtime_error(std::to_string(line)+": "+text);
}
#define CHECK(x) check((x),#x,__LINE__)
Bytes literal(const std::string& hex) {
  Bytes bytes;
  for (std::size_t i = 0; i < hex.size(); i += 2)
    bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i,2),nullptr,16)));
  return bytes;
}
const std::array<Bytes,4> golden{{
    literal("f7120e000bf8e2680600000c08000cd528000a000a"),
    literal("f7120e000ba8a6690600000c08000ea218000a000a"),
    literal("f7120e000bb1b5682900000c0803490000ffff2a00"),
    literal("f7120e000bb5a2692900000c0803be000000252a00")}};
const Bytes init(mech::mech_bringup::kPassThroughInitFrame.begin(),
                 mech::mech_bringup::kPassThroughInitFrame.end());

class FakePort final : public CdcSerialPort {
 public:
  bool opened{true};
  unsigned closes{0}, reads{0};
  std::size_t fragment{1008}, forced_size{0};
  TransportResult forced_read{TransportResult::Ok};
  std::deque<TransportResult> write_results;
  std::deque<Bytes> incoming;
  std::vector<Bytes> writes;
  std::function<void(const Bytes&)> on_write;
  bool open() noexcept override { opened = true; return true; }
  bool is_open() const noexcept override { return opened; }
  void close() noexcept override { opened = false; ++closes; }
  TransportResult write_all(const std::uint8_t* p, std::size_t n) noexcept override {
    writes.emplace_back(p,p+n);
    const auto result = write_results.empty() ? TransportResult::Ok : write_results.front();
    if (!write_results.empty()) write_results.pop_front();
    if (result == TransportResult::Ok && on_write) on_write(writes.back());
    return result;
  }
  TransportResult read_some(std::uint8_t* p, std::size_t capacity, std::size_t& size) noexcept override {
    ++reads; size = 0;
    if (forced_size) {
      size = forced_size;
      std::fill_n(p,std::min(size,capacity),0);
      return forced_read;
    }
    if (forced_read != TransportResult::Ok) return forced_read;
    if (incoming.empty()) return TransportResult::WouldBlock;
    auto& bytes = incoming.front();
    size = std::min({capacity,fragment,bytes.size()});
    std::copy_n(bytes.begin(),size,p);
    bytes.erase(bytes.begin(),bytes.begin()+static_cast<std::ptrdiff_t>(size));
    if (bytes.empty()) incoming.pop_front();
    return TransportResult::Ok;
  }
};
void link(FakePort& a, FakePort& b, bool mixed_payload = false) {
  a.on_write = [&, mixed_payload](const Bytes& bytes) {
    if (bytes == init) return;
    if (mixed_payload && bytes == golden[0]) {
      auto changed = frame(false,0);
      changed.payload = frame(false,1).payload;
      b.incoming.push_back(dual_board::encode(changed));
    } else b.incoming.push_back(bytes);
  };
  b.on_write = [&](const Bytes& bytes) { if (bytes != init) a.incoming.push_back(bytes); };
}
void closed(const FakePort& a, const FakePort& b) {
  CHECK(!a.opened && !b.opened);
  CHECK(a.closes == 1 && b.closes == 1);
}
std::uint64_t little(const Bytes& bytes, std::size_t offset, unsigned size) {
  std::uint64_t value = 0;
  for (unsigned i = 0; i < size; ++i) value |= std::uint64_t(bytes.at(offset+i)) << (8*i);
  return value;
}
void schema(const Capture& capture, const observer::Result& summary) {
  const auto& bytes = capture.bytes();
  std::size_t cursor = 0;
  std::uint64_t previous = 0, read_calls = 0, write_calls = 0, rx_bytes = 0;
  while (cursor < bytes.size()) {
    CHECK(bytes.size()-cursor >= 32);
    const auto kind = bytes[cursor], result = bytes[cursor+2];
    const auto size = little(bytes,cursor+4,4);
    const auto end = little(bytes,cursor+8,8), begin = little(bytes,cursor+16,8);
    const auto requested = little(bytes,cursor+24,4), returned = little(bytes,cursor+28,4);
    CHECK(bytes[cursor+1] == 0 && bytes[cursor+3] == 0);
    CHECK(previous <= begin && begin <= end);
    CHECK(size >= 16 && size <= 1024 && cursor+16+size <= bytes.size());
    if (kind == 1) {
      ++write_calls;
      CHECK(size == requested+16);
      CHECK(returned == (result == 0 ? requested : 0xffffffffU));
      if (write_calls == 1) {
        CHECK(requested == 13);
        CHECK(std::equal(init.begin(),init.end(),bytes.begin()+static_cast<std::ptrdiff_t>(cursor+32)));
      } else CHECK(requested == 21);
    } else {
      CHECK(kind == 2); ++read_calls;
      CHECK(requested == 1008 && returned <= requested && size == returned+16);
      CHECK(result != 1 || returned == 0); rx_bytes += returned;
    }
    previous = end;
    cursor += static_cast<std::size_t>(16+size);
  }
  CHECK(cursor == bytes.size());
  CHECK(read_calls == summary.read_calls && write_calls == summary.write_calls && rx_bytes == summary.rx_bytes);
}
Config slow() { Config c; c.seconds = 1; c.hz = 2; return c; }
void frozen_bytes() {
  for (unsigned feedback = 0; feedback < 2; ++feedback) {
    const auto group = packets(feedback != 0);
    for (unsigned lane = 0; lane < 2; ++lane) {
      CHECK(group[lane] == golden[feedback*2+lane]);
      CHECK(frame(feedback != 0,lane).direction == FrameDirection::Tx);
      CHECK(frame(feedback != 0,lane).payload_size == 8);
    }
  }
  CHECK(plan_json(slow()).find("\"nonce_on_wire\":false") != std::string::npos);
  CHECK(plan_json(slow()).find("\"synthetic_feedback\":true") != std::string::npos);
  for (bool reverse : {false,true}) {
    auto c = slow(); c.feedback_reverse = reverse;
    const auto first = reverse ? 3U : 2U, second = reverse ? 2U : 3U;
    CHECK(plan_json(c).find(std::string("\"feedback_order\":\"")+(reverse ? "reverse" : "forward")+'"') != std::string::npos);
    const auto order = std::string("\"b_first_writes_hex\":[\"")+
        dual_board::hex(golden[first].data(),golden[first].size())+"\",\""+
        dual_board::hex(golden[second].data(),golden[second].size())+"\"]";
    CHECK(plan_json(c).find(order) != std::string::npos);
    CHECK(plan_json(c).find("\"b_tx_ids\":[10600,10601]") != std::string::npos);
  }
  CHECK(plan_json(slow()).find("\"feedback_phase_ms\":0") != std::string::npos);
  auto phased = slow(); phased.feedback_hz = 1; phased.feedback_phase_ms = 5;
  CHECK(plan_json(phased).find("\"feedback_phase_ms\":5") != std::string::npos);
  CHECK(plan_json(slow()).find("\"rx_gate_ms\":0") != std::string::npos);
  for (unsigned invalid : {1U,6U}) {
    phased.feedback_phase_ms = invalid;
    bool failed = false;
    try { phased.validate(); } catch (const std::exception&) { failed = true; }
    CHECK(failed);
  }
  phased.feedback_hz = 0; phased.feedback_phase_ms = 5;
  bool failed = false;
  try { phased.validate(); } catch (const std::exception&) { failed = true; }
  CHECK(failed);
}
void gate_configs() {
  Config allowed; allowed.seconds = 2; allowed.hz = allowed.feedback_hz = 10;
  allowed.rx_gate_ms = 6;
  for (unsigned phase : {0U,5U}) {
    allowed.feedback_phase_ms = phase;
    allowed.validate();
    CHECK(plan_json(allowed).find("\"rx_gate_ms\":6") != std::string::npos);
    CHECK(plan_json(allowed).find("\"rx_gate_anchor\":\"A_nominal_tick\"") != std::string::npos);
    CHECK(plan_json(allowed).find("\"rx_gate_scope\":\"transmit_only_both_ports\"") != std::string::npos);
  }
  for (unsigned scenario = 0; scenario < 8; ++scenario) {
    auto c = allowed;
    if (scenario < 3) c.rx_gate_ms = scenario == 0 ? 1 : scenario == 1 ? 5 : 7;
    if (scenario == 3) c.seconds = 1;
    if (scenario == 4) c.hz = 9;
    if (scenario == 5) { c.feedback_hz = 0; c.feedback_phase_ms = 0; }
    if (scenario == 6) c.feedback_hz = 9;
    if (scenario == 7) c.feedback_reverse = true;
    FakePort a,b; Capture ca(1024*1024),cb(1024*1024);
    const auto r = run(a,b,c,ca,cb,[]{ return false; });
    CHECK(!r.complete() && !r.error.empty());
    CHECK(a.writes.empty() && b.writes.empty() && ca.bytes().empty() && cb.bytes().empty());
    closed(a,b);
  }
}
void gated(unsigned phase) {
  FakePort a,b; link(a,b);
  Config c; c.seconds = 2; c.hz = c.feedback_hz = 10;
  c.rx_gate_ms = 6; c.feedback_phase_ms = phase;
  Capture ca(2*1024*1024),cb(2*1024*1024);
  const auto r = run(a,b,c,ca,cb,[]{ return false; });
  if (!r.complete()) throw std::runtime_error("gated: "+r.error);
  CHECK(r.complete() && r.content_match);
  CHECK(r.sent_a[0] == 20 && r.sent_a[1] == 20 && r.sent_b == r.sent_a);
  CHECK(r.a.tx_frames == 40 && r.b.tx_frames == 40);
  CHECK(r.a.rx_frames == 40 && r.b.rx_frames == 40);
  CHECK(a.writes.size() == 41 && b.writes.size() == 41);
  for (unsigned tick = 0; tick < 20; ++tick) {
    CHECK(a.writes[1+2*tick] == golden[0] && a.writes[2+2*tick] == golden[1]);
    CHECK(b.writes[1+2*tick] == golden[2] && b.writes[2+2*tick] == golden[3]);
  }
  for (unsigned port = 0; port < 2; ++port) {
    const auto& raw = port == 0 ? ca.bytes() : cb.bytes();
    std::size_t cursor = 0;
    unsigned writes = 0, quiet_reads = 0, drain_reads = 0;
    std::array<unsigned,20> tick_reads{};
    std::array<std::uint64_t,20> last_write{};
    while (cursor < raw.size()) {
      const auto begin = little(raw,cursor+16,8), end = little(raw,cursor+8,8);
      if (raw[cursor] == 1) {
        if (writes) {
          const unsigned tick = (writes-1)/2;
          const auto deadline = r.epoch_ns+std::uint64_t(tick)*100000000ULL+
              (port == 1 ? std::uint64_t(phase)*1000000ULL : 0ULL);
          CHECK(begin >= deadline && begin < deadline+50000000ULL);
          last_write[tick] = end;
        }
        ++writes;
      } else if (begin < r.epoch_ns) ++quiet_reads;
      else if (begin >= r.transmit_end_ns) ++drain_reads;
      else {
        const auto elapsed = begin-r.epoch_ns;
        const auto tick = static_cast<unsigned>(elapsed/100000000ULL);
        CHECK(elapsed%100000000ULL >= 6000000ULL);
        CHECK(last_write[tick] != 0 && begin >= last_write[tick]);
        ++tick_reads[tick];
      }
      cursor += static_cast<std::size_t>(16+little(raw,cursor+4,4));
    }
    CHECK(writes == 41 && quiet_reads > 0 && drain_reads > 0);
    for (unsigned count : tick_reads) CHECK(count > 0);
  }
  schema(ca,r.a); schema(cb,r.b); closed(a,b);
}
void gated_stop() {
  FakePort a,b; link(a,b);
  bool stop = false;
  const auto forward = b.on_write;
  b.on_write = [&](const Bytes& bytes) { forward(bytes); if (bytes == golden[3]) stop = true; };
  Config c; c.seconds = 2; c.hz = c.feedback_hz = 10; c.rx_gate_ms = 6;
  Capture ca(1024*1024),cb(1024*1024);
  const auto r = run(a,b,c,ca,cb,[&]{ return stop; });
  CHECK(!r.complete() && r.error == "interrupted");
  CHECK(r.sent_a[0] == 1 && r.sent_a[1] == 1 && r.sent_b == r.sent_a);
  CHECK(r.a.rx_frames == 0 && r.b.rx_frames == 0);
  schema(ca,r.a); schema(cb,r.b); closed(a,b);
}
void normal(bool feedback, bool mismatch, bool reverse = false, unsigned phase = 0) {
  FakePort a,b; a.fragment = b.fragment = 7;
  link(a,b,mismatch);
  auto c = slow(); c.feedback_hz = feedback ? 1 : 0; c.feedback_reverse = reverse;
  c.feedback_phase_ms = phase;
  Capture ca(2*1024*1024), cb(2*1024*1024);
  unsigned ready = 0;
  auto result = run(a,b,c,ca,cb,[]{ return false; },[&] {
    CHECK(a.writes.size() == 1 && b.writes.size() == 1);
    CHECK(a.writes[0] == init && b.writes[0] == init);
    ++ready;
  });
  if (!result.complete()) throw std::runtime_error("normal: "+result.error);
  CHECK(result.complete()); CHECK(ready == 1);
  CHECK(result.content_match == !mismatch);
  CHECK(result.sent_a[0] == 2 && result.sent_a[1] == 2);
  CHECK(result.sent_b[0] == (feedback ? 1 : 0) && result.sent_b[1] == (feedback ? 1 : 0));
  CHECK(result.a.tx_frames == 4 && result.b.tx_frames == (feedback ? 2 : 0));
  CHECK(a.writes.size() == 5 && a.writes[1] == golden[0] && a.writes[2] == golden[1]);
  if (feedback) {
    CHECK(b.writes.size() == 3);
    CHECK(b.writes[1] == golden[reverse ? 3 : 2] && b.writes[2] == golden[reverse ? 2 : 3]);
    // The first CAN write follows the single 45-byte initialization record and
    // any number of independent read records; locate it from raw schema2.
    const auto& raw = cb.bytes();
    std::size_t cursor = 0; unsigned writes = 0; std::uint64_t first_begin = 0;
    while (cursor < raw.size()) {
      if (raw[cursor] == 1 && ++writes == 2) { first_begin = little(raw,cursor+16,8); break; }
      cursor += static_cast<std::size_t>(16+little(raw,cursor+4,4));
    }
    CHECK(first_begin >= result.epoch_ns+std::uint64_t(phase)*1000000ULL);
  }
  CHECK(result.a.rx_frames == (feedback ? 2 : 0) && result.b.rx_frames == 4);
  CHECK(result.content_b.payload_mismatches == (mismatch ? 2 : 0));
  CHECK(result.quiet_rx_a == 0 && result.quiet_rx_b == 0);
  CHECK(result.quiet_end_ns-result.quiet_begin_ns >= 2000000000ULL);
  CHECK(result.quiet_end_ns <= result.epoch_ns && result.epoch_ns < result.transmit_end_ns);
  CHECK(result.a.end_ns >= result.transmit_end_ns+1000000000ULL);
  CHECK(result.a.read_calls > 0 && result.b.read_calls > 0);
  CHECK(result.a.min_send_interval_ns >= 250000000ULL);
  schema(ca,result.a); schema(cb,result.b); closed(a,b);
}
void early_failures() {
  for (unsigned scenario = 0; scenario < 11; ++scenario) {
    FakePort a,b;
    auto c = slow();
    Capture ca(scenario == 4 ? 1000 : 1024*1024), cb(1024*1024,scenario == 9);
    if (scenario == 0) c.hz = 0;
    if (scenario == 1) a.write_results.push_back(TransportResult::Fault);
    if (scenario == 2) b.write_results.push_back(TransportResult::Fault);
    if (scenario == 3) { a.opened = false; }
    if (scenario == 5) { a.incoming.push_back(golden[2]); a.incoming.push_back(golden[3]); }
    if (scenario == 6) { auto bad = golden[2]; bad.back() ^= 1; b.incoming.push_back(bad); }
    if (scenario == 7) { a.forced_size = 1009; }
    if (scenario == 8) { b.forced_read = TransportResult::WouldBlock; b.forced_size = 1; }
    auto result = run(a,b,c,ca,cb,[&]{ return scenario == 10; });
    CHECK(!result.complete() && !result.content_match && !result.error.empty());
    CHECK(result.sent_a[0] == 0 && result.sent_b[0] == 0);
    if (scenario == 5) {
      CHECK(result.quiet_rx_a == 1 && result.a.rx_frames == 1);
      CHECK(result.error == "A_quiet_rx_not_empty");
      CHECK(ca.bytes().size() >= 45+32+21);
    }
    if (scenario == 2) { CHECK(a.writes.size() == 1 && b.writes.size() == 1); }
    closed(a,b);
  }
}
void callback_and_overrun() {
  {
    FakePort a,b; Capture ca(1024*1024),cb(1024*1024);
    auto r = run(a,b,slow(),ca,cb,[]{ return false; },[]{ throw std::runtime_error("ready \"fault\"\n"); });
    CHECK(!r.complete() && r.epoch_ns == 0 && r.a.tx_frames == 0);
    CHECK(result_json(r,ca.bytes().size(),cb.bytes().size()).find("ready \\\"fault\\\"\\u000a") != std::string::npos);
    closed(a,b);
  }
  {
    FakePort a,b; link(a,b);
    bool first = true;
    const auto forwarding = a.on_write;
    a.on_write = [&](const Bytes& bytes) {
      forwarding(bytes);
      if (bytes != init && first) { first = false; std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
    };
    auto c = slow(); c.hz = 10;
    Capture ca(1024*1024),cb(1024*1024);
    auto r = run(a,b,c,ca,cb,[]{ return false; });
    CHECK(!r.complete() && r.error == "A_scheduler_overrun_no_catchup");
    CHECK(r.a.tx_frames == 2 && a.writes.size() == 3);
    CHECK(r.a.max_lateness_ns >= 50000000ULL);
    schema(ca,r.a); schema(cb,r.b); closed(a,b);
  }
  {
    FakePort a,b; link(a,b);
    const auto forwarding = a.on_write;
    a.on_write = [&](const Bytes& bytes) {
      forwarding(bytes);
      if (bytes != init) std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    };
    auto c = slow(); c.hz = 1;  // One final tick: there is no later overrun check.
    Capture ca(1024*1024),cb(1024*1024);
    auto r = run(a,b,c,ca,cb,[]{ return false; });
    CHECK(!r.complete() && r.error == "A_write_outside_transmit_window");
    CHECK(r.a.tx_frames == 1 && r.sent_a[0] == 1 && r.sent_a[1] == 0);
    CHECK(a.writes.size() == 2 && r.a.end_ns > r.transmit_end_ns);
    schema(ca,r.a); schema(cb,r.b); closed(a,b);
  }
}
}  // namespace
int main() {
  try {
    frozen_bytes(); gate_configs(); early_failures(); normal(false,false); normal(true,false);
    normal(true,true); normal(true,false,true); normal(true,false,false,5); callback_and_overrun();
    gated(0); gated(5); gated_stop();
    std::cout << "PASS " << checks << " duplex checks (offline semantic tests)\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
