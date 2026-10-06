#include "duplex.hpp"

#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "mech_bringup/pass_through_init.hpp"
#include "mech_protocol_cubemars/ak30_servo_wire.hpp"

namespace duplex {
namespace {
using namespace mech::mech_control_core;
constexpr std::size_t kReadCapacity = 1008, kMaxIds = 256;
constexpr std::uint64_t kMaxReadCalls = 1000000, kMaxFrames = 1000000;
constexpr auto kPollSleep = std::chrono::milliseconds(1);
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
void stop_check(const std::function<bool()>& stop) {
  require(!stop || !stop(), "interrupted");
}
std::string quote(const std::string& value) {
  std::string out{"\""};
  const char* hex = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') { out += '\\'; out += static_cast<char>(c); }
    else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
    else out += static_cast<char>(c);
  }
  return out + '"';
}
void sleep_until(Clock::time_point deadline) {
  const auto remaining = deadline - Clock::now();
  if (remaining > Clock::duration::zero())
    std::this_thread::sleep_for(std::min(remaining,
        std::chrono::duration_cast<Clock::duration>(kPollSleep)));
}
std::uint64_t ns(Clock::time_point time) {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      time.time_since_epoch()).count());
}
std::string content_json(const Content& c) {
  std::ostringstream out;
  out << "{\"received_per_id\":[" << c.received_per_id[0] << ',' << c.received_per_id[1]
      << "],\"unexpected_ids\":" << c.unexpected_ids
      << ",\"payload_mismatches\":" << c.payload_mismatches
      << ",\"format_mismatches\":" << c.format_mismatches << '}';
  return out.str();
}
}  // namespace

void Config::validate() const {
  require(seconds >= 1 && seconds <= 60, "seconds_outside_1_60");
  require(hz >= 1 && hz <= 500, "command_hz_outside_1_500");
  require(feedback_hz <= 50, "feedback_hz_outside_0_50");
  require(feedback_phase_ms == 0 || feedback_phase_ms == 5, "feedback_phase_must_be_0_or_5_ms");
  require(feedback_hz != 0 || feedback_phase_ms == 0, "disabled_feedback_requires_zero_phase");
  require(rx_gate_ms == 0 || rx_gate_ms == 6, "rx_gate_must_be_0_or_6_ms");
  require(rx_gate_ms == 0 || (seconds == 2 && hz == 10 && feedback_hz == 10 && !feedback_reverse),
          "rx_gate_requires_2s_10_10_forward");
  require(quiet_ms == 2000, "quiet_must_be_2000_ms");
  require(drain_ms >= 1000 && drain_ms <= 5000, "drain_outside_1000_5000");
  require(log_mib >= 1 && log_mib <= 64, "log_mib_outside_1_64");
}

RawCanFrame frame(bool feedback, unsigned lane) {
  require(lane < 2, "lane_outside_0_1");
  const auto time = *MonotonicTime::from_nanoseconds(1);
  if (!feedback) {
    RawCanFrame out{};
    require(mech::mech_protocol_cubemars::encode_servo_position_speed(
        104+static_cast<int>(lane), {lane == 0 ? 84.1 : 95.9, 100, 100}, 1, time, out),
        "mode6_encode_failed");
    return out;
  }
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  const std::array<std::uint8_t, 8> data = lane == 0 ?
      std::array<std::uint8_t, 8>{0x03,0x49,0,0,0xff,0xff,0x2a,0} :
      std::array<std::uint8_t, 8>{0x03,0xbe,0,0,0,0x25,0x2a,0};
  std::copy(data.begin(), data.end(), payload.begin());
  return *RawCanFrame::create(1, *CanId::create(0x2968U+lane, CanFrameFormat::Extended),
      CanFrameType::Classic, FrameDirection::Tx, 8, payload, time);
}
std::array<Bytes, 2> packets(bool feedback) {
  return {dual_board::encode(frame(feedback, 0)), dual_board::encode(frame(feedback, 1))};
}

Result run(CdcSerialPort& a, CdcSerialPort& b, const Config& c,
           Capture& capture_a, Capture& capture_b,
           const std::function<bool()>& stop, const std::function<void()>& ready) {
  Result result;
  struct Close {
    CdcSerialPort& a; CdcSerialPort& b; Result& result;
    void close() {
      if (!result.closed) {
        a.close(); b.close();
        result.closed = result.a.closed = result.b.closed = true;
      }
    }
    ~Close() { close(); }
  } close{a,b,result};
  const std::array<CdcSerialPort*, 2> ports{&a,&b};
  const std::array<Capture*, 2> captures{&capture_a,&capture_b};
  const std::array<observer::Result*, 2> boards{&result.a,&result.b};
  const std::array<Content*, 2> contents{&result.content_a,&result.content_b};
  const auto start = observer::now_ns();
  result.a.start_ns = result.b.start_ns = start;
  try {
    c.validate();
    result.a.role = "A"; result.b.role = "B";
    require(&a != &b, "same_port_object");
    require(&capture_a != &capture_b, "same_capture_object");
    require(a.is_open() && b.is_open(), "port_not_open");
    require(!capture_a.count_only() && !capture_b.count_only(), "full_capture_required");
    result.a.ids.reserve(kMaxIds); result.b.ids.reserve(kMaxIds);
    const auto command_packets = packets(false), feedback_packets = packets(true);
    const std::array<std::array<RawCanFrame, 2>, 2> expected{{
        {frame(true,0), frame(true,1)}, {frame(false,0), frame(false,1)}}};
    auto write = [&](unsigned port, const Bytes& bytes) {
      stop_check(stop);
      auto& board = *boards[port];
      auto& capture = *captures[port];
      capture.reserve_record(bytes.size(), 1);
      const auto begin = observer::now_ns();
      const auto outcome = ports[port]->write_all(bytes.data(), bytes.size());
      const auto end = observer::now_ns();
      ++board.write_calls;
      capture.append(1, static_cast<std::uint8_t>(outcome), bytes.size(),
          outcome == TransportResult::Ok ? bytes.size() : std::numeric_limits<std::uint32_t>::max(),
          bytes.data(), bytes.size(), begin, end);
      require(outcome == TransportResult::Ok, port == 0 ? "A_write_incomplete_no_retry" : "B_write_incomplete_no_retry");
      return end;
    };
    const auto& init = mech::mech_bringup::kPassThroughInitFrame;
    const Bytes init_bytes(init.begin(), init.end());
    write(0, init_bytes); write(1, init_bytes);
    std::array<dual_board::Parser, 2> parsers;
    bool quiet = true;
    const auto gate_ns = std::uint64_t(c.rx_gate_ms)*1000000ULL;
    const auto command_period_ns = 1000000000ULL/c.hz;
    auto read_allowed = [&](std::uint64_t now) {
      // Only reads during the transmit window are gated. Quiet and final drain keep
      // their original reads; both ports share the A nominal tick origin.
      return !gate_ns || !result.epoch_ns || now < result.epoch_ns ||
          now >= result.transmit_end_ns ||
          (now-result.epoch_ns)%command_period_ns >= gate_ns;
    };
    auto pump = [&](unsigned port) {
      stop_check(stop);
      if (gate_ns && !read_allowed(observer::now_ns())) return;
      auto& board = *boards[port];
      auto& capture = *captures[port];
      require(board.read_calls < kMaxReadCalls, "read_count_capacity");
      std::array<std::uint8_t, kReadCapacity> bytes{};
      capture.reserve_record(bytes.size());
      std::size_t size = 0;
      const auto begin = observer::now_ns();
      // Recheck each individual I/O after preparing its capture storage. The
      // preceding port's read may have crossed into a new nominal tick.
      if (gate_ns && !read_allowed(begin)) return;
      const auto outcome = ports[port]->read_some(bytes.data(), bytes.size(), size);
      const auto end = observer::now_ns();
      ++board.read_calls;
      capture.append(2, static_cast<std::uint8_t>(outcome), bytes.size(),
          std::min(size, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())),
          bytes.data(), std::min(size, bytes.size()), begin, end);
      require(size <= bytes.size(), "read_size");
      board.rx_bytes += size;
      if (outcome == TransportResult::WouldBlock) {
        require(size == 0, "read_would_block_with_bytes");
        return;
      }
      require(outcome == TransportResult::Ok, port == 0 ? "A_read_fault" : "B_read_fault");
      if (!size) return;
      parsers[port].feed(bytes.data(), size, [&](const RawCanFrame& received) {
        require(board.rx_frames < kMaxFrames, "frame_count_capacity");
        ++board.rx_frames;
        if (!board.first_frame_ns) board.first_frame_ns = end;
        board.last_frame_ns = end;
        const bool extended = received.id.format == CanFrameFormat::Extended;
        auto it = std::find_if(board.ids.begin(), board.ids.end(), [&](const observer::IdCount& id) {
          return id.id == received.id.value && id.extended == extended;
        });
        if (it == board.ids.end()) {
          require(board.ids.size() < kMaxIds, "unique_id_capacity");
          board.ids.push_back({received.id.value, extended, 1});
        } else ++it->frames;
        auto& content = *contents[port];
        const auto lane = received.id.value == expected[port][0].id.value ? 0 :
            (received.id.value == expected[port][1].id.value ? 1 : -1);
        if (lane < 0) ++content.unexpected_ids;
        else {
          ++content.received_per_id[static_cast<unsigned>(lane)];
          const auto& wanted = expected[port][static_cast<unsigned>(lane)];
          if (!extended || received.logical_bus != 1 || received.type != CanFrameType::Classic ||
              received.error_frame || received.remote_request || received.bitrate_switch)
            ++content.format_mismatches;
          if (received.payload_size != 8 || !std::equal(wanted.payload.begin(), wanted.payload.begin()+8,
                                                       received.payload.begin()))
            ++content.payload_mismatches;
        }
        if (quiet) {
          ++(port == 0 ? result.quiet_rx_a : result.quiet_rx_b);
        }
      });
      // Finish decoding this retained read before rejecting a nonempty quiet
      // interval, so a joined packet's IDs and frame counts are all visible.
      require(!quiet || (port == 0 ? result.quiet_rx_a : result.quiet_rx_b) == 0,
              port == 0 ? "A_quiet_rx_not_empty" : "B_quiet_rx_not_empty");
    };
    bool reverse = false;
    auto pump_both = [&] {
      // One bounded nonblocking read each, alternating which side goes first.
      // No busy side can monopolize draining or indefinitely postpone sending.
      pump(reverse ? 1 : 0); pump(reverse ? 0 : 1); reverse = !reverse;
    };
    result.quiet_begin_ns = observer::now_ns();
    const auto quiet_end = Clock::now()+std::chrono::milliseconds(c.quiet_ms);
    while (Clock::now() < quiet_end) { pump_both(); sleep_until(quiet_end); }
    // No partial leftover envelope may spill across the quiet/epoch boundary.
    parsers[0].finish(); parsers[1].finish();
    result.quiet_end_ns = observer::now_ns();
    quiet = false;
    result.a.ready_ns = result.b.ready_ns = result.quiet_end_ns;
    if (ready) ready();
    stop_check(stop);
    const auto epoch = Clock::now();
    result.epoch_ns = ns(epoch);
    const auto transmit_end = epoch+std::chrono::seconds(c.seconds);
    result.transmit_end_ns = ns(transmit_end);
    const auto period_a = std::chrono::nanoseconds(1000000000LL/c.hz);
    const auto period_b = std::chrono::nanoseconds(c.feedback_hz ? 1000000000LL/c.feedback_hz : 0);
    const auto epoch_b = epoch+std::chrono::milliseconds(c.feedback_phase_ms);
    unsigned tick_a = 0, tick_b = 0;
    std::array<Clock::time_point, 2> last_send{};
    auto send_group = [&](unsigned port, unsigned tick, Clock::time_point deadline,
                          std::chrono::nanoseconds period, const std::array<Bytes,2>& group) {
      stop_check(stop);
      const auto send_start = Clock::now();
      const auto late = std::chrono::duration_cast<std::chrono::nanoseconds>(send_start-deadline).count();
      require(late >= 0, "clock_regression");
      auto& board = *boards[port];
      board.max_lateness_ns = std::max(board.max_lateness_ns, static_cast<std::uint64_t>(late));
      require(late < period.count()/2, port == 0 ? "A_scheduler_overrun_no_catchup" : "B_scheduler_overrun_no_catchup");
      if (tick) {
        const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(send_start-last_send[port]).count();
        require(interval >= period.count()/2, "scheduler_interval_no_catchup");
        const auto value = static_cast<std::uint64_t>(interval);
        board.min_send_interval_ns = board.min_send_interval_ns ? std::min(board.min_send_interval_ns,value) : value;
      }
      last_send[port] = send_start;
      for (unsigned ordinal = 0; ordinal < 2; ++ordinal) {
        const unsigned lane = port == 1 && c.feedback_reverse ? 1U-ordinal : ordinal;
        const auto ended = write(port, group[lane]);
        ++board.tx_frames;
        ++(port == 0 ? result.sent_a[lane] : result.sent_b[lane]);
        // The final tick has no following cadence check. Preserve the successful
        // raw TX/counts, but fail collection if it completed outside its window.
        require(ended < result.transmit_end_ns,
                port == 0 ? "A_write_outside_transmit_window" : "B_write_outside_transmit_window");
      }
    };
    while (Clock::now() < transmit_end) {
      stop_check(stop);
      if (tick_a < c.command_ticks() && Clock::now() >= epoch+period_a*tick_a) {
        send_group(0,tick_a,epoch+period_a*tick_a,period_a,command_packets); ++tick_a;
      }
      if (tick_b < c.feedback_ticks() && Clock::now() >= epoch_b+period_b*tick_b) {
        send_group(1,tick_b,epoch_b+period_b*tick_b,period_b,feedback_packets); ++tick_b;
      }
      pump_both();
      auto next = transmit_end;
      if (tick_a < c.command_ticks()) next = std::min(next,epoch+period_a*tick_a);
      if (tick_b < c.feedback_ticks()) next = std::min(next,epoch_b+period_b*tick_b);
      if (gate_ns) {
        const auto now = Clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now-epoch).count();
        const auto in_tick = static_cast<std::uint64_t>(elapsed)%command_period_ns;
        if (in_tick < gate_ns)
          next = std::min(next,now+std::chrono::nanoseconds(gate_ns-in_tick));
      }
      sleep_until(next);
    }
    require(tick_a == c.command_ticks() && tick_b == c.feedback_ticks(), "schedule_not_completed_no_catchup");
    const auto finish = transmit_end+std::chrono::milliseconds(c.drain_ms);
    while (Clock::now() < finish) { pump_both(); sleep_until(finish); }
    parsers[0].finish(); parsers[1].finish();
    result.finished = result.a.finished = result.b.finished = true;
    const auto matches = [](const Content& content, const std::array<std::uint64_t,2>& sent) {
      return content.received_per_id == sent && content.unexpected_ids == 0 &&
          content.payload_mismatches == 0 && content.format_mismatches == 0;
    };
    result.content_match = matches(result.content_a,result.sent_b) && matches(result.content_b,result.sent_a);
  } catch (const std::exception& error) {
    result.error = result.a.error = result.b.error = error.what();
    if (result.quiet_begin_ns && !result.quiet_end_ns) result.quiet_end_ns = observer::now_ns();
  }
  close.close();
  result.a.end_ns = result.b.end_ns = observer::now_ns();
  return result;
}

std::string plan_json(const Config& c) {
  c.validate();
  const auto commands = packets(false), feedback = packets(true);
  const unsigned feedback_first = c.feedback_reverse ? 1 : 0;
  const unsigned feedback_second = 1-feedback_first;
  std::ostringstream out;
  out << "{\"schema\":2,\"role\":\"duplex\",\"seconds\":" << c.seconds
      << ",\"command_hz\":" << c.hz << ",\"feedback_hz\":" << c.feedback_hz
      << ",\"feedback_order\":" << quote(c.feedback_reverse ? "reverse" : "forward")
      << ",\"feedback_phase_ms\":" << c.feedback_phase_ms
      << ",\"rx_gate_ms\":" << c.rx_gate_ms
      << ",\"rx_gate_anchor\":\"A_nominal_tick\",\"rx_gate_scope\":\"transmit_only_both_ports\""
      << ",\"quiet_ms\":" << c.quiet_ms << ",\"drain_ms\":" << c.drain_ms
      << ",\"capture_limit_bytes_per_port\":" << c.log_mib*1024U*1024U
      << ",\"count_only\":false,\"nonce\":" << c.nonce << ",\"nonce_on_wire\":false"
      << ",\"timestamp_domain\":\"absolute_host_steady_clock_ns\",\"can_timestamps\":false"
      << ",\"poll_sleep_ms\":1,\"read_budget_per_port_per_pass\":1,\"read_capacity\":1008"
      << ",\"max_read_calls_per_port\":1000000,\"max_frames_per_port\":1000000,\"max_unique_ids_per_port\":256"
      << ",\"packing\":\"separate\",\"command_ticks\":" << c.command_ticks()
      << ",\"feedback_ticks\":" << c.feedback_ticks()
      << ",\"a_tx_ids\":[1640,1641],\"b_tx_ids\":[10600,10601]"
      << ",\"command_payloads_hex\":[\"000cd528000a000a\",\"000ea218000a000a\"]"
      << ",\"feedback_payloads_hex\":[\"03490000ffff2a00\",\"03be000000252a00\"]"
      << ",\"synthetic_feedback\":true,\"feedback_provenance\":\"synthetic_fixture\""
      << ",\"firmware_verified\":false,\"bitrate_verified\":false,\"listen_only_verified\":false"
      << ",\"init_hex\":" << quote(dual_board::hex(mech::mech_bringup::kPassThroughInitFrame.data(),13))
      << ",\"a_first_writes_hex\":[" << quote(dual_board::hex(commands[0].data(),commands[0].size()))
      << ',' << quote(dual_board::hex(commands[1].data(),commands[1].size())) << ']'
      << ",\"b_first_writes_hex\":[" << quote(dual_board::hex(feedback[feedback_first].data(),feedback[feedback_first].size()))
      << ',' << quote(dual_board::hex(feedback[feedback_second].data(),feedback[feedback_second].size())) << ']' << '}';
  return out.str();
}
std::string result_json(const Result& r, std::size_t capture_a_bytes, std::size_t capture_b_bytes) {
  std::ostringstream out;
  out << "{\"schema\":2,\"role\":\"duplex\",\"complete\":" << (r.complete() ? "true" : "false")
      << ",\"collection_complete\":" << (r.complete() ? "true" : "false")
      << ",\"raw_complete\":" << (r.complete() ? "true" : "false")
      << ",\"content_match\":" << (r.content_match ? "true" : "false")
      << ",\"error\":" << quote(r.error) << ",\"closed\":" << (r.closed ? "true" : "false")
      << ",\"quiet_begin_ns\":" << r.quiet_begin_ns << ",\"quiet_end_ns\":" << r.quiet_end_ns
      << ",\"epoch_ns\":" << r.epoch_ns << ",\"transmit_end_ns\":" << r.transmit_end_ns
      << ",\"quiet_rx_a\":" << r.quiet_rx_a << ",\"quiet_rx_b\":" << r.quiet_rx_b
      << ",\"sent_a\":[" << r.sent_a[0] << ',' << r.sent_a[1] << ']'
      << ",\"sent_b\":[" << r.sent_b[0] << ',' << r.sent_b[1] << ']'
      << ",\"content_a\":" << content_json(r.content_a) << ",\"content_b\":" << content_json(r.content_b)
      << ",\"a\":" << observer::result_json(r.a,capture_a_bytes)
      << ",\"b\":" << observer::result_json(r.b,capture_b_bytes)
      << ",\"fixed_payload_sequence_observable\":false,\"can_delivery_verified\":false"
      << ",\"motor_feedback_verified\":false}";
  return out.str();
}
}  // namespace duplex
