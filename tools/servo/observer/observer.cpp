#include "observer.hpp"

#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "mech_bringup/pass_through_init.hpp"
#include "mech_protocol_cubemars/ak30_servo_wire.hpp"

namespace observer {
namespace {
using namespace mech::mech_control_core;
constexpr std::size_t kReadCapacity = 1008;
constexpr std::size_t kMaxIds = 256;
constexpr std::uint64_t kMaxFrames = 1000000, kMaxReadCalls = 1000000;
constexpr auto kPollSleep = std::chrono::milliseconds(1);
void require(bool ok, const char* error) {
  if (!ok) throw std::runtime_error(error);
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
template<class T> void little(std::uint8_t* out, T value) {
  for (unsigned i = 0; i < sizeof(T); ++i)
    out[i] = static_cast<std::uint8_t>(value >> (8*i));
}
void check_stop(const std::function<bool()>& stop) {
  require(!stop || !stop(), "interrupted");
}
void sleep_until(Clock::time_point deadline) {
  const auto remaining = deadline-Clock::now();
  if (remaining > Clock::duration::zero())
    std::this_thread::sleep_for(std::min(remaining,
        std::chrono::duration_cast<Clock::duration>(kPollSleep)));
}
}  // namespace

unsigned Config::count() const {
  return frames_per_id ? frames_per_id : (profile == "four-id" ? 4 : hz*seconds);
}
unsigned Config::id_count() const { return profile == "four-id" ? 4 : lanes; }
void Config::validate() const {
  require(role == "observe" || role == "fixture", "unknown_role");
  require(profile == "sequence" || profile == "four-id", "unknown_profile");
  require(seconds >= 1 && seconds <= 180, "seconds_outside_1_180");
  require(hz >= 1 && hz <= 500, "hz_outside_1_500");
  require(drain_ms <= 5000, "drain_outside_0_5000");
  require(log_mib >= 1 && log_mib <= 64, "log_mib_outside_1_64");
  require(lanes == 1 || lanes == 2, "lanes_must_be_1_or_2");
  if (role == "fixture") {
    require(!count_only, "fixture_requires_full_capture");
    require(count() >= 1 && count() <= hz*seconds && count() <= 90000,
            "fixture_count_exceeds_duration");
    if (profile == "four-id")
      require(hz <= 10 && count() >= 4 && count() <= 10,
              "four_id_requires_1_10_hz_and_4_10_frames_per_id");
  } else require(frames_per_id == 0, "frames_require_fixture");
}

std::uint64_t now_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
}

Capture::Capture(std::size_t limit, bool count_only)
    : count_only_(count_only), storage_(limit) {}
void Capture::reserve_record(std::size_t raw_size, std::uint8_t kind) const {
  require(raw_size <= kReadCapacity, "record_raw_capacity");
  storage_.reserve_record(16+((!count_only_ || kind == 1) ? raw_size : 0));
}
void Capture::append(std::uint8_t kind, std::uint8_t result,
                     std::size_t requested, std::size_t returned,
                     const std::uint8_t* raw, std::size_t raw_size,
                     std::uint64_t begin_ns, std::uint64_t end_ns) {
  require(kind == 1 || kind == 2, "record_kind");
  reserve_record(raw_size, kind);
  require(requested <= std::numeric_limits<std::uint32_t>::max() &&
      returned <= std::numeric_limits<std::uint32_t>::max(), "record_size");
  require(begin_ns <= end_ns, "record_time_order");
  require(raw_size == 0 || raw != nullptr, "record_null_bytes");
  std::array<std::uint8_t, 1024> payload{};
  little(payload.data(), begin_ns);
  little(payload.data()+8, static_cast<std::uint32_t>(requested));
  little(payload.data()+12, static_cast<std::uint32_t>(returned));
  const auto retained = (!count_only_ || kind == 1) ? raw_size : 0;
  if (retained) std::copy_n(raw, retained, payload.begin()+16);
  storage_.append(kind, 0, result, payload.data(), 16+retained, end_ns);
}

RawCanFrame fixture_frame(const Config& c, unsigned lane, unsigned sequence) {
  c.validate();
  require(c.role == "fixture" && lane < c.id_count() && sequence < c.count(),
          "fixture_frame_index");
  if (c.profile == "sequence") {
    // Reuse the established marker IDs/layout, extending its bounded sequence
    // window to this emitter's 180-second maximum without changing that tool.
    dual_board::Config marker;
    marker.hz = 1; marker.seconds = 1; marker.lanes = c.lanes;
    marker.nonce = c.nonce;
    auto f = dual_board::frame(marker, lane, 0);
    for (unsigned i = 0; i < 3; ++i)
      f.payload[4+i] = static_cast<std::uint8_t>(sequence >> (8*i));
    return f;
  }
  RawCanFrame f{};
  const auto time = *MonotonicTime::from_nanoseconds(1);
  if (lane < 2) {
    require(mech::mech_protocol_cubemars::encode_servo_position_speed(
        104+static_cast<int>(lane), {lane == 0 ? 84.1 : 90.9, 100, 100}, 1, time, f),
        "mode6_encode_failed");
    return f;
  }
  // Legal synthetic feedback: 84.1/90.9 device degrees, zero speed/current,
  // 40/41 C and normal status. These are fabricated fixture bytes, never
  // measurements or messages from a real motor.
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  const std::array<std::uint8_t, 8> data = lane == 2 ?
      std::array<std::uint8_t, 8>{0x03,0x49,0,0,0,0,0x28,0} :
      std::array<std::uint8_t, 8>{0x03,0x8d,0,0,0,0,0x29,0};
  std::copy(data.begin(), data.end(), payload.begin());
  return *RawCanFrame::create(1, *CanId::create(0x2968U+(lane-2),
      CanFrameFormat::Extended), CanFrameType::Classic, FrameDirection::Tx,
      8, payload, time);
}
std::vector<Bytes> fixture_packets(const Config& c, unsigned sequence) {
  c.validate();
  std::vector<Bytes> packets;
  packets.reserve(c.id_count());
  for (unsigned lane = 0; lane < c.id_count(); ++lane)
    packets.push_back(dual_board::encode(fixture_frame(c,lane,sequence)));
  return packets;
}

Result run(CdcSerialPort& serial, const Config& c, Capture& capture,
           const std::function<bool()>& stop, const std::function<void()>& ready) {
  Result result;
  struct Close {
    CdcSerialPort& port; Result& result;
    void close() { if (!result.closed) { port.close(); result.closed = true; } }
    ~Close() { close(); }
  } close{serial,result};
  result.start_ns = now_ns();
  try {
    c.validate();
    result.role = c.role; result.count_only = c.count_only;
    require(capture.count_only() == c.count_only, "capture_mode_mismatch");
    require(serial.is_open(), "port_not_open");
    result.ids.reserve(kMaxIds);
    auto write = [&](const Bytes& bytes) {
      check_stop(stop);
      capture.reserve_record(bytes.size(), 1);
      const auto begin = now_ns();
      const auto outcome = serial.write_all(bytes.data(), bytes.size());
      const auto end = now_ns();
      ++result.write_calls;
      const auto transferred = outcome == TransportResult::Ok ? bytes.size() :
          std::numeric_limits<std::uint32_t>::max();
      capture.append(1, static_cast<std::uint8_t>(outcome), bytes.size(), transferred,
                     bytes.data(), bytes.size(), begin, end);
      require(outcome == TransportResult::Ok, "write_incomplete_no_retry");
    };
    const auto& init = mech::mech_bringup::kPassThroughInitFrame;
    write(Bytes(init.begin(),init.end()));
    result.ready_ns = now_ns();
    if (ready) ready();
    check_stop(stop);
    dual_board::Parser parser;
    auto pump = [&] {
      check_stop(stop);
      require(result.read_calls < kMaxReadCalls, "read_count_capacity");
      std::array<std::uint8_t, kReadCapacity> bytes{};
      capture.reserve_record(bytes.size());
      std::size_t size = 0;
      const auto begin = now_ns();
      const auto outcome = serial.read_some(bytes.data(), bytes.size(), size);
      const auto end = now_ns();
      ++result.read_calls;
      capture.append(2, static_cast<std::uint8_t>(outcome), bytes.size(),
          std::min(size, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())),
          bytes.data(), std::min(size, bytes.size()), begin, end);
      require(size <= bytes.size(), "read_size");
      result.rx_bytes += size;
      if (outcome == TransportResult::WouldBlock) {
        require(size == 0, "read_would_block_with_bytes");
        return;
      }
      require(outcome == TransportResult::Ok, "read_fault");
      if (!size) return;
      parser.feed(bytes.data(), size, [&](const RawCanFrame& frame) {
        require(result.rx_frames < kMaxFrames, "frame_count_capacity");
        ++result.rx_frames;
        if (!result.first_frame_ns) result.first_frame_ns = end;
        result.last_frame_ns = end;
        const bool extended = frame.id.format == CanFrameFormat::Extended;
        auto it = std::find_if(result.ids.begin(), result.ids.end(), [&](const IdCount& id) {
          return id.id == frame.id.value && id.extended == extended;
        });
        if (it == result.ids.end()) {
          require(result.ids.size() < kMaxIds, "unique_id_capacity");
          result.ids.push_back({frame.id.value, extended, 1});
        } else ++it->frames;
        require(c.role == "observe", "unexpected_fixture_rx");
      });
    };
    const auto epoch = Clock::now();
    const auto duration_end = epoch+std::chrono::seconds(c.seconds);
    if (c.role == "fixture") {
      const auto period = std::chrono::nanoseconds(1000000000LL/c.hz);
      Clock::time_point last_send{};
      for (unsigned sequence = 0; sequence < c.count(); ++sequence) {
        auto group = fixture_packets(c, sequence);
        const auto deadline = epoch+period*sequence;
        while (Clock::now() < deadline) { check_stop(stop); sleep_until(deadline); }
        const auto send_start = Clock::now();
        const auto late = std::chrono::duration_cast<std::chrono::nanoseconds>(send_start-deadline).count();
        result.max_lateness_ns = std::max(result.max_lateness_ns, static_cast<std::uint64_t>(late));
        require(late < period.count()/2, "scheduler_overrun_no_catchup");
        if (sequence) {
          const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(send_start-last_send).count();
          const auto ns = static_cast<std::uint64_t>(interval);
          result.min_send_interval_ns = result.min_send_interval_ns ?
              std::min(result.min_send_interval_ns, ns) : ns;
          require(interval >= period.count()/2, "scheduler_overrun_no_catchup");
        }
        last_send = send_start;
        for (const auto& packet : group) { write(packet); ++result.tx_frames; }
        pump();
      }
    }
    const auto finish_at = duration_end+(c.role == "fixture" ?
        std::chrono::milliseconds(c.drain_ms) : std::chrono::milliseconds(0));
    while (Clock::now() < finish_at) { pump(); sleep_until(finish_at); }
    parser.finish();
    result.finished = true;
  } catch (const std::exception& error) { result.error = error.what(); }
  close.close();
  result.end_ns = now_ns();
  return result;
}

std::string plan_json(const Config& c) {
  c.validate();
  std::ostringstream out;
  out << "{\"schema\":2,\"role\":" << quote(c.role)
      << ",\"seconds\":" << c.seconds << ",\"drain_ms\":" << (c.role == "fixture" ? c.drain_ms : 0)
      << ",\"capture_limit_bytes\":" << c.log_mib*1024U*1024U
      << ",\"count_only\":" << (c.count_only ? "true" : "false")
      << ",\"raw_rx_retained\":" << (c.count_only ? "false" : "true")
      << ",\"timestamp_domain\":\"absolute_host_steady_clock_ns\",\"can_timestamps\":false"
      << ",\"poll_sleep_ms\":1,\"read_capacity\":" << kReadCapacity
      << ",\"max_read_calls\":" << kMaxReadCalls << ",\"max_frames\":" << kMaxFrames
      << ",\"max_unique_ids\":" << kMaxIds
      << ",\"firmware_verified\":false,\"bitrate_verified\":false"
      << ",\"listen_only_verified\":false,\"init_hex\":\""
      << dual_board::hex(mech::mech_bringup::kPassThroughInitFrame.data(),
          mech::mech_bringup::kPassThroughInitFrame.size()) << '"';
  if (c.role == "fixture") {
    out << ",\"profile\":" << quote(c.profile) << ",\"hz_per_id\":" << c.hz
        << ",\"frames_per_id\":" << c.count() << ",\"nonce\":" << c.nonce
        << ",\"sequence_observable\":" << (c.profile == "sequence" ? "true" : "false")
        << ",\"simulated_feedback\":" << (c.profile == "four-id" ? "true" : "false")
        << ",\"feedback_provenance\":" << quote(c.profile == "four-id" ? "synthetic_fixture" : "none")
        << ",\"ids\":[";
    for (unsigned lane = 0; lane < c.id_count(); ++lane)
      out << (lane ? "," : "") << fixture_frame(c,lane,0).id.value;
    out << "],\"first_writes_hex\":[";
    const auto packets = fixture_packets(c,0);
    for (unsigned i = 0; i < packets.size(); ++i)
      out << (i ? "," : "") << '"' << dual_board::hex(packets[i].data(),packets[i].size()) << '"';
    out << ']';
  }
  out << '}'; return out.str();
}
std::string result_json(const Result& r, std::size_t capture_bytes) {
  std::ostringstream out;
  out << "{\"schema\":2,\"role\":" << quote(r.role)
      << ",\"complete\":" << (r.complete() ? "true" : "false")
      << ",\"raw_complete\":" << (r.raw_complete() ? "true" : "false")
      << ",\"count_only\":" << (r.count_only ? "true" : "false")
      << ",\"error\":" << quote(r.error) << ",\"closed\":" << (r.closed ? "true" : "false")
      << ",\"capture_bytes\":" << capture_bytes << ",\"start_ns\":" << r.start_ns
      << ",\"ready_ns\":" << r.ready_ns << ",\"end_ns\":" << r.end_ns
      << ",\"first_frame_ns\":" << r.first_frame_ns << ",\"last_frame_ns\":" << r.last_frame_ns
      << ",\"read_calls\":" << r.read_calls << ",\"write_calls\":" << r.write_calls
      << ",\"rx_bytes\":" << r.rx_bytes << ",\"rx_frames\":" << r.rx_frames
      << ",\"tx_frames\":" << r.tx_frames << ",\"max_lateness_ns\":" << r.max_lateness_ns
      << ",\"min_send_interval_ns\":" << r.min_send_interval_ns
      << ",\"can_delivery_verified\":false,\"ids\":[";
  for (unsigned i = 0; i < r.ids.size(); ++i)
    out << (i ? "," : "") << "{\"id\":" << r.ids[i].id
        << ",\"extended\":" << (r.ids[i].extended ? "true" : "false")
        << ",\"frames\":" << r.ids[i].frames << '}';
  out << "]}"; return out.str();
}
}  // namespace observer
