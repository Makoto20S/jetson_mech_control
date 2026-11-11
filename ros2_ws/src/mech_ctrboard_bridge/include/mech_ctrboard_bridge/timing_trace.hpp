#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mech::mech_ctrboard_bridge {

// Opt-in, single-executor observer. Storage is allocated and touched before
// opening the device. No allocation, file IO or formatting occurs in record_*.
// A full buffer drops trace rows only; telemetry processing continues normally.
struct PacketTimingRow {
  std::uint64_t packet_seq{0};
  std::uint32_t device_ms{0};
  std::int64_t ros_stamp_ns{0};
  std::int64_t frame_host_arrival_ns{0};
  std::int64_t last_serial_read_end_ns{0};
  std::int64_t decode_begin_ns{0};
  std::int64_t decode_end_ns{0};
  std::int64_t imu1_publish_begin_ns{0};
  std::int64_t imu1_publish_end_ns{0};
  std::int64_t imu2_publish_begin_ns{0};
  std::int64_t imu2_publish_end_ns{0};
  std::int64_t packet_done_ns{0};
};

struct PollTimingRow {
  std::uint64_t poll_seq{0};
  std::int64_t begin_ns{0};
  std::int64_t end_ns{0};
  std::int64_t previous_begin_ns{0};
  std::uint64_t frames{0};
  std::uint64_t packets{0};
  std::uint32_t result{0};
};

class TimingTrace final {
 public:
  using Clock = std::int64_t (*)() noexcept;

  TimingTrace(const std::string& path, std::int64_t packet_capacity,
              std::int64_t poll_capacity, std::int64_t poll_period_ns,
              Clock clock)
      : path_(path), clock_(clock), poll_period_ns_(poll_period_ns) {
    if (!path_.is_absolute() || !clock_ || poll_period_ns <= 0 ||
        packet_capacity < 1 || packet_capacity > 1000000 ||
        poll_capacity < 1 || poll_capacity > 1000000) {
      throw std::invalid_argument("invalid timing trace path/capacity/clock");
    }
    // Refuse to replace prior evidence or follow an existing output symlink.
    if (std::filesystem::exists(path_) || std::filesystem::is_symlink(path_) ||
        !std::filesystem::is_directory(path_.parent_path())) {
      throw std::invalid_argument(
          "timing_trace_path must be a new absolute directory with an existing parent");
    }
    packets_.resize(static_cast<std::size_t>(packet_capacity));
    polls_.resize(static_cast<std::size_t>(poll_capacity));
    if (!std::filesystem::create_directory(path_)) {
      throw std::runtime_error("failed to create timing trace directory");
    }
  }

  [[nodiscard]] std::int64_t now_ns() noexcept {
    const auto stamp = clock_();
    if (stamp <= 0) ++clock_errors_;
    return stamp;
  }

  [[nodiscard]] std::uint64_t packet_observed() const noexcept {
    return packet_observed_;
  }

  void record_packet(PacketTimingRow row) noexcept {
    row.packet_seq = ++packet_observed_;
    if (packet_stored_ < packets_.size()) {
      packets_[packet_stored_++] = row;
    } else {
      ++packet_overflow_;
    }
  }

  void record_poll(std::int64_t begin, std::int64_t end,
                   std::uint64_t frames, std::uint64_t packets,
                   std::uint32_t result) noexcept {
    ++poll_observed_;
    const auto gap = previous_poll_begin_ > 0 && begin >= previous_poll_begin_
                         ? begin - previous_poll_begin_ : 0;
    const auto duration = end >= begin ? end - begin : 0;
    if (gap > max_poll_gap_ns_) max_poll_gap_ns_ = gap;
    if (duration > max_poll_duration_ns_) max_poll_duration_ns_ = duration;
    if (gap > 2 * poll_period_ns_ || duration > poll_period_ns_) {
      ++poll_anomalies_;
      if (poll_stored_ < polls_.size()) {
        polls_[poll_stored_++] = PollTimingRow{
            poll_observed_, begin, end, previous_poll_begin_, frames, packets, result};
      } else {
        ++poll_overflow_;
      }
    }
    previous_poll_begin_ = begin;
  }

  // Call only after callbacks stop and the device has closed. Metadata is
  // written last: its absence means the trace is incomplete, not a passed run.
  void finish(std::uint64_t serial_read_clock_errors = 0) {
    if (finished_) return;
    auto packet_file = output("bridge_packets.csv");
    packet_file << "packet_seq,device_ms,ros_stamp_ns,frame_host_arrival_ns,"
                   "last_serial_read_end_ns,decode_begin_ns,decode_end_ns,"
                   "imu1_publish_begin_ns,imu1_publish_end_ns,"
                   "imu2_publish_begin_ns,imu2_publish_end_ns,packet_done_ns\n";
    for (std::size_t i = 0; i < packet_stored_; ++i) {
      const auto& r = packets_[i];
      packet_file << r.packet_seq << ',' << r.device_ms << ',' << r.ros_stamp_ns
                  << ',' << r.frame_host_arrival_ns << ',' << r.last_serial_read_end_ns
                  << ',' << r.decode_begin_ns << ',' << r.decode_end_ns
                  << ',' << r.imu1_publish_begin_ns << ',' << r.imu1_publish_end_ns
                  << ',' << r.imu2_publish_begin_ns << ',' << r.imu2_publish_end_ns
                  << ',' << r.packet_done_ns << '\n';
    }
    packet_file.close();
    auto poll_file = output("bridge_polls.csv");
    poll_file << "poll_seq,begin_ns,end_ns,previous_begin_ns,frames,packets,result\n";
    for (std::size_t i = 0; i < poll_stored_; ++i) {
      const auto& r = polls_[i];
      poll_file << r.poll_seq << ',' << r.begin_ns << ',' << r.end_ns << ','
                << r.previous_begin_ns << ',' << r.frames << ',' << r.packets
                << ',' << r.result << '\n';
    }
    poll_file.close();
    auto meta = output("bridge_meta.json");
    meta << "{\n  \"schema_version\": 1,\n  \"clock\": \"CLOCK_MONOTONIC\",\n"
         << "  \"completed\": true,\n  \"poll_mode\": \"anomaly\",\n"
         << "  \"packet_observed\": " << packet_observed_ << ",\n"
         << "  \"packet_stored\": " << packet_stored_ << ",\n"
         << "  \"packet_overflow\": " << packet_overflow_ << ",\n"
         << "  \"packet_capacity\": " << packets_.size() << ",\n"
         << "  \"poll_observed\": " << poll_observed_ << ",\n"
         << "  \"poll_anomalies\": " << poll_anomalies_ << ",\n"
         << "  \"poll_stored\": " << poll_stored_ << ",\n"
         << "  \"poll_overflow\": " << poll_overflow_ << ",\n"
         << "  \"poll_capacity\": " << polls_.size() << ",\n"
         << "  \"poll_gap_threshold_ns\": " << 2 * poll_period_ns_ << ",\n"
         << "  \"poll_duration_threshold_ns\": " << poll_period_ns_ << ",\n"
         << "  \"max_poll_gap_ns\": " << max_poll_gap_ns_ << ",\n"
         << "  \"max_poll_duration_ns\": " << max_poll_duration_ns_ << ",\n"
         << "  \"clock_errors\": " << clock_errors_ << ",\n"
         << "  \"serial_read_clock_errors\": " << serial_read_clock_errors << ",\n"
         << "  \"frame_stamp_semantics\": \"USB record decode time of packet-completing CAN frame\",\n"
         << "  \"serial_stamp_semantics\": \"latest successful read before delivery of packet-completing frame; not first byte or sensor sample\"\n}\n";
    meta.close();
    finished_ = true;
  }

 private:
  std::ofstream output(const char* name) {
    std::ofstream stream(path_ / name);
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    return stream;
  }

  std::filesystem::path path_;
  Clock clock_;
  std::int64_t poll_period_ns_;
  std::vector<PacketTimingRow> packets_;
  std::vector<PollTimingRow> polls_;
  std::size_t packet_stored_{0};
  std::size_t poll_stored_{0};
  std::uint64_t packet_observed_{0};
  std::uint64_t packet_overflow_{0};
  std::uint64_t poll_observed_{0};
  std::uint64_t poll_anomalies_{0};
  std::uint64_t poll_overflow_{0};
  std::uint64_t clock_errors_{0};
  std::int64_t previous_poll_begin_{0};
  std::int64_t max_poll_gap_ns_{0};
  std::int64_t max_poll_duration_ns_{0};
  bool finished_{false};
};

}  // namespace mech::mech_ctrboard_bridge
