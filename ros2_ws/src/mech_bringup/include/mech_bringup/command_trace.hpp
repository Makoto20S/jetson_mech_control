#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <vector>
#include "mech_control_core/transport.hpp"

namespace mech::mech_bringup {
// Optional, single control-thread recorder. Allocate/touch before activation;
// no formatting, allocation or file I/O during capture. Dump after shutdown.
// Exhaustion preserves the beginning and reports every omitted record.
class CommandTrace final {
 public:
  explicit CommandTrace(std::size_t capacity = 524288U) : records_(capacity) {}
  std::uint64_t cycle{0}, generation{0}, io{0};
  std::size_t joint{0};
  static std::int64_t now() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void record(const char* stage, std::size_t index = 0, std::uint64_t gen = 0,
              double position = 0, double feedback = 0, int flags = 0,
              std::int64_t deadline = 0, int result = 0) noexcept {
    auto* r = append(stage);
    if (!r) return;
    r->joint = index; r->generation = gen; r->position = position;
    r->feedback = feedback; r->flags = flags; r->deadline = deadline;
    r->result = result;
  }
  void bytes(const char* stage, const std::uint8_t* data, std::size_t length,
             std::int64_t result, std::size_t requested, int error = 0,
             std::int64_t begin = 0, std::uint32_t id = 0) noexcept {
    auto* r = append(stage);
    if (!r) return;
    r->result = result; r->requested = requested; r->error = error;
    r->begin = begin; r->id = id;
    r->captured = data ? std::min(length, r->data.size()) : 0;
    r->length = length;
    if (data && r->captured) std::copy_n(data, r->captured, r->data.begin());
  }
  void frame(const char* stage, const mech_control_core::RawCanFrame& f,
             int result = 0) noexcept {
    bytes(stage, f.payload.data(), f.payload_size, result, f.payload_size,
          0, 0, f.id.value);
    if (total_ <= records_.size() && total_ != 0) {
      auto& r = records_[total_ - 1];
      r.flags = (f.id.format == mech_control_core::CanFrameFormat::Extended ? 4 : 0)
          | (f.type == mech_control_core::CanFrameType::FlexibleDataRate ? 2 : 0)
          | (f.bitrate_switch ? 1 : 0) | (f.remote_request ? 16 : 0);
      r.deadline = f.logical_bus;
    }
  }
  void dump(std::ostream& out) const {
    out << std::setprecision(17)
        << "{\"schema_version\":1,\"clock\":\"steady_clock_ns\",\"capacity\":"
        << records_.size() << ",\"total\":" << total_ << ",\"dropped\":"
        << (total_ > records_.size() ? total_ - records_.size() : 0) << "}\n";
    constexpr char hex[] = "0123456789abcdef";
    for (std::size_t i = 0; i < std::min(total_, records_.size()); ++i) {
      const auto& r = records_[i];
      out << "{\"seq\":" << i << ",\"stage\":\"" << r.stage
          << "\",\"ns\":" << r.ns << ",\"begin_ns\":" << r.begin
          << ",\"cycle\":" << r.cycle << ",\"joint\":" << r.joint
          << ",\"generation\":" << r.generation << ",\"io\":" << r.io
          << ",\"position\":";
      if (std::isfinite(r.position)) out << r.position; else out << "null";
      out << ",\"feedback\":";
      if (std::isfinite(r.feedback)) out << r.feedback; else out << "null";
      out << ",\"flags\":" << r.flags << ",\"deadline\":" << r.deadline
          << ",\"result\":" << r.result << ",\"errno\":" << r.error
          << ",\"requested\":" << r.requested << ",\"length\":" << r.length
          << ",\"id\":" << r.id << ",\"captured\":" << r.captured << ",\"hex\":\"";
      for (std::size_t j = 0; j < r.captured; ++j)
        out << hex[r.data[j] >> 4] << hex[r.data[j] & 15];
      out << "\"}\n";
    }
  }
 private:
  struct Record {
    const char* stage{};  // all callers supply static string literals
    std::int64_t ns{}, begin{}, deadline{}, result{};
    std::uint64_t cycle{}, generation{}, io{};
    std::size_t joint{}, requested{}, length{}, captured{};
    double position{}, feedback{};
    int flags{}, error{};
    std::uint32_t id{};
    std::array<std::uint8_t, 64> data{};
  };
  Record* append(const char* stage) noexcept;
  std::vector<Record> records_;
  std::size_t total_{0};
};

// Observe the frame the actual BusRuntime selected, immediately around the
// production USB encoder/transport. Never substitute a reconstructed command.
class CommandTraceTransport final : public mech_control_core::Transport {
 public:
  CommandTraceTransport(mech_control_core::Transport& inner, CommandTrace& trace)
      : inner_(inner), trace_(trace) {}
  mech_control_core::TransportKind kind() const noexcept override { return inner_.kind(); }
  const mech_control_core::TransportCapabilities& capabilities() const noexcept override {
    return inner_.capabilities();
  }
  bool is_open() const noexcept override { return inner_.is_open(); }
  bool open() noexcept override { return inner_.open(); }
  void close() noexcept override { inner_.close(); }
  mech_control_core::TransportStats stats() const noexcept override { return inner_.stats(); }
  mech_control_core::TransportResult try_receive(mech_control_core::RawCanFrame& f) noexcept override {
    return inner_.try_receive(f);
  }
  mech_control_core::TransportResult try_send(const mech_control_core::RawCanFrame& f) noexcept override {
    trace_.frame("transport_send", f);
    const auto result = inner_.try_send(f);
    trace_.frame("transport_result", f, static_cast<int>(result));
    return result;
  }
 private:
  mech_control_core::Transport& inner_;
  CommandTrace& trace_;
};
}  // namespace mech::mech_bringup
