#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <string>
#include <type_traits>
#include <vector>

namespace mech::mech_bringup {
// Version 1 wire contract is little-endian, LP64, IEEE754 double. Header is
// exactly 128 bytes; readers must check magic/version/kind/stride/file length.
// Readers may access only after the writer process exits. closed=0 means a
// crash/incomplete capture, never complete evidence, even with valid counters.
struct TraceSnapshotHeader {
  char magic[8];                     // offset 0: "MCHTRC01"
  std::uint32_t version;             // 8: 1
  std::uint32_t kind;                // 12: 1 command, 2 serial
  std::uint32_t record_stride;       // 16: 216 command, 1064 serial
  std::uint32_t header_bytes;        // 20: 128
  std::uint64_t capacity;            // 24
  std::uint64_t total;               // 32: all completed capture attempts
  std::uint64_t committed;           // 40: min(total, capacity)
  std::uint64_t dropped;             // 48: command omitted attempts
  std::uint64_t overwritten;         // 56: serial overwritten attempts
  std::uint64_t closed;              // 64: 1 only after runtime shutdown
  std::uint64_t reserved[7];         // 72..127: zero
};
static_assert(sizeof(TraceSnapshotHeader) == 128);
static_assert(offsetof(TraceSnapshotHeader, capacity) == 24);
static_assert(offsetof(TraceSnapshotHeader, closed) == 64);

constexpr std::uint64_t kTraceSnapshotBudget = 1536ULL * 1024 * 1024;
constexpr std::size_t kDefaultSnapshotChainCapacity = 4194304;
constexpr std::size_t kDefaultSnapshotRawCapacity = 524288;

// Startup-only admission: fail before a serial factory/open or activation.
// Only real directories under /dev/shm on tmpfs are accepted, without symlink
// traversal. Memory availability and tmpfs space are checked before mapping.
void admit_trace_snapshots(const std::string& directory, std::uint64_t bytes);

class TraceSnapshotMapping final {
 public:
  TraceSnapshotMapping(const std::string& path, std::size_t capacity,
                       std::size_t stride, std::uint32_t kind);
  ~TraceSnapshotMapping();
  TraceSnapshotMapping(const TraceSnapshotMapping&) = delete;
  TraceSnapshotMapping& operator=(const TraceSnapshotMapping&) = delete;
  void* records() noexcept;
  void publish(std::uint64_t total, bool ring) noexcept;
  void seal() noexcept;
 private:
  void* mapping_{nullptr};
  std::size_t bytes_{0};
};

// Same indexing API for the historical vector recorder and the opt-in mapped
// recorder. Allocation, reservation and prefault all happen in construction.
template<typename Record>
class TraceRecordStorage final {
 public:
  TraceRecordStorage(std::size_t capacity, const std::string& path,
                     std::uint32_t kind) : capacity_(capacity) {
    if (path.empty()) {
      memory_.resize(capacity);
      records_ = memory_.data();
    } else {
      mapping_ = std::make_unique<TraceSnapshotMapping>(path, capacity, sizeof(Record), kind);
      records_ = static_cast<Record*>(mapping_->records());
      static_assert(std::is_trivially_destructible<Record>::value);
      static_assert(std::is_trivially_copyable<Record>::value);
      for (std::size_t i = 0; i < capacity; ++i) ::new (records_ + i) Record{};
    }
  }
  std::size_t size() const noexcept { return capacity_; }
  Record& operator[](std::size_t i) noexcept { return records_[i]; }
  const Record& operator[](std::size_t i) const noexcept { return records_[i]; }
  void publish(std::uint64_t total, bool ring) noexcept {
    if (mapping_) mapping_->publish(total, ring);
  }
  void seal() noexcept { if (mapping_) mapping_->seal(); }
 private:
  std::vector<Record> memory_;
  std::unique_ptr<TraceSnapshotMapping> mapping_;
  Record* records_{nullptr};
  std::size_t capacity_;
};
}  // namespace mech::mech_bringup
