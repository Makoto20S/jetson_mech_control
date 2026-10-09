#include "mech_bringup/trace_snapshot.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <limits>
#include <new>
#include <sstream>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <unistd.h>

namespace mech::mech_bringup {
namespace {
constexpr long kTmpfsMagic = 0x01021994;
class DirectoryFd final {
 public:
  explicit DirectoryFd(const std::string& directory) {
    if (directory != "/dev/shm" && directory.compare(0, 9, "/dev/shm/") != 0)
      throw std::invalid_argument("snapshot directory must be under /dev/shm");
    fd_ = ::open("/dev/shm", O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (fd_ < 0) throw std::runtime_error("cannot open /dev/shm");
    try {
      for (std::size_t start = 9; start < directory.size();) {
        const auto end = directory.find('/', start);
        const auto component = directory.substr(start, end - start);
        if (component.empty() || component == "." || component == "..")
          throw std::invalid_argument("invalid snapshot directory component");
        const int next = ::openat(fd_, component.c_str(),
            O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        if (next < 0) throw std::runtime_error("cannot open snapshot directory without symlinks");
        ::close(fd_); fd_ = next;
        if (end == std::string::npos) break;
        start = end + 1;
      }
      struct statfs fs{};
      if (::fstatfs(fd_, &fs) != 0 || fs.f_type != kTmpfsMagic)
        throw std::invalid_argument("snapshot directory is not tmpfs");
    } catch (...) { ::close(fd_); fd_ = -1; throw; }
  }
  ~DirectoryFd() { if (fd_ >= 0) ::close(fd_); }
  int get() const noexcept { return fd_; }
 private:
  int fd_{-1};
};
std::uint64_t available_memory() {
  std::ifstream input("/proc/meminfo");
  std::string key, unit;
  std::uint64_t value = 0;
  std::uint64_t available = 0;
  while (input >> key >> value >> unit) {
    if (key == "MemAvailable:" && unit == "kB") { available = value * 1024; break; }
  }
  if (!available)
    throw std::runtime_error("cannot determine available memory for trace admission");
  // MemAvailable is host-wide. Container limits may be much smaller; include
  // every visible ancestor limit, both for cgroup v2 and legacy memory v1.
  const auto headroom = [&available](const std::string& directory, bool v2) {
    std::ifstream limit_file(directory + (v2 ? "/memory.max" : "/memory.limit_in_bytes"));
    if (!limit_file.is_open()) return;
    std::string limit_text;
    if (!(limit_file >> limit_text)) throw std::runtime_error("cannot read cgroup memory limit");
    if (limit_text == "max") return;
    std::uint64_t limit = 0;
    std::istringstream parsed(limit_text);
    if (!(parsed >> limit)) throw std::runtime_error("invalid cgroup memory limit");
    std::ifstream usage_file(directory + (v2 ? "/memory.current" : "/memory.usage_in_bytes"));
    std::uint64_t usage = 0;
    if (!(usage_file >> usage)) throw std::runtime_error("cannot read cgroup memory usage");
    available = std::min(available, limit > usage ? limit - usage : 0);
  };
  headroom("/sys/fs/cgroup", true);
  std::ifstream groups("/proc/self/cgroup");
  std::string row;
  while (std::getline(groups, row)) {
    const auto first = row.find(':');
    const auto second = row.find(':', first == std::string::npos ? 0 : first + 1);
    if (first == std::string::npos || second == std::string::npos) continue;
    const auto controllers = row.substr(first + 1, second - first - 1);
    const bool v2 = controllers.empty();
    if (!v2 && ("," + controllers + ",").find(",memory,") == std::string::npos) continue;
    const auto relative = row.substr(second + 1);
    if (relative.empty() || relative[0] != '/' || relative.find("..") != std::string::npos) continue;
    const std::string root = v2 ? "/sys/fs/cgroup" : "/sys/fs/cgroup/memory";
    std::string directory = root + (relative == "/" ? "" : relative);
    for (;;) {
      headroom(directory, v2);
      if (directory == root) break;
      const auto slash = directory.find_last_of('/');
      if (slash < root.size()) break;
      directory.resize(slash);
    }
  }
  return available;
}
}  // namespace

void admit_trace_snapshots(const std::string& directory, std::uint64_t bytes) {
  if (bytes == 0 || bytes > kTraceSnapshotBudget)
    throw std::invalid_argument("combined trace snapshot budget exceeds 1536 MiB");
  DirectoryFd dir(directory);
  struct statvfs space{};
  if (::fstatvfs(dir.get(), &space) != 0)
    throw std::runtime_error("cannot determine tmpfs space");
  const auto available = static_cast<std::uint64_t>(space.f_bavail) * space.f_frsize;
  const auto reserve = std::min<std::uint64_t>(available / 8, 16ULL * 1024 * 1024);
  if (available < bytes || available - bytes < reserve)
    throw std::runtime_error("insufficient tmpfs space for trace snapshots");
  constexpr std::uint64_t memory_reserve = 256ULL * 1024 * 1024;
  const auto memory = available_memory();
  if (memory < bytes || memory - bytes < memory_reserve)
    throw std::runtime_error("insufficient available memory for trace snapshots");
}

TraceSnapshotMapping::TraceSnapshotMapping(const std::string& path,
    std::size_t capacity, std::size_t stride, std::uint32_t kind) {
  const std::uint16_t endian = 1;
  if (*reinterpret_cast<const std::uint8_t*>(&endian) != 1 || sizeof(void*) != 8 ||
      !std::numeric_limits<double>::is_iec559 || capacity == 0 ||
      (kind != 1 && kind != 2) ||
      stride > (kTraceSnapshotBudget - sizeof(TraceSnapshotHeader)) / capacity)
    throw std::invalid_argument("unsupported trace snapshot layout/capacity");
  const auto slash = path.find_last_of('/');
  if (slash == std::string::npos) throw std::invalid_argument("absolute snapshot path required");
  DirectoryFd dir(path.substr(0, slash));
  const auto name = path.substr(slash + 1);
  const std::string prefix = kind == 1 ? "chain" : "serial";
  bool supported = name == prefix + ".snapshot";
  if (!supported && name.size() > prefix.size() + 10 &&
      name.compare(0, prefix.size() + 1, prefix + "-") == 0 &&
      name.compare(name.size() - 9, 9, ".snapshot") == 0) {
    const auto suffix = name.substr(prefix.size() + 1, name.size() - prefix.size() - 10);
    supported = suffix.size() <= 32 && suffix.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyz0123456789-") == std::string::npos;
  }
  if (!supported)
    throw std::invalid_argument("unsupported snapshot filename");
  const int fd = ::openat(dir.get(), name.c_str(),
      O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0) throw std::runtime_error("cannot exclusively create trace snapshot");
  bytes_ = sizeof(TraceSnapshotHeader) + capacity * stride;
  try {
    // Reserve tmpfs pages before prefault: failures surface at startup, rather
    // than SIGBUS in capture when another writer fills the shared filesystem.
    const int allocated = ::posix_fallocate(fd, 0, static_cast<off_t>(bytes_));
    if (allocated != 0) throw std::runtime_error("cannot reserve trace snapshot pages");
    mapping_ = ::mmap(nullptr, bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapping_ == MAP_FAILED) { mapping_ = nullptr; throw std::bad_alloc(); }
    // Startup-only touching; capture must not allocate or grow storage.
    std::memset(mapping_, 0, bytes_);
    auto* header = ::new (mapping_) TraceSnapshotHeader{};
    std::memcpy(header->magic, "MCHTRC01", 8);
    header->version = 1; header->kind = kind;
    header->record_stride = static_cast<std::uint32_t>(stride);
    header->header_bytes = sizeof(TraceSnapshotHeader);
    header->capacity = capacity;
    ::close(fd);
  } catch (...) {
    if (mapping_) { ::munmap(mapping_, bytes_); mapping_ = nullptr; }
    ::close(fd); ::unlinkat(dir.get(), name.c_str(), 0); throw;
  }
}
TraceSnapshotMapping::~TraceSnapshotMapping() {
  // Never seal implicitly: object destruction after failed initialization is
  // not proof that the runtime completed shutdown. No msync/formatting here.
  if (mapping_) ::munmap(mapping_, bytes_);
}
void* TraceSnapshotMapping::records() noexcept {
  return static_cast<std::uint8_t*>(mapping_) + sizeof(TraceSnapshotHeader);
}
void TraceSnapshotMapping::publish(std::uint64_t total, bool ring) noexcept {
  auto* header = static_cast<TraceSnapshotHeader*>(mapping_);
  std::atomic_thread_fence(std::memory_order_release);
  header->committed = std::min(total, header->capacity);
  header->dropped = ring ? 0 : total - header->committed;
  header->overwritten = ring ? total - header->committed : 0;
  header->total = total;
}
void TraceSnapshotMapping::seal() noexcept {
  std::atomic_thread_fence(std::memory_order_release);
  static_cast<TraceSnapshotHeader*>(mapping_)->closed = 1;
}
}  // namespace mech::mech_bringup
