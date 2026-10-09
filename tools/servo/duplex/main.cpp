#include "duplex.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#ifdef __linux__
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/sysmacros.h>
#include <sys/wait.h>
#include <unistd.h>
#include "mech_bringup/posix_cdc_serial_port.hpp"
#endif

namespace {
namespace fs = std::filesystem;
volatile std::sig_atomic_t stopped = 0;
void stop_handler(int) { stopped = 1; }
void require(bool condition, const std::string& error) {
  if (!condition) throw std::runtime_error(error);
}
std::string quote(const std::string& value) {
  std::ostringstream out; out << '"';
  const char* hex = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u00" << hex[c >> 4] << hex[c & 15];
    else out << c;
  }
  out << '"'; return out.str();
}
unsigned decimal(const std::string& value) {
  require(!value.empty(), "empty numeric argument");
  unsigned result = 0;
  for (unsigned char c : value) {
    require(c >= '0' && c <= '9', "numeric arguments require unsigned decimal digits");
    require(result <= (std::numeric_limits<unsigned>::max() - (c-'0')) / 10,
            "numeric argument overflow");
    result = result * 10 + (c-'0');
  }
  return result;
}
struct Options {
  duplex::Config config;
  bool run{false}, disconnected{false}, help{false};
  std::string port_a, port_b, output;
};
Options parse(int argc, char** argv) {
  Options o;
  std::set<std::string> seen;
  bool nonce_given = false;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    require(seen.insert(key).second, "duplicate option: " + key);
    if (key == "--feedback-sequence") { o.config.feedback_sequence = true; continue; }
    if (key == "--mixed-receive") { o.config.mixed_receive = true; continue; }
    if (key == "--run") { o.run = true; continue; }
    if (key == "--motors-disconnected") { o.disconnected = true; continue; }
    if (key == "--help") { o.help = true; continue; }
    require(key == "--port-a" || key == "--port-b" || key == "--output" ||
        key == "--command-hz" || key == "--feedback-hz" || key == "--feedback-order" ||
        key == "--feedback-phase-ms" || key == "--rx-gate-ms" || key == "--seconds" ||
        key == "--drain-ms" || key == "--log-mib" || key == "--nonce" || key == "--replay-schedule", "unknown option: " + key);
    require(i + 1 < argc, "missing value: " + key);
    const std::string value = argv[++i];
    require(!value.empty() && value.rfind("--", 0) != 0, "missing value: " + key);
    if (key == "--replay-schedule") o.config.replay = duplex::load_replay_schedule(value);
    else if (key == "--port-a") o.port_a = value;
    else if (key == "--port-b") o.port_b = value;
    else if (key == "--output") o.output = value;
    else if (key == "--command-hz") o.config.hz = decimal(value);
    else if (key == "--feedback-hz") o.config.feedback_hz = decimal(value);
    else if (key == "--feedback-phase-ms") o.config.feedback_phase_ms = decimal(value);
    else if (key == "--rx-gate-ms") o.config.rx_gate_ms = decimal(value);
    else if (key == "--feedback-order") {
      require(value == "forward" || value == "reverse", "feedback order must be forward or reverse");
      o.config.feedback_reverse = value == "reverse";
    }
    else if (key == "--seconds") o.config.seconds = decimal(value);
    else if (key == "--drain-ms") o.config.drain_ms = decimal(value);
    else if (key == "--log-mib") o.config.log_mib = decimal(value);
    else { o.config.nonce = decimal(value); nonce_given = true; }
  }
  if (!nonce_given) {
    std::random_device entropy;
    o.config.nonce = static_cast<std::uint32_t>(entropy());
  }
  o.config.validate();
  if (o.run) {
    require(o.disconnected, "--run requires --motors-disconnected");
    require(!o.port_a.empty() && !o.port_b.empty() && !o.output.empty(),
            "--run requires --port-a, --port-b and a new --output directory");
    require(o.port_a != o.port_b, "two distinct physical USB boards are required");
  }
  return o;
}
std::string plan(const Options& o, const std::string& identity_a = "null",
                 const std::string& identity_b = "null", const std::string& boot = "",
                 std::int64_t admission_ns = 0) {
  auto text = duplex::plan_json(o.config);
  text.pop_back();
  return text + ",\"run_requested\":" + (o.run ? "true" : "false") +
      ",\"motors_disconnected_declared\":" + (o.disconnected ? "true" : "false") +
      ",\"port_a_by_path\":" + quote(o.port_a) + ",\"port_b_by_path\":" + quote(o.port_b) +
      ",\"output\":" + quote(o.output) + ",\"actual_identity_a\":" + identity_a +
      ",\"actual_identity_b\":" + identity_b + ",\"boot_id\":" + (boot.empty() ? "null" : quote(boot)) +
      ",\"clock\":\"steady_clock_monotonic\",\"admission_monotonic_ns\":" +
      std::to_string(admission_ns) + ",\"can_wire_timestamp_available\":false}";
}
const char* help =
    "servo_duplex: default prints JSON plan; no device or output is opened.\n"
    "--run --motors-disconnected --port-a /dev/serial/by-path/NAME\n"
    "--port-b /dev/serial/by-path/NAME --output NEW_DIRECTORY\n"
    "--command-hz 1..500 --feedback-hz 0..50 --seconds 1..60\n"
    "--feedback-order forward|reverse (default forward; B writes only)\n"
    "--feedback-phase-ms 0|5 (default 0; B deadline offset only)\n"
    "--rx-gate-ms 0|6 (default 0; 6 requires 2s, 10/10Hz, forward)\n"
    "--replay-schedule FILE (reviewed 5500-event file; requires 5s,500/50Hz,no phase/gate/reverse)\n"
    "--drain-ms 1000..5000 --log-mib 1..64 (per port) --nonce 0..4294967295 --help\n"
    "Fixed separate 21-byte CAN writes, mode6 104=84.1/105=95.9, quiet preflight 2s.\n"
    "Live execution is Linux-only; root is required for complete /proc inspection.\n";

#ifdef __linux__
struct Identity {
  std::string by_path, tty, usb, interface, vendor, product, serial, bus_number, device_number;
  dev_t device{};
  std::string json() const {
    return "{\"by_path\":" + quote(by_path) + ",\"tty\":" + quote(tty) +
        ",\"usb_parent\":" + quote(usb) + ",\"interface\":" + quote(interface) +
        ",\"vendor\":" + quote(vendor) + ",\"product\":" + quote(product) +
        ",\"serial\":" + quote(serial) + ",\"usb_bus_number\":" + quote(bus_number) +
        ",\"usb_device_number\":" + quote(device_number) + ",\"firmware\":null}";
  }
};
std::string attribute(const fs::path& path) {
  std::ifstream in(path);
  require(in.is_open(), "cannot read identity attribute: " + path.string());
  std::string value; std::getline(in, value);
  require(!in.bad(), "identity attribute read failed");
  return value;
}
Identity identify(const std::string& name) {
  const fs::path path(name);
  require(path.parent_path() == fs::path("/dev/serial/by-path") &&
      path.filename() != "." && path.filename() != "..",
      "endpoint must be a direct /dev/serial/by-path symlink");
  require(fs::is_symlink(fs::symlink_status(path)), "by-path endpoint is not a symlink");
  Identity id; id.by_path = name; id.tty = fs::canonical(path).string();
  const fs::path tty(id.tty);
  const auto leaf = tty.filename().string();
  require(tty.parent_path() == "/dev" && leaf.rfind("ttyACM", 0) == 0 &&
      leaf.size() > 6 && leaf.find_first_not_of("0123456789", 6) == std::string::npos,
      "endpoint does not resolve to a ttyACM device");
  struct stat st{};
  require(::stat(id.tty.c_str(), &st) == 0 && S_ISCHR(st.st_mode), "TTY is not a character device");
  id.device = st.st_rdev;
  auto device = fs::canonical(fs::path("/sys/class/tty") / leaf / "device");
  id.interface = device.string();
  for (auto ancestor = device; ancestor != ancestor.root_path(); ancestor = ancestor.parent_path()) {
    if (!fs::exists(ancestor / "idVendor")) continue;
    id.usb = ancestor.string(); id.vendor = attribute(ancestor / "idVendor");
    id.product = attribute(ancestor / "idProduct");
    id.bus_number = attribute(ancestor / "busnum");
    id.device_number = attribute(ancestor / "devnum");
    if (fs::exists(ancestor / "serial")) id.serial = attribute(ancestor / "serial");
    break;
  }
  require(!id.usb.empty() && id.vendor == "caf1" && id.product == "ffff",
          "endpoint USB parent must be caf1:ffff");
  // Cross-check the sysfs device number rather than trusting a path name.
  const auto sysdev = attribute(fs::path("/sys/class/tty") / leaf / "dev");
  require(sysdev == std::to_string(major(id.device)) + ":" + std::to_string(minor(id.device)),
          "sysfs and TTY device numbers disagree");
  return id;
}
bool numeric_name(const fs::path& path) {
  const auto s = path.filename().string();
  return !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
}
void inspect_fds(const fs::path& task, const Identity& a,
                 const fs::path& sys_char_root = "/sys/dev/char") {
  const auto directory = task / "fd";
  std::error_code error;
  fs::directory_iterator it(directory, error), end;
  if (error) {
    // An exited task is the only permitted unreadable-directory exception.
    require(error == std::errc::no_such_file_or_directory && !fs::exists(task),
            "cannot inspect all process descriptors: " + directory.string());
    return;
  }
  while (it != end) {
    const auto fd = it->path();
    struct stat st{};
    if (::stat(fd.c_str(), &st) != 0) {
      require(errno == ENOENT, "unreadable process descriptor: " + fd.string());
    } else if (S_ISCHR(st.st_mode)) {
      require(st.st_rdev != a.device,
              "selected channel already occupied: " + fd.string());
      // Any occupied CDC interface of this physical board excludes this run.
      const auto sys = sys_char_root /
          (std::to_string(major(st.st_rdev)) + ":" + std::to_string(minor(st.st_rdev))) / "device";
      if (fs::exists(sys)) {
        auto ancestor = fs::canonical(sys);
        for (; ancestor != ancestor.root_path(); ancestor = ancestor.parent_path())
          require(ancestor.string() != a.usb,
                  "selected board has an occupied interface: " + fd.string());
      }
    }
    it.increment(error);
    require(!error, "process descriptor scan failed: " + directory.string());
  }
}
void inspect_owners(const Identity& a) {
  require(::geteuid() == 0, "live admission requires root for full /proc inspection");
  for (const auto& process : fs::directory_iterator("/proc")) {
    if (!numeric_name(process.path())) continue;
    const auto tasks = process.path() / "task";
    std::error_code error;
    fs::directory_iterator it(tasks, error), end;
    if (error) {
      require(error == std::errc::no_such_file_or_directory && !fs::exists(process.path()),
              "cannot inspect process tasks: " + tasks.string());
      continue;
    }
    while (it != end) {
      if (numeric_name(it->path())) inspect_fds(it->path(), a);
      it.increment(error);
      require(!error, "process task scan failed");
    }
  }
}
struct BoardLock {
  int fd{-1};
  explicit BoardLock(const Identity& id) {
    const auto name = fs::path(id.usb).filename().string();
    require(!name.empty() && name.find_first_not_of("0123456789-.") == std::string::npos,
            "invalid physical USB identity for board lock");
    const auto path = fs::path("/run/lock") / ("mech-servo-observer-usb-"+name+".lock");
    fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    require(fd >= 0, "cannot open runtime board lock");
    struct stat st{};
    if (::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != 0 || st.st_nlink != 1 ||
        (st.st_mode & 0022) != 0 || ::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd); fd = -1;
      throw std::runtime_error("unsafe or occupied runtime board lock");
    }
  }
  ~BoardLock() { if (fd >= 0) ::close(fd); }
  BoardLock(const BoardLock&) = delete;
  BoardLock& operator=(const BoardLock&) = delete;
};
bool same_identity(const Identity& a, const Identity& b) {
  return a.by_path == b.by_path && a.tty == b.tty && a.usb == b.usb &&
      a.interface == b.interface && a.device == b.device && a.vendor == b.vendor &&
      a.product == b.product && a.serial == b.serial && a.bus_number == b.bus_number &&
      a.device_number == b.device_number;
}
std::int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      duplex::Clock::now().time_since_epoch()).count();
}
void write_new(const fs::path& path, const char* data, std::size_t size) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
  require(fd >= 0, "cannot create evidence file: " + path.string());
  std::size_t offset = 0;
  while (offset < size) {
    const auto n = ::write(fd, data + offset, size-offset);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) { ::close(fd); throw std::runtime_error("evidence write failed: " + path.string()); }
    offset += static_cast<std::size_t>(n);
  }
  const auto synced = ::fsync(fd);
  const auto closed = ::close(fd);
  require(synced == 0 && closed == 0, "evidence persistence failed: " + path.string());
  const int directory = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  require(directory >= 0, "cannot open evidence directory for persistence");
  const auto directory_synced = ::fsync(directory);
  const auto directory_closed = ::close(directory);
  require(directory_synced == 0 && directory_closed == 0, "evidence directory persistence failed");
}
void write_new(const fs::path& path, const std::string& data) {
  write_new(path, data.data(), data.size());
}
int worker(const Options& o, const Identity& admitted_a, const Identity& admitted_b,
           const std::string& boot) {
  duplex::Capture capture_a(static_cast<std::size_t>(o.config.log_mib)*1024*1024);
  duplex::Capture capture_b(static_cast<std::size_t>(o.config.log_mib)*1024*1024);
  duplex::Result result;
  {
    mech::mech_bringup::PosixCdcSerialPort port_a(admitted_a.tty), port_b(admitted_b.tty);
    try {
      require(!stopped, "interrupted_before_open");
      const auto before_a = identify(o.port_a), before_b = identify(o.port_b);
      require(same_identity(before_a, admitted_a) && same_identity(before_b, admitted_b),
              "device identity changed during admission");
      inspect_owners(before_a); inspect_owners(before_b);
      require(!stopped, "interrupted_before_open");
      require(port_a.open(), "port_a_open_failed");
      require(port_b.open(), "port_b_open_failed");
      require(same_identity(identify(o.port_a), admitted_a) &&
          same_identity(identify(o.port_b), admitted_b), "device identity changed while opening ports");
      result = duplex::run(port_a, port_b, o.config, capture_a, capture_b,
          [] { return stopped != 0; }, [&] {
        require(!stopped, "interrupted_before_ready");
        const auto ready = std::string("{\"ready\":true,\"role\":\"duplex\",\"boot_id\":") + quote(boot) +
            ",\"monotonic_ns\":" + std::to_string(monotonic_ns()) +
            ",\"worker_pid\":" + std::to_string(::getpid()) + ",\"actual_identity_a\":" +
            admitted_a.json() + ",\"actual_identity_b\":" + admitted_b.json() + "}";
        write_new(fs::path(o.output) / "ready.json", ready+"\n");
      });
    } catch (const std::exception& error) { result.error = error.what(); }
    // Covers all open failures before the core close guard exists.
    port_b.close(); port_a.close();
  }
  const fs::path output(o.output);
  const auto& a = capture_a.bytes(); const auto& b = capture_b.bytes();
  write_new(output / "capture-a.bin", reinterpret_cast<const char*>(a.data()), a.size());
  write_new(output / "capture-b.bin", reinterpret_cast<const char*>(b.data()), b.size());
  const auto summary = duplex::result_json(result, a.size(), b.size());
  write_new(output / "summary.json", summary+"\n");
  std::cout << summary << std::endl;
  return result.complete() ? (result.content_match ? 0 : 4) : 1;
}
struct WorkerGuard {
  pid_t pid;
  bool reaped{false};
  ~WorkerGuard() {
    if (reaped) return;
    int status = 0;
    const auto state = ::waitpid(pid, &status, WNOHANG);
    if (state == pid || (state < 0 && errno == ECHILD)) return;
    // Any parent exception must stop transmission before unwinding further.
    ::kill(pid, SIGKILL);
    const auto end = duplex::Clock::now()+std::chrono::seconds(1);
    while (duplex::Clock::now() < end) {
      if (::waitpid(pid, &status, WNOHANG) == pid) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};
int supervise(pid_t child, const fs::path& output, const std::string& p,
              std::chrono::milliseconds duration) {
  WorkerGuard guard{child};
  // This parent never opens a TTY. No alarm handler can terminate capture export.
  auto deadline = duplex::Clock::now() + duration;
  bool forwarded = false, forced = false, reaped = false;
  int status = 0;
  while (duplex::Clock::now() < deadline) {
    const auto state = ::waitpid(child, &status, WNOHANG);
    if (state == child) { reaped = true; guard.reaped = true; break; }
    if (state < 0 && errno == ECHILD) guard.reaped = true;
    require(state >= 0 || errno == EINTR, "diagnostic supervisor wait failed");
    if (stopped && !forwarded) {
      ::kill(child, SIGTERM); forwarded = true;
      deadline = std::min(deadline, duplex::Clock::now()+std::chrono::seconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!reaped) {
    forced = true; ::kill(child, SIGKILL);
    const auto kill_deadline = duplex::Clock::now()+std::chrono::seconds(1);
    while (duplex::Clock::now() < kill_deadline) {
      if (::waitpid(child, &status, WNOHANG) == child) {
        reaped = true; guard.reaped = true; break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  const int worker_exit = reaped && WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  const bool exported = reaped && !forced && WIFEXITED(status) &&
      (worker_exit == 0 || worker_exit == 1 || worker_exit == 4) &&
      fs::is_regular_file(output / "summary.json") &&
      fs::is_regular_file(output / "plan.json") && fs::is_regular_file(output / "capture-a.bin") &&
      fs::is_regular_file(output / "capture-b.bin") &&
      fs::is_regular_file(output / "ready.json");
  const bool success = reaped && !forced && WIFEXITED(status) && WEXITSTATUS(status) == 0 && exported;
  const bool collection = exported && (worker_exit == 0 || worker_exit == 4);
  const auto supervisory = std::string("{\"complete\":") + (success ? "true" : "false") +
      ",\"collection_complete\":" + (collection ? "true" : "false") +
      ",\"worker_exit_code\":" + std::to_string(worker_exit) +
      ",\"worker_reaped\":" + (reaped ? "true" : "false") +
      ",\"ports_close_verified\":" + (reaped ? "true" : "false") +
      ",\"forced_timeout\":" + (forced ? "true" : "false") +
      ",\"export_valid\":" + (exported ? "true" : "false") +
      ",\"capture_exported\":" + (exported ? "true" : "false") +
      ",\"worker_pid\":" + std::to_string(child) + "}";
  // supervisor.json is incident metadata, never a substitute for raw capture.
  write_new(output / "supervisor.json", supervisory + "\n");
  if (!exported && reaped) {
    if (!fs::exists(output / "plan.json")) write_new(output / "plan.json", p+"\n");
    if (!fs::exists(output / "summary.json")) write_new(output / "summary.json",
        "{\"complete\":false,\"error\":\"worker_terminated_or_export_failed\",\"capture_recoverable\":false}\n");
  }
  if (!success) std::cerr << supervisory << std::endl;
  return success ? 0 : (collection && worker_exit == 4 ? 4 : 1);
}
int live(const Options& o) {
  require(::geteuid() == 0, "live admission requires root for full /proc inspection");
  const auto identity_a = identify(o.port_a), identity_b = identify(o.port_b);
  require(identity_a.usb != identity_b.usb && identity_a.device != identity_b.device,
          "two distinct physical USB boards are required, not interfaces of one board");
  // Nonblocking locks share the observer prefix and coordinate both new tools.
  BoardLock board_lock_a(identity_a); BoardLock board_lock_b(identity_b);
  inspect_owners(identity_a); inspect_owners(identity_b);
  require(!stopped, "interrupted_before_output");
  const fs::path output(o.output);
  require(!fs::exists(output), "output directory already exists; evidence is never overwritten");
  const auto parent = output.has_parent_path() ? output.parent_path() : fs::path(".");
  const auto free_bytes = fs::space(parent).available;
  const std::uintmax_t required_bytes = 2*static_cast<std::uintmax_t>(o.config.log_mib)*1024*1024 +
      1024U*1024U + 1024ULL*1024ULL*1024ULL;
  require(free_bytes >= required_bytes, "insufficient free space: capture plus metadata and 1 GiB reserve required");
  require(fs::create_directory(output), "cannot create unique output directory (parent must exist)");
  const auto boot = attribute("/proc/sys/kernel/random/boot_id");
  require(boot.size() == 36 && boot.find_first_not_of("0123456789abcdef-") == std::string::npos,
          "invalid kernel boot identity");
  const auto p = plan(o, identity_a.json(), identity_b.json(), boot, monotonic_ns());
  // Persist admission metadata before any TTY opens; raw capture is exported after close.
  write_new(output / "admission.json", p+"\n");
  write_new(output / "plan.json", p+"\n");
  const auto parent_pid = ::getpid();
  const auto child = ::fork();
  require(child >= 0, "cannot start supervised diagnostic process");
  if (child == 0) {
    // Prevent a parent crash or SIGKILL from leaving an unsupervised transmitter.
    if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || ::getppid() != parent_pid) ::_exit(3);
    int code = 3;
    try { code = worker(o,identity_a,identity_b,boot); }
    catch (const std::exception& error) {
      std::cerr << "evidence export failed: " << error.what() << std::endl;
    }
    ::_exit(code);
  }
  return supervise(child, output, p, std::chrono::seconds(o.config.seconds+17) +
      std::chrono::milliseconds(o.config.drain_ms));
}
#endif
}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse(argc,argv);
    if (options.help) { std::cout << help; return 0; }
    if (!options.run) { std::cout << plan(options) << '\n'; return 0; }
    std::signal(SIGINT, stop_handler); std::signal(SIGTERM, stop_handler);
#ifdef __linux__
    return live(options);
#else
    throw std::runtime_error("live diagnostic requires Linux; this build is offline-only");
#endif
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n'; return 2;
  }
}
