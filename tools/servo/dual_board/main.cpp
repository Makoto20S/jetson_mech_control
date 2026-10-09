#include "diagnostic.hpp"

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
  dual_board::Config config;
  bool run{false}, disconnected{false}, help{false};
  std::string tx, rx, output;
};
Options parse(int argc, char** argv) {
  Options o;
  std::set<std::string> seen;
  bool nonce_given = false;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    require(seen.insert(key).second, "duplicate option: " + key);
    if (key == "--run") { o.run = true; continue; }
    if (key == "--motors-disconnected") { o.disconnected = true; continue; }
    if (key == "--help") { o.help = true; continue; }
    require(key == "--tx" || key == "--rx" || key == "--output" ||
        key == "--profile" || key == "--packing" || key == "--lanes" ||
        key == "--hz" || key == "--seconds" || key == "--drain-ms" ||
        key == "--log-mib" || key == "--nonce", "unknown option: " + key);
    require(i + 1 < argc, "missing value: " + key);
    const std::string value = argv[++i];
    require(!value.empty() && value.rfind("--", 0) != 0, "missing value: " + key);
    if (key == "--tx") o.tx = value;
    else if (key == "--rx") o.rx = value;
    else if (key == "--output") o.output = value;
    else if (key == "--profile") o.config.profile = value;
    else if (key == "--packing") o.config.packing = value;
    else if (key == "--lanes") o.config.lanes = decimal(value);
    else if (key == "--hz") o.config.hz = decimal(value);
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
  require(o.tx.empty() || o.rx.empty() || o.tx != o.rx, "tx and rx must differ");
  if (o.run) {
    require(o.disconnected, "--run requires --motors-disconnected");
    require(!o.tx.empty() && !o.rx.empty() && !o.output.empty(),
            "--run requires --tx, --rx and a new --output directory");
  }
  return o;
}
std::string plan(const Options& o, const std::string& identities = "null") {
  auto text = dual_board::plan_json(o.config);
  text.pop_back();
  return text + ",\"run_requested\":" + (o.run ? "true" : "false") +
      ",\"motors_disconnected_declared\":" + (o.disconnected ? "true" : "false") +
      ",\"tx_by_path\":" + quote(o.tx) + ",\"rx_by_path\":" + quote(o.rx) +
      ",\"output\":" + quote(o.output) + ",\"actual_identities\":" + identities + "}";
}
const char* help =
    "servo_dual_board: default prints JSON plan; no device or output is opened.\n"
    "--run --motors-disconnected --tx /dev/serial/by-path/NAME\n"
    "  --rx /dev/serial/by-path/OTHER --output NEW_DIRECTORY\n"
    "--profile sequence|constant|mode6 --packing separate|joined|batch\n"
    "--lanes 1|2 --hz 1..500 --seconds 1..120 --drain-ms 200..5000\n"
    "--log-mib 1..64 --nonce 0..4294967295 --help\n"
    "Live execution is Linux-only and requires root for complete /proc inspection.\n";

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
void inspect_fds(const fs::path& task, const Identity& a, const Identity& b) {
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
      require(st.st_rdev != a.device && st.st_rdev != b.device,
              "selected channel already occupied: " + fd.string());
      // Other CDC interfaces of either selected physical board also count.
      const auto sys = fs::path("/sys/dev/char") /
          (std::to_string(major(st.st_rdev)) + ":" + std::to_string(minor(st.st_rdev))) / "device";
      if (fs::exists(sys)) {
        auto ancestor = fs::canonical(sys);
        for (; ancestor != ancestor.root_path(); ancestor = ancestor.parent_path())
          require(ancestor.string() != a.usb && ancestor.string() != b.usb,
                  "selected board has an occupied interface: " + fd.string());
      }
    }
    it.increment(error);
    require(!error, "process descriptor scan failed: " + directory.string());
  }
}
void inspect_owners(const Identity& a, const Identity& b) {
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
      if (numeric_name(it->path())) inspect_fds(it->path(), a, b);
      it.increment(error);
      require(!error, "process task scan failed");
    }
  }
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
}
void write_new(const fs::path& path, const std::string& data) {
  write_new(path, data.data(), data.size());
}
int worker(const Options& o, const Identity& txid, const Identity& rxid, const std::string& p) {
  dual_board::Capture capture(static_cast<std::size_t>(o.config.log_mib)*1024*1024);
  dual_board::Result result(o.config);
  {
    mech::mech_bringup::PosixCdcSerialPort tx(txid.tty), rx(rxid.tty);
    try {
      require(!stopped, "interrupted_before_open");
      // Re-check after fork and before opening either channel. This is still read-only.
      const auto a = identify(o.tx), b = identify(o.rx);
      require(a.tty == txid.tty && b.tty == rxid.tty && a.usb == txid.usb &&
          b.usb == rxid.usb && a.device == txid.device && b.device == rxid.device &&
          a.device_number == txid.device_number && b.device_number == rxid.device_number,
          "device identity changed during admission");
      inspect_owners(a,b);
      require(!stopped, "interrupted_before_open");
      require(tx.open(), "sender_open_failed");
      require(rx.open(), "receiver_open_failed");
      const auto opened_a = identify(o.tx), opened_b = identify(o.rx);
      require(opened_a.tty == txid.tty && opened_b.tty == rxid.tty &&
          opened_a.usb == txid.usb && opened_b.usb == rxid.usb &&
          opened_a.device_number == txid.device_number && opened_b.device_number == rxid.device_number,
          "device identity changed while opening ports");
      result = dual_board::run(tx, rx, o.config, capture, [] { return stopped != 0; });
    } catch (const std::exception& error) { result.error = error.what(); }
    // Includes open failures and exceptions before core's close guard exists.
    tx.close(); rx.close();
  }
  const fs::path output(o.output);
  write_new(output / "plan.json", p + "\n");
  const auto& bytes = capture.bytes();
  write_new(output / "capture.bin", reinterpret_cast<const char*>(bytes.data()), bytes.size());
  const auto summary = dual_board::result_json(result, bytes.size());
  write_new(output / "summary.json", summary + "\n");
  std::cout << summary << std::endl;
  return result.complete() ? 0 : 1;
}
struct WorkerGuard {
  pid_t pid;
  bool reaped{false};
  ~WorkerGuard() {
    if (reaped) return;
    // Any parent exception must stop transmission before unwinding further.
    ::kill(pid, SIGKILL);
    const auto end = dual_board::Clock::now()+std::chrono::seconds(1);
    int status = 0;
    while (dual_board::Clock::now() < end) {
      if (::waitpid(pid, &status, WNOHANG) == pid) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};
int supervise(pid_t child, const fs::path& output, const std::string& p,
              std::chrono::milliseconds duration) {
  WorkerGuard guard{child};
  // This parent never opens a TTY. No alarm handler can terminate capture export.
  auto deadline = dual_board::Clock::now() + duration;
  bool forwarded = false, forced = false, reaped = false;
  int status = 0;
  while (dual_board::Clock::now() < deadline) {
    const auto state = ::waitpid(child, &status, WNOHANG);
    if (state == child) { reaped = true; guard.reaped = true; break; }
    require(state >= 0 || errno == EINTR, "diagnostic supervisor wait failed");
    if (stopped && !forwarded) {
      ::kill(child, SIGTERM); forwarded = true;
      deadline = std::min(deadline, dual_board::Clock::now()+std::chrono::seconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!reaped) {
    forced = true; ::kill(child, SIGKILL);
    const auto kill_deadline = dual_board::Clock::now()+std::chrono::seconds(1);
    while (dual_board::Clock::now() < kill_deadline) {
      if (::waitpid(child, &status, WNOHANG) == child) {
        reaped = true; guard.reaped = true; break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  const bool exported = reaped && !forced && WIFEXITED(status) && WEXITSTATUS(status) <= 1 &&
      fs::is_regular_file(output / "summary.json") &&
      fs::is_regular_file(output / "plan.json") && fs::is_regular_file(output / "capture.bin");
  const bool success = reaped && !forced && WIFEXITED(status) && WEXITSTATUS(status) == 0 && exported;
  const auto supervisory = std::string("{\"complete\":") + (success ? "true" : "false") +
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
  return success ? 0 : 1;
}
int live(const Options& o) {
  require(::geteuid() == 0, "live admission requires root for full /proc inspection");
  const auto a = identify(o.tx), b = identify(o.rx);
  require(a.usb != b.usb && a.device != b.device, "tx/rx must be different physical USB devices");
  inspect_owners(a,b);
  require(!stopped, "interrupted_before_output");
  const fs::path output(o.output);
  require(!fs::exists(output), "output directory already exists; evidence is never overwritten");
  const auto parent = output.has_parent_path() ? output.parent_path() : fs::path(".");
  const auto free_bytes = fs::space(parent).available;
  const std::uintmax_t required_bytes = static_cast<std::uintmax_t>(o.config.log_mib)*1024*1024 +
      1024U*1024U + 1024ULL*1024ULL*1024ULL;
  require(free_bytes >= required_bytes, "insufficient free space: capture plus metadata and 1 GiB reserve required");
  require(fs::create_directory(output), "cannot create unique output directory (parent must exist)");
  const auto p = plan(o, "[" + a.json() + "," + b.json() + "]");
  // Persist admission metadata before any TTY opens; raw capture is exported after close.
  write_new(output / "admission.json", p+"\n");
  const auto parent_pid = ::getpid();
  const auto child = ::fork();
  require(child >= 0, "cannot start supervised diagnostic process");
  if (child == 0) {
    // Prevent a parent crash or SIGKILL from leaving an unsupervised transmitter.
    if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || ::getppid() != parent_pid) ::_exit(3);
    int code = 3;
    try { code = worker(o,a,b,p); }
    catch (const std::exception& error) {
      std::cerr << "evidence export failed: " << error.what() << std::endl;
    }
    ::_exit(code);
  }
  return supervise(child, output, p, std::chrono::seconds(o.config.seconds+15) +
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
