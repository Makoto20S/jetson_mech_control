// Test-only composition: the production final plugin still owns serial I/O,
// lifecycle, command tracing and watchdogs. Only its injected clock is synthetic.
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "mech_bringup/ak30_servo_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mech::mech_bringup::test {
class ServoPtyTestSystem final : public hardware_interface::SystemInterface {
 public:
  ~ServoPtyTestSystem() override {
    save_clock();
    if (peer_ >= 0) ::close(peer_);
  }

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override {
    const auto path = info.hardware_parameters.find("device_path");
    const auto log = std::getenv("MECH_TEST_CLOCK_DIR");
    // This plugin must never become an alternative physical-device entrypoint.
    if (path == info.hardware_parameters.end() ||
        path->second.rfind("/dev/pts/", 0) != 0 ||
        path->second.find_first_not_of("0123456789", 9) != std::string::npos ||
        path->second.size() <= 9 || log == nullptr || !*log) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto trace = info.hardware_parameters.find("trace_name");
    const auto name = trace == info.hardware_parameters.end() ? "single" : trace->second;
    if (name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-") !=
                            std::string::npos)
      return hardware_interface::CallbackReturn::ERROR;
    clock_path_ = std::string(log) + "/" + name + "-" + std::to_string(getpid()) + ".csv";
    const auto peer_path = std::string(log) + "/" + name + ".sock";
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (peer_path.size() >= sizeof(address.sun_path))
      return hardware_interface::CallbackReturn::ERROR;
    std::memcpy(address.sun_path, peer_path.c_str(), peer_path.size() + 1);
    peer_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    const timeval timeout{1, 0};
    if (peer_ < 0 || ::setsockopt(peer_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) ||
        ::setsockopt(peer_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) ||
        ::connect(peer_, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
      return hardware_interface::CallbackReturn::ERROR;
    samples_.reserve(1000000);
    sample();
    inner_.set_clock_for_testing([this] {
      // A read can drain several queued PTY feedback frames. Their arrival
      // stamps must remain strictly increasing, just as on the host clock.
      if (++ticks_ >= cycle_ticks_ + 2000000)
        throw std::runtime_error("test clock exhausted a hardware cycle");
      sample();
      return *mech_control_core::MonotonicTime::from_nanoseconds(ticks_);
    });
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    return inner_.on_init(info);
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    return inner_.export_state_interfaces();
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    return inner_.export_command_interfaces();
  }

#define FORWARD_LIFECYCLE(method) \
  hardware_interface::CallbackReturn method(const rclcpp_lifecycle::State& state) override { \
    return inner_.method(state); \
  }
  FORWARD_LIFECYCLE(on_configure)
  FORWARD_LIFECYCLE(on_cleanup)
  FORWARD_LIFECYCLE(on_activate)
  FORWARD_LIFECYCLE(on_deactivate)
  FORWARD_LIFECYCLE(on_error)
#undef FORWARD_LIFECYCLE
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override {
    const auto result = inner_.on_shutdown(state);
    save_clock();
    return result;
  }
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start, const std::vector<std::string>& stop) override {
    return inner_.prepare_command_mode_switch(start, stop);
  }
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start, const std::vector<std::string>& stop) override {
    return inner_.perform_command_mode_switch(start, stop);
  }
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override {
    char reply{};
    if (::send(peer_, "R", 1, MSG_NOSIGNAL) != 1 || ::recv(peer_, &reply, 1, 0) != 1 || reply != 'A') {
      RCLCPP_ERROR(rclcpp::get_logger("servo_pty_fixture"), "Synthetic gateway handshake failed");
      return hardware_interface::return_type::ERROR;
    }
    cycle_ticks_ += 2000000;
    ticks_ = cycle_ticks_;
    sample();
    return inner_.read(time, period);
  }
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override {
    const auto result = inner_.write(time, period);
    // Pause with a pending command, before the next read sends it. All three
    // durations exceed the unchanged production 3/6 ms command deadlines.
    if (inner_.motor_command_frames() > 0 && pause_index_ < pauses_.size() &&
        ++writes_after_send_ % 100 == 0) {
      const auto pause = pauses_[pause_index_++];
      std::this_thread::sleep_for(std::chrono::milliseconds(pause));
      sample(pause);
    }
    return result;
  }

 private:
  struct Sample { std::int64_t host, hardware; int pause; };
  void sample(int pause = 0) {
    if (samples_.size() == 1000000) throw std::runtime_error("test clock evidence exhausted");
    const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    samples_.push_back({host, ticks_, pause});
  }
  void save_clock() noexcept {
    if (clock_path_.empty()) return;
    try {
      std::ofstream out(clock_path_);
      out << "host_ns,hardware_ns,pause_ms\n";
      for (const auto& s : samples_) out << s.host << ',' << s.hardware << ',' << s.pause << '\n';
    } catch (...) { /* missing/invalid clock evidence fails the Python audit */ }
  }
  Ak30ServoSystem inner_;
  std::int64_t ticks_{1000000000};
  std::int64_t cycle_ticks_{1000000000};
  std::string clock_path_;
  int peer_{-1};
  std::vector<Sample> samples_;
  const std::array<int, 3> pauses_{{20, 50, 100}};
  std::size_t pause_index_{0}, writes_after_send_{0};
};
}  // namespace mech::mech_bringup::test

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::test::ServoPtyTestSystem,
                      hardware_interface::SystemInterface)
