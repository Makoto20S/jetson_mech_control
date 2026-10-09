#include "mech_bringup/servo_feedback_reader.hpp"
#include "mech_bringup/ak30_servo_system.hpp"
#include "mech_bringup/servo_resources.hpp"
#include <memory>
#include "hardware_interface/component_parser.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <iterator>
#include <thread>
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
int main(int argc, char** argv) {
  try {
    std::string path;
    double duration = 0;
    bool check = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (arg == "--urdf" && i + 1 < argc) path = argv[++i];
      else if (arg == "--duration" && i + 1 < argc) {
        std::size_t end = 0;
        const std::string value(argv[++i]);
        duration = std::stod(value, &end);
        if (end != value.size() || !std::isfinite(duration) || duration <= 0) throw std::runtime_error("duration must be positive finite seconds");
      } else if (arg == "--check") check = true;
      else throw std::runtime_error("usage: servo_feedback_reader --urdf FILE [--duration SECONDS] [--check]");
    }
    if (path.empty()) throw std::runtime_error("--urdf is required");
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot read URDF file");
    const std::string xml((std::istreambuf_iterator<char>(file)), {});
    const auto resources = hardware_interface::parse_control_resources_from_urdf(xml);
    const auto configs = mech::mech_bringup::parse_servo_resources(resources);
    // Check returns before constructing any plugin or serial port.
    if (check) { std::cout << "{\"schema_version\":1,\"configuration_valid\":true,\"motor_command_frames\":0}\n"; return 0; }
    std::vector<std::uint16_t> ids;
    std::vector<std::unique_ptr<mech::mech_bringup::Ak30ServoSystem>> plugins;
    const rclcpp_lifecycle::State state;
    using hardware_interface::CallbackReturn;
    for (std::size_t bus = 0; bus < configs.size(); ++bus) {
      for (const auto& joint : configs[bus].config.joints) ids.push_back(joint.drive_id);
      auto plugin = std::make_unique<mech::mech_bringup::Ak30ServoSystem>();
      if (plugin->on_init(resources[bus]) != CallbackReturn::SUCCESS ||
          plugin->on_configure(state) != CallbackReturn::SUCCESS ||
          plugin->on_activate(state) != CallbackReturn::SUCCESS)
        throw std::runtime_error("servo reader activation failed");
      plugins.push_back(std::move(plugin));
    }
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    const auto start = now_ns();
    auto next_output = start;
    bool acquired = false;
    int result = 0;
    while (!interrupted) {
      const auto now = now_ns();
      std::vector<std::optional<mech::mech_protocol_cubemars::ServoPositionSnapshot>> samples;
      bool fresh = true, bad = false;
      std::uint64_t command_frames = 0;
      for (std::size_t bus = 0; bus < plugins.size(); ++bus) {
        auto& plugin = *plugins[bus];
        const auto read_result = plugin.read(rclcpp::Time(now), rclcpp::Duration(std::chrono::milliseconds(2)));
        command_frames += plugin.motor_command_frames();
        bad = bad || read_result != hardware_interface::return_type::OK || plugin.motor_command_frames() != 0;
        for (std::size_t i = 0; i < configs[bus].config.joints.size(); ++i) {
          auto sample = plugin.diagnostic_snapshot(i);
          fresh = fresh && sample && sample->availability == mech::mech_protocol_cubemars::ServoPositionAvailability::Fresh;
          if (sample && sample->availability != mech::mech_protocol_cubemars::ServoPositionAvailability::Fresh && sample->availability != mech::mech_protocol_cubemars::ServoPositionAvailability::Unknown) bad = true;
          samples.push_back(sample);
        }
      }
      acquired = acquired || fresh;
      const bool failed = bad || (!fresh && (acquired || now - start >= 3000000000LL));
      if (now >= next_output || failed) {
        std::cout << mech::mech_bringup::format_servo_feedback(now_ns(), command_frames, ids, samples) << std::endl;
        next_output = now + 50000000;
      }
      if (failed) { result = 2; break; }
      if (duration > 0 && static_cast<double>(now - start) / 1e9 >= duration) { if (!acquired) result = 2; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // These lifecycle callbacks only stop reception and release ownership;
    // no command claims, write(), implicit disable or reset are performed.
    for (auto& plugin : plugins) {
      if (plugin->on_deactivate(state) != CallbackReturn::SUCCESS) result = 2;
      if (plugin->on_cleanup(state) != CallbackReturn::SUCCESS) result = 2;
    }
    return result;
  } catch (const std::exception& error) {
    std::cerr << "servo_feedback_reader: " << error.what() << '\n';
    return 2;
  }
}
