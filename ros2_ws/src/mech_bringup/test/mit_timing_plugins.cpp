#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <vector>
#include "mech_bringup/ak30_system.hpp"
#include "mech_controllers/single_joint_command_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

namespace mech::mech_bringup::test {
static std::int64_t timing_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
class MitTimingSystem final : public Ak30System {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
    if (info.joints.size() != 1U) return hardware_interface::CallbackReturn::ERROR;
    auto copy = info;
    joint_ = info.joints[0].name;
    std::array<unsigned, 3> found{};
    auto& states = copy.joints[0].state_interfaces;
    for (auto it = states.begin(); it != states.end();) {
      bool removed = false;
      for (std::size_t i = 0; i < names_.size(); ++i) {
        if (it->name == names_[i]) { ++found[i]; it = states.erase(it); removed = true; break; }
      }
      if (!removed) ++it;
    }
    for (auto count : found) if (count != 1U) return hardware_interface::CallbackReturn::ERROR;
    const auto result = Ak30System::on_init(copy);
    if (result != hardware_interface::CallbackReturn::SUCCESS || !timing_trace())
      return hardware_interface::CallbackReturn::ERROR;
    commands_ = Ak30System::export_command_interfaces();
    return result;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    auto states = Ak30System::export_state_interfaces();
    for (std::size_t i = 0; i < names_.size(); ++i) states.emplace_back(joint_, names_[i], &metadata_[i]);
    return states;
  }
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override {
    ++timing_trace()->cycle;
    const auto result = Ak30System::read(time, period);
    metadata_.fill(std::numeric_limits<double>::quiet_NaN());
    const auto* status = timing_status();
    if (result == hardware_interface::return_type::OK && status && status->host_rx_time) {
      metadata_[0] = static_cast<double>(status->sequence);
      metadata_[1] = static_cast<double>(status->host_rx_time->nanoseconds());
      metadata_[2] = static_cast<double>(timing_now());
    }
    timing_trace()->record("hw_read", 0, 0, metadata_[0], metadata_[1], 0, 0, static_cast<int>(result));
    return result;
  }
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override {
    std::uint64_t generation = 0;
    for (const auto& command : commands_) {
      if (command.get_interface_name() == "command_generation") {
        const auto value = command.get_value();
        if (std::isfinite(value) && value >= 0 && value <= 9007199254740991.0)
          generation = static_cast<std::uint64_t>(value);
      }
    }
    timing_trace()->generation = generation;
    timing_trace()->record("hw_write_begin", 0, generation, 0, 0, authorized(0) ? 1 : 0);
    const auto result = Ak30System::write(time, period);
    timing_trace()->record("hw_write_end", 0, generation, 0, 0, authorized(0) ? 1 : 0, 0, static_cast<int>(result));
    return result;
  }
 private:
  std::string joint_;
  const std::array<std::string, 3> names_{{"timing_rx_sequence", "timing_rx_ns", "timing_read_ns"}};
  std::array<double, 3> metadata_{{0, 0, 0}};
  std::vector<hardware_interface::CommandInterface> commands_;
};

class MitZeroTimingController final : public mech_controllers::SingleJointCommandController {
 public:
  MitZeroTimingController() : SingleJointCommandController("effort", "~/unused_scalar_target") {}
  ~MitZeroTimingController() override {
    if (path_.empty()) return;
    std::ofstream out(path_);
    out << "# total=" << total_ << " capacity=" << records_.size() << "\n";
    out << "seq,publish_ns,receive_ns,update_ns,command_generation\n";
    for (std::size_t i = 0; i < total_ && i < records_.size(); ++i) {
      const auto& r = records_[i];
      out << r.sequence << ',' << r.publish << ',' << r.receive << ',' << r.update << ',' << r.generation << '\n';
    }
  }
  controller_interface::CallbackReturn on_init() override {
    const auto result = SingleJointCommandController::on_init();
    if (result != controller_interface::CallbackReturn::SUCCESS) return result;
    auto_declare<std::string>("timing_path", "");
    auto_declare<int64_t>("timing_nonce", 0);
    return result;
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override {
    const auto result = SingleJointCommandController::on_configure(state);
    if (result != controller_interface::CallbackReturn::SUCCESS) return result;
    path_ = get_node()->get_parameter("timing_path").as_string();
    nonce_ = get_node()->get_parameter("timing_nonce").as_int();
    if (path_.empty() || path_[0] != '/' || nonce_ <= 0)
      return controller_interface::CallbackReturn::ERROR;
    records_.resize(4096);
    subscription_ = get_node()->create_subscription<std_msgs::msg::UInt64MultiArray>(
        "~/timed_zero", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::UInt64MultiArray::SharedPtr msg) {
          const auto now = timing_now();
          if (!msg || msg->data.size() != 3U || msg->data[0] != static_cast<std::uint64_t>(nonce_) ||
              msg->data[1] == 0 || msg->data[1] <= last_received_ ||
              msg->data[1] > 9007199254740991ULL || msg->data[2] == 0 ||
              msg->data[2] > static_cast<std::uint64_t>(now)) return;
          TargetCommand command;
          command.value = 0.0;
          command.parameters[0] = static_cast<double>(msg->data[1]);
          command.parameters[1] = static_cast<double>(msg->data[2]);
          command.parameters[2] = static_cast<double>(now);
          if (set_command(command)) last_received_ = msg->data[1];
        });
    return result;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override {
    const auto result = SingleJointCommandController::update(time, period);
    if (result != controller_interface::return_type::OK) return result;
    const auto& p = command_parameters();
    const auto seq = static_cast<std::uint64_t>(p[0]);
    if (seq && seq != last_recorded_) {
      last_recorded_ = seq;
      if (total_ < records_.size()) records_[total_] = {
        seq, static_cast<std::int64_t>(p[1]), static_cast<std::int64_t>(p[2]), timing_now(),
        static_cast<std::uint64_t>(command_interfaces_[1].get_value())};
      ++total_;
    }
    return result;
  }
 private:
  bool initial_command(double& value) const noexcept override { value = 0.0; return state_interfaces_.empty(); }
  bool valid_limits(const mech_controllers::BoundedTarget& l) const noexcept override {
    return l.minimum == 0.0 && l.maximum == 0.0;
  }
  bool scalar_subscription() const noexcept override { return false; }
  struct Record { std::uint64_t sequence; std::int64_t publish, receive, update; std::uint64_t generation; };
  std::vector<Record> records_;
  std::size_t total_{0};
  std::uint64_t last_received_{0}, last_recorded_{0};
  std::int64_t nonce_{0};
  std::string path_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr subscription_;
};
}
PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::test::MitTimingSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::test::MitZeroTimingController, controller_interface::ControllerInterface)
