// Offline replay of the production controller; no manager or hardware plugin.
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mech_controllers/resistance_training_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  // Prepared input: elapsed_s,position_rad,velocity_rad_s, at 2 ms intervals.
  if (argc != 3) {
    std::cerr << "usage: resistance_controller_replay INPUT.csv OUTPUT.csv\n";
    return 2;
  }
  rclcpp::init(0, nullptr);
  int result = 0;
  try {
    mech::mech_controllers::ResistanceTrainingController controller;
    if (controller.init("offline_resistance_replay") != controller_interface::return_type::OK)
      throw std::runtime_error("controller init");
    const auto parameters = controller.get_node()->set_parameters({
      {"minimum", 0.0}, {"maximum", 1.0}, {"max_slew_per_second", 1000000.0},
      {"damping", 24.0}, {"torque_limit_nm", 8.0}, {"buildup_nm_per_second", 40.0},
      {"velocity_filter_tau_s", .025}, {"max_velocity_rad_s", 50.0},
      {"position_min_rad", -3.0}, {"position_max_rad", 3.0}, {"run_limit_s", 30.0}});
    for (const auto& parameter : parameters)
      if (!parameter.successful) throw std::runtime_error(parameter.reason);
    std::int64_t now = 1000000000;
    controller.set_clock_for_testing([&now]() { return now; });
    rclcpp_lifecycle::State state;
    if (controller.on_configure(state) != controller_interface::CallbackReturn::SUCCESS)
      throw std::runtime_error("configure");
    double q = 0.0, v = 0.0, effort = 0.0, generation = 0.0;
    hardware_interface::CommandInterface effort_interface("joint_1", "effort", &effort);
    hardware_interface::CommandInterface generation_interface("joint_1", "command_generation", &generation);
    hardware_interface::StateInterface position_interface("joint_1", "position", &q);
    hardware_interface::StateInterface velocity_interface("joint_1", "velocity", &v);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(effort_interface); commands.emplace_back(generation_interface);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(position_interface); states.emplace_back(velocity_interface);
    controller.assign_interfaces(std::move(commands), std::move(states));
    if (controller.on_activate(state) != controller_interface::CallbackReturn::SUCCESS)
      throw std::runtime_error("activate");
    std::ifstream input(argv[1]);
    std::ofstream output(argv[2]);
    if (!input || !output) throw std::runtime_error("input/output file");
    output << "elapsed_s,position_rad,velocity_rad_s,effort_nm\n" << std::setprecision(17);
    std::string line;
    std::getline(input, line);
    std::size_t count = 0U;
    while (std::getline(input, line)) {
      std::istringstream fields(line);
      double elapsed = 0.0;
      char comma1 = 0, comma2 = 0;
      if (!(fields >> elapsed >> comma1 >> q >> comma2 >> v) || comma1 != ',' || comma2 != ',' ||
          !std::isfinite(elapsed) || !std::isfinite(q) || !std::isfinite(v) ||
          std::abs(elapsed - .002 * static_cast<double>(count)) > 1e-8)
        throw std::runtime_error("invalid 2 ms replay row");
      if (count % 10U == 0U && !controller.set_target(1.0))
        throw std::runtime_error("permission");
      now += 2000000;
      if (controller.update(rclcpp::Time(0), rclcpp::Duration(0, 2000000)) !=
          controller_interface::return_type::OK)
        throw std::runtime_error("controller rejected replay");
      output << elapsed << ',' << q << ',' << v << ',' << effort << '\n';
      ++count;
    }
    if (count == 0U) throw std::runtime_error("empty replay");
    std::cout << "replayed " << count << " rows (fixed input; not a plant simulation)\n";
    (void)controller.on_deactivate(state);
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n'; result = 1;
  }
  rclcpp::shutdown();
  return result;
}
