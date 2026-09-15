#include <gtest/gtest.h>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(ProductionPluginLoading, HardwareThenControllerHaveIndependentFactories) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware_loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto hardware = hardware_loader.createSharedInstance("mech_bringup/Ak30System");
  ASSERT_NE(hardware, nullptr);
  // Construction only: no on_init/configure/activate and no device access.
  pluginlib::ClassLoader<controller_interface::ControllerInterface> controller_loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_NE(controller_loader.createSharedInstance("mech_controllers/PositionCommandController"),
            nullptr);
  EXPECT_NE(controller_loader.createSharedInstance("mech_controllers/VelocityCommandController"),
            nullptr);
}
