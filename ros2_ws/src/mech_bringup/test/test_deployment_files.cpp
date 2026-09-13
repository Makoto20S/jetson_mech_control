// Structure validation for the deployment example files. CI never launches
// the example (launching arms position commands on real hardware); these
// checks keep the URDF's ros2_control block, the controllers YAML, and the
// launch file consistent with what the code actually parses:
//
// - every URDF <param> name must be a key Ak30RuntimeParams accepts
//   (unknown keys reject configure in production, so the example must not
//   carry any);
// - the URDF's interface set must be exactly what CompositeSystem validates
//   (position command + position/velocity/effort states);
// - the URDF's hardware plugin must be the Ak30System composition point
//   (mech_bringup/Ak30System), not the bare CompositeSystem (which would
//   silently run the loopback runtime against real-hardware parameters);
// - the YAML controller type must be the registered plugin name;
// - the URDF's watchdog parameters must satisfy the ADR-012 budget.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "mech_bringup/ak30_runtime_params.hpp"

#include "mech_protocol_cubemars/ak30_mapping.hpp"

namespace mech::mech_bringup {
namespace {

// The URDF example lives in the source tree, not the install space: CI
// checks the source of truth that ships.
[[nodiscard]] std::string read_file(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    return {};
  }
  return std::string(std::istreambuf_iterator<char>(stream),
                     std::istreambuf_iterator<char>());
}

class DeploymentFilesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string root = MECH_BRINGUP_SOURCE_DIR;
    controllers_ = read_file(root + "/config/motor1_controllers.yaml");
    launch_ = read_file(root + "/launch/motor1_bringup.launch.py");
    ASSERT_FALSE(controllers_.empty());
    ASSERT_FALSE(launch_.empty());
    for (const auto& [name, sub_mode] : urdf_variants()) {
      const std::string urdf = read_file(root + "/config/" + name);
      ASSERT_FALSE(urdf.empty()) << name;
      urdfs_[name] = urdf;
      variants_.push_back({name, sub_mode});
    }
  }

  // Each deployment variant's URDF and the sub-mode its hardware block
  // declares. The command interface in the URDF must equal
  // expected_command_interface_name(sub_mode) - two spellings of the
  // same choice, checked below (ADR-014).
  struct Variant {
    std::string file;
    mech::mech_protocol_cubemars::ForceControlSubMode sub_mode;
  };

  std::map<std::string, std::string> urdfs_;
  std::vector<Variant> variants_;
  std::string controllers_;
  std::string launch_;

  [[nodiscard]] static const std::vector<
      std::pair<std::string,
                mech::mech_protocol_cubemars::ForceControlSubMode>>&
  urdf_variants() {
    using mech::mech_protocol_cubemars::ForceControlSubMode;
    static const std::vector<
        std::pair<std::string, ForceControlSubMode>>
        variants{
            {"motor1.urdf.xacro", ForceControlSubMode::Position},
            {"motor1_torque.urdf.xacro", ForceControlSubMode::Torque},
            {"motor1_velocity.urdf.xacro", ForceControlSubMode::Velocity},
        };
    return variants;
  }
};

TEST_F(DeploymentFilesTest, UrdfParamsAreAllKnownToTheParser) {
  for (const auto& [name, urdf] : urdfs_) {
    SCOPED_TRACE(name);
    // Extract every <param name="..."> from the ros2_control hardware block.
    std::set<std::string> names;
    std::size_t position = 0;
    while (true) {
      const auto hit = urdf.find("<param name=\"", position);
      if (hit == std::string::npos) {
        break;
      }
      const auto start = hit + 13;
      const auto end = urdf.find('"', start);
      ASSERT_NE(end, std::string::npos);
      names.insert(urdf.substr(start, end - start));
      position = end;
    }
    ASSERT_FALSE(names.empty());

    // Round-trip: the URDF's own parameter map must parse cleanly.
    std::map<std::string, std::string> params;
    for (const auto& param_name : names) {
      const auto tag = "<param name=\"" + param_name + "\">";
      const auto value_start = urdf.find(tag) + tag.size();
      const auto value_end = urdf.find("</param>", value_start);
      ASSERT_NE(value_end, std::string::npos);
      params[param_name] = urdf.substr(value_start, value_end - value_start);
    }
    const auto parsed = Ak30RuntimeParams::parse(params);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->device_path, "/dev/ttyACM0");
    EXPECT_EQ(parsed->config.drive_id, 104U);
    // ADR-012: hard TTL <= 6 ms (<=3 control cycles at 500 Hz) in every
    // shipped variant.
    EXPECT_LE(parsed->config.command_hard_ttl_nanoseconds, 6000000);
    EXPECT_GT(parsed->config.command_ttl_nanoseconds, 0);
    EXPECT_GT(parsed->config.command_hard_ttl_nanoseconds,
              parsed->config.command_ttl_nanoseconds);
  }
}

// The single command interface in each variant's URDF must be exactly the
// interface its sub_mode requires (ADR-014): Position -> position,
// Velocity -> velocity, Torque -> effort. States stay [position, velocity,
// effort] in every variant.
TEST_F(DeploymentFilesTest, UrdfCommandInterfaceMatchesSubMode) {
  for (const auto& variant : variants_) {
    SCOPED_TRACE(variant.file);
    const std::string& urdf = urdfs_[variant.file];
    const std::string expected_interface =
        expected_command_interface_name(variant.sub_mode);
    EXPECT_NE(urdf.find("<command_interface name=\"" + expected_interface +
                        "\"/>"),
              std::string::npos);
    // Exactly one command interface: the three canonical names occur only
    // once combined (the expected one), and the other two never appear as
    // command interfaces. <state_interface> lines reuse the same names, so
    // count command_interface tags specifically.
    std::size_t command_tags = 0;
    std::size_t position = 0;
    while (true) {
      const auto hit = urdf.find("<command_interface name=\"", position);
      if (hit == std::string::npos) {
        break;
      }
      ++command_tags;
      position = urdf.find('>', hit);
      ASSERT_NE(position, std::string::npos);
    }
    EXPECT_EQ(command_tags, 1U);
    for (const auto& states :
         {std::string("position"), std::string("velocity"),
          std::string("effort")}) {
      EXPECT_NE(urdf.find("<state_interface name=\"" + states + "\"/>"),
                std::string::npos);
    }
  }
}

// The hardware plugin in every variant must be the Ak30System composition
// point: pluginlib-constructing the bare CompositeSystem would run its
// built-in loopback runtime, which is not a real-device bring-up shape.
TEST_F(DeploymentFilesTest, UrdfUsesAk30SystemCompositionPlugin) {
  for (const auto& [name, urdf] : urdfs_) {
    SCOPED_TRACE(name);
    EXPECT_NE(urdf.find("<plugin>mech_bringup/Ak30System</plugin>"),
              std::string::npos);
    EXPECT_EQ(urdf.find("<plugin>mech_hardware_ros2_control/CompositeSystem"
                        "</plugin>"),
              std::string::npos);
  }
}

TEST_F(DeploymentFilesTest, ControllersYamlUsesTheRegisteredPluginName) {
  EXPECT_NE(controllers_.find("mech_controllers/DemoController"),
            std::string::npos);
  EXPECT_NE(controllers_.find("joint_state_broadcaster/JointStateBroadcaster"),
            std::string::npos);
}

TEST_F(DeploymentFilesTest, LaunchFileReferencesExistingFilesAndStaysSafe) {
  EXPECT_NE(launch_.find("motor1.urdf.xacro"), std::string::npos);
  EXPECT_NE(launch_.find("motor1_controllers.yaml"), std::string::npos);
  // The position-controller spawner must stay commented out: uncommenting it
  // arms position commands, which the file's warning and ADR-006 both gate.
  // The state broadcaster alone is safe to spawn.
  //
  // Asserted over EVERY mention of the controller rather than one exact
  // spawner literal. The literal form broke the moment the spawner line grew
  // a '--inactive' argument, and a guard that a harmless edit can silently
  // stop matching is not a guard.
  const std::string controller = "motor1_position_controller";
  auto position = launch_.find(controller);
  ASSERT_NE(position, std::string::npos);
  std::size_t mentions = 0U;
  while (position != std::string::npos) {
    ++mentions;
    const auto newline = launch_.rfind('\n', position);
    const std::size_t line_start =
        newline == std::string::npos ? 0U : newline + 1U;
    const auto line = launch_.substr(line_start, position - line_start);
    EXPECT_NE(line.find('#'), std::string::npos)
        << "uncommented mention of " << controller << " at offset " << position;
    position = launch_.find(controller, position + 1U);
  }
  EXPECT_GT(mentions, 0U);
  EXPECT_NE(launch_.find("arguments=['joint_state_broadcaster']"),
            std::string::npos);
}

// Deliberately a separate test from the one above, which is due to fail at T7:
// that guard says the position-controller spawner must stay commented out, so
// arming it legitimately turns it red and whoever is at the bench will edit it
// away. This guard has to outlive that edit, because arming is exactly when it
// starts to matter.
//
// ADR-016 refuses the command claim until valid feedback has flowed, and the
// spawner asks switch_controller exactly once with STRICT strictness
// (controller_manager/spawner.py): a refused claim is a valid service response,
// so its max_attempts retry does not apply and it logs "Failed to activate
// controller" and exits 1. A plainly spawned position controller therefore
// races the first feedback frame - up to 20 ms at motor1's configured 50 Hz -
// and losing leaves the controller off with a failed launch process.
//
// Scanned per arguments= list rather than per line so reformatting the call
// across several lines cannot quietly drop the guard.
TEST_F(DeploymentFilesTest, PositionControllerSpawnerIsNeverArmedActive) {
  const std::string controller = "motor1_position_controller";
  const std::string marker = "arguments=[";
  std::size_t spawners = 0U;
  auto open = launch_.find(marker);
  while (open != std::string::npos) {
    const auto close = launch_.find(']', open);
    ASSERT_NE(close, std::string::npos)
        << "unterminated " << marker << " at offset " << open;
    const auto arguments = launch_.substr(open, close - open);
    // A nested list would end the scan at the inner ']' and could hide the
    // controller name, turning this guard into a vacuous pass. Fail loudly
    // and rewrite the scan instead.
    ASSERT_EQ(arguments.find('[', marker.size()), std::string::npos)
        << "nested list inside " << marker << " at offset " << open
        << "; this guard's bracket scan stops early - update it";
    if (arguments.find(controller) != std::string::npos) {
      ++spawners;
      EXPECT_NE(arguments.find("--inactive"), std::string::npos)
          << "the " << controller << " spawner must load it inactive and be "
          << "activated only after feedback is confirmed; found: " << arguments;
    }
    open = launch_.find(marker, close);
  }
  // Without this the loop passes vacuously on a file that stopped spawning the
  // controller in any form.
  EXPECT_GT(spawners, 0U)
      << "no spawner arguments mention " << controller
      << "; this guard is no longer guarding anything";
}

}  // namespace
}  // namespace mech::mech_bringup
