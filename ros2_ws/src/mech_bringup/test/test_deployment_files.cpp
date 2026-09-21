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

#include "mech_hardware_ros2_control/composite_system.hpp"

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
    trajectory_controllers_ =
        read_file(root + "/config/motor1_trajectory_controllers.yaml");
    position_velocity_controllers_ = read_file(
        root + "/config/motor1_position_velocity_trajectory_controllers.yaml");
    trajectory_launch_ =
        read_file(root + "/launch/motor1_trajectory_bringup.launch.py");
    ASSERT_FALSE(controllers_.empty());
    ASSERT_FALSE(launch_.empty());
    ASSERT_FALSE(trajectory_controllers_.empty());
    ASSERT_FALSE(position_velocity_controllers_.empty());
    ASSERT_FALSE(trajectory_launch_.empty());
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
  std::string trajectory_controllers_;
  std::string position_velocity_controllers_;
  std::string trajectory_launch_;

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
            {"motor1_position_velocity.urdf.xacro",
             ForceControlSubMode::Position},
            {"motor1_position_velocity_effort.urdf.xacro",
             ForceControlSubMode::Position},
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

TEST_F(DeploymentFilesTest, TargetAndHardwareLifetimesRemainDistinct) {
  EXPECT_NE(controllers_.find("target_ttl_nanoseconds: 100000000"),
            std::string::npos);
  EXPECT_NE(controllers_.find("target_hard_ttl_nanoseconds: 106000000"),
            std::string::npos);
  EXPECT_EQ(controllers_.find("\n    ttl_nanoseconds:"), std::string::npos);
  EXPECT_EQ(controllers_.find("\n    hard_ttl_nanoseconds:"),
            std::string::npos);

  for (const auto& [name, urdf] : urdfs_) {
    SCOPED_TRACE(name);
    EXPECT_NE(urdf.find("<param name=\"command_ttl_ns\">4000000</param>"),
              std::string::npos);
    EXPECT_NE(
        urdf.find("<param name=\"command_hard_ttl_ns\">6000000</param>"),
        std::string::npos);
    EXPECT_EQ(urdf.find("100000000"), std::string::npos);
    EXPECT_EQ(urdf.find("106000000"), std::string::npos);
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
    // ADR-017: exactly one MOTION command interface (the expected one; the
    // other two canonical names never appear as command interfaces) plus
    // exactly one command_generation interface. <state_interface> lines reuse
    // the same names, so count command_interface tags specifically.
    //
    // The generation interface is required in the URDF because the hardware
    // exports it unconditionally - a deployment that omitted it would disagree
    // with what CompositeSystem exports. That is separate from whether a
    // controller claims it, which stays the controller's choice.
    std::size_t command_tags = 0;
    std::size_t generation_tags = 0;
    std::size_t position = 0;
    while (true) {
      const auto hit = urdf.find("<command_interface name=\"", position);
      if (hit == std::string::npos) {
        break;
      }
      ++command_tags;
      const auto close = urdf.find('>', hit);
      ASSERT_NE(close, std::string::npos);
      if (urdf.compare(hit, close - hit,
                       std::string("<command_interface name=\"") +
                           mech::mech_hardware_ros2_control::
                               kCommandGenerationInterface +
                           "\"/") == 0) {
        ++generation_tags;
      }
      position = close;
    }
    const bool position_velocity =
        variant.file == "motor1_position_velocity.urdf.xacro";
    const bool position_velocity_effort =
        variant.file == "motor1_position_velocity_effort.urdf.xacro";
    EXPECT_EQ(command_tags,
              position_velocity_effort ? 4U : (position_velocity ? 3U : 2U));
    EXPECT_EQ(generation_tags, 1U);
    for (const auto& states :
         {std::string("position"), std::string("velocity"),
          std::string("effort")}) {
      EXPECT_NE(urdf.find("<state_interface name=\"" + states + "\"/>"),
                std::string::npos);
    }
  }
}

TEST_F(DeploymentFilesTest, PositionVelocityJtcClaimsTheCompleteDeclaredBundle) {
  const auto& urdf = urdfs_["motor1_position_velocity.urdf.xacro"];
  EXPECT_NE(urdf.find("<command_interface name=\"position\"/>"),
            std::string::npos);
  EXPECT_NE(urdf.find("<command_interface name=\"velocity\"/>"),
            std::string::npos);
  EXPECT_EQ(urdf.find("<command_interface name=\"effort\"/>"),
            std::string::npos);
  EXPECT_NE(position_velocity_controllers_.find(
                "type: joint_trajectory_controller/JointTrajectoryController"),
            std::string::npos);
  const auto position = position_velocity_controllers_.find("- position");
  const auto velocity = position_velocity_controllers_.find("- velocity");
  ASSERT_NE(position, std::string::npos);
  ASSERT_NE(velocity, std::string::npos);
  EXPECT_LT(position, velocity);
  EXPECT_EQ(position_velocity_controllers_.find("- effort"), std::string::npos);
}

TEST_F(DeploymentFilesTest, FullTupleTemplateDeclaresOneCompleteBundle) {
  const auto& urdf = urdfs_["motor1_position_velocity_effort.urdf.xacro"];
  for (const auto* interface : {"position", "velocity", "effort"}) {
    EXPECT_NE(urdf.find(std::string("<command_interface name=\"") + interface +
                        "\"/>"),
              std::string::npos);
  }
  EXPECT_NE(urdf.find("position_max_abs_velocity_rad_s\">1.0"),
            std::string::npos);
  EXPECT_NE(urdf.find("position_max_abs_feedforward_nm\">0.1"),
            std::string::npos);
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
  EXPECT_NE(controllers_.find("mech_controllers/PositionCommandController"),
            std::string::npos);
  EXPECT_NE(controllers_.find("joint_state_broadcaster/JointStateBroadcaster"),
            std::string::npos);
}

TEST_F(DeploymentFilesTest, TrajectoryVariantLoadsUpstreamJtcInactive) {
  EXPECT_NE(trajectory_controllers_.find(
                "type: joint_trajectory_controller/JointTrajectoryController"),
            std::string::npos);
  EXPECT_NE(trajectory_controllers_.find("command_interfaces:"),
            std::string::npos);
  EXPECT_NE(trajectory_controllers_.find("- position"), std::string::npos);
  EXPECT_EQ(trajectory_controllers_.find("- velocity"), std::string::npos);
  EXPECT_NE(trajectory_controllers_.find("allow_partial_joints_goal: false"),
            std::string::npos);
  EXPECT_NE(trajectory_controllers_.find(
                "allow_nonzero_velocity_at_trajectory_end: false"),
            std::string::npos);
  EXPECT_NE(trajectory_launch_.find("motor1.urdf.xacro"), std::string::npos);
  EXPECT_NE(trajectory_launch_.find("motor1_trajectory_controllers.yaml"),
            std::string::npos);
  EXPECT_NE(trajectory_launch_.find("'--inactive'"), std::string::npos);
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

// ADR-019: the Position variant ships an explicit hardware envelope; the
// Velocity and Torque variants do not carry position bounds they never use.
TEST_F(DeploymentFilesTest, PositionVariantShipsTheHardwareEnvelope) {
  for (const auto& variant : variants_) {
    SCOPED_TRACE(variant.file);
    const std::string& urdf = urdfs_[variant.file];
    const bool position_mode = variant.sub_mode ==
        mech::mech_protocol_cubemars::ForceControlSubMode::Position;
    for (const auto& [key, value] :
         std::vector<std::pair<std::string, std::string>>{
             {"position_min_rad", "-12.0"},
             {"position_max_rad", "6.0"},
             {"position_max_error_rad", "0.5"}}) {
      const auto tag = "<param name=\"" + key + "\">" + value + "</param>";
      EXPECT_EQ(urdf.find(tag) != std::string::npos, position_mode) << key;
    }
  }
}

// The hardware envelope and the controller's own clamp are two spellings of
// one mechanical decision, written in two files that nothing else ties
// together. They are allowed to differ in kind - the controller clamps, the
// hardware refuses and latches - but not in value: a controller whose clamp
// let a target past the hardware bound would turn an ordinary command into a
// latched component, and a controller clamped tighter than the hardware would
// leave the envelope untestable from the deployment.
TEST_F(DeploymentFilesTest, PositionEnvelopeBoundsMatchControllerTargetBounds) {
  const std::string& urdf = urdfs_["motor1.urdf.xacro"];
  std::map<std::string, std::string> params;
  std::size_t position = 0;
  while (true) {
    const auto hit = urdf.find("<param name=\"", position);
    if (hit == std::string::npos) {
      break;
    }
    const auto start = hit + 13;
    const auto name_end = urdf.find('"', start);
    ASSERT_NE(name_end, std::string::npos);
    const auto value_start = urdf.find('>', name_end) + 1;
    const auto value_end = urdf.find("</param>", value_start);
    ASSERT_NE(value_end, std::string::npos);
    params[urdf.substr(start, name_end - start)] =
        urdf.substr(value_start, value_end - value_start);
    position = value_end;
  }
  const auto parsed = Ak30RuntimeParams::parse(params);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_DOUBLE_EQ(parsed->config.position_min_rad, -12.0);
  EXPECT_DOUBLE_EQ(parsed->config.position_max_rad, 6.0);

  // Matched against the shipped YAML text rather than a parsed controller
  // config: this file is read by the controller's own parameter loading at
  // runtime, and the point here is that the two files cannot drift.
  EXPECT_NE(controllers_.find("minimum: -12.0"), std::string::npos);
  EXPECT_NE(controllers_.find("maximum: 6.0"), std::string::npos);
}

}  // namespace
}  // namespace mech::mech_bringup
