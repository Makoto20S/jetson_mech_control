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

TEST_F(DeploymentFilesTest, ControllersYamlUsesTheRegisteredPluginName) {
  EXPECT_NE(controllers_.find("mech_controllers/DemoController"),
            std::string::npos);
  EXPECT_NE(controllers_.find("joint_state_broadcaster/JointStateBroadcaster"),
            std::string::npos);
}

TEST_F(DeploymentFilesTest, LaunchFileReferencesExistingFilesAndStaysSafe) {
  EXPECT_NE(launch_.find("motor1.urdf.xacro"), std::string::npos);
  EXPECT_NE(launch_.find("motor1_controllers.yaml"), std::string::npos);
  // The position-controller spawner must stay commented out: uncommenting
  // it arms position commands, which the file's warning and ADR-006 gate
  // both call out. The state broadcaster alone is safe to spawn.
  // The only occurrence of the spawner argument must be inside a comment.
  const std::string spawner_arg = "arguments=['motor1_position_controller']";
  auto position = launch_.find(spawner_arg);
  ASSERT_NE(position, std::string::npos);
  EXPECT_EQ(launch_.find(spawner_arg, position + 1), std::string::npos);
  const auto line_start = launch_.rfind('\n', position);
  const auto line = launch_.substr(line_start + 1, position - line_start - 1);
  EXPECT_NE(line.find('#'), std::string::npos);
  EXPECT_NE(launch_.find("arguments=['joint_state_broadcaster']"),
            std::string::npos);
}

}  // namespace
}  // namespace mech::mech_bringup
