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
    urdf_ = read_file(root + "/config/motor1.urdf.xacro");
    controllers_ = read_file(root + "/config/motor1_controllers.yaml");
    launch_ = read_file(root + "/launch/motor1_bringup.launch.py");
    ASSERT_FALSE(urdf_.empty());
    ASSERT_FALSE(controllers_.empty());
    ASSERT_FALSE(launch_.empty());
  }

  std::string urdf_;
  std::string controllers_;
  std::string launch_;
};

TEST_F(DeploymentFilesTest, UrdfParamsAreAllKnownToTheParser) {
  // Extract every <param name="..."> from the ros2_control hardware block.
  std::set<std::string> names;
  std::size_t position = 0;
  while (true) {
    const auto hit = urdf_.find("<param name=\"", position);
    if (hit == std::string::npos) {
      break;
    }
    const auto start = hit + 13;
    const auto end = urdf_.find('"', start);
    ASSERT_NE(end, std::string::npos);
    names.insert(urdf_.substr(start, end - start));
    position = end;
  }
  ASSERT_FALSE(names.empty());

  // Round-trip: the URDF's own parameter map must parse cleanly.
  std::map<std::string, std::string> params;
  for (const auto& name : names) {
    const auto tag = "<param name=\"" + name + "\">";
    const auto value_start = urdf_.find(tag) + tag.size();
    const auto value_end = urdf_.find("</param>", value_start);
    ASSERT_NE(value_end, std::string::npos);
    params[name] = urdf_.substr(value_start, value_end - value_start);
  }
  const auto parsed = Ak30RuntimeParams::parse(params);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->device_path, "/dev/ttyACM0");
  EXPECT_EQ(parsed->config.drive_id, 104U);
}

TEST_F(DeploymentFilesTest, UrdfInterfacesMatchCompositeSystemShape) {
  EXPECT_NE(urdf_.find("<command_interface name=\"position\"/>"),
            std::string::npos);
  EXPECT_EQ(urdf_.find("<command_interface name=\"velocity\""),
            std::string::npos);
  EXPECT_EQ(urdf_.find("<command_interface name=\"effort\""),
            std::string::npos);
  EXPECT_NE(urdf_.find("<state_interface name=\"position\"/>"),
            std::string::npos);
  EXPECT_NE(urdf_.find("<state_interface name=\"velocity\"/>"),
            std::string::npos);
  EXPECT_NE(urdf_.find("<state_interface name=\"effort\"/>"),
            std::string::npos);
}

TEST_F(DeploymentFilesTest, UrdfWatchdogStaysInsideTheAdr012Budget) {
  const auto ttl = urdf_.find("<param name=\"command_ttl_ns\">4000000");
  const auto hard = urdf_.find("<param name=\"command_hard_ttl_ns\">6000000");
  ASSERT_NE(ttl, std::string::npos);
  ASSERT_NE(hard, std::string::npos);
  // ADR-012: hard TTL <= 6 ms (<=3 control cycles at 500 Hz).
  EXPECT_TRUE(urdf_.find("<param name=\"command_hard_ttl_ns\">6000000") <
              urdf_.find("</hardware>"));
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
