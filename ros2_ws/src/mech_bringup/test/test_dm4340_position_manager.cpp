// Production Dm4340System + USB-CDC transport + two installed generic position
// controller plugins under a real ControllerManager. No physical device I/O.
#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_bringup/dm4340_system.hpp"
#include "mech_controllers/position_command_controller.hpp"
#include "mech_simulation/fake_serial.hpp"

namespace {
using mech::mech_bringup::Dm4340System;
using namespace mech::mech_control_core;

class PositionReplySerial final : public CdcSerialPort {
 public:
  bool is_open() const noexcept override {return port.is_open();}
  bool open() noexcept override {++opens;return port.open();}
  void close() noexcept override {port.close();}
  TransportResult read_some(std::uint8_t* d,std::size_t c,std::size_t& n) noexcept override {
    return port.read_some(d,c,n);
  }
  TransportResult write_all(const std::uint8_t* d,std::size_t n) noexcept override {
    if(n!=21) return port.write_all(d,n);
    const bool query=d[7]==0xff&&d[8]==7;
    const auto id=query?d[13]:d[7];
    if(id<1||id>2) return TransportResult::Ok;
    if(!query) {
      bool special=true;for(int i=13;i<20;++i) special&=d[i]==0xff;
      if(special&&d[20]==0xfc) state[id-1]=1;
      else if(special&&d[20]==0xfd) {state[id-1]=0;++disables[id-1];}
      else if(!special) {
        std::array<std::uint8_t,8> p{};
        for(int i=0;i<8;++i)p[i]=d[13+i];
        motion[id-1].push_back(p);
      }
    }
    if(query||state[id-1]) inject(id);
    return TransportResult::Ok;
  }
  void inject(std::uint8_t id) {
    std::array<std::uint8_t,kMaxCanPayloadBytes> p{};
    p[0]=static_cast<std::uint8_t>((state[id-1]<<4)|id);
    p[1]=0x7f;p[2]=0xff;p[3]=0x7f;p[4]=0xf7;p[5]=0xff;
    auto f=RawCanFrame::create(1,*CanId::create(0,CanFrameFormat::Standard),
        CanFrameType::Classic,FrameDirection::Tx,8,p,*MonotonicTime::from_nanoseconds(0));
    std::array<std::uint8_t,528> encoded{};std::size_t count=0;
    if(f&&UsbCdcCodec::encode(*f,encoded,count))
      (void)port.inject_rx({encoded.begin(),encoded.begin()+count});
  }
  mech::mech_simulation::FakeSerial port;
  std::array<std::uint8_t,2> state{};
  std::array<int,2> disables{};
  std::array<std::vector<std::array<std::uint8_t,8>>,2> motion;
  int opens=0;
};
hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  result.size = 1;
  return result;
}

hardware_interface::HardwareInfo info() {
  hardware_interface::HardwareInfo result;
  result.name = "dm4340_test";
  result.type = "system";
  result.hardware_class_type = "mech_bringup/Dm4340System";
  result.hardware_parameters = {
      {"profile", "dm4340_mit_standard"},
      {"device_path", "/dev/ttyACM1"},
      {"logical_bus", "1"},
      {"control_period_ns", "2000000"},
      {"command_ttl_ns", "4000000"},
      {"command_hard_ttl_ns", "6000000"},
      {"feedback_ttl_ns", "40000000"},
      {"query_period_ns", "20000000"},
      {"motor_side_timeout_verified", "true"},
      {"enable_on_activate", "false"},
      {"board_version_major", "4"},
      {"board_version_minor", "8"},
      {"board_version_patch", "8"},
      {"board_version_verified", "true"},
      {"can_bitrate_verified", "true"}};
  hardware_interface::ComponentInfo joint;
  joint.name = "knee_joint";
  joint.type = "joint";
  joint.command_interfaces = {
      interface(hardware_interface::HW_IF_VELOCITY),
      interface(mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  joint.state_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(hardware_interface::HW_IF_VELOCITY),
      interface(hardware_interface::HW_IF_EFFORT)};
  joint.parameters = {
      {"command_can_id", "2"}, {"feedback_can_id", "0"},
      {"motor_id", "2"}, {"firmware_id", "5010107"},
      {"firmware_min", "5010107"}, {"firmware_max", "5010107"},
      {"firmware_evidence_declared", "true"},
      {"limit_position_rad", "12.5"},
      {"limit_velocity_rad_s", "10"}, {"limit_torque_nm", "28"},
      {"limits_evidence_declared", "true"},
      {"joint_scale", "1"}, {"joint_offset_rad", "0"},
      {"joint_mapping_verified", "true"},
      {"position_min_rad", "-12.5"}, {"position_max_rad", "12.5"},
      {"position_guard_margin_rad", "0.02"},
      {"max_target_error_rad", "0.3"},
      {"kp", "0"}, {"kd", "1"},
      {"max_command_velocity_rad_s", "1.5"},
      {"max_feedback_velocity_rad_s", "2"},
      {"max_feedback_torque_nm", "5"},
      {"velocity_feedforward_nm", "0"}};
  result.joints.push_back(std::move(joint));
  return result;
}

class DmPositionManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const char* args[]={"dm_position_manager_test","--ros-args","--params-file",
      MECH_BRINGUP_SOURCE_DIR "/config/dm4340_dual_controllers.yaml"};
    rclcpp::init(4,args);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    auto h=info();h.name="dm_position_bench";
    h.hardware_parameters["enable_on_activate"]="true";
    h.joints[0].command_interfaces[0]=interface("position");
    h.joints[0].parameters["kp"]="1";
    h.joints[0].parameters["kd"]="0.2";
    h.joints[0].parameters["max_command_velocity_rad_s"]="0";
    h.joints.push_back(h.joints[0]);
    for(int i=0;i<2;++i) {
      h.joints[i].name="dm4340_motor"+std::to_string(i+1)+"_joint";
      h.joints[i].parameters["command_can_id"]=std::to_string(i+1);
      h.joints[i].parameters["motor_id"]=std::to_string(i+1);
    }
    serial=std::make_shared<PositionReplySerial>();
    auto sys=std::make_unique<Dm4340System>();
    sys->set_serial_factory_for_testing([this](const std::string&){return serial;});
    sys->set_clock_for_testing([this]{
      // Lifecycle polling must advance so staggered second-axis queries run.
      if(initializing) ns+=1000000;
      return *MonotonicTime::from_nanoseconds(ns);
    });
    sys->set_registry_for_testing(ownership);
    auto resources=std::make_unique<hardware_interface::ResourceManager>();
    resources->import_component(std::move(sys),h);
    rclcpp_lifecycle::State active{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE};
    ASSERT_EQ(resources->set_component_state(h.name,active),hardware_interface::return_type::OK);
    initializing=false;
    executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto options=controller_manager::get_cm_node_options();
    manager=std::make_shared<controller_manager::ControllerManager>(
      std::move(resources),executor,"controller_manager","",options);
    for(int i=0;i<2;++i) {
      controllers[i]=std::dynamic_pointer_cast<mech::mech_controllers::PositionCommandController>(
        manager->load_controller(names[i]));
      ASSERT_NE(controllers[i],nullptr);
      controllers[i]->set_clock_for_testing([this]{return ns;});
      ASSERT_EQ(manager->configure_controller(names[i]),controller_interface::return_type::OK);
    }
    cycle(1);
  }
  void TearDown() override {
    if(manager) {
      std::vector<std::string> stop;
      for(int i=0;i<2;++i) if(controllers[i]&&controllers[i]->get_state().id()==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)stop.push_back(names[i]);
      if(!stop.empty())(void)switch_to({},stop);
    }
    controllers={};manager.reset();executor.reset();serial.reset();
  }
  void cycle(int n) {
    for(int j=0;j<n;++j) {
      ns+=2000000;
      for(int i=0;i<2;++i) {
        serial->inject(i+1);
        if(refresh) (void)controllers[i]->set_target(i==0?.02:-.02);
      }
      rclcpp::Time time(ns,RCL_STEADY_TIME);rclcpp::Duration period(0,2000000);
      manager->read(time,period);manager->update(time,period);manager->write(time,period);
    }
  }
  controller_interface::return_type switch_to(const std::vector<std::string>& start,
                                              const std::vector<std::string>& stop) {
    auto f=std::async(std::launch::async,[&]{return manager->switch_controller(start,stop,
      controller_manager_msgs::srv::SwitchController::Request::STRICT);});
    for(int i=0;i<2000&&f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready;++i) {
      cycle(1);std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return f.get();
  }
  std::array<std::string,2> names{"dm4340_motor1_position_controller","dm4340_motor2_position_controller"};
  std::array<std::shared_ptr<mech::mech_controllers::PositionCommandController>,2> controllers;
  std::shared_ptr<PositionReplySerial> serial;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  BusOwnershipRegistry ownership;
  std::int64_t ns=1000000000000;
  bool refresh=false;
  bool initializing=true;
};

TEST_F(DmPositionManagerTest, DualNativeImpedanceNeedsTargetAndDoesNotReplayAfterRelease) {
  ASSERT_EQ(serial->opens,1);
  EXPECT_TRUE(serial->motion[0].empty());EXPECT_TRUE(serial->motion[1].empty());
  ASSERT_EQ(switch_to({names[0],names[1]},{}),controller_interface::return_type::OK);
  cycle(2);
  EXPECT_TRUE(serial->motion[0].empty());EXPECT_TRUE(serial->motion[1].empty());
  refresh=true;cycle(55);
  for(int i=0;i<2;++i) {
    ASSERT_FALSE(serial->motion[i].empty());
    const auto& p=serial->motion[i].back();
    const auto raw=static_cast<unsigned>((p[0]<<8)|p[1]);
    EXPECT_NEAR(raw*25.0/65535-12.5,i==0?.02:-.02,25.0/65535);
    EXPECT_EQ(((p[3]&15)<<8)|p[4],8); // Kp=1
    EXPECT_EQ((p[5]<<4)|(p[6]>>4),163); // Kd=.2
  }
  refresh=false;
  ASSERT_EQ(switch_to({},{names[0],names[1]}),controller_interface::return_type::OK);
  const auto n0=serial->motion[0].size(),n1=serial->motion[1].size();
  cycle(10);
  EXPECT_EQ(serial->motion[0].size(),n0);EXPECT_EQ(serial->motion[1].size(),n1);
  EXPECT_GE(serial->disables[0],1);EXPECT_GE(serial->disables[1],1);
  EXPECT_EQ(serial->state[0],0);EXPECT_EQ(serial->state[1],0);
}
}  // namespace
