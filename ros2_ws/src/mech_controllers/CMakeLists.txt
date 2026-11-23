cmake_minimum_required(VERSION 3.16)
project(mech_controllers LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(mech_control_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ph_exo_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/package_marker.cpp
  src/position_command_controller.cpp
  src/single_joint_command_controller.cpp
  src/velocity_command_controller.cpp
  src/effort_command_controller.cpp
  src/resistance_training_controller.cpp
  src/fixed_position_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  mech_control_core)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools std_msgs std_srvs ph_exo_interfaces)

# Keep registration out of the linkable implementation: the hardware library
# uses TargetLimiter and must not register controller factories in its loader.
add_library(${PROJECT_NAME}_plugin SHARED
  src/position_command_controller_plugin.cpp
  src/velocity_command_controller_plugin.cpp
  src/effort_command_controller_plugin.cpp
  src/resistance_training_controller_plugin.cpp
  src/fixed_position_impedance_controller_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_plugin controller_interface pluginlib)

pluginlib_export_plugin_description_file(
  controller_interface mech_controllers_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_plugin LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface mech_control_core pluginlib rclcpp rclcpp_lifecycle
  realtime_tools std_msgs std_srvs ph_exo_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_position_command_controller.cpp
    test/test_velocity_command_controller.cpp
    test/test_effort_command_controller.cpp
    test/test_resistance_training_controller.cpp
    test/test_fixed_position_impedance_controller.cpp
    test/test_package_marker.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  add_executable(resistance_controller_replay test/resistance_controller_replay.cpp)
  target_link_libraries(resistance_controller_replay ${PROJECT_NAME})
  # The plugin-level tests stand up a real rclcpp node to exercise the
  # ~/target_position subscription, which under -fsanitize=address trips a
  # mismatch inside ROS 2 itself: rcutils_string_map_init() allocates 72 bytes
  # through operator new while rcutils_string_map_fini() releases it through
  # operator delete(void*, size_t) with size 1. The stack is entirely in
  # librcutils/librcl/librclcpp - no project code appears in it. This mirrors
  # the identical exemption mech_bringup's controller_manager test needs.
  # Only that single check is switched off, and by appending rather than
  # replacing, so the value run_sanitizers.sh sets survives and every other
  # AddressSanitizer check plus all of UndefinedBehaviorSanitizer stay active.
  set_tests_properties(${PROJECT_NAME}_test PROPERTIES
    ENVIRONMENT_MODIFICATION
      "ASAN_OPTIONS=string_append::new_delete_type_mismatch=0")
  ament_lint_cmake()
  ament_xmllint()
endif()


# Opt-in REF01 native software path; other controller builds need no MuJoCo SDK.
option(PH_EXO_ENABLE_MUJOCO "Build the REF01 native software controller and loop" OFF)
if(PH_EXO_ENABLE_MUJOCO)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "${MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
  file(GLOB MUJOCO_LIBRARIES "${MUJOCO_ROOT}/libmujoco.so.*")
  list(LENGTH MUJOCO_LIBRARIES MUJOCO_LIBRARY_COUNT)
  if(NOT MUJOCO_LIBRARY_COUNT EQUAL 1)
    message(FATAL_ERROR "MUJOCO_ROOT must contain exactly one pinned libmujoco.so")
  endif()
  list(GET MUJOCO_LIBRARIES 0 MUJOCO_LIBRARY)
  set(REF01_MODEL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco/ref01/cpp")
  add_library(${PROJECT_NAME}_ref01 SHARED src/ref01_impedance_controller.cpp)
  target_include_directories(${PROJECT_NAME}_ref01 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${REF01_MODEL_INCLUDE}>
    $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_ref01 "${MUJOCO_LIBRARY}")
  ament_target_dependencies(${PROJECT_NAME}_ref01
    controller_interface rclcpp rclcpp_lifecycle realtime_tools
    std_msgs std_srvs ph_exo_interfaces)
  set_target_properties(${PROJECT_NAME}_ref01 PROPERTIES INSTALL_RPATH "${MUJOCO_ROOT}")
  add_library(${PROJECT_NAME}_ref01_plugin SHARED src/ref01_impedance_controller_plugin.cpp)
  target_link_libraries(${PROJECT_NAME}_ref01_plugin ${PROJECT_NAME}_ref01)
  ament_target_dependencies(${PROJECT_NAME}_ref01_plugin controller_interface pluginlib)
  pluginlib_export_plugin_description_file(controller_interface ref01_impedance_plugin.xml)
  if(BUILD_TESTING)
    find_package(mech_exo_state REQUIRED)
    find_package(hardware_interface REQUIRED)
  add_executable(ref01_impedance_software_loop test/ref01_impedance_software_loop.cpp)
  target_link_libraries(ref01_impedance_software_loop ${PROJECT_NAME}_ref01)
  ament_target_dependencies(ref01_impedance_software_loop mech_exo_state hardware_interface rclcpp rclcpp_lifecycle)
  set_target_properties(ref01_impedance_software_loop PROPERTIES INSTALL_RPATH "${MUJOCO_ROOT}")
  install(TARGETS ref01_impedance_software_loop RUNTIME DESTINATION lib/${PROJECT_NAME})
  endif()
  install(TARGETS ${PROJECT_NAME}_ref01 ${PROJECT_NAME}_ref01_plugin LIBRARY DESTINATION lib)
endif()

ament_package()
