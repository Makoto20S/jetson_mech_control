cmake_minimum_required(VERSION 3.16)
project(mech_bringup LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(mech_control_core REQUIRED)
find_package(mech_simulation REQUIRED)
find_package(mech_hardware_ros2_control REQUIRED)
find_package(mech_controllers REQUIRED)
find_package(mech_protocol_cubemars REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

# SHARED is required for the pluginlib-loaded Ak30System: pluginlib dlopens
# the library by name, which cannot work against a static archive.
add_library(${PROJECT_NAME} SHARED
  src/package_marker.cpp
  src/foundation_harness.cpp
  src/posix_cdc_serial_port.cpp
  src/ak30_force_runtime.cpp
  src/ak30_runtime_params.cpp
  src/pass_through_init.cpp
  src/ak30_system.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  mech_control_core
  mech_simulation
  mech_hardware_ros2_control
  mech_controllers
  mech_protocol_cubemars
  hardware_interface
  pluginlib
  realtime_tools)

pluginlib_export_plugin_description_file(
  hardware_interface mech_bringup_plugins.xml)

# First real-device entry point: drives a motor through the repository's own
# UsbCdcTransport and Ak30ForceControlSession. Guarded behind an explicit
# option so default builds and CI never produce a device-touching binary.
option(MECH_BUILD_DEVICE_PROBES "Build device-touching bring-up probes" OFF)
if(MECH_BUILD_DEVICE_PROBES)
  add_executable(ak30_torque_probe src/ak30_torque_probe.cpp)
  add_executable(ak30_position_probe src/ak30_position_probe.cpp)
  target_include_directories(ak30_torque_probe PRIVATE include)
  target_include_directories(ak30_position_probe PRIVATE include)
  target_link_libraries(ak30_torque_probe ${PROJECT_NAME})
  target_link_libraries(ak30_position_probe ${PROJECT_NAME})
  ament_target_dependencies(ak30_torque_probe
    mech_control_core mech_protocol_cubemars)
  ament_target_dependencies(ak30_position_probe
    mech_control_core mech_protocol_cubemars)
  install(TARGETS ak30_torque_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(TARGETS ak30_position_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/development/foundation_validation.md
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/development/adapter_contract_v1.md
  DESTINATION share/${PROJECT_NAME}/docs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mech_control_core mech_simulation mech_hardware_ros2_control
  mech_controllers mech_protocol_cubemars realtime_tools)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  # Process-test-only hardware plugin. It exercises the production composite
  # system and AK3 runtime over FakeTransport, and is absent from production
  # builds and the production plugin manifest.
  add_library(${PROJECT_NAME}_e5_test_system SHARED
    test/e5_test_system.cpp
    src/ak30_force_runtime.cpp)
  target_include_directories(${PROJECT_NAME}_e5_test_system PRIVATE include)
  ament_target_dependencies(${PROJECT_NAME}_e5_test_system
    hardware_interface
    mech_control_core
    mech_hardware_ros2_control
    mech_protocol_cubemars
    mech_simulation
    pluginlib
    rclcpp
    std_msgs
    std_srvs)
  pluginlib_export_plugin_description_file(
    hardware_interface test/e5_test_plugins.xml)
  install(TARGETS ${PROJECT_NAME}_e5_test_system
    LIBRARY DESTINATION lib)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_foundation_integration.cpp
    test/test_ak30_runtime.cpp
    test/test_ak30_runtime_params.cpp
    test/test_ak30_system_integration.cpp
    test/test_ak30_system_plugin.cpp
    test/test_deployment_files.cpp
    test/test_package_marker.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_test PRIVATE
    MECH_BRINGUP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  # ADR-015 E1's last layer: the authorization boundary driven by a real
  # controller_manager. Kept in its own target because it needs an rclcpp
  # context and a ControllerManager node, which the offline unit tests do not.
  find_package(controller_manager REQUIRED)
  find_package(controller_interface REQUIRED)
  # Do not link either project plugin here: that would register factories before
  # the class loaders and hide the production hardware-first loading order.
  ament_add_gtest(${PROJECT_NAME}_plugin_loading_test
    test/test_plugin_loading.cpp)
  ament_target_dependencies(${PROJECT_NAME}_plugin_loading_test
    hardware_interface controller_interface pluginlib)
  ament_add_gtest(${PROJECT_NAME}_controller_manager_test
    test/test_controller_manager_integration.cpp
  )
  target_link_libraries(${PROJECT_NAME}_controller_manager_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_controller_manager_test
    controller_manager
    controller_interface)
  # This is the first test in the repository to construct an rclcpp::Node, and
  # under -fsanitize=address that trips a mismatch inside ROS 2 itself:
  # rcutils_string_map_init() allocates 72 bytes through operator new while
  # rcutils_string_map_fini() releases it through
  # operator delete(void*, size_t) with size 1. The stack is entirely in
  # librcutils/librcl/librclcpp, reached from ControllerManager's constructor -
  # no project code appears in it. Only that single check is switched off, and
  # by appending rather than replacing, so the value run_sanitizers.sh sets
  # survives and every other AddressSanitizer check plus all of
  # UndefinedBehaviorSanitizer stay active on this test.
  set_tests_properties(${PROJECT_NAME}_controller_manager_test PROPERTIES
    ENVIRONMENT_MODIFICATION
      "ASAN_OPTIONS=string_append::new_delete_type_mismatch=0")

  ament_add_test(${PROJECT_NAME}_activate_position_controller
    COMMAND bash
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/bench/test_activate_position_controller.sh
    TIMEOUT 35
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  add_launch_test(test/test_e5_process.launch.py
    TARGET ${PROJECT_NAME}_e5_process_lifetime
    TIMEOUT 45)
  add_launch_test(test/test_e5_process.launch.py
    TARGET ${PROJECT_NAME}_e5_process_feedback_loss
    TIMEOUT 35)
  set_tests_properties(${PROJECT_NAME}_e5_process_lifetime PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=41"
    RUN_SERIAL TRUE)
  set_tests_properties(${PROJECT_NAME}_e5_process_feedback_loss PROPERTIES
    ENVIRONMENT "E5_SCENARIO=feedback_loss;ROS_DOMAIN_ID=42"
    RUN_SERIAL TRUE)
  if(CMAKE_CXX_FLAGS MATCHES "fsanitize=address")
    execute_process(
      COMMAND "${CMAKE_CXX_COMPILER}" -print-file-name=libasan.so
      OUTPUT_VARIABLE MECH_ASAN_RUNTIME
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(EXISTS "${MECH_ASAN_RUNTIME}")
      set_tests_properties(${PROJECT_NAME}_e5_process_lifetime PROPERTIES
        ENVIRONMENT "ROS_DOMAIN_ID=41;MECH_ASAN_RUNTIME=${MECH_ASAN_RUNTIME}")
      set_tests_properties(${PROJECT_NAME}_e5_process_feedback_loss PROPERTIES
        ENVIRONMENT "E5_SCENARIO=feedback_loss;ROS_DOMAIN_ID=42;MECH_ASAN_RUNTIME=${MECH_ASAN_RUNTIME}")
    endif()
  endif()

  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
