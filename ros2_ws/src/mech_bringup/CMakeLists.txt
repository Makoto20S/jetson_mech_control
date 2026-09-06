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

add_library(${PROJECT_NAME}
  src/package_marker.cpp
  src/foundation_harness.cpp
  src/posix_cdc_serial_port.cpp
  src/ak30_force_runtime.cpp
  src/ak30_runtime_params.cpp
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
  mech_protocol_cubemars)

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
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/development/foundation_validation.md
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/development/adapter_contract_v1.md
  DESTINATION share/${PROJECT_NAME}/docs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mech_control_core mech_simulation mech_hardware_ros2_control
  mech_controllers mech_protocol_cubemars)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_foundation_integration.cpp
    test/test_ak30_runtime.cpp
    test/test_ak30_runtime_params.cpp
    test/test_package_marker.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
