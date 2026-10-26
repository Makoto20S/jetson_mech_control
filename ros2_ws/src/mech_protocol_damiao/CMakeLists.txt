cmake_minimum_required(VERSION 3.16)
project(mech_protocol_damiao LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(mech_control_core REQUIRED)

add_library(${PROJECT_NAME}
  src/package_marker.cpp
  src/dm4340_mit_wire.cpp
  src/dm4340_mit_session.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} mech_control_core)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mech_control_core)

option(MECH_BUILD_DAMIAO_RX_PROBE "Build receive-only USB-CAN bench probe" OFF)
if(MECH_BUILD_DAMIAO_RX_PROBE)
  add_executable(dm4340_rx_probe tools/dm4340_rx_probe.cpp)
  target_link_libraries(dm4340_rx_probe ${PROJECT_NAME})
  ament_target_dependencies(dm4340_rx_probe mech_control_core)
  install(TARGETS dm4340_rx_probe DESTINATION lib/${PROJECT_NAME})
endif()

option(MECH_BUILD_DAMIAO_ENABLE_BENCH "Build gated single-motor enable/disable bench tool" OFF)
if(MECH_BUILD_DAMIAO_ENABLE_BENCH)
  add_executable(dm4340_enable_bench tools/dm4340_enable_bench.cpp)
  target_link_libraries(dm4340_enable_bench ${PROJECT_NAME})
  ament_target_dependencies(dm4340_enable_bench mech_control_core)
  install(TARGETS dm4340_enable_bench DESTINATION lib/${PROJECT_NAME})
endif()

option(MECH_BUILD_DAMIAO_ROS_MONITOR "Build receive-only ROS 2 bench monitor" OFF)
if(MECH_BUILD_DAMIAO_ROS_MONITOR)
  find_package(diagnostic_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  add_executable(dm4340_ros_monitor tools/dm4340_ros_monitor.cpp)
  target_link_libraries(dm4340_ros_monitor ${PROJECT_NAME})
  ament_target_dependencies(dm4340_ros_monitor
    diagnostic_msgs mech_control_core rclcpp sensor_msgs)
  install(TARGETS dm4340_ros_monitor DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mech_simulation REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_package_marker.cpp
    test/test_dm4340_mit_wire.cpp
    test/test_dm4340_cdc_receive.cpp
    test/test_dm4340_mit_session.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  add_executable(${PROJECT_NAME}_offline_smoke test/offline_wire_smoke.cpp)
  target_link_libraries(${PROJECT_NAME}_offline_smoke ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_offline_smoke mech_simulation)
  add_test(NAME ${PROJECT_NAME}_offline_smoke
    COMMAND ${PROJECT_NAME}_offline_smoke)
endif()

ament_package()
