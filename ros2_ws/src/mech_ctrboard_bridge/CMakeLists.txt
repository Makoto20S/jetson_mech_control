cmake_minimum_required(VERSION 3.16)
project(mech_ctrboard_bridge LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mech_control_core REQUIRED)
find_package(mech_protocol_ctrboard REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ctrboard_bridge_node src/bridge_node.cpp)
target_include_directories(ctrboard_bridge_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(ctrboard_bridge_node
  geometry_msgs
  mech_control_core
  mech_protocol_ctrboard
  rclcpp
  sensor_msgs
  std_msgs)

install(TARGETS ctrboard_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_marker_test test/test_package_marker.cpp)
  target_include_directories(${PROJECT_NAME}_marker_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
