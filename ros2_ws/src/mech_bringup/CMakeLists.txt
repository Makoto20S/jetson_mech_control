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

add_library(${PROJECT_NAME} src/package_marker.cpp)
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
  mech_controllers)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mech_control_core mech_simulation mech_hardware_ros2_control mech_controllers)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_package_marker.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
