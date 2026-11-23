cmake_minimum_required(VERSION 3.16)
project(mech_simulation LANGUAGES CXX)

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
  src/loopback.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} mech_control_core)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mech_control_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_package_marker.cpp
    test/test_fake_clock_transport.cpp
    test/test_router_runtime.cpp
    test/test_usb_cdc_transport.cpp
    test/test_loopback.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_lint_cmake()
  ament_xmllint()
endif()

# Explicit opt-in; the default simulation package needs no native MuJoCo SDK.
option(PH_EXO_ENABLE_MUJOCO "Build REF01 software-only MuJoCo SystemInterface" OFF)
if(PH_EXO_ENABLE_MUJOCO)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "${MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
  file(GLOB MUJOCO_LIBRARIES "${MUJOCO_ROOT}/libmujoco.so.*")
  list(LENGTH MUJOCO_LIBRARIES MUJOCO_LIBRARY_COUNT)
  if(NOT MUJOCO_LIBRARY_COUNT EQUAL 1)
    message(FATAL_ERROR "MUJOCO_ROOT must contain exactly one pinned libmujoco.so")
  endif()
  list(GET MUJOCO_LIBRARIES 0 MUJOCO_LIBRARY)
  add_library(${PROJECT_NAME}_ref01 SHARED src/ref01_mujoco_system.cpp)
  target_include_directories(${PROJECT_NAME}_ref01 PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco/ref01/cpp"
    "${MUJOCO_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME}_ref01 "${MUJOCO_LIBRARY}")
  ament_target_dependencies(${PROJECT_NAME}_ref01
    hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  set_target_properties(${PROJECT_NAME}_ref01 PROPERTIES INSTALL_RPATH "${MUJOCO_ROOT}")
  pluginlib_export_plugin_description_file(hardware_interface ref01_mujoco_plugin.xml)
  install(TARGETS ${PROJECT_NAME}_ref01 LIBRARY DESTINATION lib)
  if(BUILD_TESTING)
    add_executable(ref01_sim_receiver_test test/test_ref01_sim_receiver.cpp)
    target_include_directories(ref01_sim_receiver_test PRIVATE include)
    add_test(NAME ref01_sim_receiver COMMAND ref01_sim_receiver_test)
  endif()
endif()

ament_package()
