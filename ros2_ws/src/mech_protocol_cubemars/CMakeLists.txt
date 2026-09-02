cmake_minimum_required(VERSION 3.16)
project(mech_protocol_cubemars LANGUAGES CXX)

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
  src/ak30_force_wire.cpp
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
  # mech_simulation is required by the Task 8 simulator integration test; the
  # current marker test does not use it yet. Do not remove it as unused.
  find_package(mech_simulation REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_package_marker.cpp
    test/test_ak30_force_wire.cpp
    test/test_ak30_feedback_decode.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test mech_simulation)
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
