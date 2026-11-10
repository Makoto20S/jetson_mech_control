cmake_minimum_required(VERSION 3.16)
project(mech_exo_state)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SourceStatus.msg"
  "msg/MitMotorFeedback.msg"
  "msg/RobotState.msg"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs)
# Install the canonical MuJoCo FK implementation and its model assets.
get_filename_component(REF01_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco/ref01" ABSOLUTE)
install(FILES "${REF01_MODEL_DIR}/ref01_mujoco_kinematics.py" DESTINATION lib/${PROJECT_NAME})
install(FILES "${REF01_MODEL_DIR}/config/mujoco/ref01_dynamics_cad_v1.xml"
  DESTINATION share/${PROJECT_NAME}/mujoco)
install(DIRECTORY "${REF01_MODEL_DIR}/config/mujoco/assets/ref01_cad_v1"
  DESTINATION share/${PROJECT_NAME}/mujoco/assets)
install(PROGRAMS scripts/state_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/state_core.py scripts/mit_feedback.py scripts/timing_trace.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ref01_fk test/test_ref01_fk.py
    APPEND_ENV "PYTHONPATH=${REF01_MODEL_DIR}" TIMEOUT 120)
  ament_add_pytest_test(state_core test/test_state_core.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:${REF01_MODEL_DIR}" TIMEOUT 30)
  ament_add_pytest_test(state_graph test/test_state_graph.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:${REF01_MODEL_DIR}" TIMEOUT 45)
  ament_add_pytest_test(timing_trace test/test_timing_trace.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:${REF01_MODEL_DIR}" TIMEOUT 30)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
