cmake_minimum_required(VERSION 3.16)
project(ph_exo_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FixedPosePlan.msg" "msg/ImpedanceStatus.msg" "msg/ImpedanceRef.msg"
  "srv/PrepareFixedPose.srv")
# Keep CTest output available to the workspace sanitizer build audit.
if(BUILD_TESTING)
  enable_testing()
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
