cmake_minimum_required(VERSION 3.5)
project(bicycle_sim)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  gazebo_ros
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclpy
  rcutils
  realtime_tools
  rosidl_default_generators
  sensor_msgs
  std_msgs
  std_srvs
  steering_controllers_library
  tf2_ros
  urdf
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# generate_parameter_library(bicycle_steering_controller_parameters
#   src/bicycle_steering_controller.yaml
# )

# add_library(
#   bicycle_steering_controller
#   SHARED
#   src/bicycle_steering_controller.cpp
# )

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY config launch meshes rviz src urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
