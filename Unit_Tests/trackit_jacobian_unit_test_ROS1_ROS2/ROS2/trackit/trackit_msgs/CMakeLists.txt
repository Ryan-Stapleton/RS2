cmake_minimum_required(VERSION 3.8)
project(trackit_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(trackit_msgs
  "msg/EDLSParam.msg"
  "msg/JacobianStamped.msg"
  "msg/JointLimitsStamped.msg"
  "msg/JointVelocityStamped.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs std_msgs)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_package()