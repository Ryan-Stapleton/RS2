cmake_minimum_required(VERSION 3.8)
project(trackit_nullspace)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trackit_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

ament_export_dependencies(rclcpp rclpy std_msgs sensor_msgs trackit_msgs)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(nullspace_projection src/nullspace_projection.cpp)
ament_target_dependencies(nullspace_projection rclcpp std_msgs sensor_msgs trackit_msgs)

add_executable(task_joint_middle src/task_joint_middle.cpp)
ament_target_dependencies(task_joint_middle rclcpp std_msgs sensor_msgs trackit_msgs)

# target_link_libraries(nullspace_projection task_joint_middle)

ament_package()